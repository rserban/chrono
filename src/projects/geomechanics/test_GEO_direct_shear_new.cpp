#include <omp.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "chrono/ChConfig.h"

#include <chrono/functions/ChFunctionSetpoint.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMotorLinearPosition.h>
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/utils/ChFilters.h"

#include <chrono_multicore/physics/ChSystemMulticore.h>
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#include <chrono/utils/ChUtilsCreators.h>
#include <chrono/utils/ChUtilsSamplers.h>

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using std::cout;
using std::endl;
using std::string;

using namespace chrono;

string file_name;
string file_name_prefix("shear_results");
string csv_header("t,x,fm,fc,ffm,A,iter");
string settings_file_name("shear_settings.txt");
string data_file_name;
string data_file_name_prefix("final_data");

// UNIT SYSTEM: SI (kg, m, s)
// Conversion factors for specifications
double ft2in = 12;
double in2m = 0.0254;
double min2s = 60;
double g2kg = 1.0 / 1000.0;

double grav = 9.81;  // Magnitude of gravity in the -z direction

double sphere_inflation = 20;                     // Multiplier on particle radius
double sphere_radius = sphere_inflation * 50e-6;  // Particle radius 50um = 50e-6m
double sphere_density = 400;                      // Particle density 0.4 g/cm^3 = 400 kg/m^3
double sphere_mass = 4 * CH_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density / 3;
ChVector3d sphere_inertia = 2 * sphere_mass * sphere_radius * sphere_radius / 3 * ChVector3d(1, 1, 1);

// Particle material: Parameters to tune
float sphere_mu;         // Coefficient of friction
float sphere_cr = 0.0f;  // Coefficient of restitution

double uncompressed_volume =
    0.003873 * ft2in * ft2in * ft2in * in2m * in2m * in2m;  // 0.003873 ft^3 sample before compression

// Interior dimensions of shear chamber
double box_dim_X = 2.416 * in2m;  // 2.416in shear box diameter
double box_dim_Y = 2.416 * in2m;  // 2.416in shear box diameter
double box_dim_Z = uncompressed_volume / (box_dim_X * box_dim_Y);

double sampling_to_settled_ratio = 1.85;  // TODO tune

double sampling_dim_Z = box_dim_Z * sampling_to_settled_ratio;

double box_mass = 100;

// Box material
float box_mu = 0;
float box_cr = 0;

double dt = 1e-4;  // Simulation timestep
double tolerance = 0.1;
int max_iteration_normal = 0;
int max_iteration_sliding = 600;
int max_iteration_spinning = 0;
int max_iteration_bilateral = 100;
double contact_recovery_speed = 10e30;

bool clamp_bilaterals = false;
double bilateral_clamp_speed = 0.1;

double out_interval = 1e-2;  // Prints a status at this interval
unsigned int out_steps = static_cast<unsigned int>(out_interval / dt);

double plate_area = box_dim_X * box_dim_Y;
// Confining pressures expressed as mass per area kg/m2
// NOTE confining mass selected at runtime to allow batch runs
double confining_masses[] = {25, 100, 250, 500, 1000, 2000, 2500};
double plate_mass;  // confining_mass * plate_area;  // Plate mass necessary to generate the confining pressure

double settling_time = 0.11;

double max_compression_time = 0.35;  // Alternative stop condition for ending the compression phase

double shear_velocity_inflation = 1000;  // Multiplier on shear_velocity speed for shorter simulation
double shear_velocity = shear_velocity_inflation * 0.01 * in2m / min2s;  // X velocity of top shear section 0.01in/min
double shear_displacement = 0.25 * in2m;                                 // X displacement at which the test ends 0.25in
double shear_time = shear_displacement / shear_velocity;

double box_thick = 1.5 * shear_displacement;  // Thickness of walls so that no material spills during shearing

void AddBox(ChSystemMulticore& m_sys, std::shared_ptr<ChBody>& top) {
    auto box_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    box_mat->SetFriction(box_mu);
    box_mat->SetRestitution(box_cr);

    double hx = box_dim_X / 2;
    double hy = box_dim_Y / 2;
    double hz = box_dim_Z / 2;

    double hthick = box_thick / 2;

    // Top half of shear box
    bool top_vis = false;
    ChVector3d pos(0, 0, 0);
    top = chrono_types::make_shared<ChBody>();
    top->SetPos(pos);
    top->SetMass(box_mass / 2);
    top->SetFixed(true);
    utils::AddBoxGeometry(top.get(), box_mat, ChVector3d(hthick, hy, hz / 2), ChVector3d(-(hx + hthick), 0, hz / 2),
                          QUNIT, top_vis);  // Low X
    utils::AddBoxGeometry(top.get(), box_mat, ChVector3d(hthick, hy, hz / 2), ChVector3d(hx + hthick, 0, hz / 2), QUNIT,
                          top_vis);  // High X
    utils::AddBoxGeometry(top.get(), box_mat, ChVector3d(hx, hthick, hz / 2), ChVector3d(0, -(hy + hthick), hz / 2),
                          QUNIT, top_vis);  // Low Y
    utils::AddBoxGeometry(top.get(), box_mat, ChVector3d(hx, hthick, hz / 2), ChVector3d(0, hy + hthick, hz / 2), QUNIT,
                          top_vis);  // High Y
    top->EnableCollision(true);
    top->GetCollisionModel()->SetFamily(1);
    top->GetCollisionModel()->DisallowCollisionsWith(1);
    m_sys.AddBody(top);

    // Bottom half of shear box
    auto bot = chrono_types::make_shared<ChBody>();
    bot->SetPos(pos);
    bot->SetMass(box_mass / 2);
    bot->SetFixed(true);
    utils::AddBoxGeometry(bot.get(), box_mat, ChVector3d(hx, hy, hthick), ChVector3d(0, 0, -(hz + hthick)));  // Bottom
    utils::AddBoxGeometry(bot.get(), box_mat, ChVector3d(hthick, hy, hz / 2),
                          ChVector3d(-(hx + hthick), 0, -hz / 2));  // Low X
    utils::AddBoxGeometry(bot.get(), box_mat, ChVector3d(hthick, hy, hz / 2),
                          ChVector3d(hx + hthick, 0, -hz / 2));  // High X
    utils::AddBoxGeometry(bot.get(), box_mat, ChVector3d(hx, hthick, hz / 2),
                          ChVector3d(0, -(hy + hthick), -hz / 2));  // Low Y
    utils::AddBoxGeometry(bot.get(), box_mat, ChVector3d(hx, hthick, hz / 2),
                          ChVector3d(0, hy + hthick, -hz / 2));  // High Y
    bot->EnableCollision(true);
    bot->GetCollisionModel()->SetFamily(1);
    bot->GetCollisionModel()->DisallowCollisionsWith(1);
    m_sys.AddBody(bot);
}

void AddPlate(ChSystemMulticoreNSC& m_sys,
              std::shared_ptr<ChBody>& plate,
              std::shared_ptr<ChBody>& top,
              double plate_bottom) {
    auto box_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    box_mat->SetFriction(box_mu);
    box_mat->SetRestitution(box_cr);

    double hx = box_dim_X / 2;
    double hy = box_dim_Y / 2;
    double hz = box_dim_Z / 2;

    double hthick = box_thick / 2;

    // Plate for applying confining pressure
    plate = chrono_types::make_shared<ChBody>();
    plate->SetPos(ChVector3d(0, 0, plate_bottom + hthick));
    plate->SetMass(plate_mass);
    plate->SetFixed(false);
    utils::AddBoxGeometry(plate.get(), box_mat, ChVector3d(hx, hy, hthick));
    plate->EnableCollision(true);
    plate->GetCollisionModel()->SetFamily(1);
    plate->GetCollisionModel()->DisallowCollisionsWith(1);
    m_sys.AddBody(plate);

    auto prismatic_plate_box = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic_plate_box->Initialize(plate, top, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    m_sys.AddLink(prismatic_plate_box);
}

void AddMotor(ChSystemMulticoreNSC& m_sys,
              std::shared_ptr<ChBody>& top,
              std::shared_ptr<ChLinkMotorLinearPosition>& motor) {
    double hz = box_dim_Z / 2;

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->SetPos(ChVector3d(0, 0, 0));
    m_sys.AddBody(ground);

    auto pos_func = chrono_types::make_shared<ChFunctionSetpoint>();

    motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor->Initialize(top, ground, ChFrame<>(ChVector3d(0, 0, hz / 2), QUNIT));
    motor->SetMotionFunction(pos_func);
    m_sys.AddLink(motor);
}

void FixPlate(ChSystemMulticoreNSC& m_sys, std::shared_ptr<ChBody>& plate, std::shared_ptr<ChBody>& top) {
    ChQuaternion<> z2y;
    z2y.SetFromAngleAxis(-CH_PI / 2, ChVector3d(1, 0, 0));

    auto pin = chrono_types::make_shared<ChLinkLockPrismatic>();
    pin->Initialize(plate, top, ChFrame<>(ChVector3d(0, 0, 0), z2y));
    m_sys.AddLink(pin);
}

size_t AddParticles(ChSystemMulticoreNSC& m_sys, ChVector3d box_center, ChVector3d hdims) {
    auto sphere_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    sphere_mat->SetFriction(sphere_mu);
    sphere_mat->SetRestitution(sphere_cr);

    utils::ChPDSampler<> sampler(2.01 * sphere_radius);
    auto points = sampler.SampleBox(box_center, ChVector3d(hdims.x(), hdims.y(), hdims.z()));
    for (unsigned int i = 0; i < points.size(); i++) {
        auto sphere = chrono_types::make_shared<ChBody>();

        sphere->SetMass(sphere_mass);
        sphere->SetInertiaXX(sphere_inertia);
        sphere->SetPos(points[i]);
        sphere->SetRot(ChQuaternion<>(1, 0, 0, 0));
        sphere->SetFixed(false);
        sphere->EnableCollision(true);

        utils::AddSphereGeometry(sphere.get(), sphere_mat, sphere_radius);

        m_sys.AddBody(sphere);
    }

    return points.size();
}

void WriteParticles(ChSystemMulticoreNSC& m_sys, string file_name) {
    std::ofstream ostream;
    ostream.open(file_name);
    ostream << "x,y,z,U" << endl;
    for (auto body : m_sys.GetBodies()) {
        ChVector3d pos = body->GetPos();
        ostream << pos.x() << "," << pos.y() << "," << pos.z() << "," << body->GetPosDt().Length() << endl;
    }

    ostream.close();
}

int main(int argc, char* argv[]) {
    bool render = false;

    if (argc < 3) {
        cout << "usage: " << argv[0] << " <pressure_index: 0-6> <mu> [true]" << endl;
        return 1;
    } else if (argc > 3) {
        render = true;
    }

    double approx_particles = (box_dim_X - 2 * sphere_radius) * (box_dim_Y - 2 * sphere_radius) *
                              (box_dim_Z - 2 * sphere_radius) / (8 * sphere_radius * sphere_radius * sphere_radius);

    plate_mass = confining_masses[std::stoi(argv[1])] * plate_area;
    sphere_mu = std::stof(argv[2]);
    file_name = file_name_prefix + argv[1] + ".csv";
    data_file_name = data_file_name_prefix + argv[2] + ".dat";

    std::ofstream settings_stream;
    settings_stream.open(settings_file_name);
    settings_stream << "Unit system: SI (mks)" << endl;
    settings_stream << "Pressure index: " << std::stoi(argv[1]) << endl;
    settings_stream << "Plate mass: " << plate_mass << endl;
    settings_stream << "Data file: " << file_name << endl;
    settings_stream << "Final data file: " << data_file_name << endl;
    settings_stream << "Particle friction: " << sphere_mu << endl;
    settings_stream << "Particle radius: " << sphere_radius << endl;
    settings_stream << "Time step: " << dt << endl;
    settings_stream << "Box dimensions: " << box_dim_X << " X " << box_dim_Y << " X " << box_dim_Z << endl;
    settings_stream << "Approximate number of particles: " << static_cast<int>(approx_particles) << endl;
    settings_stream << "Settling time: " << settling_time << endl;
    settings_stream << "Compression time: " << max_compression_time << endl;
    settings_stream << "Shear velocity: " << shear_velocity << endl;
    settings_stream << "Shear displacement: " << shear_displacement << endl;
    settings_stream << "Shear time: " << shear_time << endl;

    ChSystemMulticoreNSC m_sys;

    m_sys.SetGravitationalAcceleration(ChVector3d(0, 0, -grav));
    m_sys.SetNumThreads(omp_get_num_procs());

    m_sys.GetSettings()->solver.use_full_inertia_tensor = false;
    m_sys.GetSettings()->solver.tolerance = tolerance;
    m_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    m_sys.GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
    m_sys.GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

    m_sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    m_sys.GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    m_sys.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    m_sys.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    m_sys.GetSettings()->solver.alpha = 0;
    m_sys.GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    m_sys.ChangeSolverType(SolverType::APGD);

    m_sys.GetSettings()->collision.collision_envelope = 0.1 * sphere_radius;

    double factor = 2 * 2 * sphere_radius;  // Length of each dimension of a bin
    double total_x = box_dim_X + 2 * box_thick + shear_displacement;
    double total_y = box_dim_Y + 2 * box_thick;
    double total_z = box_dim_Z + box_thick;

    int bins_x = static_cast<int>(std::ceil(total_x / factor));
    int bins_y = static_cast<int>(std::ceil(total_y / factor));
    int bins_z = static_cast<int>(std::ceil(total_z / factor));

    settings_stream << "Bins: " << bins_x << " X " << bins_y << " X " << bins_z << endl;

    m_sys.GetSettings()->collision.bins_per_axis = vec3(bins_x, bins_y, bins_z);
    m_sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    // Add containing box and compression plate
    std::shared_ptr<ChBody> plate;  // Weighted plate - top face of shear box
    std::shared_ptr<ChBody> top;    // Top section of the shear box
    AddBox(m_sys, top);

    // Add spherical particles
    auto num_particles = AddParticles(
        m_sys, ChVector3d(0, 0, -box_dim_Z / 2 + sampling_dim_Z / 2),
        ChVector3d(box_dim_X / 2 - sphere_radius, box_dim_Y / 2 - sphere_radius, sampling_dim_Z / 2 - sphere_radius));

    settings_stream << "Actual number of particles: " << num_particles << endl;
    settings_stream.close();

#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL vis;
    if (render) {
        vis.AttachSystem(&m_sys);
        vis.SetWindowTitle("Test");
        vis.SetWindowSize(1280, 720);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.AddCamera(ChVector3d(3 * box_dim_Y, 0, 0), ChVector3d(0, 0, 0));
        vis.SetCameraVertical(CameraVerticalDir::Z);
    }
#endif

    // Settle material under its own weight for a fixed amount of time
    cout << endl << "Running settling..." << endl;
    double m_time = 0;
    unsigned int step = 0;
    while (m_time < settling_time) {
        m_sys.DoStepDynamics(dt);
#ifdef CHRONO_OPENGL
        if (render) {
            if (vis.Run())
                vis.Render();
            else
                return 1;
        }
#endif
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            // WriteParticles(m_sys, string("points_compression") + std::to_string(step) + string(".csv"));
        }
        step++;
        m_time += dt;
    }

    // Add a weighted top plate
    double highest = -10e30;
    for (unsigned int i = 2; i < m_sys.GetBodies().size(); i++) {
        highest = std::max(highest, m_sys.GetBodies()[i]->GetPos().z());
    }

    highest += 2 * sphere_radius;

    AddPlate(m_sys, plate, top, highest);

    // Compress the material under the weight of the plate
    cout << endl << "Running compression..." << endl;
    m_time = 0;

    do {
        m_sys.DoStepDynamics(dt);
#ifdef CHRONO_OPENGL
        if (render) {
            if (vis.Run())
                vis.Render();
            else
                return 1;
        }
#endif
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            cout << std::setprecision(4) << "\tPlate height: " << plate->GetPos().z() << endl;
            // WriteParticles(m_sys, string("points_compression") + std::to_string(step) + string(".csv"));
        }
        m_time += dt;
        step++;
    } while (m_time < max_compression_time);

    // Create a motor to slide the top of the box in the +x direction
    std::shared_ptr<ChLinkMotorLinearPosition> motor;
    top->SetFixed(false);
    AddMotor(m_sys, top, motor);

    // Fix the plate to the top of the box
    ////FixPlate(m_sys, plate, top);

    // Run shearing for specified displacement
    cout << endl << "Running shear test..." << endl;

    std::ofstream ostream;
    ostream.open(file_name);
    ostream << csv_header << endl;

    // 5 Hz low pass filter
    utils::ChButterworthLowpass fm_lowpass5(1, dt, 5.0);
    double shear_force_motor_filtered;
    double shear_area;

    step = 0;
    m_time = 0;
    while (m_time < shear_time) {
        auto pos_func = std::static_pointer_cast<ChFunctionSetpoint>(motor->GetMotorFunction());
        double pos = step * dt * shear_velocity;
        double pos_dt = shear_velocity;
        pos_func->SetSetpointAndDerivatives(pos, pos_dt, 0);

        m_sys.DoStepDynamics(dt);
        m_time += dt;

#ifdef CHRONO_OPENGL
        if (render) {
            if (vis.Run())
                vis.Render();
            else
                break;
        }
#endif

        m_sys.CalculateContactForces();

        double shear_force_motor = motor->GetMotorForce();
        double shear_force_contact = m_sys.GetBodyContactForce(top).x;
        int iters = std::static_pointer_cast<ChIterativeSolverMulticore>(m_sys.GetSolver())->GetIterations();

        shear_force_motor_filtered = fm_lowpass5.Filter(shear_force_motor);
        shear_area = box_dim_Y * (box_dim_X - 2 * top->GetPos().x());

        // Output displacement and force
        if (step % out_steps == 0) {
            cout << std::setprecision(4) << "Time: " << m_time << endl;
            cout << std::setprecision(4) << "\tShear displacement: " << top->GetPos().x() << endl;
            cout << std::setprecision(4) << "\tShear force (motor): " << shear_force_motor << endl;
            cout << std::setprecision(4) << "\tShear force (contact): " << shear_force_contact << endl;
            cout << std::setprecision(4) << "\tShear area: " << shear_area << endl;
            cout << std::setprecision(4) << "\tIterations: " << iters << endl;

            // WriteParticles(m_sys, string("points_shear") + std::to_string(step) + string(".csv"));
        }
        step++;

        ostream << m_time << "," << top->GetPos().x() << "," << shear_force_motor << "," << shear_force_contact << ","
                << shear_force_motor_filtered << "," << shear_area << "," << iters << endl;
    }
    ostream.close();

    double normal_stress = (plate_mass * grav) / shear_area;
    double shear_stress = shear_force_motor_filtered / shear_area;

    cout << "\n\n" << endl;
    cout << "Normal stress: " << normal_stress << endl;
    cout << "Shear stress:  " << shear_stress << endl;

    std::ofstream data;
    data.open(data_file_name);
    data << "mu: " << sphere_mu << endl;
    data << "Normal stress: " << normal_stress << endl;
    data << "Shear stress:  " << shear_stress << endl;
    data.close();

    return 0;
}