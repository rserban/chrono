// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Chrono::Vehicle + Chrono::Multicore program for simulating a HMMWV vehicle
// on granular terrain.
//
// Contact uses non-smooth (DVI) formulation.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdlib>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/physics/Ch3DOFContainer.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "driver_model.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Use 3DOF particles?
//#define USE_PARTICLES

// Container
double hdimX = 6;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.05;

// Fixed base layer?
bool rough = true;

// Depth of granular material (mm)
double depth = 100;
int num_layers_min = 4;
int num_layers_max = 8;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Type of wheel/tire (controls both contact and visualization)
enum WheelType { CYLINDRICAL, LUGGED };
WheelType wheel_type = CYLINDRICAL;

// Vehicle horizontal offset
double horizontal_offset = 2.5;
double horizontal_pos = hdimX - horizontal_offset;

// Initial vehicle position, orientation, and forward velocity
ChVector3d initLoc(-horizontal_pos, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);
double initSpeed = 0;

// Contact material properties for tires
float mu_t = 0.8f;
float cr_t = 0;

// -----------------------------------------------------------------------------
// Timed events
// -----------------------------------------------------------------------------

// Total simulation duration.
double time_end = 7;

// Time when the vehicle is created (allows for granular material settling)
double time_create_vehicle = 0.25;

// Duration before starting to apply throttle (allows for vehicle settling)
double delay_start_engine = 0.25;
double time_start_engine = time_create_vehicle + delay_start_engine;

// Delay before throttle reaches maximum (linear ramp)
double delay_max_throttle = 0.5;
double time_max_throttle = time_start_engine + delay_max_throttle;

// Time when terrain is pitched (rotate gravity)
double time_pitch = time_start_engine;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Enable thread tuning?
bool thread_tuning = false;

// Integration step size
double time_step = 1e-3;

// Solver settings
int max_iteration_bilateral = 100;
int max_iteration_normal = 0;
int max_iteration_sliding = 50;
int max_iteration_spinning = 0;

double tolerance = 1e-3;
double alpha = 0;
float contact_recovery_speed = 1000;

// Output
bool output = true;
double output_frequency = 100.0;
std::string out_dir = "../GONOGO_HMMWV";

// =============================================================================

// Custom material composition law.
// Use the maximum coefficient of friction.
class CustomCompositionStrategy : public ChContactMaterialCompositionStrategy {
  public:
    virtual float CombineFriction(float a1, float a2) const override { return std::max<float>(a1, a2); }
};

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& file,
                     int& line,
                     int& threads,
                     bool& render,
                     bool& copy,
                     bool& pov_output);

double CreateContainer(ChSystem* system, double mu, double coh, double radius);
int CreateParticles(ChSystemMulticoreNSC* system,
                    int num_layers,
                    double radius,
                    double rho,
                    double mu,
                    double coh,
                    double top_height);
double FindHighestParticle(ChSystem* system);
HMMWV_Full* CreateVehicle(ChSystem* system, double vertical_offset);
GONOGO_Driver* CreateDriver(ChVehicle& vehicle);

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50);
void TimingOutput(chrono::ChSystem* mSys, std::ofstream* ofile = NULL);

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    
    // ----------------------------
    // Parse command line arguments
    // ----------------------------

    std::string input_file = "";  // Name of input file
    int line_number = 0;          // Line of inputs
    int threads = 0;              // Number of threads
    bool render = true;           // Render?
    bool copy = true;             // Copy input file?
    bool pov_output = false;      // Currently not used

    // Extract arguments
    if (!GetProblemSpecs(argc, argv, input_file, line_number, threads, render, copy, pov_output)) {
        return 1;
    }

    // Check that input file exists
    filesystem::path inpath(input_file);
    if (!inpath.exists()) {
        cout << "Input file " << input_file << " does not exist" << endl;
        return 1;
    }
    else if (!inpath.is_file()) {
        cout << "Input file " << input_file << " is not a regular file" << endl;
        return 1;
    }

    if (line_number <= 0) {
        cout << "Incorrect line number." << endl;
        return 1;
    }

    // Check that the number of threads was specified
    if (threads <= 0) {
        cout << "Incorrect number of threads." << endl;
        return 1;
    }

    // ----------------
    // Parse input file
    // ----------------

    // Extract the filename, the basename, and extension of the input file
    std::string filename = inpath.filename();
    std::string stem = inpath.stem();
    std::string extension = inpath.extension();

    // Open input file
    std::ifstream ifile;
    ifile.open(input_file.c_str());

    std::string line;
    for (int i = 0; i < line_number; i++) {
        if (!std::getline(ifile, line) || line.length() == 0) {
            cout << "Incorrect line number." << endl;
            return 1;
        }
    }
    ifile.clear();
    ifile.seekg(0);

    // Extract input data
    std::istringstream iss(line);
    double slope_val, r_val, rho_val, mu_val, coh_val;
    iss >> slope_val >> r_val >> rho_val >> mu_val >> coh_val;

    double slope = slope_val * (CH_PI / 180);
    double r_g = r_val / 1000;
    double rho_g = rho_val;
    double mu_g = mu_val;
    double area = CH_PI * r_g * r_g;
    double coh_force = area * (coh_val * 1e3);
    double coh_g = coh_force * time_step;
    double envelope = 0.1 * r_g;

    int num_layers = static_cast<int>(depth / (2 * r_val));
    ChClampValue(num_layers, num_layers_min, num_layers_max);

    cout << "Set up" << endl;
    cout << "  File:     " << input_file << "  Line: " << line_number << endl;
    cout << "  Slope:    " << slope_val << "  " << slope << endl;
    cout << "  Radius:   " << r_val << "  " << r_g << endl;
    cout << "  Density:  " << rho_val << "  " << rho_g << endl;
    cout << "  Friction: " << mu_val << "  " << mu_g << endl;
    cout << "  Cohesion: " << coh_val << "  " << coh_g << endl;

    // --------------------------------
    // Create output directory and file
    // --------------------------------

    std::ofstream ofile;
    std::string del("  ");

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }

        out_dir += "/" + stem;

        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }

        // Copy input file to output directory
        if (copy) {
            std::ofstream dst(out_dir + "/" + filename, std::ios::binary);
            dst << ifile.rdbuf();
        }

        // Open the output file stream
        ofile.open(out_dir + "/results_" + std::to_string(line_number) + ".out", std::ios::out);
    }

    // -------------
    // Create system
    // -------------

    // Prepare rotated acceleration vector
    ChVector3d gravity(0, 0, -9.81);
    ChVector3d gravityR = ChMatrix33<>(slope, ChVector3d(0, 1, 0)) * gravity;

    ChSystemMulticoreNSC* system = new ChSystemMulticoreNSC();
    system->SetGravitationalAcceleration(gravity);
    system->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Use a custom material property composition strategy.
    // This ensures that tire-terrain interaction always uses the same coefficient of friction.
    std::unique_ptr<CustomCompositionStrategy> strategy(new CustomCompositionStrategy);
    system->SetMaterialCompositionStrategy(std::move(strategy));

    // Set number of threads
    ////cout << "Environment variables:" << endl;
    ////cout << "  OMP_NUM_THREADS = " << std::getenv("OMP_NUM_THREADS") << endl;

    // Set number of threads
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    if (thread_tuning)
        system->SetNumThreads(1, 1, threads);
    else
        system->SetNumThreads(threads);

    // --------------------
    // Edit system settings
    // --------------------

    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.compute_N = false;
    system->GetSettings()->solver.alpha = alpha;
    system->GetSettings()->solver.cache_step_length = true;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system->ChangeSolverType(SolverType::BB);
    ////system->SetLoggingLevel(LoggingLevel::LOG_INFO);
    ////system->SetLoggingLevel(LoggingLevel::LOG_TRACE);

    system->GetSettings()->collision.collision_envelope = envelope;
    system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    system->GetSettings()->collision.bins_per_axis = vec3(100, 30, 2);

    // Specify active box.
    system->GetSettings()->collision.use_aabb_active = false;
    system->GetSettings()->collision.aabb_min = real3(-1.1 * hdimX, -1.1 * hdimY, 0);
    system->GetSettings()->collision.aabb_max = real3(+1.1 * hdimX, +1.1 * hdimY, 10 * hdimZ);

    // ------------------
    // Create the terrain
    // ------------------

    double top_height = CreateContainer(system, mu_g, coh_g, (rough ? r_g : -1));
    int actual_num_particles = CreateParticles(system, num_layers, r_g, rho_g, mu_g, coh_g, top_height);
    cout << "Created " << actual_num_particles << " particles." << endl;

    // Save parameters and problem setup to output file
    if (output) {
        ofile << "# File: " << filename << endl;
        ofile << "# Line: " << line_number << endl;
        ofile << "# " << endl;
        ofile << "# " << line << endl;
        ofile << "# " << endl;
        ofile << "# Slope (deg):     " << slope_val << endl;
        ofile << "# Radius (mm):     " << r_val << endl;
        ofile << "# Density (kg/m3): " << rho_val << endl;
        ofile << "# Friction:        " << mu_val << endl;
        ofile << "# Cohesion (kPa):  " << coh_val << endl;
        ofile << "# " << endl;
        ofile << "# Num threads:     " << threads << endl;
        ofile << "# Num layers:      " << num_layers << endl;
        ofile << "# Num particles:   " << actual_num_particles << endl;
        ofile << "# " << endl;

        ofile.precision(7);
        ofile << std::scientific;
    }

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    opengl::ChVisualSystemOpenGL vis;
    if (render) {
        vis.AttachSystem(system);
        vis.SetWindowTitle("Test");
        vis.SetWindowSize(1280, 720);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.AddCamera(ChVector3d(0, -10, 0), ChVector3d(0, 0, 0));
        vis.SetCameraVertical(CameraVerticalDir::Z);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------

    HMMWV_Full* hmmwv = nullptr;
    GONOGO_Driver* driver = nullptr;
    FlatTerrain terrain(0);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil((1 / output_frequency) / time_step);

    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;

    bool is_pitched = false;
    double x_pos = -horizontal_pos;

    while (true) {
        // Check exit conditions
        if (x_pos >= horizontal_pos) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Reached maximum x position" << endl;
            }
            break;
        }

        if (x_pos <= -hdimX) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Vehicle sliding backward" << endl;
            }
            break;
        }

        if (time >= time_end) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Reached maximum time" << endl;
            }
            break;
        }

        // Create the vehicle
        if (!hmmwv && time > time_create_vehicle) {
            cout << time << "    Create vehicle" << endl;

            double max_height = FindHighestParticle(system);
            hmmwv = CreateVehicle(system, max_height);
            driver = CreateDriver(hmmwv->GetVehicle());

            next_out_frame = sim_frame;
        }

        // Rotate gravity vector
        if (!is_pitched && time > time_pitch) {
            cout << time << "    Pitch: " << gravityR.x() << " " << gravityR.y() << " " << gravityR.z() << endl;
            system->SetGravitationalAcceleration(gravityR);
            is_pitched = true;
        }

        if (hmmwv) {
            // Extract current driver inputs
            DriverInputs driver_inputs = driver->GetInputs();

            // Synchronize vehicle systems
            driver->Synchronize(time);
            hmmwv->Synchronize(time, driver_inputs, terrain);

            // Update vehicle x position
            x_pos = hmmwv->GetChassis()->GetPos().x();

            // Save output
            if (output && sim_frame == next_out_frame) {
                ChVector3d pv = hmmwv->GetChassisBody()->GetFrameRefToAbs().GetPos();
                ChVector3d vv = hmmwv->GetChassisBody()->GetFrameRefToAbs().GetPosDt();
                ChVector3d av = hmmwv->GetChassisBody()->GetFrameRefToAbs().GetPosDt2();

                ChVector3d v0 = hmmwv->GetVehicle().GetSpindleLinVel(0, LEFT);
                ChVector3d v1 = hmmwv->GetVehicle().GetSpindleLinVel(0, RIGHT);
                ChVector3d v2 = hmmwv->GetVehicle().GetSpindleLinVel(1, LEFT);
                ChVector3d v3 = hmmwv->GetVehicle().GetSpindleLinVel(1, RIGHT);

                ofile << system->GetChTime() << del;
                ofile << driver_inputs.m_throttle << del << driver_inputs.m_steering << del;

                ofile << pv.x() << del << pv.y() << del << pv.z() << del;
                ofile << vv.x() << del << vv.y() << del << vv.z() << del;
                ////ofile << av.x() << del << av.y() << del << av.z() << del;

                ////ofile << v0.x() << del << v0.y() << del << v0.z() << del;
                ////ofile << v1.x() << del << v1.y() << del << v1.z() << del;
                ////ofile << v2.x() << del << v2.y() << del << v2.z() << del;
                ////ofile << v3.x() << del << v3.y() << del << v3.z() << del;

                ofile << endl;

                out_frame++;
                next_out_frame += output_steps;
            }

            // Advance vehicle systems
            driver->Advance(time_step);
            hmmwv->Advance(time_step);
        } else {
            // Advance system state (no vehicle created yet)
            system->DoStepDynamics(time_step);
        }

#ifdef CHRONO_OPENGL
        if (render) {
            if (vis.Run()) {
                vis.Render();
            } else {
                break;
            }
        }
#endif

        // Display performance metrics
        TimingOutput(system);

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;

    if (output) {
        ofile << "# " << endl;
        ofile << "# Simulation time (s): " << exec_time << endl;
        ofile.close();
    }

    delete hmmwv;
    delete driver;

    return 0;
}

// =============================================================================

double CreateContainer(ChSystem* system,  // containing system
                       double mu,         // coefficient of friction
                       double coh,        // cohesion (constant impulse)
                       double radius      // radius of roughness spheres (if positive)
                       ) {
    bool visible_walls = false;

    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction((float)mu);
    material->SetCohesion((float)coh);
    material->SetCompliance(1e-9f);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetTag(-1);
    ground->SetMass(1000);
    ground->SetFixed(true);
    ground->EnableCollision(true);

    // Attention: collision family for ground should be >= 5 (first values used in Chrono::Vehicle)
    ground->GetCollisionModel()->SetFamily(5);
    ground->GetCollisionModel()->DisallowCollisionsWith(WheeledCollisionFamily::TIRE);

    // Bottom box
    utils::AddBoxGeometry(ground.get(), material, ChVector3d(hdimX, hdimY, hthick), ChVector3d(0, 0, -hthick),
                          ChQuaternion<>(1, 0, 0, 0), true);
    // Left box
    utils::AddBoxGeometry(ground.get(), material, ChVector3d(hdimX, hthick, hdimZ + hthick),
                          ChVector3d(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
    // Right box
    utils::AddBoxGeometry(ground.get(), material, ChVector3d(hdimX, hthick, hdimZ + hthick),
                          ChVector3d(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);

    // Front box
    utils::AddBoxGeometry(ground.get(), material, ChVector3d(hthick, hdimY, hdimZ + hthick),
                          ChVector3d(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
    // Rear box
    utils::AddBoxGeometry(ground.get(), material, ChVector3d(hthick, hdimY, hdimZ + hthick),
                          ChVector3d(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);

    // If a positive radius was provided, create a "rough" surface
    if (radius > 0) {
        double d = 4 * radius;
        int nx = (int)std::floor(hdimX / d);
        int ny = (int)std::floor(hdimY / d);
        for (int ix = -nx; ix <= nx; ix++) {
            for (int iy = -ny; iy <= ny; iy++) {
                utils::AddSphereGeometry(ground.get(), material, radius, ChVector3d(ix * d, iy * d, radius));
            }
        }
    }

    system->AddBody(ground);

    return (radius > 0) ? 2 * radius : 0;
}

// =============================================================================

int CreateParticles(ChSystemMulticoreNSC* system,  // containing system
                    int num_layers,               // number of layers
                    double radius,                // particle radius
                    double rho,                   // particle density
                    double mu,                    // coefficient of friction
                    double coh,                   // cohesion (constant impulse)
                    double top_height             // top height of rigid container
                    ) {
#ifdef USE_PARTICLES
    auto particle_container = chrono_types::make_shared<ChParticleContainer>();
    system->Add3DOFContainer(particle_container);

    particle_container->kernel_radius = 2 * radius;
    particle_container->mass = rho * (4 * CH_PI / 3) * std::pow(radius, 3);

    particle_container->contact_mu = mu;
    particle_container->contact_cohesion = coh;
    particle_container->contact_compliance = 1e-9;

    particle_container->mu = mu;
    particle_container->cohesion = coh;
    particle_container->compliance = 1e-9;

    particle_container->alpha = alpha;

    particle_container->contact_recovery_speed = contact_recovery_speed;
    particle_container->collision_envelope = 0.1 * radius;

    std::vector<real3> pos;
    std::vector<real3> vel;

    double r = 1.01 * radius;
    ChVector3d hdims(hdimX - r, hdimY - r, 0);
    ChVector3d center(0, 0, top_height + 2 * r);

    utils::ChPDSampler<> sampler(2 * r);
    for (int il = 0; il < num_layers; il++) {
        utils::ChGenerator::PointVector points = sampler.SampleBox(center, hdims);
        center.z() += 2 * r;
        for (int i = 0; i < points.size(); i++) {
            pos.push_back(real3(points[i].x(), points[i].y(), points[i].z()));
            vel.push_back(real3(0, 0, 0));
        }
    }

    particle_container->UpdatePosition(0);
    particle_container->AddBodies(pos, vel);

    return (int)pos.size();
#else
    // Create a material
    auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
    mat_g->SetFriction((float)mu);
    mat_g->SetCohesion((float)coh);
    mat_g->SetCompliance(1e-9f);

    // Create a particle generator and a mixture entirely made out of spheres
    double r = 1.01 * radius;
    utils::ChPDSampler<double> sampler(2 * r);
    utils::ChGenerator gen(system);
    std::shared_ptr<utils::ChMixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->SetDefaultMaterial(mat_g);
    m1->SetDefaultDensity(rho);
    m1->SetDefaultSize(radius);

    // Set starting value for body identifiers
    gen.SetStartTag(1);

    // Create particles in layers until reaching the desired number of particles
    ChVector3d hdims(hdimX - r, hdimY - r, 0);
    ChVector3d center(0, 0, top_height + 2 * r);

    for (int il = 0; il < num_layers; il++) {
        gen.CreateObjectsBox(sampler, center, hdims);
        center.z() += 2 * r;
    }

    return gen.GetTotalNumBodies();
#endif
}

double FindHighestParticle(ChSystem* system) {
    double highest = 0;
    for (auto body : system->GetBodies()) {
        if (body->GetTag() > 0 && body->GetPos().z() > highest)
            highest = body->GetPos().z();
    }
    return highest;
}

// =============================================================================

HMMWV_Full* CreateVehicle(ChSystem* system, double vertical_offset) {
    auto hmmwv = new HMMWV_Full(system);

    hmmwv->SetContactMethod(ChContactMethod::NSC);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(ChCoordsys<>(initLoc + ChVector3d(0, 0, vertical_offset), initRot));
    hmmwv->SetInitFwdVel(initSpeed);
    hmmwv->SetEngineType(EngineModelType::SIMPLE_MAP);
    hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    hmmwv->SetTireType(TireModelType::RIGID);

    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);

    return hmmwv;
}

GONOGO_Driver* CreateDriver(ChVehicle& vehicle) {
    double height = initLoc.z();
    auto path = StraightLinePath(ChVector3d(-10 * hdimX, 0, height), ChVector3d(10 * hdimX, 0, height));

    auto driver = new GONOGO_Driver(vehicle, path, time_start_engine, time_max_throttle);
    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;
    driver->SetLookAheadDistance(look_ahead_dist);
    driver->SetGains(Kp_steering, Ki_steering, Kd_steering);
    driver->Initialize();

    return driver;
}
