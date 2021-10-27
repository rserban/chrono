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
// Authors: Radu Serban
// =============================================================================
//
//
// =============================================================================

#include <iostream>

// Chrono::Engine header files
#include "chrono/ChConfig.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono::Multicore header files
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

// Chrono::Multicore OpenGL header files
//#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

// Chrono utility header files
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono vehicle header files
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

// M113 model header files
#include "chrono_models/vehicle/m113a/M113a_Vehicle.h"
#include "chrono_models/vehicle/m113a/M113a_SimplePowertrain.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Comment the following line to use Chrono::Multicore
//#define USE_SEQ

// Comment the following line to use NSC contact
#define USE_SMC

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

enum TerrainType { RIGID_TERRAIN, GRANULAR_TERRAIN };

// Type of terrain
TerrainType terrain_type = GRANULAR_TERRAIN;

// Control visibility of containing bin walls
bool visible_walls = true;

// Dimensions
double hdimX = 10;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.25;
double numLayers = 12;

// Parameters for granular material
int Id_g = 100;
double r_g = 0.008;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_C_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1, 1, 1);
double coh_pressure = 8e4;
double coh_force = CH_C_PI * r_g * r_g * coh_pressure * 0;

float mu_g = 0.9f;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Initial vehicle position and orientation
ChVector<> initLoc(-hdimX + 4.5, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Simple powertrain model
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Total simulation duration.
double time_end = 7;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.2;

// Solver parameters
#ifdef USE_SMC
double time_step = 5e-5;
#else
double time_step = 1e-3;  // 2e-4;
#endif

double tolerance = 0.1;

int max_iteration_bilateral = 100;  // 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 200;  // 2000;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// Output directories
bool povray_output = true;

const std::string out_dir = "../M113_MULTICORE_ACCEL";
const std::string pov_dir = out_dir + "/POVRAY";

int out_fps = 60;

// =============================================================================

class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 4.2) {
            m_throttle = 0;
            m_braking = 1;
        } else if (eff_time > 0.2) {
            m_throttle = 0.8;
        } else {
            m_throttle = 4 * eff_time;
        }
    }

  private:
    double m_delay;
};

// =============================================================================

double CreateParticles(ChSystem* system) {
// Create a material
#ifdef USE_SMC
    auto mat_g = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(0.0f);
    mat_g->SetYoungModulus(8e5f);
    mat_g->SetPoissonRatio(0.3f);
    mat_g->SetAdhesion(static_cast<float>(coh_force));
    mat_g->SetKn(1.0e6f);
    mat_g->SetGn(6.0e1f);
    mat_g->SetKt(4.0e5f);
    mat_g->SetGt(4.0e1f);
#else
    auto mat_g = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(0.0f);
    mat_g->SetCohesion(coh_force);
#endif

    // Create a particle generator and a mixture entirely made out of spheres
    double r = 1.01 * r_g;
    utils::PDSampler<double> sampler(2 * r);
    utils::Generator gen(system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->setDefaultMaterial(mat_g);
    m1->setDefaultDensity(rho_g);
    m1->setDefaultSize(r_g);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(Id_g);

    // Create particles in layers until reaching the desired number of particles
    ChVector<> hdims(hdimX - r, hdimY - r, 0);
    ChVector<> center(0, 0, 2 * r);

    double layerCount = 0;
    while (layerCount < numLayers) {
        gen.CreateObjectsBox(sampler, center, hdims);
        center.z() += 2 * r;
        layerCount++;
    }

    std::cout << "Created " << gen.getTotalNumBodies() << " particles." << std::endl;

    return center.z();
}

// =============================================================================
// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50) {
    if ((x != n) && (x % (n / 100 + 1) != 0))
        return;

    float ratio = x / (float)n;
    unsigned int c = (unsigned int)(ratio * w);

    std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
    for (unsigned int x = 0; x < c; x++)
        std::cout << "=";
    for (unsigned int x = c; x < w; x++)
        std::cout << " ";
    std::cout << "]\r" << std::flush;
}

// =============================================================================
int main(int argc, char* argv[]) {
    // -----------------
    // Initialize output
    // -----------------

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

// --------------
// Create system.
// --------------

#ifdef USE_SEQ
// ----  Sequential
#ifdef USE_SMC
    std::cout << "Create SMC system" << std::endl;
    ChSystemSMC* system = new ChSystemSMC();
#else
    std::cout << "Create NSC system" << std::endl;
    ChSystemNSC* system = new ChSystemNSC();
#endif

#else
// ----  Multicore
#ifdef USE_SMC
    std::cout << "Create multicore SMC system" << std::endl;
    ChSystemMulticoreSMC* system = new ChSystemMulticoreSMC();
#else
    std::cout << "Create multicore NSC system" << std::endl;
    ChSystemMulticoreNSC* system = new ChSystemMulticoreNSC();
#endif

#endif

    system->Set_G_acc(ChVector<>(0, 0, -9.80665));

// ---------------------
// Edit system settings.
// ---------------------

#ifdef USE_SEQ

    ////system->SetSolverType(ChSolver::Type::MINRES);
    system->SetMaxItersSolverSpeed(50);
    system->SetMaxItersSolverStab(50);
////system->SetTol(0);
////system->SetMaxPenetrationRecoverySpeed(1.5);
////system->SetMinBounceSpeed(2.0);
////system->SetSolverOverrelaxationParam(0.8);
////system->SetSolverSharpnessParam(1.0);

#else

    // Set number of threads
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    if (thread_tuning)
        system->SetNumThreads(1, 1, threads);
    else
        system->SetNumThreads(threads);

    // Set solver parameters
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

#ifndef USE_SMC
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->ChangeSolverType(SolverType::APGD);
    system->GetSettings()->collision.collision_envelope = 0.001;
#else
    system->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
    system->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
    system->GetSettings()->solver.use_material_properties = true;
#endif

    system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    int factor = 2;
    int binsX = (int)std::ceil(hdimX / r_g) / factor;
    int binsY = (int)std::ceil(hdimY / r_g) / factor;
    int binsZ = 1;
    system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    cout << "broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << endl;

#endif

// -------------------
// Create the terrain.
// -------------------

// Contact material
#ifdef USE_SMC
    auto mat_g = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat_g->SetYoungModulus(1e8f);
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(0.4f);
#else
    auto mat_g = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat_g->SetFriction(mu_g);
#endif

    // Ground body
    auto ground = std::shared_ptr<ChBody>(system->NewBody());
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->GetCollisionModel()->ClearModel();

    // Bottom box
    utils::AddBoxGeometry(ground.get(), mat_g,                                    //
                          ChVector<>(hdimX, hdimY, hthick),                       //
                          ChVector<>(0, 0, -hthick), ChQuaternion<>(1, 0, 0, 0),  //
                          true);
    if (terrain_type == GRANULAR_TERRAIN) {
        // Front box
        utils::AddBoxGeometry(ground.get(), mat_g,                                                         //
                              ChVector<>(hthick, hdimY, hdimZ + hthick),                                   //
                              ChVector<>(+hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
        // Rear box
        utils::AddBoxGeometry(ground.get(), mat_g,                                                         //
                              ChVector<>(hthick, hdimY, hdimZ + hthick),                                   //
                              ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
        // Left box
        utils::AddBoxGeometry(ground.get(), mat_g,                                                         //
                              ChVector<>(hdimX, hthick, hdimZ + hthick),                                   //
                              ChVector<>(0, +hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
        // Right box
        utils::AddBoxGeometry(ground.get(), mat_g,                                                         //
                              ChVector<>(hdimX, hthick, hdimZ + hthick),                                   //
                              ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),  //
                              visible_walls);
    }

    ground->GetCollisionModel()->BuildModel();

    system->AddBody(ground);

    // Create the granular material.
    double vertical_offset = 0;

    if (terrain_type == GRANULAR_TERRAIN) {
        vertical_offset = CreateParticles(system);
    }

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    // Create and initialize vehicle system
    M113a_Vehicle vehicle(true, system);
    ////vehicle.SetStepsize(0.0001);
    vehicle.Initialize(ChCoordsys<>(initLoc + ChVector<>(0.0, 0.0, vertical_offset), initRot));

    // Set visualization type for subsystems
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSprocketVisualizationType(VisualizationType::MESH);
    vehicle.SetIdlerVisualizationType(VisualizationType::MESH);
    vehicle.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::MESH);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::MESH);

    ////vehicle.SetCollide(TrackCollide::NONE);
    ////vehicle.SetCollide(TrackCollide::WHEELS_LEFT | TrackCollide::WHEELS_RIGHT);
    ////vehicle.SetCollide(TrackCollide::ALL & (~TrackCollide::SPROCKET_LEFT) & (~TrackCollide::SPROCKET_RIGHT));

    // Create the powertrain system
    auto powertrain = chrono_types::make_shared<M113a_SimplePowertrain>("Powertrain");
    vehicle.InitializePowertrain(powertrain);

    // Create the driver system
    MyDriver driver(vehicle, 0.5);
    driver.Initialize();

    // ------------------------------------
    // Prepare output directories and files
    // ------------------------------------

    // Create output directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

// ---------------
// Simulation loop
// ---------------

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "M113", system);
    gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

    // Number of simulation steps between two 3D view render frames
    int out_steps = (int)std::ceil((1.0 / time_step) / out_fps);

    // Run simulation for specified time.
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    while (time < time_end) {
        // Collect output data from modules
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        ChVector<> vel_CG = vehicle.GetChassisBody()->GetPos_dt();
        vel_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(vel_CG);

        // Vehicle and Control Values
        csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
        csv << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetAxleSpeed()
            << vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetAxleSpeed();
        csv << powertrain->GetMotorSpeed() << powertrain->GetMotorTorque();
        csv << powertrain->GetOutputTorque() << vehicle.GetDriveline()->GetDriveshaftSpeed();
        // Chassis Position & Velocity
        csv << vehicle.GetChassis()->GetPos().x() << vehicle.GetChassis()->GetPos().y()
            << vehicle.GetChassis()->GetPos().z();
        csv << vel_CG.x() << vel_CG.y() << vel_CG.z();
        csv << std::endl;

        // Output
        if (sim_frame == next_out_frame) {
            cout << endl;
            cout << "---- Frame:          " << out_frame + 1 << endl;
            cout << "     Sim frame:      " << sim_frame << endl;
            cout << "     Time:           " << time << endl;
            cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
            cout << "     Throttle input: " << driver_inputs.m_throttle << endl;
            cout << "     Braking input:  " << driver_inputs.m_braking << endl;
            cout << "     Steering input: " << driver_inputs.m_steering << endl;
            cout << "     Execution time: " << exec_time << endl;

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame + 1);
                utils::WriteVisualizationAssets(system, filename);
            }

            out_frame++;
            next_out_frame += out_steps;
            num_contacts = 0;

            csv.write_to_file(out_dir + "/output.dat");
        }

        // Release the vehicle chassis at the end of the hold time.
        if (vehicle.GetChassis()->IsFixed() && time > time_hold) {
            std::cout << std::endl << "Release vehicle t = " << time << std::endl;
            vehicle.GetChassisBody()->SetBodyFixed(false);
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance simulation for one timestep for all modules
        driver.Advance(time_step);
        vehicle.Advance(time_step);

#ifdef CHRONO_OPENGL
        if (gl_window.Active())
            gl_window.Render();
        else
            break;
#endif

        progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);

        // Periodically display maximum constraint violation
        if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
            vehicle.LogConstraintViolations();
        }

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
        num_contacts += system->GetNcontacts();
    }

    // Final stats
    std::cout << "==================================" << std::endl;
    std::cout << "Simulation time:   " << exec_time << std::endl;
    std::cout << "Number of threads: " << threads << std::endl;

    csv.write_to_file(out_dir + "/output.dat");

    return 0;
}
