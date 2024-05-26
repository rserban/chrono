// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
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
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono::Multicore header files
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

// Chrono utility header files
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono vehicle header files
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

// M113 model header files
#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Comment the following line to use NSC contact
#define USE_SMC

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Dimensions
double hdimX = 60;  ////5.5; //// 2.5;
double hdimY = 3;   ////2.5;
double hthick = 0.25;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Initial vehicle position and orientation
ChVector3d initLoc(-hdimX + 8, 0, 1);
// ChVector3d initLoc(0, 0, 1);
ChQuaternion<> initRot(1, 0, 0, 0);

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 8;

// Total simulation duration.
double time_end = 30;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.2;

// Solver parameters
double time_step = 1e-3;  // 2e-4;

double tolerance = 0.01;

int max_iteration_bilateral = 1000;  // 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 200;  // 2000;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// Output
int out_fps = 60;

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
    for (unsigned int ix = 0; ix < c; ix++)
        std::cout << "=";
    for (unsigned int ix = c; ix < w; ix++)
        std::cout << " ";
    std::cout << "]\r" << std::flush;
}

// =============================================================================
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create system.
    // --------------

#ifdef USE_SMC
    std::cout << "Create Multicore SMC system" << std::endl;
    ChSystemMulticoreSMC* system = new ChSystemMulticoreSMC();
#else
    std::cout << "Create Multicore NSC system" << std::endl;
    ChSystemMulticoreNSC* system = new ChSystemMulticoreNSC();
#endif

    system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // ---------------------
    // Edit system settings.
    // ---------------------

    // Set number of threads
    system->SetNumThreads(threads);

    // Set solver parameters
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = tolerance;

#ifndef USE_SMC
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->ChangeSolverType(SolverType::APGD);
    system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
#else
    system->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
#endif

    system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // -------------------
    // Create the terrain.
    // -------------------

    // Contact material
    float mu_g = 0.8f;

#ifdef USE_SMC
    auto mat_g = chrono_types::make_shared<ChContactMaterialSMC>();
    mat_g->SetYoungModulus(1e8f);
    mat_g->SetFriction(mu_g);
    mat_g->SetRestitution(0.4f);
#else
    auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
    mat_g->SetFriction(mu_g);
#endif

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);

    chrono::utils::AddBoxGeometry(ground.get(),                                           //
                                  mat_g,                                                  //
                                  ChVector3d(hdimX, hdimY, hthick),                       //
                                  ChVector3d(0, 0, -hthick), ChQuaternion<>(1, 0, 0, 0),  //
                                  true);

    system->AddBody(ground);

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    bool fix_chassis = false;
    bool create_track = true;

    // Create and initialize vehicle system
    M113 m113(system);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetDrivelineType(DrivelineTypeTV::SIMPLE);
    m113.SetBrakeType(BrakeType::SIMPLE);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    m113.SetChassisCollisionType(CollisionType::NONE);

    m113.SetChassisFixed(fix_chassis);

    ////m113.GetVehicle().SetStepsize(0.0001);

    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();

    m113.SetChassisVisualizationType(VisualizationType::NONE);
    m113.SetSprocketVisualizationType(VisualizationType::MESH);
    m113.SetIdlerVisualizationType(VisualizationType::MESH);
    m113.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetRoadWheelVisualizationType(VisualizationType::MESH);
    m113.SetTrackShoeVisualizationType(VisualizationType::MESH);

    ////m113.GetVehicle().EnableCollision(TrackCollide::NONE);
    ////m113.GetVehicle().EnableCollision(TrackCollide::WHEELS_LEFT | TrackCollide::WHEELS_RIGHT);
    ////m113.GetVehicle().EnableCollision(TrackCollide::ALL & (~TrackCollide::SPROCKET_LEFT) &
    ///(~TrackCollide::SPROCKET_RIGHT));

    // Create the driver system
    ////ChDataDriver driver(m113.GetVehicle(), vehicle::GetDataFile("M113/driver/Acceleration.txt"));

    std::vector<ChDataDriver::Entry> driver_data;
    driver_data.push_back({0.0, 0, 0.0, 0, 0});
    driver_data.push_back({0.5, 0, 0.0, 0, 0});
    driver_data.push_back({2.0, 0, 0.5, 0, 0});
    driver_data.push_back({3.0, 0, 0.8, 0, 0});
    driver_data.push_back({4.0, 0, 1.0, 0, 0});
    ChDataDriver driver(m113.GetVehicle(), driver_data);

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize OpenGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(system);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(initLoc - ChVector3d(3.5, 4, 0), initLoc - ChVector3d(3.5, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

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
    BodyStates shoe_states_left(m113.GetVehicle().GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(m113.GetVehicle().GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(m113.GetVehicle().GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(m113.GetVehicle().GetNumTrackShoes(RIGHT));

    while (time < time_end) {
        // Collect output data from modules
        DriverInputs driver_inputs = driver.GetInputs();
        m113.GetVehicle().GetTrackShoeStates(LEFT, shoe_states_left);
        m113.GetVehicle().GetTrackShoeStates(RIGHT, shoe_states_right);

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
            cout << "     Vehicle speed:  " << m113.GetVehicle().GetSpeed() << endl;
            cout << "     Execution time: " << exec_time << endl;

            out_frame++;
            next_out_frame += out_steps;
            num_contacts = 0;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance simulation for one timestep for all modules
        driver.Advance(time_step);
        m113.Advance(time_step);
        system->DoStepDynamics(time_step);

        if (vis.Run())
            vis.Render();
        else
            break;

        progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);

        // Periodically display maximum constraint violation
        if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
            m113.GetVehicle().LogConstraintViolations();
        }

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
        num_contacts += system->GetNumContacts();
    }

    // Final stats
    std::cout << "==================================" << std::endl;
    std::cout << "Simulation time:   " << exec_time << std::endl;
    std::cout << "Number of threads: " << threads << std::endl;

    return 0;
}
