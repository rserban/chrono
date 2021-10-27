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
// Test influence of terrain friction
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/m113/M113_SimpleCVTPowertrain.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================
// Problem parameters

// Terrain friction coefficients
float friction_1 = 0.1f;
float friction_2 = 0.9f;

// Simulation step size, end time
double step_size = 1e-3;

// Output
bool output = false;
double out_fps = 10;
const std::string out_dir = "../M113_FRICTION";

// =============================================================================

int main(int argc, char* argv[]) {
    // Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetSolverMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Create the terrain (will contain 2 side-by-side patches)
    RigidTerrain terrain(&sys);

    // Create and initialize the first vehicle
    M113_Vehicle vehicle_1(false, TrackShoeType::SINGLE_PIN, DrivelineTypeTV::SIMPLE, BrakeType::SIMPLE, false, &sys,
                           CollisionType::NONE);
    vehicle_1.Initialize(ChCoordsys<>(ChVector<>(-90.0, -5.5, 1.0), QUNIT));
    vehicle_1.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle_1.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_1.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    auto powertrain_1 = chrono_types::make_shared<M113_SimpleCVTPowertrain>("Powertrain1");
    vehicle_1.InitializePowertrain(powertrain_1);

    MaterialInfo minfo_1;
    minfo_1.mu = friction_1;
    minfo_1.cr = 0.01f;
    minfo_1.Y = 2e7f;
    auto mat_1 = minfo_1.CreateMaterial(sys.GetContactMethod());
    auto patch_1 = terrain.AddPatch(mat_1, ChVector<>(0, -5.5, 0), ChVector<>(0, 0, 1), 200, 10);
    patch_1->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch_1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_1 = StraightLinePath(ChVector<>(-90, -5.5, 0.1), ChVector<>(+90, -5.5, 0.1));
    ChPathFollowerDriver driver_1(vehicle_1, path_1, "path_1", 10.0);
    driver_1.GetSteeringController().SetLookAheadDistance(5);
    driver_1.GetSteeringController().SetGains(0.5, 0, 0);
    driver_1.GetSpeedController().SetGains(0.4, 0, 0);
    driver_1.Initialize();

    // Create and initialize the second vehicle
    M113_Vehicle vehicle_2(false, TrackShoeType::SINGLE_PIN, DrivelineTypeTV::SIMPLE, BrakeType::SIMPLE, false, &sys,
                           CollisionType::NONE);
    vehicle_2.Initialize(ChCoordsys<>(ChVector<>(-90.0, +5.5, 1.0), QUNIT));
    vehicle_2.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle_2.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle_2.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    auto powertrain_2 = chrono_types::make_shared<M113_SimpleCVTPowertrain>("Powertrain2");
    vehicle_2.InitializePowertrain(powertrain_2);

    MaterialInfo minfo_2;
    minfo_2.mu = friction_2;
    minfo_2.cr = 0.01f;
    minfo_2.Y = 2e7f;
    auto mat_2 = minfo_2.CreateMaterial(sys.GetContactMethod());
    auto patch_2 = terrain.AddPatch(mat_2, ChVector<>(0, +5.5, 0), ChVector<>(0, 0, 1), 200, 10);
    patch_2->SetColor(ChColor(1.0f, 0.8f, 0.8f));
    patch_2->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_2 = StraightLinePath(ChVector<>(-90, +5.5, 0.1), ChVector<>(+90, +5.5, 0.1));
    ChPathFollowerDriver driver_2(vehicle_2, path_2, "path_2", 0.0);
    driver_2.GetSteeringController().SetLookAheadDistance(5);
    driver_2.GetSteeringController().SetGains(0.5, 0, 0);
    driver_2.GetSpeedController().SetGains(0.4, 0, 0);
    driver_2.Initialize();

    terrain.Initialize();

    // Create the vehicle Irrlicht interface (associated with 2nd vehicle)
    ChTrackedVehicleIrrApp app(&vehicle_2, L"Terrain friction test");
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    app.SetChaseCameraState(utils::ChChaseCamera::Track);
    app.SetChaseCameraPosition(ChVector<>(-50, -10, 2.0));
    app.SetTimestep(step_size);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left_1(vehicle_1.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right_1(vehicle_1.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left_1(vehicle_1.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right_1(vehicle_1.GetNumTrackShoes(RIGHT));

    BodyStates shoe_states_left_2(vehicle_2.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right_2(vehicle_2.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left_2(vehicle_2.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right_2(vehicle_2.GetNumTrackShoes(RIGHT));

    // Output interval
    double out_step_size = 1 / out_fps;
    int out_steps = (int)std::ceil(out_step_size / step_size);
    int sim_frame = 0;

    while (app.GetDevice()->run()) {
        double time = sys.GetChTime();

        double veh_pos_1 = vehicle_1.GetVehiclePos().x();
        double veh_pos_2 = vehicle_2.GetVehiclePos().x();
        double veh_speed_1 = vehicle_1.GetVehicleSpeed();
        double veh_speed_2 = vehicle_2.GetVehicleSpeed();

        // Extract output
        if (output && sim_frame % out_steps == 0) {
            csv << time;
            csv << veh_pos_1 << veh_speed_1;
            csv << veh_pos_2 << veh_speed_2;
            csv << std::endl;

            std::cout << veh_pos_1 << "   " << veh_pos_2 << std::endl;
        }

        // Stop befoe end of terrain patches
        if (veh_pos_1 > 80 || veh_pos_2 > 80) {
            break;
        }

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        double throttle_input = 0;
        if (time > 1 && time < 2)
            throttle_input = 0.75 * (time - 1);
        else if (time > 2)
            throttle_input = 0.75;

        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs_1 = driver_1.GetInputs();
        ChDriver::Inputs driver_inputs_2 = driver_2.GetInputs();
        driver_inputs_1.m_throttle = throttle_input;
        driver_inputs_2.m_throttle = throttle_input;
        driver_inputs_1.m_braking = 0;
        driver_inputs_2.m_braking = 0;

        vehicle_1.GetTrackShoeStates(LEFT, shoe_states_left_1);
        vehicle_1.GetTrackShoeStates(RIGHT, shoe_states_right_1);

        vehicle_2.GetTrackShoeStates(LEFT, shoe_states_left_2);
        vehicle_2.GetTrackShoeStates(RIGHT, shoe_states_right_2);

        // Update modules (process inputs from other modules)
        driver_1.Synchronize(time);
        driver_2.Synchronize(time);
        vehicle_1.Synchronize(time, driver_inputs_1, shoe_forces_left_1, shoe_forces_right_1);
        vehicle_2.Synchronize(time, driver_inputs_2, shoe_forces_left_2, shoe_forces_right_2);
        terrain.Synchronize(time);
        app.Synchronize("", driver_inputs_2);

        // Advance simulation for one timestep for all modules.
        driver_1.Advance(step_size);
        driver_2.Advance(step_size);
        vehicle_1.Advance(step_size);
        vehicle_2.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    if (output) {
        csv.write_to_file(out_dir + "/m113.out");
    }

    return 0;
}
