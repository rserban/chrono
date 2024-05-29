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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Problem parameters

// Type of tire model (RIGID, TMEASY, FIALA, PAC89)
TireModelType tire_model = TireModelType::TMEASY;

// Terrain friction coefficients
float friction_1 = 0.6f;
float friction_2 = 0.9f;

// Simulation step size, end time
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Output
bool output = true;
double out_fps = 50;
const std::string out_dir = "../HMMWV_FRICTION";

// =============================================================================

int main(int argc, char* argv[]) {
    // Chrono system
    ChSystemNSC sys;
    ////ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Create the terrain (will contain 2 side-by-side patches)
    RigidTerrain terrain(&sys);

    // Create and initialize the first vehicle
    HMMWV_Full hmmwv_1(&sys);
    hmmwv_1.SetInitPosition(ChCoordsys<>(ChVector3d(-90, -5.5, 1.0), QUNIT));
    hmmwv_1.SetEngineType(EngineModelType::SHAFTS);
    hmmwv_1.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv_1.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv_1.SetTireType(tire_model);
    hmmwv_1.SetTireStepSize(tire_step_size);
    hmmwv_1.Initialize();
    hmmwv_1.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv_1.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_1.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    ChContactMaterialData minfo_1;
    minfo_1.mu = friction_1;
    minfo_1.cr = 0.01f;
    minfo_1.Y = 2e7f;
    auto mat_1 = minfo_1.CreateMaterial(sys.GetContactMethod());
    auto patch_1 = terrain.AddPatch(mat_1, ChCoordsys<>(ChVector3d(0, -5.5, 0), QUNIT), 200, 10);
    patch_1->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch_1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_1 = StraightLinePath(ChVector3d(-90, -5.5, 0.1), ChVector3d(+90, -5.5, 0.1));
    ChPathFollowerDriver driver_1(hmmwv_1.GetVehicle(), path_1, "path_1", 10.0);
    driver_1.GetSteeringController().SetLookAheadDistance(5);
    driver_1.GetSteeringController().SetGains(0.5, 0, 0);
    driver_1.GetSpeedController().SetGains(0.4, 0, 0);
    driver_1.Initialize();

    // Create and initialize the second vehicle
    HMMWV_Full hmmwv_2(&sys);
    hmmwv_2.SetInitPosition(ChCoordsys<>(ChVector3d(-90, +5.5, 1.0), QUNIT));
    hmmwv_2.SetEngineType(EngineModelType::SHAFTS);
    hmmwv_2.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv_2.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv_2.SetTireType(tire_model);
    hmmwv_2.SetTireStepSize(tire_step_size);
    hmmwv_2.Initialize();
    hmmwv_2.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv_2.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_2.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    ChContactMaterialData minfo_2;
    minfo_2.mu = friction_2;
    minfo_2.cr = 0.01f;
    minfo_2.Y = 2e7f;
    auto mat_2 = minfo_2.CreateMaterial(sys.GetContactMethod());
    auto patch_2 = terrain.AddPatch(mat_2, ChCoordsys<>(ChVector3d(0, +5.5, 0), QUNIT), 200, 10);
    patch_2->SetColor(ChColor(1.0f, 0.8f, 0.8f));
    patch_2->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto path_2 = StraightLinePath(ChVector3d(-90, +5.5, 0.1), ChVector3d(+90, +5.5, 0.1));
    ChPathFollowerDriver driver_2(hmmwv_2.GetVehicle(), path_2, "path_2", 0.0);
    driver_2.GetSteeringController().SetLookAheadDistance(5);
    driver_2.GetSteeringController().SetGains(0.5, 0, 0);
    driver_2.GetSpeedController().SetGains(0.4, 0, 0);
    driver_2.Initialize();

    terrain.Initialize();

    std::string modelname;
    switch (tire_model) {
        case TireModelType::TMEASY:
            modelname = "tmeasy";
            break;
        case TireModelType::FIALA:
            modelname = "fiala";
            break;
        case TireModelType::PAC89:
            modelname = "pac89";
            break;
        case TireModelType::RIGID:
            modelname = "rigid";
            break;
    }

    // Create the vehicle Irrlicht interface (associated with 2nd vehicle)
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Terrain friction test");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
    vis->SetChaseCameraState(utils::ChChaseCamera::Track);
    vis->SetChaseCameraPosition(ChVector3d(-50, -10, 2.0));
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&hmmwv_2.GetVehicle());

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    // ---------------
    // Simulation loop
    // ---------------

    double out_step_size = 1 / out_fps;
    int out_steps = (int)std::ceil(out_step_size / step_size);
    int sim_frame = 0;

    while (vis->Run()) {
        double time = sys.GetChTime();

        double veh_pos_1 = hmmwv_1.GetVehicle().GetPos().x();
        double veh_pos_2 = hmmwv_2.GetVehicle().GetPos().x();

        // Extract output
        if (output && sim_frame % out_steps == 0) {
            double veh_speed_1 = hmmwv_1.GetVehicle().GetSpeed();
            double fr_omega_1 = hmmwv_1.GetVehicle().GetSpindleOmega(0, RIGHT);
            double rr_omega_1 = hmmwv_1.GetVehicle().GetSpindleOmega(1, RIGHT);

            double veh_speed_2 = hmmwv_2.GetVehicle().GetSpeed();
            double fr_omega_2 = hmmwv_2.GetVehicle().GetSpindleOmega(0, RIGHT);
            double rr_omega_2 = hmmwv_2.GetVehicle().GetSpindleOmega(1, RIGHT);

            csv << time;
            csv << veh_pos_1 << veh_speed_1 << fr_omega_1 << rr_omega_1;
            csv << veh_pos_2 << veh_speed_2 << fr_omega_2 << rr_omega_2;
            csv << std::endl;
        }

        // Stop befoe end of terrain patches
        if (veh_pos_1 > 80 || veh_pos_2 > 80) {
            break;
        }

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        double throttle_input = 0;
        if (time > 1 && time < 2)
            throttle_input = 0.75 * (time - 1);
        else if (time > 2)
            throttle_input = 0.75;

        // Driver inputs
        DriverInputs driver_inputs_1 = driver_1.GetInputs();
        DriverInputs driver_inputs_2 = driver_2.GetInputs();
        driver_inputs_1.m_throttle = throttle_input;
        driver_inputs_2.m_throttle = throttle_input;
        driver_inputs_1.m_braking = 0;
        driver_inputs_2.m_braking = 0;

        // Update modules (process inputs from other modules)
        driver_1.Synchronize(time);
        driver_2.Synchronize(time);
        hmmwv_1.Synchronize(time, driver_inputs_1, terrain);
        hmmwv_2.Synchronize(time, driver_inputs_2, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs_2);

        // Advance simulation for one timestep for all modules.
        driver_1.Advance(step_size);
        driver_2.Advance(step_size);
        hmmwv_1.Advance(step_size);
        hmmwv_2.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    if (output) {
        csv.WriteToFile(out_dir + "/" + modelname + ".out");
    }

    return 0;
}
