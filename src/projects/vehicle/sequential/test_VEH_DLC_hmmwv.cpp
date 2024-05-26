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
// Double lane change.  Tire kinematics check.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Type of tire model (FIALA, PAC89, PAC02, or TMEASY)
TireModelType tire_model = TireModelType::FIALA;

// Simulation step size
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Output directories
const std::string out_dir = "../DLC_TIRE_TEST";

// =============================================================================

int main(int argc, char* argv[]) {
    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-75, 0, 0.5), QUNIT));
    my_hmmwv.SetEngineType(EngineModelType::SHAFTS);
    my_hmmwv.SetTransmissionType(TransmissionModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    my_hmmwv.SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Get handle to FR tire
    std::string outfile = "tire.out";
    auto FR_tire = my_hmmwv.GetVehicle().GetWheel(0, RIGHT)->GetTire();
    ChTMeasyTire* FR_TMeasy = nullptr;
    ChFialaTire* FR_Fiala = nullptr;
    ChPac89Tire* FR_Pac89 = nullptr;
    switch (tire_model) {
        case TireModelType::TMEASY:
            FR_TMeasy = static_cast<ChTMeasyTire*>(FR_tire.get());
            outfile = "tmeasy.out";
            break;
        case TireModelType::FIALA:
            FR_Fiala = static_cast<ChFialaTire*>(FR_tire.get());
            outfile = "fiala.out";
            break;
        case TireModelType::PAC89:
            FR_Pac89 = static_cast<ChPac89Tire*>(FR_tire.get());
            outfile = "pac89.out";
            break;
    }

    // Create the terrain
    ChContactMaterialData minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(my_hmmwv.GetSystem()->GetContactMethod());
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 20);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 20);
    terrain.Initialize();

    // Parameterized ISO double lane change (to left)
    auto path = DoubleLaneChangePath(ChVector3d(-75, 0, 0.1), 13.5, 4.0, 11.0, 50.0, true);
    auto npoints = path->GetNumPoints();
    double x_max = path->GetPoint(npoints - 1).x();

    // Parameterized NATO double lane change (to right)
    ////auto path = DoubleLaneChangePath(ChVector3d(-125, 0, 0.1), 28.93, 3.6105, 25.0, 50.0, false);

    // Create the driver system
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", 10.0);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle Irrlicht application
    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Suspension Test Rig");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&my_hmmwv.GetVehicle());

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // -----------------
    // Initialize output
    // -----------------
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counter and simulation time
    int sim_frame = 0;

    while (vis->Run()) {
        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        if (my_hmmwv.GetVehicle().GetPos().x() > x_max - 20) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_steering = 0;
            driver_inputs.m_braking = 1;
        }

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector3d& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector3d& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment simulation frame number
        sim_frame++;

    }

    return 0;
}
