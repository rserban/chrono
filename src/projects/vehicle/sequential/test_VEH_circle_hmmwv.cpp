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
// HMMWV constant radius turn.
// This program uses explicitly a ChPathSteeringControler.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono/geometry/ChLineBezier.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

double step_size = 2e-3;

double throttle_value = 0.3;

// =============================================================================

int main(int argc, char* argv[]) {
    // Create the HMMWV vehicle
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-75, 0, 0.5), QUNIT));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetSteeringType(SteeringType::PITMAN_ARM);
    my_hmmwv.SetTireType(TireModelType::TMEASY);
    my_hmmwv.SetTireStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    MaterialInfo minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(my_hmmwv.GetSystem()->GetContactMethod());
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 200, 200);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Left circle path
    auto path = CirclePath(ChVector<>(-75, 0, 0.6), 20, 40, true, 10);
    auto npoints = (unsigned int)path->getNumPoints();

    auto path_asset = chrono_types::make_shared<ChLineShape>();
    path_asset->SetLineGeometry(chrono_types::make_shared<geometry::ChLineBezier>(path));
    path_asset->SetName("test path");
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * npoints, 400));
    patch->GetGroundBody()->AddAsset(path_asset);

    // Create the PID lateral controller
    ChPathSteeringController steeringPID(path, false);
    steeringPID.SetLookAheadDistance(5);
    steeringPID.SetGains(0.8, 0, 0);
    steeringPID.Reset(my_hmmwv.GetVehicle());

    // Create the vehicle Irrlicht application
    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"Constant radius test");
    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, 0.f, 200.f), irr::core::vector3df(-150.f, 0.f, 200.f), 100, 100);
    app.AddTypicalLights(irr::core::vector3df(150.f, 0.f, 200.f), irr::core::vector3df(150.0f, 0.f, 200.f), 100, 100);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------

    double steeringPID_output = 0;

    while (app.GetDevice()->run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();

        // Driver inputs
        ChDriver::Inputs driver_inputs;
        driver_inputs.m_steering = ChClamp(steeringPID_output, -1.0, +1.0);
        driver_inputs.m_throttle = throttle_value;
        driver_inputs.m_braking = 0.0;

        // Update sentinel and target location markers for the path-follower controller.
        const ChVector<>& pS = steeringPID.GetSentinelLocation();
        const ChVector<>& pT = steeringPID.GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Update modules (process inputs from other modules)
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        steeringPID_output = steeringPID.Advance(my_hmmwv.GetVehicle(), step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);
    }

    return 0;
}