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
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtils.h"

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
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-75, 0, 0.5), QUNIT));
    my_hmmwv.SetEngineType(EngineModelType::SHAFTS);
    my_hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    my_hmmwv.SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    my_hmmwv.SetTireType(TireModelType::TMEASY);
    my_hmmwv.SetTireStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    ChContactMaterialData minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(my_hmmwv.GetSystem()->GetContactMethod());
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 200);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Left circle path
    auto path = CirclePath(ChVector3d(-75, 0, 0.6), 20, 40, true, 10);
    auto npoints = (unsigned int)path->GetNumPoints();

    auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<ChLineBezier>(path));
    path_asset->SetName("test path");
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * npoints, 400));
    patch->GetGroundBody()->AddVisualShape(path_asset);

    // Create the PID lateral controller
    ChPathSteeringController steeringPID(path);
    steeringPID.SetLookAheadDistance(5);
    steeringPID.SetGains(0.8, 0, 0);
    steeringPID.Reset(my_hmmwv.GetVehicle().GetRefFrame());

    // Create the vehicle Irrlicht application
    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Constant radius test");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->SetHUDLocation(500, 20);
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

    // ---------------
    // Simulation loop
    // ---------------

    double steeringPID_output = 0;

    while (vis->Run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();

        // Driver inputs
        DriverInputs driver_inputs;
        driver_inputs.m_steering = ChClamp(steeringPID_output, -1.0, +1.0);
        driver_inputs.m_throttle = throttle_value;
        driver_inputs.m_braking = 0.0;

        // Update sentinel and target location markers for the path-follower controller.
        const ChVector3d& pS = steeringPID.GetSentinelLocation();
        const ChVector3d& pT = steeringPID.GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Update modules (process inputs from other modules)
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        steeringPID_output = steeringPID.Advance(my_hmmwv.GetVehicle().GetRefFrame(), time, step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}
