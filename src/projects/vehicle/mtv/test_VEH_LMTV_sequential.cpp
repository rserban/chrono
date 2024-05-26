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
// Authors: Rainer Gericke
// =============================================================================
//
// Demo program for LMTV simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/mtv/LMTV.h"
#include "chrono_models/vehicle/mtv/LMTV_LeafspringAxle.h"
#include "chrono_models/vehicle/mtv/FMTV_ToebarLeafspringAxle.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::fmtv;

// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(0, 0, 0.4);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType chassis_rear_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "LMTV";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the vehicle, set parameters, and initialize
    LMTV lmtv;
    lmtv.SetContactMethod(ChContactMethod::NSC);
    lmtv.SetChassisFixed(false);
    lmtv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    lmtv.SetTireType(tire_model);
    lmtv.SetTireStepSize(tire_step_size);
    lmtv.SetInitFwdVel(0.0);
    lmtv.Initialize();

    lmtv.SetChassisVisualizationType(chassis_vis_type);
    lmtv.SetChassisRearVisualizationType(chassis_rear_vis_type);
    lmtv.SetSuspensionVisualizationType(suspension_vis_type);
    lmtv.SetSteeringVisualizationType(steering_vis_type);
    lmtv.SetWheelVisualizationType(wheel_vis_type);
    lmtv.SetTireVisualizationType(tire_vis_type);

    {
        auto suspF = std::static_pointer_cast<ChToeBarLeafspringAxle>(lmtv.GetVehicle().GetSuspension(0));
        double leftAngle = suspF->GetKingpinAngleLeft();
        double rightAngle = suspF->GetKingpinAngleRight();

        auto springFL = suspF->GetSpring(VehicleSide::LEFT);
        auto shockFL = suspF->GetShock(VehicleSide::RIGHT);

        std::cout << "Spring rest length front: " << springFL->GetRestLength() << std::endl;
        std::cout << "Shock rest length front:  " << shockFL->GetRestLength() << std::endl;
    }
    {
        auto suspR = std::static_pointer_cast<ChLeafspringAxle>(lmtv.GetVehicle().GetSuspension(1));
        auto springRL = suspR->GetSpring(VehicleSide::LEFT);
        auto shockRL = suspR->GetShock(VehicleSide::RIGHT);

        std::cout << "Spring rest length rear: " << springRL->GetRestLength() << std::endl;
        std::cout << "Shock rest length rear:  " << shockRL->GetRestLength() << std::endl;
    }

    std::cout << "Vehicle mass:               " << lmtv.GetVehicle().GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.8f);
    patch_mat->SetRestitution(0.01f);
    RigidTerrain terrain(lmtv.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("LMTV demo");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&lmtv.GetVehicle());

    // Create the interactive driver system
    ChInteractiveDriverIRR driver(*vis);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;
    int render_frame = 0;

    double maxKingpinAngle = 0.0;

    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        double time = lmtv.GetSystem()->GetChTime();

        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output && step_number % render_steps == 0) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(lmtv.GetSystem(), filename);
                render_frame++;
            }
        }

        // Collect output data from modules (for inter-module communication)
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        lmtv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Test for validity of kingpin angles (max. allowed by lmtv: 27 deg)
        auto suspF = std::static_pointer_cast<ChToeBarLeafspringAxle>(lmtv.GetVehicle().GetSuspension(0));
        double leftAngle = suspF->GetKingpinAngleLeft() * 180.0 / CH_PI;
        double rightAngle = suspF->GetKingpinAngleRight() * 180.0 / CH_PI;
        if (std::abs(leftAngle) > maxKingpinAngle) {
            maxKingpinAngle = std::abs(leftAngle);
        }
        if (std::abs(rightAngle) > maxKingpinAngle) {
            maxKingpinAngle = std::abs(rightAngle);
        }

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        lmtv.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    std::cout << "Maximum Kingpin Angle = " << maxKingpinAngle << " deg" << std::endl;
    return 0;
}
