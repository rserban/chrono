// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// LMTV acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChTimer.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/mtv/LMTV.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::fmtv;

// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType chassis_rear_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of powertrain model (SHAFTS, SIMPLE_MAP)
EngineModelType engine_model = EngineModelType::SIMPLE_MAP;
TransmissionModelType transmission_model = TransmissionModelType::AUTOMATIC_SIMPLE_MAP;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89, PAC02, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Terrain length (X direction)
double terrainLength = 300.0;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for LMTV: Cd = 0.5 and area ~5 m2
    LMTV lmtv;
    lmtv.SetContactMethod(ChContactMethod::SMC);
    lmtv.SetChassisFixed(false);
    lmtv.SetInitPosition(ChCoordsys<>(ChVector3d(-terrainLength / 2 + 5, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    lmtv.SetEngineType(engine_model);
    lmtv.SetTransmissionType(transmission_model);
    // lmtv.SetDriveType(drive_type);
    lmtv.SetTireType(tire_model);
    lmtv.SetTireStepSize(tire_step_size);
    lmtv.SetAerodynamicDrag(0.8, 5.76, 1.2);
    lmtv.Initialize();

    // Set subsystem visualization mode
    if (tire_model == TireModelType::RIGID_MESH)
        tire_vis_type = VisualizationType::MESH;

    lmtv.SetChassisVisualizationType(chassis_vis_type);
    lmtv.SetChassisRearVisualizationType(chassis_rear_vis_type);
    lmtv.SetSuspensionVisualizationType(suspension_vis_type);
    lmtv.SetSteeringVisualizationType(steering_vis_type);
    lmtv.SetWheelVisualizationType(wheel_vis_type);
    lmtv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    RigidTerrain terrain(lmtv.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, 5);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);
    terrain.Initialize();

    // Create the straight path and the driver system
    auto path = StraightLinePath(ChVector3d(-terrainLength / 2, 0, 0.5), ChVector3d(terrainLength * 10, 0, 0.5), 1);
    ChPathFollowerDriver driver(lmtv.GetVehicle(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("LMTV acceleration test");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&lmtv.GetVehicle());

    // ---------------
    // Simulation loop
    // ---------------

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunctionInterp speed_recorder;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    bool done = false;

    ChTimer timer;
    timer.start();
    while (vis->Run()) {
        time = lmtv.GetSystem()->GetChTime();

        double speed = speed_filter.Add(lmtv.GetVehicle().GetSpeed());
        if (!done) {
            speed_recorder.AddPoint(time, speed);
            if (time > 6 && std::abs((speed - last_speed) / step_size) < 2e-4) {
                done = true;
                timer.stop();
                std::cout << "Simulation time: " << timer() << std::endl;
                std::cout << "Maximum speed: " << speed << std::endl;
#ifdef CHRONO_POSTPROCESS
                postprocess::ChGnuPlot gplot;
                gplot.SetGrid();
                gplot.SetLabelX("time (s)");
                gplot.SetLabelY("speed (m/s)");
                gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif
            }
        }
        last_speed = speed;

        // End simulation
        if (time >= 100)
            break;

        vis->BeginScene();
        vis->Render();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        if (done) {
            driver_inputs.m_throttle = 0.1;
            driver_inputs.m_braking = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        lmtv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        lmtv.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        vis->EndScene();
    }

    return 0;
}
