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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Main driver function for a mrole specified through JSON files + example for
// obtaining shock effect results presented by ISO 2631-5
//
// Halfround shaped obstacles
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/mrole/mrole.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
    // specify whether the demo should actually use Irrlicht
    #define USE_IRRLICHT
#endif

// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;
using namespace chrono::vehicle::mrole;

// Tire collision type
ChTire::CollisionType collision_type = ChTire::CollisionType::ENVELOPE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector3d initLoc(-40, 0, 0.7);

// Simulation step size (should not be too high!)
double step_size = 1e-3;

const double mph_to_ms = 0.44704;

// Road visualization (mesh or boundary lines)
bool useMesh = true;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const int heightVals[16] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30};
    int iObstacle = 1;
    double target_speed = 5.0;
    // CRG files for terrain
    std::string crg_terrain_file("terrain/crg_roads/halfround_2in.crg");

    switch (argc) {
        default:
        case 1:
            std::cout << "usage: demo_VEH_Shock [ObstacleNumber [Speed_mph]]\n\n";
            std::cout << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " in Obstacle Height)\n"
                     << "Speed       = " << target_speed << " mph\n";
            break;
        case 2:
            iObstacle = ChClamp(atoi(argv[1]), 1, 15);
            std::cout << "usage: demo_VEH_Shock [ObstacleNumber [Speed_mph]]\n\n";
            std::cout << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " in Obstacle Height)\n"
                     << "Speed       = " << target_speed << " mph\n";
            break;
        case 3:
            iObstacle = ChClamp(atoi(argv[1]), 1, 15);
            target_speed = ChClamp<double>(atof(argv[2]), 1, 100);
            std::cout << "usage: demo_VEH_Shock [ObstacleNumber [Speed_mph]]\n\n";
            std::cout << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " in Obstacle Height)\n"
                     << "Speed       = " << target_speed << " mph\n";
            break;
    }
    switch (iObstacle) {
        case 2:
            crg_terrain_file = "terrain/crg_roads/halfround_4in.crg";
            break;
        case 3:
            crg_terrain_file = "terrain/crg_roads/halfround_6in.crg";
            break;
        case 4:
            crg_terrain_file = "terrain/crg_roads/halfround_8in.crg";
            break;
        case 5:
            crg_terrain_file = "terrain/crg_roads/halfround_10in.crg";
            break;
        case 6:
            crg_terrain_file = "terrain/crg_roads/halfround_12in.crg";
            break;
        case 7:
            crg_terrain_file = "terrain/crg_roads/halfround_14in.crg";
            break;
        case 8:
            crg_terrain_file = "terrain/crg_roads/halfround_16in.crg";
            break;
        case 9:
            crg_terrain_file = "terrain/crg_roads/halfround_18in.crg";
            break;
        case 10:
            crg_terrain_file = "terrain/crg_roads/halfround_20in.crg";
            break;
        case 11:
            crg_terrain_file = "terrain/crg_roads/halfround_22in.crg";
            break;
        case 12:
            crg_terrain_file = "terrain/crg_roads/halfround_24in.crg";
            break;
        case 13:
            crg_terrain_file = "terrain/crg_roads/halfround_26in.crg";
            break;
        case 14:
            crg_terrain_file = "terrain/crg_roads/halfround_28in.crg";
            break;
        case 15:
            crg_terrain_file = "terrain/crg_roads/halfround_30in.crg";
            break;
    }
    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    mrole_Full mrole;
    mrole.SetContactMethod(ChContactMethod::NSC);
    mrole.SetChassisFixed(false);
    mrole.SetInitPosition(ChCoordsys<>(initLoc, QUNIT));
    mrole.SetTireType(tire_model);
    mrole.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    mrole.SetInitFwdVel(target_speed * mph_to_ms);
    mrole.Initialize();

    mrole.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetWheelVisualizationType(VisualizationType::NONE);
    mrole.SetTireVisualizationType(VisualizationType::MESH);
    ////mrole.GetChassis()->SetFixed(true);

    std::cout << "\nBe patient - startup may take some time... \n";

    // Create the ground
    // RigidTerrain terrain(mrole.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    CRGTerrain terrain(mrole.GetSystem());
    terrain.UseMeshVisualization(useMesh);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.Initialize(vehicle::GetDataFile(crg_terrain_file));

    ChISO2631_Shock_SeatCushionLogger seat_logger(step_size);

#ifdef USE_IRRLICHT
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Multi Role Vehicle Shock Test");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&mrole.GetVehicle());
#endif

    // Create the driver
    auto path = ChBezierCurve::Read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(mrole.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // Logging of seat acceleration data on flat road surface is useless
    double xstart = 40.0;  // start logging when the mrole crosses this x position
    double xend = 100.0;   // end logging here, this also the end of our world

#ifdef USE_IRRLICHT

    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = mrole.GetSystem()->GetChTime();
        driver.Synchronize(time);
        mrole.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        mrole.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);

        double xpos = mrole.GetVehicle().GetPos().x();
        if (xpos >= xend) {
            break;
        }
        if (xpos >= xstart) {
            ChVector3d seat_acc = mrole.GetVehicle().GetPointAcceleration(
                mrole.GetVehicle().GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(seat_acc);
        }

        vis->EndScene();
    }

#else
    const double ms_to_mph = 2.2369362921;

    double v_pos;
    while ((v_pos = mrole.GetVehicle().GetPos().x()) < xend) {
        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = mrole.GetSystem()->GetChTime();
        driver.Synchronize(time);
        mrole.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        mrole.Advance(step_size);
        terrain.Advance(step_size);

        if (v_pos >= xstart) {
            double speed = mrole.GetSpeed();
            ChVector3d seat_acc = mrole.GetPointAcceleration(mrole.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(seat_acc);
        }
    }

#endif

    const double ms_to_mph = 2.2369362921;
    double se_low = 0.5;
    double se_high = 0.8;
    double se = seat_logger.GetSe();

    double az_limit = 2.5;
    double az = seat_logger.GetLegacyAz();

    std::cout << "Shock Simulation Results #1 (ISO 2631-5 Method):\n";
    std::cout << "  Significant Speed                        Vsig = " << target_speed << " m/s\n";
    std::cout << "  Equivalent Static Spine Compressive Stress Se = " << se << " MPa\n";
    if (se <= se_low) {
        std::cout << "Se <= " << se_low << " MPa (ok) - low risc of health effect, below limit for average occupants\n";
    } else if (se >= se_high) {
        std::cout << "Se >= " << se_high << " MPa - severe risc of health effect!\n";
    } else {
        std::cout << "Se is between [" << se_low << ";" << se_high << "] - risc of health effects, above limit!\n";
    }
    std::cout << "\nShock Simulation Results #2 (Traditional NRMM Method):\n";
    std::cout << "Significant Speed = " << target_speed << " mph\n";
    std::cout << "Obstacle Height   = " << heightVals[iObstacle] << " in\n";
    std::cout << "  Maximum Vertical Seat Acceleration = " << az << " g\n";
    if (az <= az_limit) {
        std::cout << "Az <= " << az_limit << " g (ok)\n";
    } else {
        std::cout << "Az > " << az_limit << " g - severe risk for average occupant!\n";
    }
    return 0;
}