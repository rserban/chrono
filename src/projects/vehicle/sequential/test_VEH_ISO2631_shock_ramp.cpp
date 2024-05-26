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
// Main driver function for a vehicle specified through JSON files + example for
// obtaining shock effect results presented by ISO 2631-5
//
// Ramp shaped obstacles
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
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");

// JSON file for powertrain (simple)
std::string engine_file("generic/powertrain/EmgineSimpleMap.json");
std::string transmission_file("generic/powertrain/AutomaticTransmissionSimpleMap.json");

// JSON files tire models
std::string tmeasy_tire_file("hmmwv/tire/HMMWV_TMeasy_converted.json");
std::string fiala_tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");

// Tire collision type
ChTire::CollisionType collision_type = ChTire::CollisionType::ENVELOPE;

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector3d initLoc(90, 0, 0.6);

// Simulation step size (should not be too high!)
double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const int heightVals[7] = {0, 1000, 2000, 3000, 4000, 5000, 6000};
    int iObstacle = 1;
    double target_speed = 5.0;
    int iTire = 1;

    // JSON files for terrain
    std::string rigidterrain_file("terrain/RigidRamp1.json");

    switch (argc) {
        default:
        case 1:
            std::cout << "usage: demo_VEH_ShockRamp [ObstacleNumber [Speed]]\n\n";
            std::cout << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala) = " << iTire << "\n";
            break;
        case 2:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 6) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRamp" + std::to_string(iObstacle) + ".json";
            }
            std::cout << "Using values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala) = " << iTire << "\n";
            break;
        case 3:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 6) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRamp" + std::to_string(iObstacle) + ".json";
            }
            target_speed = atof(argv[2]);
            std::cout << "Using values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala) = " << iTire << "\n";
            break;
        case 4:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 6) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRamp" + std::to_string(iObstacle) + ".json";
            }
            target_speed = atof(argv[2]);
            if (atoi(argv[3]) >= 1 && atoi(argv[3]) <= 2) {
                iTire = atoi(argv[3]);
            }
            std::cout << "Using values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala) = " << iTire << "\n";
            break;
    }
    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, QUNIT));
    ////vehicle.GetChassis()->SetFixed(true);
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    std::cout << "\nBe patient - startup may take some time... \n";

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_file));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_file));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    // handling tire works, but still too high sesults
    // a validated flexible tire model would be the best choice
    for (auto& axle : vehicle.GetAxles()) {
        switch (iTire) {
            default:
            case 1: {
                auto tireL = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_tire_file));
                auto tireR = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_tire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
            case 2: {
                auto tireL = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fiala_tire_file));
                auto tireR = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fiala_tire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
        }
    }

    ChISO2631_Shock_SeatCushionLogger seat_logger(step_size);

    // Create Irrlicht visualization
    std::string windowTitle = "Vehicle Shock Test Demo ";
    switch (iTire) {
        default:
        case 1:
            windowTitle.append("(TMeasy Tire) - " + std::to_string(heightVals[iObstacle]) + " mm Obstacle Height");
            break;
        case 2:
            windowTitle.append("(Fiala Tire) - " + std::to_string(heightVals[iObstacle]) + " mm Obstacle Height");
            break;
    }

    // Create the driver
    auto path = ChBezierCurve::Read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();

    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle(windowTitle);
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // ---------------
    // Simulation loop
    // ---------------

    // Logging of seat acceleration data on flat road surface is useless
    double xstart = 100.0;  // start logging when the vehicle crosses this x position
    double xend = 180;      // end logging here, this also the end of our world

    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);

        double xpos = vehicle.GetSpindlePos(0, LEFT).x();
        if (xpos >= xend) {
            break;
        }
        if (xpos >= xstart) {
            double speed = vehicle.GetSpeed();
            ChVector3d seat_acc = vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(seat_acc);
        }
    }

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
    std::cout << "  Maximum Vertical Seat Acceleration = " << az << " g\n";
    if (az <= az_limit) {
        std::cout << "Az <= " << az_limit << " g (ok)\n";
    } else {
        std::cout << "Az > " << az_limit << " g - severe risk for average occupant!\n";
    }
    return 0;
}