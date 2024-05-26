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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Sample test program for sequential FEDA simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

//  The vehicle runs on the smalles possible turn radius

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/feda/FEDA.h"
#include "chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"

#include <chrono>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, PAC02)
TireModelType tire_model = TireModelType::PAC02;

// Type of steering model (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringTypeWV steering_model = SteeringTypeWV::PITMAN_ARM;

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size = 2e-4;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// =============================================================================

int main(int argc, char* argv[]) {
    // --------------
    // Create systems
    // --------------
    std::string mode;
    if (argc == 2) {
        mode = argv[1];
    } else {
        std::cout << "usage: test_LMTV_wall2wall left | right\n";
        std::cout << "using \"left\"...\n";
        mode = "left";
    }
    bool left_turn = true;
    std::string driver_file;
    if (mode.compare("right") == 0) {
        left_turn = false;
        std::cout << "Right turn selected.\n";
        driver_file = "feda/driver/right_slow_turn.txt";
    } else {
        std::cout << "Left turn selected.\n";
        driver_file = "feda/driver/left_slow_turn.txt";
    }

    // corner locations on the chassis
    ChVector3d FrontLeftCornerLoc(0.8, 1.144, 0.5);
    ChVector3d FrontRightCornerLoc(0.8, -1.144, 0.5);

    // Parameter for aerodynamic force
    double Cd = 0.6;
    double area = 3.8;
    double air_density = 1.2041;
    // Create the vehicle, set parameters, and initialize
    FEDA feda;
    feda.SetContactMethod(ChContactMethod::NSC);
    feda.SetChassisFixed(false);
    feda.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    feda.SetTireType(tire_model);
    feda.SetTireStepSize(tire_step_size);
    feda.SetAerodynamicDrag(Cd, area, air_density);
    feda.SetRideHeight_OnRoad();
    // feda.SetRideHeight_ObstacleCrossing();
    // feda.SetRideHeight_Low();

    feda.SetInitFwdVel(0.0);
    feda.Initialize();

    feda.SetChassisVisualizationType(chassis_vis_type);
    feda.SetSuspensionVisualizationType(suspension_vis_type);
    feda.SetSteeringVisualizationType(steering_vis_type);
    feda.SetWheelVisualizationType(wheel_vis_type);
    feda.SetTireVisualizationType(tire_vis_type);

    std::cout << "Vehicle mass: " << feda.GetVehicle().GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.8f);
    patch_mat->SetRestitution(0.01f);
    RigidTerrain terrain(feda.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------
    std::string wTitle = "FED Alpha wall2wall turning test";
    if (left_turn)
        wTitle.append(" (left)");
    else
        wTitle.append(" (right)");

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle(wTitle);
    vis->SetChaseCamera(trackPoint, 8.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&feda.GetVehicle());

    ChDataDriver driver(feda.GetVehicle(), vehicle::GetDataFile(driver_file), true);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;

    std::vector<double> turn_x, turn_y;

    while (vis->Run()) {
        double time = feda.GetSystem()->GetChTime();

        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        feda.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        feda.Advance(step_size);
        vis->Advance(step_size);

        auto susp0 = std::static_pointer_cast<ChDoubleWishbone>(feda.GetVehicle().GetSuspension(0));
        auto susp1 = std::static_pointer_cast<ChDoubleWishbone>(feda.GetVehicle().GetSuspension(1));

        ChCoordsys<> vehCoord = ChCoordsys<>(feda.GetVehicle().GetPos(), feda.GetVehicle().GetRot());
        ChVector3d vehCOM = vehCoord.TransformPointParentToLocal(feda.GetVehicle().GetCOMFrame().GetPos());

        if (left_turn) {
            // left turn + right corner
            turn_x.push_back(feda.GetVehicle().GetPointLocation(FrontRightCornerLoc).x());
            turn_y.push_back(feda.GetVehicle().GetPointLocation(FrontRightCornerLoc).y());
        } else {
            // right turn + left corner
            turn_x.push_back(feda.GetVehicle().GetPointLocation(FrontLeftCornerLoc).x());
            turn_y.push_back(feda.GetVehicle().GetPointLocation(FrontLeftCornerLoc).y());
        }
        // std::cout << "y = " << turn_y.back() << "\n";
        // Increment frame number
        step_number++;
        if (time > 140.0)
            break;
    }

    double xmax = turn_x[0];
    double xmin = turn_x[0];
    for (int i = 1; i < turn_x.size(); i++) {
        if (turn_x[i] > xmax)
            xmax = turn_x[i];
        if (turn_x[i] < xmin)
            xmin = turn_x[i];
    }
    double real_radius_left = 0.3048 * 50.8 / 2.0;
    double real_radius_right = 0.3048 * 51.1 / 2.0;
    double turn_radius = (xmax - xmin) / 2.0;
    double test_error;
    if (left_turn) {
        test_error = 100.0 * (real_radius_left - turn_radius) / real_radius_left;
    } else {
        test_error = 100.0 * (real_radius_right - turn_radius) / real_radius_right;
    }
    // std::cout << "Xmin = " << xmin << "    Xmax = " << xmax << "\n";
    if (left_turn) {
        std::cout << "Left Turn circle radius (simulation) = " << turn_radius << " m\n";
        std::cout << "Left Turn circle radius (measured) = " << real_radius_left << " m\n";
        std::cout << "Left Turn circle radius (error) = " << test_error << " %\n";
    } else {
        std::cout << "Right Turn circle radius (simulation) = " << turn_radius << " m\n";
        std::cout << "Right Turn circle radius (measured) = " << real_radius_right << " m\n";
        std::cout << "Right Turn circle radius (error) = " << test_error << " %\n";
    }
    return 0;
}
