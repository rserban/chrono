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
// Test for HMMWV full vehicle with PITMAN_ARM_SHAFTS steering
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.6);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

enum DriverMode {INTERACTIVE, DATAFILE};
DriverMode driver_mode = DATAFILE;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringType steering_type = SteeringType::PITMAN_ARM_SHAFTS;

// Simulation step sizes
double step_size = 1e-3;

// Simulation end time
double t_end = 10;

// Output directories
const std::string out_dir = "../TEST_steering_hmmwv";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChMaterialSurface::SMC);
    my_hmmwv.SetChassisCollisionType(ChassisCollisionType::NONE);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineType::AWD);
    my_hmmwv.SetSteeringType(steering_type);
    my_hmmwv.SetRigidSteeringColumn(true);
    my_hmmwv.SetTireType(TireModelType::RIGID);
    my_hmmwv.SetTireStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    my_hmmwv.GetSystem()->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    my_hmmwv.GetSystem()->SetMaxItersSolverSpeed(200);


    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(100, 100, 10));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"HMMWV Steering Test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Initialize output file for driver inputs
    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    std::string out_file = out_dir + "/output_" + std::to_string(static_cast<int>(steering_type)) + ".txt";
    utils::CSV_writer out_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    ChDriver* driver;

    switch (driver_mode) {
        case INTERACTIVE: {
            ChIrrGuiDriver* driver_irr = new ChIrrGuiDriver(app);

            // Set the time response for steering and throttle keyboard inputs.
            double steering_time = 0.4;  // time to go from 0 to +1 (or from 0 to -1)
            double throttle_time = 0.3;  // time to go from 0 to +1
            double braking_time = 0.3;   // time to go from 0 to +1
            driver_irr->SetSteeringDelta(step_size / steering_time);
            driver_irr->SetThrottleDelta(step_size / throttle_time);
            driver_irr->SetBrakingDelta(step_size / braking_time);

            driver = driver_irr;
            break;
        }

        case DATAFILE: {
            std::vector<ChDataDriver::Entry> data;
            data.push_back(ChDataDriver::Entry(0.0, 0.0, 0.0, 0.0));
            data.push_back(ChDataDriver::Entry(2.0, 0.0, 0.0, 0.0));
            data.push_back(ChDataDriver::Entry(3.0, 0.0, 0.4, 0.0));
            data.push_back(ChDataDriver::Entry(4.0, 0.0, 0.4, 0.0));
            data.push_back(ChDataDriver::Entry(5.0, 0.2, 0.0, 0.0));
            data.push_back(ChDataDriver::Entry(10.0, 0.2, 0.4, 0.0));
            ChDataDriver* driver_data = new ChDataDriver(my_hmmwv.GetVehicle(), data);

            driver = driver_data;
            break;
        }
    }

    driver->Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;

    while (app.GetDevice()->run()) {
        time = my_hmmwv.GetSystem()->GetChTime();
        if (time >= t_end)
            break;

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver->GetThrottle();
        double steering_input = driver->GetSteering();
        double braking_input = driver->GetBraking();

        auto wheel_omgG = my_hmmwv.GetVehicle().GetWheelBody(FRONT_LEFT)->GetWvel_par();
        auto wheel_omgL = my_hmmwv.GetVehicle().GetWheelBody(FRONT_LEFT)->GetWvel_loc();
        auto wheel_nrm = my_hmmwv.GetVehicle().GetWheelBody(FRONT_LEFT)->TransformDirectionLocalToParent(ChVector<>(0, 1, 0));
        out_csv << time << steering_input    // time and steering input
                << wheel_omgG << wheel_omgL  // wheel angular velocity (in global and local frame)
                << wheel_nrm;                // wheel normal (global frame)
        if (steering_type == SteeringType::PITMAN_ARM_SHAFTS) {
            double m, m_d;             // motor input and time derivative
            std::vector<double> a, v;  // shaft angles and angular velocities
            double ra, rv;             // relative chassis-arm angle and agular velocity
            std::vector<double> c;     // constraint violations
            auto steering = std::static_pointer_cast<ChPitmanArmShafts>(my_hmmwv.GetVehicle().GetSteering(0));
            steering->GetDebugInformation(m, m_d, a, v, ra, rv, c);
            out_csv << m << m_d << a << v << ra << rv << c;
        }
        out_csv << std::endl;

        // Update modules (process inputs from other modules)
        driver->Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    out_csv.write_to_file(out_file);

    delete driver;
    return 0;
}
