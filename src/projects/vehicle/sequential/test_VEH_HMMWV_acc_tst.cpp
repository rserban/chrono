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
// HMMWV acceleration test
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Type of powertrain model (SHAFTS, SIMPLE)
EngineModelType engine_model = EngineModelType::SHAFTS;
TransmissionModelType transmission_model = TransmissionModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Type of tire model (RIGID, RIGID_MESH, ANCF, REISSNER, TMEASY, FIALA, PAC89, PAC02)
TireModelType tire_model = TireModelType::TMEASY;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Simulation end time
double t_end = 18;

// Speed limit
double speed_end = 25.0;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = "../HMMWV_ACC_TST";

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisCollisionType(CollisionType::NONE);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-180, 0, 0.6), QUNIT));
    my_hmmwv.SetEngineType(engine_model);
    my_hmmwv.SetTransmissionType(transmission_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.UseTierodBodies(true);
    my_hmmwv.SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    my_hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // Create the terrain
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 400, 200);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 400, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("HMMWV Special Test");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&my_hmmwv.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    utils::ChWriterCSV driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------
    std::string driverFile = GetChronoDataFile("vehicle/hmmwv/driver/AccTest_Maneuver.txt");
    std::cout << driverFile << "\n";
    ChDataDriver driver(my_hmmwv.GetVehicle(), driverFile);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    my_hmmwv.GetVehicle().LogSubsystemTypes();

    // Recorders
    ChFunctionInterp speed_recorder;
    ChFunctionInterp engine_speed_recorder;
    ChFunctionInterp engine_torque_recorder;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    my_hmmwv.GetVehicle().EnableRealtime(true);
    utils::ChRunningAverage RTF_filter(50);
    utils::ChRunningAverage AVG_filter(20);
    while (vis->Run()) {
        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        double time = my_hmmwv.GetSystem()->GetChTime();
        double speed = my_hmmwv.GetVehicle().GetSpeed();
        double engine_torque = my_hmmwv.GetVehicle().GetEngine()->GetOutputMotorshaftTorque();
        double engine_speed = my_hmmwv.GetVehicle().GetEngine()->GetMotorSpeed();
        double throttle = driver_inputs.m_throttle;
        double fx = my_hmmwv.GetVehicle().GetWheel(0, LEFT)->GetTire()->ReportTireForce(&terrain).force.x() +
                    my_hmmwv.GetVehicle().GetWheel(0, RIGHT)->GetTire()->ReportTireForce(&terrain).force.x() +
                    my_hmmwv.GetVehicle().GetWheel(1, LEFT)->GetTire()->ReportTireForce(&terrain).force.x() +
                    my_hmmwv.GetVehicle().GetWheel(1, RIGHT)->GetTire()->ReportTireForce(&terrain).force.x();
        driver_csv << time << throttle << speed << engine_torque << engine_speed << AVG_filter.Add(fx) << std::endl;

        speed_recorder.AddPoint(time, speed);
        engine_speed_recorder.AddPoint(time, engine_speed);
        engine_torque_recorder.AddPoint(time, engine_torque);

        // End simulation
        if (speed >= speed_end)
            break;
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

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

        // Increment frame number
        step_number++;
    }

    std::string tire_model_str;
    switch (tire_model) {
        case TireModelType::RIGID:
            tire_model_str = "rigid";
            break;
        case TireModelType::TMEASY:
            tire_model_str = "tmeasy";
            break;
        case TireModelType::PAC89:
            tire_model_str = "pac89";
            break;
        case TireModelType::PAC02:
            tire_model_str = "pac02";
            break;
        default:
            break;
    }
    driver_csv.WriteToFile(out_dir + "/output_" + tire_model_str + ".csv",
                             "#time throttle speed engine_torque engine_speed");

#ifdef CHRONO_POSTPROCESS
    std::string title_command = "set title 'Tire model: " + tire_model_str + "'";
    {
        postprocess::ChGnuPlot gplot(out_dir + "/speed.gpl");
        gplot.SetCommand(title_command.c_str());
        gplot.SetGrid();
        gplot.SetLabelX("time (s)");
        gplot.SetLabelY("speed (m/s)");
        gplot.OutputPNG(out_dir + "/speed.png", 800, 600);
        gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/engine_speed.gpl");
        gplot.SetCommand(title_command.c_str());
        gplot.SetGrid();
        gplot.SetLabelX("time (s)");
        gplot.SetLabelY("engine speed (rad/s)");
        gplot.OutputPNG(out_dir + "/engin_speed.png", 800, 600);
        gplot.Plot(engine_speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
    }
    {
        postprocess::ChGnuPlot gplot(out_dir + "/engine_torque.gpl");
        gplot.SetCommand(title_command.c_str());
        gplot.SetGrid();
        gplot.SetLabelX("time (s)");
        gplot.SetLabelY("engine torque (N.m)");
        gplot.OutputPNG(out_dir + "/engine_torque.png", 800, 600);
        gplot.Plot(engine_torque_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
    }
#endif

    return 0;
}
