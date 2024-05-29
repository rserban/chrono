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
// Program to generate POV-Ray renderings of various suspension templates.
// Uses the suspension test rig to save rendering data at the configuration
// corresponding to the specified ride height or at the design configuration
// (if a negative ride_height value is set).
//
// Usage:
// - Set the JSON vehicle specification file and axle (starting with 0 in front)
// - Run this test program
// - Output files (with names based on the suspension template name) are
//   generated in the directory ../RENDER_SUSPENSION
// - For a suspension template name 'XXX', the following files are generated:
//      XXX.dat     - contains the data with body and asset information
//      XXX_inc.pov - an automatically generated POV-Ray header which specifies
//                    the data filename and camera settings (the view is always
//                    pointing at the axle center, from behind)
//      XXX.ini     - the POV-Ray ini file
// - Copy the POV-Ray script 'render_suspension.pov' into the output directory
// - Run POV-Ray specifying the ini file; for example:
//      povray /EXIT /RENDER XXX.ini
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDataDriver.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// USER SETTINGS
// =============================================================================

// JSON file for vehicle and axle index (axle_index=0: front axle, axle_index=1: rear axle)
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
////std::string vehicle_file("uaz/vehicle/UAZBUS_Vehicle.json");
////std::string vehicle_file("uaz/vehicle/UAZBUS_SAEVehicle.json");
////std::string vehicle_file("MAN_Kat1/vehicle/MAN_5t_Vehicle_4WD.json");
////std::string vehicle_file("generic/vehicle/Vehicle_MacPhersonStruts.json");
////std::string vehicle_file("generic/vehicle/Vehicle_MultiLinks.json");
////std::string vehicle_file("generic/vehicle/Vehicle_SemiTrailingArm.json");
////std::string vehicle_file("generic/vehicle/Vehicle_ThreeLinkIRS.json");
////std::string vehicle_file("generic/vehicle/Vehicle_SolidAxles.json");
////std::string vehicle_file("generic/vehicle/Vehicle_RigidSuspension.json");
////std::string vehicle_file("generic/vehicle/Vehicle_RigidPinnedAxle.json");
////std::string vehicle_file("generic/vehicle/Vehicle_HendricksonPRIMAXX.json");

// Vehicle axle (starting with 0 in front)
int axle_index = 1;

// Set ride height
// If a negative value, the suspension is rendered in the design configuration.
double ride_height = -1;

// Output directory
std::string out_dir = "../RENDER_SUSPENSION";

// =============================================================================
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the vehicle and the test rig
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile(vehicle_file), ChContactMethod::SMC);
    ChSuspensionTestRigPushrod rig(vehicle, {axle_index}, 0.1);

    // Create the tires
    std::string tire_file("hmmwv/tire/HMMWV_TMeasyTire.json");

    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_file));
            vehicle->InitializeTire(tire, wheel, VisualizationType::NONE);
        }
    }

    if (ride_height > 0)
        rig.SetInitialRideHeight(ride_height);

    rig.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig.SetWheelVisualizationType(VisualizationType::NONE);
    rig.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    rig.SetTireVisualizationType(VisualizationType::NONE);

    // Create the vehicle Irrlicht application.
    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Suspension Test Rig");
    vis->SetChaseCamera(0.5 * (rig.GetSpindlePos(0, LEFT) + rig.GetSpindlePos(0, RIGHT)), 2.0, 0.5);

    // Create and attach the driver system.
    std::string driver_file("hmmwv/suspensionTest/ST_inputs.dat");
    auto driver = chrono_types::make_shared<ChSuspensionTestRigDataDriver>(vehicle::GetDataFile(driver_file));
    rig.SetDriver(driver);

    // Initialize suspension test rig.
    rig.Initialize();

    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&rig.GetVehicle());

    // Output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    auto suspension = vehicle->GetAxle(axle_index);
    auto susp_name = suspension->m_suspension->GetTemplateName();
    auto susp_pos = rig.GetSpindlePos(0, LEFT) + rig.GetSpindlePos(0, RIGHT);

    // Simulation loop
    double step_size = 1e-3;
    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Advance simulation of the rig
        rig.Advance(step_size);

        // Update visualization app
        vis->Synchronize(rig.GetVehicle().GetChTime(), {rig.GetSteeringInput(), 0, 0});
        vis->Advance(step_size);

        // Save POV-Ray file once the rig is at specified ride height
        if (ride_height < 0 || driver->Started()) {
            // Hack: create a dummy body (to render a global reference frame)
            auto dummy = chrono_types::make_shared<ChBody>();
            dummy->SetPos(susp_pos);
            vehicle->GetSystem()->AddBody(dummy);

            std::string filename = out_dir + "/" + susp_name + ".dat";
            utils::WriteVisualizationAssets(vehicle->GetSystem(), filename);
            break;
        }
    }

    // Left spindle location
    auto locL = rig.GetSpindlePos(0, LEFT);
    std::cout << "\n\nSuspension template:  " << susp_name << "\n";
    std::cout << "Suspension left spindle location:  " << locL << std::endl;

    // Generate wrapper scripts
    {
        std::string filename = out_dir + "/" + susp_name + ".ini";
        std::ofstream ini_file(filename);
        ini_file << "Input_File_Name=\"render_suspension.pov\"\n";
        ini_file << "Include_Header=\"" << susp_name << "_inc.pov\"\n";
        ini_file << "Output_File_Name=\"" << susp_name << ".png\"\n";
        ini_file << "Height = 1200\n";
        ini_file << "Width = 1600\n";
        ini_file << "Antialias = On\n";
        ini_file << "Antialias_Threshold = 0.3\n";
    }

    // Camera position for viewing entire suspension assembly
    {
        std::string filename = out_dir + "/" + susp_name + "_inc.pov";
        std::ofstream inc_file(filename);
        inc_file << "#declare datafile = \"" << susp_name << ".dat\"\n";
        inc_file << "#declare cam_perspective = true;\n";
        inc_file << "#declare cam_lookat = <" << locL.x() << ", 0, 0>;\n";
        inc_file << "#declare cam_loc = <" << locL.x() - 2 << ", -1.75, 1>;\n";
        inc_file << "#declare cam_angle = 50;\n";
    }

    // Camera position for zooming on left side
    ////{
    ////    std::string filename = out_dir + "/" + susp_name + "_inc.pov";
    ////    std::ofstream inc_file(filename);
    ////    inc_file << "#declare datafile = \"" << susp_name << ".dat\"\n";
    ////    inc_file << "#declare cam_perspective = true;\n";
    ////    inc_file << "#declare cam_lookat = <" << locL.x() << "," << locL.y() - 0.25 << "," << locL.z() << ">;\n";
    ////    inc_file << "#declare cam_loc = <" << locL.x() - 1 << ", -0.7, 1>;\n";
    ////    inc_file << "#declare cam_angle = 50;\n";
    ////}

    return 0;
}
