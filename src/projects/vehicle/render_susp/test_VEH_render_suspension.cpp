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

#include "chrono/core/ChStream.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

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
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create tires.
    std::string tire_file("hmmwv/tire/HMMWV_TMeasyTire.json");
    auto tire_L = ReadTireJSON(vehicle::GetDataFile(tire_file));
    auto tire_R = ReadTireJSON(vehicle::GetDataFile(tire_file));

    // Create the suspension test rig.
    auto rig = std::unique_ptr<ChSuspensionTestRigPushrod>(
        new ChSuspensionTestRigPushrod(vehicle::GetDataFile(vehicle_file), axle_index, 0.1, tire_L, tire_R));

    if (ride_height > 0)
        rig->SetInitialRideHeight(ride_height);

    rig->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rig->SetWheelVisualizationType(VisualizationType::NONE);
    if (rig->HasSteering()) {
        rig->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    }
    rig->SetTireVisualizationType(VisualizationType::NONE);

    // Create the vehicle Irrlicht application.
    ChVehicleIrrApp app(rig.get(), L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(0.5 * (rig->GetSpindlePos(LEFT) + rig->GetSpindlePos(RIGHT)), 2.0, 0.5);

    // Create and attach the driver system.
    std::string driver_file("hmmwv/suspensionTest/ST_inputs.dat");
    auto driver = chrono_types::make_shared<ChDataDriverSTR>(vehicle::GetDataFile(driver_file));
    rig->SetDriver(driver);

    // Initialize suspension test rig.
    rig->Initialize();

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string susp_name = rig->GetSuspension()->GetTemplateName();

    // Simulation loop
    double step_size = 1e-3;
    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Advance simulation of the rig
        rig->Advance(step_size);

        // Update visualization app
        app.Synchronize(tire_L->GetTemplateName(), {rig->GetSteeringInput(), 0, 0});
        app.Advance(step_size);

        // Save POV-Ray file once the rig is at specified ride height
        if (ride_height < 0 || driver->Started()) {
            // Hack: create a dummy body (to render a global reference frame)
            auto dummy = std::shared_ptr<ChBody>(rig->GetSystem()->NewBody());
            dummy->SetPos(rig->GetSuspension()->GetLocation());
            dummy->SetIdentifier(-1);
            rig->GetSystem()->AddBody(dummy);

            std::string filename = out_dir + "/" + susp_name + ".dat";
            utils::WriteVisualizationAssets(rig->GetSystem(), filename);
            break;
        }
    }

    // Left spindle location
    auto locL = rig->GetSuspension()->GetSpindlePos(LEFT);
    std::cout << "\n\nSuspension template:  " << susp_name << "\n";
    std::cout << "Suspension left spindle location:  " << locL << std::endl;

    // Generate wrapper scripts
    {
        std::string filename = out_dir + "/" + susp_name + ".ini";
        ChStreamOutAsciiFile ini_file(filename.c_str());
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
        ChStreamOutAsciiFile inc_file(filename.c_str());
        inc_file << "#declare datafile = \"" << susp_name << ".dat\"\n";
        inc_file << "#declare cam_perspective = true;\n";
        inc_file << "#declare cam_lookat = <" << locL.x() << ", 0, 0>;\n";
        inc_file << "#declare cam_loc = <" << locL.x() - 2 << ", -1.75, 1>;\n";
        inc_file << "#declare cam_angle = 50;\n";
    }

    // Camera position for zooming on left side
    ////{
    ////    std::string filename = out_dir + "/" + susp_name + "_inc.pov";
    ////    ChStreamOutAsciiFile inc_file(filename.c_str());
    ////    inc_file << "#declare datafile = \"" << susp_name << ".dat\"\n";
    ////    inc_file << "#declare cam_perspective = true;\n";
    ////    inc_file << "#declare cam_lookat = <" << locL.x() << "," << locL.y() - 0.25 << "," << locL.z() << ">;\n";
    ////    inc_file << "#declare cam_loc = <" << locL.x() - 1 << ", -0.7, 1>;\n";
    ////    inc_file << "#declare cam_angle = 50;\n";
    ////}

    return 0;
}
