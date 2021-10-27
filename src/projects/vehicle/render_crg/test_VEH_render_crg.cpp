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
// Program for exporting POV-Ray visualization data for CRG terrain
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Problem parameters

// Type of tire model (LUGRE, FIALA, PACEJKA, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// OpenCRG input file
std::string crg_road_file = "terrain/crg_roads/Barber.crg";
////std::string crg_road_file = "terrain/crg_roads/Horstwalde.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_arc.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_banked.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_circle.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_sloped.crg";

// Road visualization (mesh or boundary lines)
bool useMesh = false;

// Desired vehicle speed (m/s)
double target_speed = 12;

// Simulation step size
double step_size = 3e-3;
double tire_step_size = 1e-3;

// Output frame images
const std::string out_dir = "../RENDER_CRG";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ---------------------------------------
    // Create the vehicle, terrain, and driver
    // ---------------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(2, 0, 0.5), QUNIT));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    CRGTerrain terrain(my_hmmwv.GetSystem());
    terrain.UseMeshVisualization(useMesh);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.Initialize(vehicle::GetDataFile(crg_road_file));

    // Get the vehicle path (middle of the road)
    auto path = terrain.GetRoadCenterLine();
    bool path_is_closed = terrain.IsPathClosed();
    double road_length = terrain.GetLength();
    double road_width = terrain.GetWidth();

    ChPathFollowerDriverSR driver(my_hmmwv.GetVehicle(), path, "road_center", target_speed, path_is_closed,
                                  my_hmmwv.GetVehicle().GetMaxSteeringAngle(), 3.2);
    driver.GetSteeringController().SetGains(0.1, 5);
    driver.GetSteeringController().SetPreviewTime(0.5);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Export paths to POV-Ray
    driver.ExportPathPovray(out_dir);
    terrain.ExportCurvesPovray(out_dir);

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"OpenCRG Demo SR Steering",
                               irr::core::dimension2d<irr::u32>(1600, 1200));

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.SetTimestep(step_size);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Final time
    double t_end = 2 + road_length / target_speed;
    if (path_is_closed) {
        t_end += 30.0;
    }
    std::cout << "Road length:     " << road_length << std::endl;
    std::cout << "Road width:      " << road_width << std::endl;
    std::cout << "Closed loop?     " << path_is_closed << std::endl;
    std::cout << "Set end time to: " << t_end << std::endl;

    app.SetChaseCameraPosition(ChVector<>(450, 35, 15));
    app.SetChaseCameraState(utils::ChChaseCamera::State::Free);
    app.SetChaseCameraAngle(-150.0 * CH_C_DEG_TO_RAD);
    app.EnableStats(false);

    while (app.GetDevice()->run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();
        if (time >= t_end)
            break;

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        if (time > 1) {
            break;
        }
    }

    std::cout << my_hmmwv.GetVehicle().GetVehiclePos() << std::endl;

    std::string filename1 = out_dir + "/" + filesystem::path(crg_road_file).stem() + ".dat";
    utils::WriteVisualizationAssets(my_hmmwv.GetSystem(), filename1);

    std::string filename2 = out_dir + "/" + filesystem::path(crg_road_file).stem() + ".jpg";
    app.WriteImageToFile(filename2);


    return 0;
}
