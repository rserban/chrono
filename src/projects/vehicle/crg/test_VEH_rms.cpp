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
// Demonstration of an OpenCRG terrain.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Problem parameters

// OpenCRG input file
std::string crg_road_file = "terrain/crg_roads/rms_2p4.crg";
////std::string crg_road_file = "terrain/crg_roads/rms_3p6.crg";

// Desired vehicle speed (m/s)
double target_speed = 5;

// Simulation step size
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Output frame images
bool output_images = false;
double fps = 60;

// Output directories
const std::string out_dir = "../RMS_CRG";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string mitsuba_dir = out_dir + "/MITSUBA";

// POV-Ray output
bool povray_output = false;

// =============================================================================
void WriteShapesMitsuba(ChSystem* system, const std::string& filename);

int main(int argc, char* argv[]) {
    // ----------------
    // Output directory
    // ----------------

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

    // ---------------------------------------
    // Create the vehicle, terrain, and driver
    // ---------------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(80, 0, 0.5), QUNIT));
    my_hmmwv.SetEngineType(EngineModelType::SHAFTS);
    my_hmmwv.SetTransmissionType(TransmissionModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    my_hmmwv.SetTireType(TireModelType::TMEASY);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    CRGTerrain terrain(my_hmmwv.GetSystem());
    terrain.UseMeshVisualization(true);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.Initialize(vehicle::GetDataFile(crg_road_file));

    terrain.ExportMeshWavefront(out_dir);
    terrain.ExportMeshPovray(out_dir);

    // Get the vehicle path (middle of the road)
    auto path = terrain.GetRoadCenterLine();
    bool path_is_closed = terrain.IsPathClosed();
    double road_length = terrain.GetLength();

    // Create the driver system based on PID steering controller
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("HMMWV RMS");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&my_hmmwv.GetVehicle());


    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // ---------------
    // Simulation loop
    // ---------------

    // Final time
    double t_end = 2 + road_length / target_speed;
    if (path_is_closed) {
        t_end += 30.0;
    }
    std::cout << "Road length:     " << road_length << std::endl;
    std::cout << "Closed loop?     " << path_is_closed << std::endl;
    std::cout << "Set end time to: " << t_end << std::endl;

    // Number of simulation steps between image outputs
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize frame counters
    int step_number = 0;
    int render_frame = 0;

    while (vis->Run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();
        if (time >= t_end)
            break;

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector3d& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector3d& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Render scene and output images
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (step_number % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(my_hmmwv.GetSystem(), filename);
            }
            render_frame++;
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

        // Increment simulation frame number
        step_number++;
    }

    return 0;
}
