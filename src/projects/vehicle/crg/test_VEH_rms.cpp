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
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

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
bool mitsuba_output = false;

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
    if (mitsuba_output) {
        if (!filesystem::create_directory(filesystem::path(mitsuba_dir))) {
            std::cout << "Error creating directory " << mitsuba_dir << std::endl;
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
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(80, 0, 0.5), QUNIT));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
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
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed, path_is_closed);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"HMMWV RMS", irr::core::dimension2d<irr::u32>(800, 640));

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    app.SetTimestep(step_size);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

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

    while (app.GetDevice()->run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();
        if (time >= t_end)
            break;

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Render scene and output images
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        if (step_number % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(my_hmmwv.GetSystem(), filename);
            }
            if (mitsuba_output) {
                char filename[100];
                sprintf(filename, "%s/data_%04d.dat", mitsuba_dir.c_str(), render_frame + 1);
                WriteShapesMitsuba(my_hmmwv.GetSystem(), filename);
            }
            render_frame++;
        }

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

        // Increment simulation frame number
        step_number++;

        app.EndScene();
    }

    return 0;
}

void WriteShapesMitsuba(ChSystem* system, const std::string& filename) {
    utils::CSV_writer csv;

    // Loop over all bodies and over all their assets.
    int a_count = 0;
    for (auto body : system->Get_bodylist()) {
        const ChVector<>& body_pos = body->GetFrame_REF_to_abs().GetPos();
        const ChQuaternion<>& body_rot = body->GetFrame_REF_to_abs().GetRot();

        // Loop over assets -- write information for supported types.
        for (auto asset : body->GetAssets()) {
            auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset);
            if (!visual_asset)
                continue;

            const Vector& asset_pos = visual_asset->Pos;
            Quaternion asset_rot = visual_asset->Rot.Get_A_quaternion();

            Vector pos = body_pos + body_rot.Rotate(asset_pos);
            Quaternion rot = body_rot % asset_rot;
            ChVector<> axis;
            double angle;
            rot.Q_to_AngAxis(angle, axis);

            bool supported = true;
            std::string type;
            std::stringstream gss;

            if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(visual_asset)) {
                type = "sphere";
                gss << sphere->GetSphereGeometry().rad;
                a_count++;
            } else if (auto box = std::dynamic_pointer_cast<ChBoxShape>(visual_asset)) {
                const Vector& size = box->GetBoxGeometry().Size;
                type = "box";
                gss << size.x() << "," << size.y() << "," << size.z();
                a_count++;
            } else if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(visual_asset)) {
                const geometry::ChCylinder& geom = cylinder->GetCylinderGeometry();
                type = "cylinder";
                gss << geom.p1.x() << "," << geom.p1.y() << "," << geom.p1.z() << "," << geom.p2.x() << ","
                    << geom.p2.y() << "," << geom.p2.z() << "," << geom.rad;
                a_count++;
            } else if (auto mesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(visual_asset)) {
                type = "obj_mesh";
                gss << mesh->GetName();
                a_count++;
            } else {
                supported = false;
            }

            if (supported) {
                csv << type << pos << angle << axis << gss.str() << std::endl;
            }
        }  // end loop over assets
    }      // end loop over bodies

    csv.write_to_file(filename);
}
