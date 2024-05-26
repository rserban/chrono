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
// Authors: Radu Serban
// =============================================================================
//
// Test of vehicle driving over a bump
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Problem parameters

// Contact method type
ChContactMethod contact_method = ChContactMethod::SMC;

// Type of tire model (RIGID, FIALA)
TireModelType tire_model = TireModelType::RIGID;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Initial vehicle location and orientation
ChVector3d initLoc(5, 5, 0.5);
ChQuaternion<> initRot = QuatFromAngleZ(0);

// Desired vehicle speed (m/s)
double target_speed = 12;

// Bump parameters
double dist_to_bump = 40;
double bump_radius = 2;
double bump_height = 0.2;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 50.0;   // size in Y direction

// Simulation step size, end time
double step_size = 1e-3;

// Render FPS
double fps = 60;

// Output
bool state_output = true;
int filter_window_size = 40;
const std::string out_dir = "../HMMWV_BUMP";

// =============================================================================

std::shared_ptr<ChBezierCurve> CreateBezierPath(const ChVector3d& start, const ChVector3d& dir, double length) {
    std::vector<ChVector3d> points;
    std::vector<ChVector3d> inCV;
    std::vector<ChVector3d> outCV;

    // Start point
    ChVector3d P1 = start;
    points.push_back(P1);
    inCV.push_back(P1);
    outCV.push_back(P1 + 0.1 * length * dir);

    // End point
    ChVector3d P2 = start + length * dir;
    points.push_back(P2);
    inCV.push_back(P2 - 0.1 * length * dir);
    outCV.push_back(P2);

    // Generate Bezier path
    return chrono_types::make_shared<ChBezierCurve>(points, inCV, outCV);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetEngineType(EngineModelType::SHAFTS);
    my_hmmwv.SetTransmissionType(TransmissionModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    ChContactMaterialData minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), (float)terrainLength, (float)terrainWidth);
    terrain.Initialize();

    // ---------------
    // Add bump
    // ---------------

    ChVector3d fwd_dir = initRot.GetXaxis();
    ChVector3d loc = ChVector3d(initLoc.x(), initLoc.y(), bump_height - bump_radius) + fwd_dir * dist_to_bump;

    std::cout << loc.x() << "  " << loc.y() << "  " << loc.z() << std::endl;

    auto bump = chrono_types::make_shared<ChBody>();
    bump->SetPos(loc);
    bump->SetRot(initRot);
    bump->SetFixed(true);
    bump->EnableCollision(true);

    auto bump_mat = ChContactMaterial::DefaultMaterial(my_hmmwv.GetSystem()->GetContactMethod());
    utils::AddCylinderGeometry(bump.get(), bump_mat, bump_radius, 4.0);

    bump->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.4f));

    my_hmmwv.GetSystem()->AddBody(bump);

    // ------------------------
    // Create the driver system
    // ------------------------

    auto path = CreateBezierPath(initLoc, fwd_dir, 2 * dist_to_bump);

    ChPathFollowerDriver driver_follower(my_hmmwv.GetVehicle(), path, "my_path", target_speed);
    driver_follower.GetSteeringController().SetLookAheadDistance(5);
    driver_follower.GetSteeringController().SetGains(0.5, 0, 0);
    driver_follower.GetSpeedController().SetGains(0.4, 0, 0);
    driver_follower.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Steering Controller Demo");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->SetHUDLocation(500, 20);
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

    // -----------------
    // Initialize output
    // -----------------

    if (state_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    utils::ChRunningAverage filter_drv_acc(filter_window_size);
    utils::ChRunningAverage filter_com_acc(filter_window_size);
    utils::ChRunningAverage filter_flw_acc(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------

    auto chassis = my_hmmwv.GetVehicle().GetChassis();
    auto chassis_body = my_hmmwv.GetVehicle().GetChassisBody();
    auto wheel_body = my_hmmwv.GetVehicle().GetAxle(0)->GetWheel(LEFT)->GetSpindle();

    // Driver location in vehicle local frame
    ChVector3d drv_pos_loc = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int sim_frame = 0;
    int render_frame = 0;

    while (vis->Run()) {
        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();

        ChVector3d drv_pos_abs = my_hmmwv.GetVehicle().GetDriverPos();
        ChVector3d veh_pos_abs = my_hmmwv.GetVehicle().GetPos();
        ChVector3d com_pos_abs = chassis_body->GetPos();
        ChVector3d flw_pos_abs = wheel_body->GetPos();

        ChVector3d drv_acc_loc = my_hmmwv.GetVehicle().GetPointAcceleration(drv_pos_loc);
        ChVector3d drv_acc_abs = chassis_body->GetFrameRefToAbs().PointAccelerationLocalToParent(drv_pos_loc);
        ChVector3d com_acc_abs = chassis_body->GetPosDt2();
        ChVector3d flw_acc_abs = wheel_body->GetPosDt2();

        double vert_drv_acc_abs = filter_drv_acc.Add(drv_acc_abs.z());
        double vert_com_acc_abs = filter_com_acc.Add(com_acc_abs.z());
        double vert_flw_acc_abs = filter_flw_acc.Add(flw_acc_abs.z());

        // Collect output
        if (state_output) {
            csv << time << my_hmmwv.GetVehicle().GetSpeed();
            csv << drv_pos_loc << drv_pos_abs << com_pos_abs << flw_pos_abs;
            csv << drv_acc_loc << drv_acc_abs << com_acc_abs << flw_acc_abs;
            csv << vert_drv_acc_abs << vert_com_acc_abs << vert_flw_acc_abs;
            csv << std::endl;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver_follower.GetInputs();

        /*
        // Hack for acceleration-braking maneuver
        static bool braking = false;
        if (my_hmmwv.GetVehicle().GetSpeed() > target_speed)
            driver_inputs.m_braking = true;
        if (braking) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            driver_inputs.m_throttle = 1;
            driver_inputs.m_braking = 0;
        }
        */

        // Update sentinel and target location markers for the path-follower controller.
        const ChVector3d& pS = driver_follower.GetSteeringController().GetSentinelLocation();
        const ChVector3d& pT = driver_follower.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Render scene and output POV-Ray data
        if (sim_frame % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            render_frame++;
        }

        // Update modules (process inputs from other modules)
        driver_follower.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver_follower.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    if (state_output)
        csv.WriteToFile(out_dir + "/state.out");

    return 0;
}
