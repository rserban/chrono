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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Test program for M113 rough terrain test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>
#include <string>

#include "chrono/core/ChStream.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/m113a/M113a_Vehicle.h"
#include "chrono_models/vehicle/m113a/M113a_SimplePowertrain.h"

#include "chrono_thirdparty/filesystem/path.h"

// Uncomment the following line to unconditionally disable Irrlicht support
//#undef CHRONO_IRRLICHT
#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================

// Initial vehicle position
ChVector<> initLoc(-195.0, 0.0, 0.75);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
// set in main function or as a calling argument

// Chassis Corner Point Locations
ChVector<> FrontLeftCornerLoc(13.5 * 0.0254, 53.0 * 0.0254, 0.0);
ChVector<> FrontRightCornerLoc(13.5 * 0.0254, -53.0 * 0.0254, 0.0);
ChVector<> RearLeftCornerLoc(-178.2 * 0.0254, 53.0 * 0.0254, -5.9 * 0.0254);
ChVector<> RearRightCornerLoc(-178.2 * 0.0254, -53.0 * 0.0254, -5.9 * 0.0254);

// Input file names for the path-follower driver model
std::string path_file("M113a_benchmark/paths/straight10km.txt");
std::string steering_controller_file("M113a_benchmark/SteeringController_M113_roughTerrain.json");
std::string speed_controller_file("M113a_benchmark/SpeedController.json");

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1e-4;  // output step size in seconds

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Simulation length (set to a negative value to disable for Irrlicht)
// double tend = 40.0;

// Output directories (Povray only)
const std::string out_dir = "../M113_ROUGHTERRAIN";
const std::string pov_dir = out_dir + "/POVRAY";

// Visualization type for all vehicle subsystems (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::PRIMITIVES;

// POV-Ray output
bool povray_output = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = true;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    // Default target speed and profile code (1,2,3).
    double target_speed = 15.0;
    int profile_code = 1;
    double swept_radius = 0.01;

    // Simple argument parsing.
    // Unless both speed and profile code are specified, use default values.
    if (argc == 3) {
        target_speed = std::stod(argv[1]);
        profile_code = std::stoi(argv[2]);
    }

    // Set name of profile input file
    std::string profile_filename = "";
    switch (profile_code) {
        case 1:
            profile_filename = "M113a_benchmark/meshes/RandomRoad_3cm.obj";
            break;
        case 2:
            profile_filename = "M113a_benchmark/meshes/RandomRoad_6cm.obj";
            break;
        case 3:
            profile_filename = "M113a_benchmark/meshes/RandomRoad_9cm.obj";
            break;
        case 4:
            profile_filename = "M113a_benchmark/meshes/RandomRoad_3cm.obj";
            swept_radius = 0.002;
            break;
        case 5:
            profile_filename = "M113a_benchmark/meshes/RandomRoad_6cm.obj";
            swept_radius = 0.002;
            break;
        case 6:
            profile_filename = "M113a_benchmark/meshes/RandomRoad_9cm.obj";
            swept_radius = 0.002;
            break;
        default:
            std::cout << "Incorrect profile code.  Must be one of: 1, 2, 3, 4, 5, or 6." << std::endl;
            return 1;
    }

    std::cout << "Target speed: " << target_speed << std::endl;
    std::cout << "Road profile file: " << profile_filename << std::endl;

    // ---------------
    // Create the M113
    // ---------------

    // Create the vehicle system
    CollisionType chassis_collision_type = CollisionType::PRIMITIVES;
    M113a_Vehicle vehicle(false, ChContactMethod::SMC, chassis_collision_type);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Set visualization type for subsystems
    vehicle.SetChassisVisualizationType(vis_type);
    vehicle.SetSprocketVisualizationType(vis_type);
    vehicle.SetIdlerVisualizationType(vis_type);
    vehicle.SetRoadWheelAssemblyVisualizationType(vis_type);
    vehicle.SetRoadWheelVisualizationType(vis_type);
    vehicle.SetTrackShoeVisualizationType(vis_type);

    // Control steering type (enable crossdrive capability).
    vehicle.GetDriveline()->SetGyrationMode(true);

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<M113a_SimplePowertrain>("Powertrain");
    vehicle.InitializePowertrain(powertrain);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->EnableWarmStart(true);
    solver->SetTolerance(1e-10);
    vehicle.GetSystem()->SetSolver(solver);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////vehicle.SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////vehicle.SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////vehicle.SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    float mu = 0.8f;
    float restitution = 0.01f;
    float E = 2e7f;
    float nu = 0.3f;

    RigidTerrain terrain(vehicle.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(mu);
    patch_mat->SetRestitution(restitution);
    patch_mat->SetYoungModulus(E);
    patch_mat->SetPoissonRatio(nu);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile(profile_filename), swept_radius);
    patch->SetColor(ChColor(0.4f, 0.2f, 0.0f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 12, 12);
    terrain.Initialize();

    // -------------------------------------
    // Create the path and the driver system
    // -------------------------------------

    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, true);
    driver.Initialize();

#ifdef CHRONO_IRRLICHT

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp app(&vehicle, L"M113 rough terrain");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

#endif

    // ------------------------------------
    // Prepare output directories and files
    // ------------------------------------

    state_output = state_output || povray_output;

    // Create output directories
    if (state_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        driver.ExportPathPovray(out_dir);
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage vert_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage vert_acc_driver_filter(filter_window_size);

    // Driver location in vehicle local frame
    ChVector<> driver_pos = vehicle.GetChassis()->GetLocalDriverCoordsys().pos;

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));
    ChDriver::Inputs driver_inputs = {0, 0, 0};

    bool ChassisContact = false;
    bool reset_arm_angle = true;
    double maximum_arm_angle = 0;
    double minimum_arm_angle = 0;

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    double time = 0;
    long step_number = 0;
    int render_frame = 0;
    double theta = 0;
    bool org_time = true;

    while ((vehicle.GetChassis()->GetPos().x()) <= 1005) {
        time = vehicle.GetChTime();

#ifdef CHRONO_IRRLICHT
        if (!app.GetDevice()->run())
            break;
#endif

        // Extract accelerations to add to the filter
        ChVector<> acc_CG = vehicle.GetChassisBody()->GetPos_dtdt();
        acc_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(acc_CG);
        ChVector<> acc_driver = vehicle.GetVehiclePointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double vert_acc_CG = vert_acc_GC_filter.Add(acc_CG.z());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());
        double vert_acc_driver = vert_acc_driver_filter.Add(acc_driver.z());

        // Check for Chassis Contacts:
        ChassisContact |= vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS);

        if (reset_arm_angle) {
            reset_arm_angle = false;
            maximum_arm_angle = vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetRoadWheelAssembly(0)->GetCarrierAngle();
            minimum_arm_angle = maximum_arm_angle;
        }
        for (size_t i = 0; i < vehicle.GetTrackAssembly(LEFT)->GetNumRoadWheelAssemblies(); i++) {
            maximum_arm_angle =
                (maximum_arm_angle >=
                vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetRoadWheelAssembly(i)->GetCarrierAngle())
                ? maximum_arm_angle
                : vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetRoadWheelAssembly(i)->GetCarrierAngle();
            minimum_arm_angle =
                (minimum_arm_angle <=
                vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetRoadWheelAssembly(i)->GetCarrierAngle())
                ? minimum_arm_angle
                : vehicle.GetTrackAssembly(VehicleSide::LEFT)->GetRoadWheelAssembly(i)->GetCarrierAngle();
        }
        for (size_t i = 0; i < vehicle.GetTrackAssembly(RIGHT)->GetNumRoadWheelAssemblies(); i++) {
            maximum_arm_angle =
                (maximum_arm_angle >=
                vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetRoadWheelAssembly(i)->GetCarrierAngle())
                ? maximum_arm_angle
                : vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetRoadWheelAssembly(i)->GetCarrierAngle();
            minimum_arm_angle =
                (minimum_arm_angle <=
                vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetRoadWheelAssembly(i)->GetCarrierAngle())
                ? minimum_arm_angle
                : vehicle.GetTrackAssembly(VehicleSide::RIGHT)->GetRoadWheelAssembly(i)->GetCarrierAngle();
        }

#ifdef CHRONO_IRRLICHT
        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));
#endif


		if ((state_output) && (step_number % output_steps == 0)) {
			ChVector<> vel_CG = vehicle.GetChassisBody()->GetPos_dt();
			vel_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(vel_CG);

			ChVector<> vel_driver_abs =
				vehicle.GetChassisBody()->GetFrame_REF_to_abs().PointSpeedLocalToParent(driver_pos);
			ChVector<> vel_driver_local =
				vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformDirectionParentToLocal(vel_driver_abs);

			ChVector<> FrontLeftCornerPos =
				vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontLeftCornerLoc);
			ChVector<> FrontRightCornerPos =
				vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontRightCornerLoc);
			ChVector<> RearLeftCornerPos =
				vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearLeftCornerLoc);
			ChVector<> RearRightCornerPos =
				vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearRightCornerLoc);

			// Vehicle and Control Values
            csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
            csv << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetAxleSpeed()
				<< vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetAxleSpeed();
			csv << powertrain->GetMotorSpeed() << powertrain->GetMotorTorque();
			// Chassis Position, Velocity, & Acceleration (Unfiltered and Filtered)
			csv << vehicle.GetChassis()->GetPos().x() << vehicle.GetChassis()->GetPos().y()
				<< vehicle.GetChassis()->GetPos().z();
			csv << vel_CG.x() << vel_CG.y() << vel_CG.z();
			csv << acc_CG.x() << acc_CG.y() << acc_CG.z();
			csv << fwd_acc_CG << lat_acc_CG << vert_acc_CG;
			// Driver Position, Velocity, & Acceleration (Unfiltered and Filtered)
			csv << vehicle.GetDriverPos().x() << vehicle.GetDriverPos().y() << vehicle.GetDriverPos().z();
			csv << vel_driver_local.x() << vel_driver_local.y() << vel_driver_local.z();
			csv << acc_driver.x() << acc_driver.y() << acc_driver.z();   // Chassis CSYS
			csv << fwd_acc_driver << lat_acc_driver << vert_acc_driver;  // filtered Chassis CSYS
																		 // Chassis Corner Point Positions
			csv << FrontLeftCornerPos.x() << FrontLeftCornerPos.y() << FrontLeftCornerPos.z();
			csv << FrontRightCornerPos.x() << FrontRightCornerPos.y() << FrontRightCornerPos.z();
			csv << RearLeftCornerPos.x() << RearLeftCornerPos.y() << RearLeftCornerPos.z();
			csv << RearRightCornerPos.x() << RearRightCornerPos.y() << RearRightCornerPos.z();
			if (ChassisContact) {
				csv << 1;
			}
			else {
				csv << 0;
			}
			csv << maximum_arm_angle << minimum_arm_angle;
			csv << std::endl;

			ChassisContact = false;
			reset_arm_angle = true;
		}


        // Render scene
        if (step_number % render_steps == 0) {
#ifdef CHRONO_IRRLICHT
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
#endif

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);
            }

            render_frame++;

            // if (org_time && (profile_code <= 3) && (vehicle.GetChassis()->GetPos().x() >= -5)) {
            //    step_size = 1e-4;
            //    // Number of simulation steps between two 3D view render frames
            //    render_steps = (int)std::ceil(render_step_size / step_size);
            //    step_number = 0;
            //    org_time = false;
            //}

            if (step_number % int(std::round(2 / step_size)) == 0) {
                char filename[100];
                sprintf(filename, "%s/output_%.2f_%d.dat", out_dir.c_str(), target_speed, profile_code);
                csv.write_to_file(filename);
            }
        }

        // Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        terrain.Synchronize(time);
#ifdef CHRONO_IRRLICHT
        app.Synchronize("Follower driver", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        app.Advance(step_size);
#endif

        // Increment frame number
        step_number++;

		if (vehicle.GetChassis()->GetPos().x() > -5) {
			step_size = 1e-4;
			output_steps = (int)std::ceil(output_step_size / step_size);
		}
    }

    // Write output to file.
    if (state_output) {
        char filename[100];
        sprintf(filename, "%s/output_%.2f_%d.dat", out_dir.c_str(), target_speed, profile_code);
        csv.write_to_file(filename);
    }

    return 0;
}