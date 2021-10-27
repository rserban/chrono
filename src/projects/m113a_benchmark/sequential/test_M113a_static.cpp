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
// Test program for reporting M113 static component heights.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/m113a/M113a_Vehicle.h"
#include "chrono_models/vehicle/m113a/M113a_SimplePowertrain.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

// Uncomment the following line to unconditionally disable Irrlicht support
//#undef CHRONO_IRRLICHT
#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"
#endif

// VehicleTests configuration header
// (for definitions of CHRONO_DATA_DIR and CHRONO_VEHICLE_DATA_DIR)
#include "VehicleTestsConfig.h"
#include "utils/VehicleTestsData.h"                     // path to test data

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================

// Initial vehicle position
ChVector<> initLoc(0, 0, 0.75);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
double target_speed = 0;

//Chassis Corner Point Locations
ChVector<> FrontLeftCornerLoc(13.5 * 0.0254, 53.0 * 0.0254, 0.0);
ChVector<> FrontRightCornerLoc(13.5 * 0.0254, -53.0 * 0.0254, 0.0);
ChVector<> RearLeftCornerLoc(-178.2 * 0.0254, 53.0 * 0.0254, -5.9 * 0.0254);
ChVector<> RearRightCornerLoc(-178.2 * 0.0254, -53.0 * 0.0254, -5.9 * 0.0254);
ChVector<> FrontLeftBellyLoc(0.214, 1.70/2, 0.343);
ChVector<> FrontRightBellyLoc(0.214, 1.70 / 2, 0.343);
ChVector<> MidLeftBellyLoc(0.041, 1.70 / 2, -0.143);
ChVector<> MidRightBellyLoc(0.041, -1.70 / 2, -0.143);
ChVector<> RearLeftBellyLoc(-4.170, 1.70 / 2, -0.143);
ChVector<> RearRightBellyLoc(-4.170, -1.70 / 2, -0.143);
ChVector<> ForwardLoc(1, 0, 0);
ChVector<> LeftLoc(0, 1, 0);

// Input file names for the path-follower driver model
std::string path_file("paths/straight10km.txt");
std::string steering_controller_file("M113/SteeringController_M113_doublelanechange.json");
std::string speed_controller_file("M113/SpeedController.json");

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 50.0;  // size in X direction
double terrainWidth = 50.0;   // size in Y direction

// Simulation step size
double step_size = 1e-3;

// Use MKL
bool use_mkl = false;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 100;  // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Simulation length (set to a negative value to disable for Irrlicht)
double tend = 10;

// Output directories (Povray only)
const std::string out_dir = "../M113_STATIC";
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
    // -------------------------------------------------------
    // Set path to Chrono and Chrono::Vehicle data directories
    // -------------------------------------------------------
    
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(CHRONO_VEHICLE_DATA_DIR);

    // ---------------
    // Create the M113
    // ---------------

    // Create the vehicle system
    ChassisCollisionType chassis_collision_type = ChassisCollisionType::PRIMITIVES;
    M113a_Vehicle vehicle(false, ChMaterialSurfaceBase::DEM, chassis_collision_type);
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
    M113a_SimplePowertrain powertrain;
    powertrain.Initialize(vehicle.GetChassisBody(), vehicle.GetDriveshaft());

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
    terrain.SetContactFrictionCoefficient(mu);
    terrain.SetContactRestitutionCoefficient(restitution);
    terrain.SetContactMaterialProperties(E, nu);
    terrain.SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth);

    // -------------------------------------
    // Create the path and the driver system
    // -------------------------------------

    ChBezierCurve* path = ChBezierCurve::read(tests::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, tests::GetDataFile(steering_controller_file),
                                tests::GetDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();


    // ------------------------------
    // Solver and integrator settings
    // ------------------------------
#ifndef CHRONO_MKL
    // Do not use MKL if not available
    use_mkl = false;
#endif

    if (use_mkl) {
#ifdef CHRONO_MKL
        auto mkl_solver = std::make_shared<ChSolverMKL<>>();
        mkl_solver->SetSparsityPatternLock(true);
        vehicle.GetSystem()->SetSolver(mkl_solver);

        vehicle.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(vehicle.GetSystem()->GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
        integrator->SetMode(ChTimestepperHHT::POSITION);
        integrator->SetModifiedNewton(false);
        integrator->SetScaling(true);
        integrator->SetVerbose(true);
#endif
    }
    else {
        vehicle.GetSystem()->SetSolverType(ChSolver::Type::MINRES);
        vehicle.GetSystem()->SetSolverWarmStarting(true);
        ////vehicle.GetSystem()->SetMaxItersSolverSpeed(100);
        ////vehicle.GetSystem()->SetMaxItersSolverStab(100);
        ////vehicle.GetSystem()->SetTol(0);
        ////vehicle.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
        ////vehicle.GetSystem()->SetMinBounceSpeed(2.0);
        ////vehicle.GetSystem()->SetSolverOverrelaxationParam(0.8);
        ////vehicle.GetSystem()->SetSolverSharpnessParam(1.0);
    }



#ifdef CHRONO_IRRLICHT

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp app(&vehicle, &powertrain, L"M113 acceleration");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(-70.f, -150.f, 100.f), irr::core::vector3df(-70.f, -50.f, 100.f), 250, 130);
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
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
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
    TrackShoeForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TrackShoeForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input = 0;
    double steering_input = 0;
    double braking_input = 0;

    bool ChassisContact = false;

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    double time = 0;
    int step_number = 0;
    int render_frame = 0;
    double theta = 0;

#ifdef CHRONO_IRRLICHT

    while (app.GetDevice()->run()) {
        time = vehicle.GetChTime();

        // End simulation
        if ((time > tend) && (tend > 0))
            break;
#else

    while (time <= tend) {
        time = vehicle.GetChTime();

#endif

        // Extract accelerations to add to the filter
        ChVector<> acc_CG = vehicle.GetChassisBody()->GetPos_dtdt();
        acc_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(acc_CG);
        ChVector<> acc_driver = vehicle.GetVehicleAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double vert_acc_CG = vert_acc_GC_filter.Add(acc_CG.z());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());
        double vert_acc_driver = vert_acc_driver_filter.Add(acc_driver.z());

        //Check for Chassis Contacts:
        ChassisContact |= vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS);

#ifdef CHRONO_IRRLICHT
        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));
#endif

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

            if (state_output) {
                ChVector<> vel_CG = vehicle.GetChassisBody()->GetPos_dt();
                vel_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(vel_CG);

                ChVector<> vel_driver_abs = vehicle.GetChassisBody()->GetFrame_REF_to_abs().PointSpeedLocalToParent(driver_pos);
                ChVector<> vel_driver_local = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformDirectionParentToLocal(vel_driver_abs);

                ChVector<> FrontLeftCornerPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontLeftCornerLoc);
                ChVector<> FrontRightCornerPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontRightCornerLoc);
                ChVector<> RearLeftCornerPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearLeftCornerLoc);
                ChVector<> RearRightCornerPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearRightCornerLoc);

                ChVector<> FrontLeftBellyPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontLeftBellyLoc);
                ChVector<> FrontRightBellyPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontRightBellyLoc);
                ChVector<> MidLeftBellyPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(MidLeftBellyLoc);
                ChVector<> MidRightBellyPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(MidRightBellyLoc);
                ChVector<> RearLeftBellyPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearLeftBellyLoc);
                ChVector<> RearRightBellyPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearRightBellyLoc);

                ChVector<> ForwardPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(ForwardLoc);
                ChVector<> LeftPos = vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(LeftLoc);

                //Vehicle and Control Values
                csv << time << steering_input << throttle_input << braking_input;
                csv << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetAxleSpeed()
                    << vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetAxleSpeed();
                csv << powertrain.GetMotorSpeed() << powertrain.GetMotorTorque();
                //Chassis Position, Velocity, & Acceleration (Unfiltered and Filtered)
                csv << vehicle.GetChassis()->GetPos().x() << vehicle.GetChassis()->GetPos().y()
                    << vehicle.GetChassis()->GetPos().z();
                csv << vel_CG.x() << vel_CG.y() << vel_CG.z();
                csv << acc_CG.x() << acc_CG.y() << acc_CG.z();
                csv << fwd_acc_CG << lat_acc_CG << vert_acc_CG;
                //Driver Position, Velocity, & Acceleration (Unfiltered and Filtered)
                csv << vehicle.GetDriverPos().x() << vehicle.GetDriverPos().y() << vehicle.GetDriverPos().z();
                csv << vel_driver_local.x() << vel_driver_local.y() << vel_driver_local.z();
                csv << acc_driver.x() << acc_driver.y() << acc_driver.z(); //Chassis CSYS
                csv << fwd_acc_driver << lat_acc_driver << vert_acc_driver; //filtered Chassis CSYS
                //Chassis Corner Point Positions
                csv << FrontLeftCornerPos.x() << FrontLeftCornerPos.y() << FrontLeftCornerPos.z();
                csv << FrontRightCornerPos.x() << FrontRightCornerPos.y() << FrontRightCornerPos.z();
                csv << RearLeftCornerPos.x() << RearLeftCornerPos.y() << RearLeftCornerPos.z();
                csv << RearRightCornerPos.x() << RearRightCornerPos.y() << RearRightCornerPos.z();
                //Extra Static Output Points
                csv << FrontLeftBellyPos.x() << FrontLeftBellyPos.y() << FrontLeftBellyPos.z();
                csv << FrontRightBellyPos.x() << FrontRightBellyPos.y() << FrontRightBellyPos.z();
                csv << MidLeftBellyPos.x() << MidLeftBellyPos.y() << MidLeftBellyPos.z();
                csv << MidRightBellyPos.x() << MidRightBellyPos.y() << MidRightBellyPos.z();
                csv << RearLeftBellyPos.x() << RearLeftBellyPos.y() << RearLeftBellyPos.z();
                csv << RearRightBellyPos.x() << RearRightBellyPos.y() << RearRightBellyPos.z();
                csv << vehicle.GetChassisBody()->GetPos().x() << vehicle.GetChassisBody()->GetPos().y() << vehicle.GetChassisBody()->GetPos().z();
                csv << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos().x()
                    << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos().y()
                    << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos().z();
                csv << vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos().x()
                    << vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos().y()
                    << vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos().z();
                csv << vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->GetPos().x()
                    << vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->GetPos().y()
                    << vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->GetPos().z();
                csv << vehicle.GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->GetPos().x()
                    << vehicle.GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->GetPos().y()
                    << vehicle.GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->GetPos().z();
                csv << ForwardPos.x() << ForwardPos.y() << ForwardPos.z();
                csv << LeftPos.x() << LeftPos.y() << LeftPos.z();
                if (ChassisContact) {
                    csv << 1;
                }
                else{
                    csv << 0;
                }
                csv << std::endl;
            }

            ChassisContact = false;
            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        throttle_input = 0;
        steering_input = 0;
        braking_input = 0;
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, shoe_forces_left, shoe_forces_right);
        terrain.Synchronize(time);
#ifdef CHRONO_IRRLICHT
        app.Synchronize("Follower driver", steering_input, throttle_input, braking_input);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        powertrain.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        app.Advance(step_size);
#endif

        // Increment frame number
        step_number++;
    }

    if (state_output) {
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
