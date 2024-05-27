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
// Test program for M113 half round obstacle test.
//
// Vehicle reference frame: X forward, Y left, Z up.
//
// =============================================================================

#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

// Uncomment the following line to unconditionally disable Irrlicht support
////#undef CHRONO_IRRLICHT

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================

// Simulation step size (acceleration phase)
double step_size = 1e-3;

// Simulation step size (obstacle crossing phase)
double step_size_O = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;

// Time interval between two output frames
double output_step_size = 1.0 / 100;

// Output directory
const std::string out_dir = "../M113_HALFROUNDOBSTACLE";

// Visualization type for all vehicle subsystems (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::PRIMITIVES;

// Vehicle state output
bool state_output = false;
int filter_window_size = 20;

// =============================================================================

// Forward declarations
void AddHalfround(ChSystem* system, double radius, double obstacle_distance);

// =============================================================================

int main(int argc, char* argv[]) {
    double radius = 4.0;            // inches
    double target_speed = 8;        // m/s
    double obstacle_distance = 50;  // m

    // Check for input arguments for running this test in batch
    if (argc > 1)  // vehicle target speed (m/s)
        target_speed = std::atof(argv[1]);
    if (argc > 2)  // bump radius (inches)
        radius = std::atof(argv[2]);
    if (argc > 3)  // distance to bump (m)
        obstacle_distance = std::atof(argv[3]);

    std::cout << target_speed << radius << obstacle_distance << std::endl;

    // ---------------
    // Create the M113
    // ---------------

    // Create the vehicle system
    M113 m113;
    m113.SetContactMethod(ChContactMethod::SMC);
    m113.SetChassisCollisionType(CollisionType::PRIMITIVES);
    m113.SetChassisFixed(false);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetDrivelineType(DrivelineTypeTV::SIMPLE);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);

    m113.SetInitPosition(ChCoordsys<>(ChVector3d(0.0, 0.0, 1.0), QUNIT));
    m113.Initialize();
    auto& vehicle = m113.GetVehicle();
    auto engine = vehicle.GetEngine();

    // Set visualization type for subsystems
    vehicle.SetChassisVisualizationType(vis_type);
    vehicle.SetSprocketVisualizationType(vis_type);
    vehicle.SetIdlerVisualizationType(vis_type);
    vehicle.SetSuspensionVisualizationType(vis_type);
    vehicle.SetRoadWheelVisualizationType(vis_type);
    vehicle.SetTrackShoeVisualizationType(vis_type);

    // Control steering type (enable crossdrive capability)
    vehicle.GetDriveline()->SetGyrationMode(true);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////vehicle.EnableCollision(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////vehicle.EnableCollision(TrackedCollisionFlag::NONE);

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
    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    material->SetFriction(mu);
    material->SetRestitution(restitution);
    material->SetYoungModulus(E);
    material->SetPoissonRatio(nu);

    RigidTerrain terrain(vehicle.GetSystem());
    auto patch = terrain.AddPatch(material, CSYSNORM, 400, 10);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 800, 20);
    terrain.Initialize();

    AddHalfround(vehicle.GetSystem(), radius * 0.0254, obstacle_distance);

    // -------------------------------------
    // Create the path and the driver system
    // -------------------------------------

    auto path = StraightLinePath(ChVector3d(-10, 0, 0.25), ChVector3d(200, 0, 0.25));
    ChPathFollowerDriver driver(vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(1.0, 0.5, 0.5);
    driver.GetSpeedController().SetGains(20.0, 0.0, 0.0);
    driver.Initialize();

#ifdef CHRONO_IRRLICHT

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("M113 half round obstacle");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 0.0), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 128, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 128, 0);

#endif

    // ----------------------------------
    // Prepare output directory and files
    // ----------------------------------

    // Create output directory
    if (state_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage vert_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage vert_acc_driver_filter(filter_window_size);

    // Driver location in vehicle local frame
    ChVector3d driver_pos = vehicle.GetChassis()->GetLocalDriverCoordsys().pos;

    // ---------------
    // Solver settings
    // ---------------

    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(50);
    solver->SetTolerance(1e-10);
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    vehicle.GetSystem()->SetSolver(solver);

    vehicle.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input = 0;
    double steering_input = 0;
    double braking_input = 0;

    bool contact_detected = false;

    // Number of simulation steps between two render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    while (true) {
        // Current time
        double time = vehicle.GetChTime();

        // Extract chassis CG and driver accelerations
        ChVector3d acc_CG = vehicle.GetChassisBody()->GetPosDt2();                 // global frame
        acc_CG = vehicle.GetChassisBody()->TransformDirectionParentToLocal(acc_CG);  // local frame
        ChVector3d acc_driver = vehicle.GetPointAcceleration(driver_pos);     // local frame

        // Filter accelerations
        double acc_CG_x = fwd_acc_GC_filter.Add(acc_CG.x());
        double acc_CG_y = lat_acc_GC_filter.Add(acc_CG.y());
        double acc_CG_z = vert_acc_GC_filter.Add(acc_CG.z());
        double acc_driver_x = fwd_acc_driver_filter.Add(acc_driver.x());
        double acc_driver_y = lat_acc_driver_filter.Add(acc_driver.y());
        double acc_driver_z = vert_acc_driver_filter.Add(acc_driver.z());

        // Check for chassis contacts
        contact_detected = contact_detected || vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS);

        if (state_output && (step_number % output_steps == 0)) {
            // Vehicle control values
            csv << time << steering_input << throttle_input << braking_input;
            csv << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetAxleSpeed()
                << vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetAxleSpeed();
            csv << vehicle.GetEngine()->GetMotorSpeed() << vehicle.GetEngine()->GetOutputMotorshaftTorque();

            // Chassis CG position and local acceleration (raw and filtered)
            csv << vehicle.GetCOMFrame().GetPos() << acc_CG;
            csv << acc_CG_x << acc_CG_y << acc_CG_z;

            // Driver position and local acceleration (raw and filtered)
            csv << vehicle.GetDriverPos() << acc_driver;
            csv << acc_driver_x << acc_driver_y << acc_driver_z;

            // Chassis contact since last output time?
            csv << (contact_detected ? 1 : 0);
            csv << std::endl;

            contact_detected = false;
        }

#ifdef CHRONO_IRRLICHT
        if (step_number % render_steps == 0) {
            // Update sentinel and target location markers for the path-follower controller.
            const ChVector3d& pS = driver.GetSteeringController().GetSentinelLocation();
            const ChVector3d& pT = driver.GetSteeringController().GetTargetLocation();
            ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
            ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

            if (!vis->Run())
                break;

            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }
#endif

        // Collect output data from sub-systems
        DriverInputs driver_inputs = driver.GetInputs();
        powertrain_torque = vehicle.GetTransmission()->GetOutputDriveshaftTorque();
        driveshaft_speed = vehicle.GetDriveline()->GetOutputDriveshaftSpeed();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update sub-systems (process inputs from other sub-systems)
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        terrain.Synchronize(time);
#ifdef CHRONO_IRRLICHT
        vis->Synchronize(time, driver_inputs);
#endif

        // Adjust step size for obstacle crossing
        if (vehicle.GetPos().x() > (obstacle_distance - 2)) {
            step_size = step_size_O;
            render_steps = (int)std::ceil(render_step_size / step_size);
            output_steps = (int)std::ceil(output_step_size / step_size);
        }

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        vis->Advance(step_size);
#endif

        // Increment frame number
        step_number++;

        // End simulation after clearing obstacle
        if (vehicle.GetPos().x() > obstacle_distance + 10)
            break;
    }

    // Write output file to disk
    if (state_output) {
        char filename[100];
        sprintf(filename, "%s/output_%.2f_%d.dat", out_dir.c_str(), target_speed, int(std::round(radius)));
        csv.WriteToFile(filename);
    }

    return 0;
}

// =============================================================================
void AddHalfround(ChSystem* system, double radius, double obstacle_distance) {
    double length = 10;

    float friction_coefficient = 0.8f;
    float restitution_coefficient = 0.01f;

    auto obstacle = chrono_types::make_shared<ChBody>();
    obstacle->SetPos(ChVector3d(0, 0, 0));
    obstacle->SetFixed(true);
    obstacle->EnableCollision(true);

    // Visualization
    auto shape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, length);
    shape->SetColor(ChColor(1, 1, 1));
    obstacle->AddVisualShape(shape, ChFrame<>(ChVector3d(obstacle_distance, 0, 0), QuatFromAngleX(CH_PI_2)));

    obstacle->GetVisualShape(0)->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 10, 10);

    // Contact
    auto obst_mat = ChContactMaterial::DefaultMaterial(system->GetContactMethod());
    obst_mat->SetFriction(friction_coefficient);
    obst_mat->SetRestitution(restitution_coefficient);

    obstacle->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCylinder>(obst_mat, radius, length * 0.5),
                                ChVector3d(obstacle_distance, 0, 0));

    system->AddBody(obstacle);
}
