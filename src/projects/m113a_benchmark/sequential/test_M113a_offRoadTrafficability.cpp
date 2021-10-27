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
// Authors: Radu Serban, Daniel Melanz
// =============================================================================
//
// Test program for M113 off-road trafficability.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChStream.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
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
ChVector<> initLoc(-3, 0, 0.8);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
// double target_speed = 15.6464;  // 35 mph
// double target_speed = 11.176;  // 25 mph
// double target_speed = 6.7056;  // 15 mph
double target_speed = 4;  // 5 mph
// double target_speed = 2.2352;  // 5 mph

// Control slip?
bool controlSlip = false;
double slip = 1;
double sprocketRadius = 0.2605 + 0.5 * 0.06;
double rotSpeed = 1;

// Use deformable soil?
bool useDefSoil = true;

double traversalLength = 3;
double lookAheadDistance = 2;

int numPasses = 1;

double chassisMass = 2086.524902;  // Default value is 2086.524902 kg

int terrainType = 1;  // 1: LETE Sand, 2: Muskeg, 3: Snow

float mu = 0.8f;
float restitution = 0.01f;
float E = 2e7f;
float nu = 0.3f;

double depth = 10;
double factor = 10;  // How many soil nodes per meter

// Chassis Corner Point Locations
ChVector<> FrontLeftCornerLoc(13.5 * 0.0254, 53.0 * 0.0254, 0.0);
ChVector<> FrontRightCornerLoc(13.5 * 0.0254, -53.0 * 0.0254, 0.0);
ChVector<> RearLeftCornerLoc(-178.2 * 0.0254, 53.0 * 0.0254, -5.9 * 0.0254);
ChVector<> RearRightCornerLoc(-178.2 * 0.0254, -53.0 * 0.0254, -5.9 * 0.0254);

// Input file names for the path-follower driver model
std::string path_file("M113a_benchmark/paths/straightOrigin.txt");
std::string steering_controller_file("M113a_benchmark/SteeringController_M113_sideSlopeStability.json");
std::string speed_controller_file("M113a_benchmark/SpeedController.json");

// Rigid terrain dimensions
double terrainLength = 18.0;  // size in X direction
double terrainWidth = 3.0;    // size in Y direction

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 100;  // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Simulation length (set to a negative value to disable for Irrlicht)
double tend = 2;

// Output directories (Povray only)
std::string out_dir = "../M113_OFFROADTRAFFICABILITY";
std::string pov_dir = out_dir + "/POVRAY";

// Visualization type for all vehicle subsystems (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::PRIMITIVES;

// POV-Ray output
bool povray_output = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = true;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    if (argc > 1) {
        numPasses = atoi(argv[1]);
        chassisMass = atof(argv[2]);
        controlSlip = (atoi(argv[3]) != 0);
        slip = atof(argv[4]);
        factor = atof(argv[5]);

        std::stringstream dataFolderStream;
        dataFolderStream << "../M113_OFFROADTRAFFICABILITY_nPasses" << numPasses << "_ChassisWeight" << chassisMass
                         << "_slipControlled" << controlSlip << "_slip" << slip << "_nodes" << factor << "/";
        out_dir = dataFolderStream.str();
        pov_dir = out_dir + "/POVRAY";
    }

    // If controlling slip, create a smaller contact patch
    if (controlSlip) {
        terrainLength = 6.0;  // size in X direction
        initLoc = ChVector<>(1, 0, 0.6);
        terrainWidth = 3.0;  // size in Y direction
    }

    std::stringstream outputFileStream;
    outputFileStream << out_dir << "/output_nPasses" << numPasses << "_ChassisWeight" << chassisMass
                     << "_slipControlled" << controlSlip << "_slip" << slip << "_nodes" << factor << ".dat";

    // ---------------
    // Create the M113
    // ---------------

    // Create the vehicle system
    M113a_Vehicle vehicle(false, ChContactMethod::SMC);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Set visualization type for subsystems
    vehicle.SetChassisVisualizationType(vis_type);
    vehicle.SetSprocketVisualizationType(vis_type);
    vehicle.SetIdlerVisualizationType(vis_type);
    vehicle.SetRoadWheelAssemblyVisualizationType(vis_type);
    vehicle.SetRoadWheelVisualizationType(vis_type);
    vehicle.SetTrackShoeVisualizationType(vis_type);

    if (chassisMass > 0)
        vehicle.GetChassisBody()->SetMass(chassisMass);

    vehicle.GetChassisBody()->SetCollide(true);

    // Add collision geometry to vehicle
    /*
    auto chassis_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    chassis_mat->SetFriction(mu);
    chassis_mat->SetRestitution(restitution);
    chassis_mat->SetYoungModulus(E);
    chassis_mat->SetPoissonRatio(nu);

    vehicle.GetChassisBody()->GetCollisionModel()->ClearModel();

    // Add flat bottom plate
    vehicle.GetChassisBody()->GetCollisionModel()->AddBox(chassis_mat, 0.5 * 4.25, 0.5 * 1.65, 0.05,
                                                          ChVector<>(-1.925, 0, -.153 + 0.1));

    auto bottomPlate = chrono_types::make_shared<ChBoxShape>();
    bottomPlate->GetBoxGeometry().Size = ChVector<>(0.5 * 4.25, 0.5 * 1.65, 0.05);
    bottomPlate->GetBoxGeometry().Pos = ChVector<>(-1.925, 0, -.153 + 0.05);
    vehicle.GetChassisBody()->AddAsset(bottomPlate);

    // Add angled bottom plate
    // ChQuaternion<> plateAngleQuat = chrono::Q_from_AngAxis(1.0474, VECT_Y);
    ChMatrix33<> plateAngle;
    plateAngle.Set_A_quaternion(chrono::Q_from_AngAxis(-1.0474, VECT_Y));
    vehicle.GetChassisBody()->GetCollisionModel()->AddBox(chassis_mat, 0.5 * .3521, 0.5 * 1.65, 0.05,
                                                          ChVector<>(.2457, 0, 0.0245), plateAngle);

    auto angledPlate = chrono_types::make_shared<ChBoxShape>();
    angledPlate->GetBoxGeometry().Size = ChVector<>(0.5 * .3521, 0.5 * 1.65, 0.05);
    angledPlate->GetBoxGeometry().Pos = ChVector<>(.2457, 0, 0.0245);
    angledPlate->GetBoxGeometry().Rot = plateAngle;
    vehicle.GetChassisBody()->AddAsset(angledPlate);

    vehicle.GetChassisBody()->GetCollisionModel()->BuildModel();
    */

    // Control steering type (enable crossdrive capability).
    vehicle.GetDriveline()->SetGyrationMode(true);

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<M113a_SimplePowertrain>("Powertrain");
    if (!controlSlip)
        vehicle.InitializePowertrain(powertrain);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->EnableWarmStart(true);
    solver->SetTolerance(1e-10);
    vehicle.GetSystem()->SetSolver(solver);

    // ------------------
    // Create the terrain
    // ------------------

    // Deformable terrain properties (default is LETE sand)
    double Kphi = 5301e3;
    double Kc = 102e3;
    double n = 0.793;
    double c = 1.3e3;
    double phi = 31.1;
    double K = 1.2e-2;
    double E_elastic = 2e8;
    double damping = 3e4;

    switch (terrainType) {
        case 1:
            // LETE sand
            Kphi = 5301e3;
            Kc = 102e3;
            n = 0.793;
            c = 1.3e3;
            phi = 31.1;
            K = 1.2e-2;
            E_elastic = 2e8;
            damping = 3e4;
            break;
        case 2:
            // Petawawa muskeg A
            Kphi = 5301e3;
            Kc = 102e3;
            n = 0.793;
            c = 2.8e3;
            phi = 39.4;
            K = 3.1;
            E_elastic = 2e8;
            damping = 3e4;
            break;
        case 3:
            // Petawawa snow A (Rubber-snow)
            Kphi = 5301e3;
            Kc = 102e3;
            n = 0.793;
            c = 0.12e3;
            phi = 16.4;
            K = 0.4e-2;
            E_elastic = 2e8;
            damping = 3e4;
            break;
    }

    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ground_mat->SetFriction(mu);
    ground_mat->SetRestitution(restitution);
    ground_mat->SetYoungModulus(E);
    ground_mat->SetPoissonRatio(nu);

    ChTerrain* terrain;
    if (useDefSoil) {
        auto terrain_D = new SCMDeformableTerrain(vehicle.GetSystem());
        terrain_D->SetPlane(ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
        terrain_D->SetSoilParameters(Kphi,       // Bekker Kphi
                                     Kc,         // Bekker Kc
                                     n,          // Bekker n exponent
                                     c,          // Mohr cohesive limit (Pa)
                                     phi,        // Mohr friction limit (degrees)
                                     K,          // Janosi shear coefficient (m)
                                     E_elastic,  // Elastic stiffness (Pa/m), before plastic yeld
                                     damping     // Damping coefficient (Pa*s/m)
        );
        ////terrain_D->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 80, 16);
        terrain_D->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
        ////terrain_D->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
        terrain_D->Initialize(terrainLength, terrainWidth, 1.0 / factor);
        terrain = terrain_D;
    } else {
        auto terrain_R = new RigidTerrain(vehicle.GetSystem());
        auto patch = terrain_R->AddPatch(ground_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1),
                                         terrainLength, terrainWidth);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
        terrain_R->Initialize();
        terrain = terrain_R;
    }

    // -------------------
    // Create the rigid ground
    // -------------------

    auto ground1 = std::shared_ptr<ChBody>(vehicle.GetSystem()->NewBody());
    ground1->SetIdentifier(-1);
    ground1->SetName("ground1");
    ground1->SetPos(ChVector<>(-terrainLength, 0, 0));
    ground1->SetBodyFixed(true);
    ground1->SetCollide(true);
    vehicle.GetSystem()->AddBody(ground1);

    ground1->GetCollisionModel()->ClearModel();
    ground1->GetCollisionModel()->AddBox(ground_mat, 0.5 * terrainLength, 0.5 * terrainWidth, 0.5 * depth,
                                         ChVector<>(0, 0, -0.5 * depth));

    auto box1 = chrono_types::make_shared<ChBoxShape>();
    box1->GetBoxGeometry().Size = ChVector<>(0.5 * terrainLength, 0.5 * terrainWidth, 0.5 * depth);
    box1->GetBoxGeometry().Pos = ChVector<>(0, 0, -0.5 * depth);
    ground1->AddAsset(box1);
    ground1->GetCollisionModel()->BuildModel();

    auto ground2 = std::shared_ptr<ChBody>(vehicle.GetSystem()->NewBody());
    ground2->SetIdentifier(-2);
    ground2->SetName("ground2");
    ground2->SetPos(ChVector<>(terrainLength, 0, 0));
    ground2->SetBodyFixed(true);
    ground2->SetCollide(true);
    vehicle.GetSystem()->AddBody(ground2);

    ground2->GetCollisionModel()->ClearModel();
    ground2->GetCollisionModel()->AddBox(ground_mat, 0.5 * terrainLength, 0.5 * terrainWidth, 0.5 * depth,
                                         ChVector<>(0, 0, -0.5 * depth));

    auto box2 = chrono_types::make_shared<ChBoxShape>();
    box2->GetBoxGeometry().Size = ChVector<>(0.5 * terrainLength, 0.5 * terrainWidth, 0.5 * depth);
    box2->GetBoxGeometry().Pos = ChVector<>(0, 0, -0.5 * depth); 
    ground2->AddAsset(box1);
    ground2->GetCollisionModel()->BuildModel();

    // Control the slip of the vehicle
    auto lockJoint = chrono_types::make_shared<ChLinkLockLock>();
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    if (controlSlip) {
        auto slipRig = std::shared_ptr<ChBody>(vehicle.GetSystem()->NewBody());
        slipRig->SetIdentifier(-3);
        slipRig->SetName("slipRig");
        slipRig->SetPos(initLoc);
        slipRig->SetBodyFixed(false);
        slipRig->SetCollide(false);
        vehicle.GetSystem()->AddBody(slipRig);

        slipRig->GetCollisionModel()->ClearModel();

        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(.2, .1, .1);
        slipRig->AddAsset(box);

        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
        cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -.2);
        cyl->GetCylinderGeometry().rad = .05;
        slipRig->AddAsset(cyl);

        slipRig->GetCollisionModel()->BuildModel();

        auto transJoint = chrono_types::make_shared<ChLinkLockPrismatic>();
        transJoint->SetNameString("_transJoint");
        transJoint->Initialize(slipRig, vehicle.GetChassisBody(), ChCoordsys<>(initLoc, QUNIT));
        vehicle.GetSystem()->AddLink(transJoint);

        lockJoint->SetNameString("_transJoint");
        lockJoint->Initialize(slipRig, ground1, ChCoordsys<>(initLoc, QUNIT));
        vehicle.GetSystem()->AddLink(lockJoint);

        double transSpeed = (1.0 - slip) * sprocketRadius * rotSpeed;
        auto motion = chrono_types::make_shared<ChFunction_Ramp>(0, transSpeed);
        lockJoint->SetMotion_X(motion);

        motor->Initialize(
            vehicle.GetTrackAssembly(chrono::vehicle::RIGHT)->GetSprocket()->GetGearBody(), vehicle.GetChassisBody(),
            ChFrame<>(vehicle.GetTrackAssembly(chrono::vehicle::RIGHT)->GetSprocket()->GetGearBody()->GetPos(),
                      chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X)));
        auto speedFunc = chrono_types::make_shared<ChFunction_Ramp>(0, -rotSpeed);
        motor->SetAngleFunction(speedFunc);
        vehicle.GetSystem()->AddLink(motor);

        motor2->Initialize(
            vehicle.GetTrackAssembly(chrono::vehicle::LEFT)->GetSprocket()->GetGearBody(), vehicle.GetChassisBody(),
            ChFrame<>(vehicle.GetTrackAssembly(chrono::vehicle::LEFT)->GetSprocket()->GetGearBody()->GetPos(),
                      chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X)));
        auto speedFunc2 = chrono_types::make_shared<ChFunction_Ramp>(0, -rotSpeed);
        motor2->SetAngleFunction(speedFunc2);
        vehicle.GetSystem()->AddLink(motor2);
    } else {
        // add joint to keep the vehicle centered
        auto planeJoint = chrono_types::make_shared<ChLinkLockPointPlane>();
        planeJoint->SetNameString("_planeJoint");
        planeJoint->Initialize(ground1, vehicle.GetChassisBody(),
                               ChCoordsys<>(initLoc, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X)));
        vehicle.GetSystem()->AddLink(planeJoint);

        // add joint to keep the vehicle centered
        auto planeJoint2 = chrono_types::make_shared<ChLinkLockPointPlane>();
        planeJoint2->SetNameString("_planeJoint2");
        planeJoint2->Initialize(
            ground1, vehicle.GetChassisBody(),
            ChCoordsys<>(initLoc + ChVector<>(1, 0, 0), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X)));
        vehicle.GetSystem()->AddLink(planeJoint2);
    }

    // -------------------------------------
    // Create the path and the driver system
    // -------------------------------------

    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();

#ifdef CHRONO_IRRLICHT

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp app(&vehicle, L"M113 off-road trafficability");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(-70.f, -150.f, 100.f), irr::core::vector3df(-70.f, -50.f, 100.f), 250,
                         130);
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
    ChStreamOutAsciiFile csv(outputFileStream.str().c_str());
    csv.SetNumFormat("%16.4e");

    // utils::CSV_writer csv("\t");
    // csv.stream().setf(std::ios::scientific | std::ios::showpos);
    // csv.stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

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

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    double time = 0;
    int step_number = 0;
    int render_frame = 0;
    double theta = 0;
    bool performBrakingManeuver = false;
    double brakeStart = 0;
    bool goingForward = true;
    double drawbarPull = 0;
    double torque = 0;
    double torque2 = 0;
    double motionResistance = 0;
    double tractiveEffort = 0;

#ifdef CHRONO_IRRLICHT
    while (app.GetDevice()->run()) {
        time = vehicle.GetChTime();

        // End simulation
        if (numPasses <= 0)
            break;

        if (controlSlip && time > tend)
            break;

        // Forward Mode
        if (vehicle.GetChassis()->GetPos().x() > traversalLength && !performBrakingManeuver) {
            if (!controlSlip) powertrain->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
            if (goingForward) {
                performBrakingManeuver = true;
                goingForward = false;
                brakeStart = time;
            }
            // driver.GetSteeringController().SetLookAheadDistance(-lookAheadDistance);
            // driver.Reset();
        }

        // Reverse mode
        if (vehicle.GetChassis()->GetPos().x() < -traversalLength && !performBrakingManeuver) {
            if (!controlSlip)
                powertrain->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
            if (!goingForward) {
                performBrakingManeuver = true;
                goingForward = true;
                brakeStart = time;
                numPasses--;
                std::cout << "Number of passes to go: " << numPasses << " (Time: " << time << ")" << std::endl;
            }
            // driver.GetSteeringController().SetLookAheadDistance(lookAheadDistance);
            // driver.Reset();
        }

        // Extract system state
        ChVector<> vel_CG = vehicle.GetChassisBody()->GetPos_dt();
        ChVector<> acc_CG = vehicle.GetChassisBody()->GetPos_dt();
        acc_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(acc_CG);
        ChVector<> acc_driver = vehicle.GetVehiclePointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        ChVector<> FrontLeftCornerPos =
            vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(FrontLeftCornerLoc);
        ChVector<> FrontRightCornerPos =
            vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(FrontRightCornerLoc);
        ChVector<> RearLeftCornerPos = vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(RearLeftCornerLoc);
        ChVector<> RearRightCornerPos =
            vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(RearRightCornerLoc);

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);
            }

            render_frame++;
        }

        if (state_output) {
            if (controlSlip)
            {
                drawbarPull = lockJoint->Get_react_force().x();
                torque = motor->GetMotorTorque();
                torque2 = motor2->GetMotorTorque();
            }
            csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
            csv << vehicle.GetVehicleSpeed() << ", ";
            csv << acc_CG.x() << ", " << fwd_acc_CG << ", " << acc_CG.y() << ", " << lat_acc_CG << ", ";
            csv << acc_driver.x() << ", " << fwd_acc_driver << ", " << acc_driver.y() << ", " << lat_acc_driver << ", ";
            csv << acc_CG.z() << ", ";  // vertical acceleration
            csv << vel_CG.x() << ", " << vel_CG.y() << ", " << vel_CG.z() << ", ";
            csv << vehicle.GetChassis()->GetPos().x() << ", " << vehicle.GetChassis()->GetPos().y() << ", "
                << vehicle.GetChassis()->GetPos().z() << ", ";
            csv << 180.0 * theta / CH_C_PI << ", ";                                                    // ground angle
            csv << 180.0 * (theta - vehicle.GetChassisBody()->GetA().Get_A_Rxyz().x()) / CH_C_PI << ", ";  // vehicle roll
            csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().y()) / CH_C_PI << ", ";          // vehicle pitch
            csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().z()) / CH_C_PI << ", ";          // vehicle yaw
            csv << FrontLeftCornerPos.x() << ", " << FrontLeftCornerPos.y() << ", " << FrontLeftCornerPos.z() << ", ";
            csv << FrontRightCornerPos.x() << ", " << FrontRightCornerPos.y() << ", " << FrontRightCornerPos.z()
                << ", ";
            csv << RearLeftCornerPos.x() << ", " << RearLeftCornerPos.y() << ", " << RearLeftCornerPos.z() << ", ";
            csv << RearRightCornerPos.x() << ", " << RearRightCornerPos.y() << ", " << RearRightCornerPos.z() << ", ";
            csv << vehicle.GetDriveline()->GetSprocketSpeed(chrono::vehicle::RIGHT) << ", ";
            csv << vehicle.GetDriveline()->GetSprocketSpeed(chrono::vehicle::LEFT) << ", ";
            csv << drawbarPull << ", " << torque << ", " << torque2 << ", ";
            csv << "\n";
            csv.GetFstream().flush();
        }

        // Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs();
        if (controlSlip) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 0;
        } else {
            driver_inputs.m_throttle = driver.GetThrottle();
            driver_inputs.m_braking = driver.GetBraking();
        }
        driver_inputs.m_steering = driver.GetSteering();
        if (!goingForward)
            driver_inputs.m_steering = 0;  //-steering_input;
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);
        driver_inputs.m_throttle = 1;
        driver_inputs.m_braking = 0;
        driver_inputs.m_steering = 0;

        if (performBrakingManeuver && !controlSlip) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
            driver_inputs.m_steering = 0;

            if (time > brakeStart + 1) {
                performBrakingManeuver = false;
            }
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        terrain->Synchronize(time);
        app.Synchronize("Follower driver", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

#else

    while (numPasses > 0) {
        if (controlSlip && time > tend)
            break;

        // Forward Mode
        if (vehicle.GetChassis()->GetPos().x() > traversalLength && !performBrakingManeuver) {
            if (!controlSlip)
                powertrain->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
            if (goingForward) {
                performBrakingManeuver = true;
                goingForward = false;
                brakeStart = time;
            }
            // driver.GetSteeringController().SetLookAheadDistance(-lookAheadDistance);
            // driver.Reset();
        }

        // Reverse mode
        if (vehicle.GetChassis()->GetPos().x() < -traversalLength && !performBrakingManeuver) {
            if (!controlSlip)
                powertrain->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
            if (!goingForward) {
                performBrakingManeuver = true;
                goingForward = true;
                brakeStart = time;
                numPasses--;
                std::cout << "Number of passes to go: " << numPasses << " (Time: " << time << ")" << std::endl;
            }
            // driver.GetSteeringController().SetLookAheadDistance(lookAheadDistance);
            // driver.Reset();
        }

        // Extract system state
        ChVector<> vel_CG = vehicle.GetChassisBody()->GetPos_dt();
        ChVector<> acc_CG = vehicle.GetChassisBody()->GetPos_dtdt();
        acc_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(acc_CG);
        ChVector<> acc_driver = vehicle.GetVehiclePointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        ChVector<> FrontLeftCornerPos =
            vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(FrontLeftCornerLoc);
        ChVector<> FrontRightCornerPos =
            vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(FrontRightCornerLoc);
        ChVector<> RearLeftCornerPos =
            vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(RearLeftCornerLoc);
        ChVector<> RearRightCornerPos =
            vehicle.GetChassisBody()->GetCoord().TransformPointLocalToParent(RearRightCornerLoc);

        //        if (step_number % output_steps == 0)
        //            std::cout << "Time = " << time << "   % Complete = " << time / tend * 100.0 << std::endl;

        // Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs();
        if (controlSlip) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 0;
        } else {
            driver_inputs.m_throttle = driver.GetThrottle();
            driver_inputs.m_braking = driver.GetBraking();
        }
        driver_inputs.m_steering = driver.GetSteering();
        if (!goingForward)
            driver_inputs.m_steering = 0;  //-steering_input;
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);
        driver_inputs.m_throttle = 1;
        driver_inputs.m_braking = 0;
        driver_inputs.m_steering = 0;

        if (performBrakingManeuver && !controlSlip) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
            driver_inputs.m_steering = 0;

            if (time > brakeStart + 1) {
                performBrakingManeuver = false;
            }
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle.Advance(step_size);

        if (step_number % render_steps == 0) {
            // Output render data
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);
            std::cout << "Output frame:   " << render_frame << std::endl;
            std::cout << "Sim frame:      " << step_number << std::endl;
            std::cout << "Time:           " << time << "   # Passes Left = " << numPasses << std::endl;
            std::cout << "   throttle: " << driver.GetThrottle() << "   steering: " << driver.GetSteering()
                      << "   braking:  " << driver.GetBraking() << std::endl;
            std::cout << "   xPos: " << vehicle.GetChassis()->GetPos().x()
                      << "   yPos: " << vehicle.GetChassis()->GetPos().y()
                      << "   zPos:  " << vehicle.GetChassis()->GetPos().z() << std::endl;
            std::cout << std::endl;

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);
            }

            render_frame++;
        }

        if (state_output) {
            if (controlSlip)
            {
                drawbarPull = lockJoint->Get_react_force().x();
                torque = motor->GetMotorTorque();
                torque2 = motor2->GetMotorTorque();

                // sum up horizontal component of normal contact force acting on each tread (TODO: Currently Chrono hack)
                motionResistance = 0;
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(LEFT); ++ii)
                {
                   //motionResistance += ((SCMDeformableTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(LEFT, ii)->GetShoeBody()).fN.x();
                }
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(RIGHT); ++ii)
                {
                   //motionResistance += ((SCMDeformableTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(RIGHT, ii)->GetShoeBody()).fN.x();
                }

                // sum up horizontal component of friction contact force acting on each tread (TODO: Currently Chrono hack)
                tractiveEffort = 0;
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(LEFT); ++ii)
                {
                   //tractiveEffort += ((SCMDeformableTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(LEFT, ii)->GetShoeBody()).fT.x();
                }
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(RIGHT); ++ii)
                {
                   //tractiveEffort += ((SCMDeformableTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(RIGHT, ii)->GetShoeBody()).fT.x();
                }

            }
            csv << time << driver_inputs.m_steering << ", " << driver_inputs.m_throttle <<", " << driver_inputs.m_braking << ", ";
            csv << vehicle.GetVehicleSpeed() << ", ";
            csv << acc_CG.x() << ", " << fwd_acc_CG << ", " << acc_CG.y() << ", " << lat_acc_CG << ", ";
            csv << acc_driver.x() << ", " << fwd_acc_driver << ", " << acc_driver.y() << ", " << lat_acc_driver << ", ";
            csv << acc_CG.z() << ", ";  // vertical acceleration
            csv << vel_CG.x() << ", " << vel_CG.y() << ", " << vel_CG.z() << ", ";
            csv << vehicle.GetChassis()->GetPos().x() << ", " << vehicle.GetChassis()->GetPos().y() << ", "
                << vehicle.GetChassis()->GetPos().z() << ", ";
            csv << 180.0 * theta / CH_C_PI << ", ";  // ground angle
            csv << 180.0 * (theta - vehicle.GetChassisBody()->GetA().Get_A_Rxyz().x()) / CH_C_PI
                << ", ";                                                                           // vehicle roll
            csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().y()) / CH_C_PI << ", ";  // vehicle pitch
            csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().z()) / CH_C_PI << ", ";  // vehicle yaw
            csv << FrontLeftCornerPos.x() << ", " << FrontLeftCornerPos.y() << ", " << FrontLeftCornerPos.z() << ", ";
            csv << FrontRightCornerPos.x() << ", " << FrontRightCornerPos.y() << ", " << FrontRightCornerPos.z()
                << ", ";
            csv << RearLeftCornerPos.x() << ", " << RearLeftCornerPos.y() << ", " << RearLeftCornerPos.z() << ", ";
            csv << RearRightCornerPos.x() << ", " << RearRightCornerPos.y() << ", " << RearRightCornerPos.z() << ", ";
            csv << vehicle.GetDriveline()->GetSprocketSpeed(chrono::vehicle::RIGHT) << ", ";
            csv << vehicle.GetDriveline()->GetSprocketSpeed(chrono::vehicle::LEFT) << ", ";
            csv << drawbarPull << ", " << torque << ", " << torque2 << ", " << motionResistance << ", " << tractiveEffort << ", ";
            csv << "\n";
            csv.GetFstream().flush();
        }

        // Increment frame number
        time += step_size;
        step_number++;
    }

#endif

    //  if (state_output) {
    //      std::stringstream outputFileStream;
    // outputFileStream << out_dir << "/output_nPasses" << numPasses << "_ChassisWeight" << chassisMass <<
    // "_slipControlled" << controlSlip << "_slip" << slip << "_nodes" << factor << ".dat";
    //      csv.write_to_file(outputFileStream.str());
    //  }

    return 0;
}
