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

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

// Uncomment the following line to unconditionally disable Irrlicht support
//#undef CHRONO_IRRLICHT
#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================

// Initial vehicle position
ChVector3d initLoc(-3, 0, 0.8);

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
ChVector3d FrontLeftCornerLoc(13.5 * 0.0254, 53.0 * 0.0254, 0.0);
ChVector3d FrontRightCornerLoc(13.5 * 0.0254, -53.0 * 0.0254, 0.0);
ChVector3d RearLeftCornerLoc(-178.2 * 0.0254, 53.0 * 0.0254, -5.9 * 0.0254);
ChVector3d RearRightCornerLoc(-178.2 * 0.0254, -53.0 * 0.0254, -5.9 * 0.0254);

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
ChVector3d trackPoint(0.0, 0.0, 0.0);

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
        initLoc = ChVector3d(1, 0, 0.6);
        terrainWidth = 3.0;  // size in Y direction
    }

    std::stringstream outputFileStream;
    outputFileStream << out_dir << "/output_nPasses" << numPasses << "_ChassisWeight" << chassisMass
                     << "_slipControlled" << controlSlip << "_slip" << slip << "_nodes" << factor << ".dat";

    // ---------------
    // Create the M113
    // ---------------

    // Create the vehicle system
    M113 m113;
    m113.SetContactMethod(ChContactMethod::SMC);
    m113.SetChassisFixed(false);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetDrivelineType(DrivelineTypeTV::SIMPLE);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);

    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();
    auto& vehicle = m113.GetVehicle();
    auto engine = vehicle.GetEngine();
    auto transmission = std::static_pointer_cast<ChAutomaticTransmission>(vehicle.GetTransmission());

    // Set visualization type for subsystems
    m113.SetChassisVisualizationType(vis_type);
    m113.SetSprocketVisualizationType(vis_type);
    m113.SetIdlerVisualizationType(vis_type);
    m113.SetSuspensionVisualizationType(vis_type);
    m113.SetRoadWheelVisualizationType(vis_type);
    m113.SetTrackShoeVisualizationType(vis_type);

    if (chassisMass > 0)
        m113.GetChassisBody()->SetMass(chassisMass);

    m113.GetChassisBody()->EnableCollision(true);

    // Add collision geometry to vehicle
    /*
    auto chassis_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    chassis_mat->SetFriction(mu);
    chassis_mat->SetRestitution(restitution);
    chassis_mat->SetYoungModulus(E);
    chassis_mat->SetPoissonRatio(nu);

    // Add flat bottom plate
    m113.GetChassisBody()->GetCollisionModel()->AddBox(chassis_mat, 0.5 * 4.25, 0.5 * 1.65, 0.05,
                                                          ChVector3d(-1.925, 0, -.153 + 0.1));

    auto bottomPlate = chrono_types::make_shared<ChVisualShapeBox>();
    bottomPlate->GetBoxGeometry().Size = ChVector3d(0.5 * 4.25, 0.5 * 1.65, 0.05);
    m113.GetChassisBody()->AddVisualShape(bottomPlate, ChFrame<>(ChVector3d(-1.925, 0, -.153 + 0.05)));

    // Add angled bottom plate
    // ChQuaternion<> plateAngleQuat = chrono::QuatFromAngleAxis(1.0474, VECT_Y);
    ChMatrix33<> plateAngle;
    plateAngle.Set_A_quaternion(chrono::QuatFromAngleAxis(-1.0474, VECT_Y));
    m113.GetChassisBody()->GetCollisionModel()->AddBox(chassis_mat, 0.5 * .3521, 0.5 * 1.65, 0.05,
                                                          ChVector3d(.2457, 0, 0.0245), plateAngle);

    auto angledPlate = chrono_types::make_shared<ChVisualShapeBox>();
    angledPlate->GetBoxGeometry().Size = ChVector3d(0.5 * .3521, 0.5 * 1.65, 0.05);
    m113.GetChassisBody()->AddVisualShape(angledPlate, ChFrame<>(ChVector3d(.2457, 0, 0.0245), plateAngle));
    */

    // Control steering type (enable crossdrive capability).
    m113.GetDriveline()->SetGyrationMode(true);

    // Create and initialize the powertrain system
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->EnableWarmStart(true);
    solver->SetTolerance(1e-10);
    m113.GetSystem()->SetSolver(solver);

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

    auto ground_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    ground_mat->SetFriction(mu);
    ground_mat->SetRestitution(restitution);
    ground_mat->SetYoungModulus(E);
    ground_mat->SetPoissonRatio(nu);

    ChTerrain* terrain;
    if (useDefSoil) {
        auto terrain_D = new SCMTerrain(m113.GetSystem());
        terrain_D->SetPlane(ChCoordsys<>(VNULL, QuatFromAngleX(CH_PI_2)));
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
        terrain_D->SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
        ////terrain_D->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);
        terrain_D->Initialize(terrainLength, terrainWidth, 1.0 / factor);
        terrain = terrain_D;
    } else {
        auto terrain_R = new RigidTerrain(m113.GetSystem());
        auto patch = terrain_R->AddPatch(ground_mat, CSYSNORM, terrainLength, terrainWidth);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
        terrain_R->Initialize();
        terrain = terrain_R;
    }

    // -------------------
    // Create the rigid ground
    // -------------------

    auto ground1 = chrono_types::make_shared<ChBody>();
    ground1->SetName("ground1");
    ground1->SetPos(ChVector3d(-terrainLength, 0, 0));
    ground1->SetFixed(true);
    ground1->EnableCollision(true);
    m113.GetSystem()->AddBody(ground1);

    ground1->GetCollisionModel()->AddBox(ground_mat, terrainLength, terrainWidth, depth,
                                         ChVector3d(0, 0, -0.5 * depth));

    auto box1 = chrono_types::make_shared<ChVisualShapeBox>(terrainLength, terrainWidth, depth);
    ground1->AddVisualShape(box1, ChFrame<>(ChVector3d(0, 0, -0.5 * depth)));

    auto ground2 = chrono_types::make_shared<ChBody>();
    ground2->SetName("ground2");
    ground2->SetPos(ChVector3d(terrainLength, 0, 0));
    ground2->SetFixed(true);
    ground2->EnableCollision(true);
    m113.GetSystem()->AddBody(ground2);

    ground2->GetCollisionModel()->AddBox(ground_mat, terrainLength, terrainWidth, depth,
                                         ChVector3d(0, 0, -0.5 * depth));

    auto box2 = chrono_types::make_shared<ChVisualShapeBox>(terrainLength, terrainWidth, depth);
    ground2->AddVisualShape(box1, ChFrame<>(ChVector3d(0, 0, -0.5 * depth)));

    // Control the slip of the vehicle
    auto lockJoint = chrono_types::make_shared<ChLinkLockLock>();
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    if (controlSlip) {
        auto slipRig = chrono_types::make_shared<ChBody>();
        slipRig->SetName("slipRig");
        slipRig->SetPos(initLoc);
        slipRig->SetFixed(false);
        slipRig->EnableCollision(false);
        m113.GetSystem()->AddBody(slipRig);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(0.4, 0.2, 0.2);
        slipRig->AddVisualShape(box);

        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.05, 0.2);
        slipRig->AddVisualShape(cyl, ChFrame<>(ChVector3d(0, 0, -0.1), QUNIT));

        auto transJoint = chrono_types::make_shared<ChLinkLockPrismatic>();
        transJoint->SetName("_transJoint");
        transJoint->Initialize(slipRig, m113.GetChassisBody(), ChCoordsys<>(initLoc, QUNIT));
        m113.GetSystem()->AddLink(transJoint);

        lockJoint->SetName("_transJoint");
        lockJoint->Initialize(slipRig, ground1, ChCoordsys<>(initLoc, QUNIT));
        m113.GetSystem()->AddLink(lockJoint);

        double transSpeed = (1.0 - slip) * sprocketRadius * rotSpeed;
        auto motion = chrono_types::make_shared<ChFunctionRamp>(0, transSpeed);
        lockJoint->SetMotion_X(motion);

        motor->Initialize(
            vehicle.GetTrackAssembly(chrono::vehicle::RIGHT)->GetSprocket()->GetGearBody(), m113.GetChassisBody(),
            ChFrame<>(vehicle.GetTrackAssembly(chrono::vehicle::RIGHT)->GetSprocket()->GetGearBody()->GetPos(),
                      chrono::QuatFromAngleAxis(CH_PI / 2, VECT_X)));
        auto speedFunc = chrono_types::make_shared<ChFunctionRamp>(0, -rotSpeed);
        motor->SetAngleFunction(speedFunc);
        m113.GetSystem()->AddLink(motor);

        motor2->Initialize(
            vehicle.GetTrackAssembly(chrono::vehicle::LEFT)->GetSprocket()->GetGearBody(), m113.GetChassisBody(),
            ChFrame<>(vehicle.GetTrackAssembly(chrono::vehicle::LEFT)->GetSprocket()->GetGearBody()->GetPos(),
                      chrono::QuatFromAngleAxis(CH_PI / 2, VECT_X)));
        auto speedFunc2 = chrono_types::make_shared<ChFunctionRamp>(0, -rotSpeed);
        motor2->SetAngleFunction(speedFunc2);
        m113.GetSystem()->AddLink(motor2);
    } else {
        // add joint to keep the vehicle centered
        auto planeJoint = chrono_types::make_shared<ChLinkLockPointPlane>();
        planeJoint->SetName("_planeJoint");
        planeJoint->Initialize(ground1, m113.GetChassisBody(),
                               ChCoordsys<>(initLoc, chrono::QuatFromAngleAxis(CH_PI / 2, VECT_X)));
        m113.GetSystem()->AddLink(planeJoint);

        // add joint to keep the vehicle centered
        auto planeJoint2 = chrono_types::make_shared<ChLinkLockPointPlane>();
        planeJoint2->SetName("_planeJoint2");
        planeJoint2->Initialize(
            ground1, m113.GetChassisBody(),
            ChCoordsys<>(initLoc + ChVector3d(1, 0, 0), chrono::QuatFromAngleAxis(CH_PI / 2, VECT_X)));
        m113.GetSystem()->AddLink(planeJoint2);
    }

    // -------------------------------------
    // Create the path and the driver system
    // -------------------------------------

    auto path = ChBezierCurve::Read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();

#ifdef CHRONO_IRRLICHT

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("M113 off-road trafficability");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
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

    // utils::ChWriterCSV csv("\t");
    // csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    // csv.Stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    // Driver location in vehicle local frame
    ChVector3d driver_pos = m113.GetChassis()->GetLocalDriverCoordsys().pos;

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));
    DriverInputs driver_inputs = {0, 0, 0};

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
    while (vis->Run()) {
        time = vehicle.GetChTime();

        // End simulation
        if (numPasses <= 0)
            break;

        if (controlSlip && time > tend)
            break;

        // Forward Mode
        if (m113.GetChassis()->GetPos().x() > traversalLength && !performBrakingManeuver) {
            if (!controlSlip) transmission->SetDriveMode(ChAutomaticTransmission::DriveMode::REVERSE);
            if (goingForward) {
                performBrakingManeuver = true;
                goingForward = false;
                brakeStart = time;
            }
            // driver.GetSteeringController().SetLookAheadDistance(-lookAheadDistance);
            // driver.Reset();
        }

        // Reverse mode
        if (m113.GetChassis()->GetPos().x() < -traversalLength && !performBrakingManeuver) {
            if (!controlSlip)
                transmission->SetDriveMode(ChAutomaticTransmission::DriveMode::FORWARD);
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
        ChVector3d vel_CG = m113.GetChassisBody()->GetPosDt();
        ChVector3d acc_CG = m113.GetChassisBody()->GetPosDt();
        acc_CG = m113.GetChassisBody()->TransformDirectionParentToLocal(acc_CG);
        ChVector3d acc_driver = vehicle.GetPointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        ChVector3d FrontLeftCornerPos =
            m113.GetChassisBody()->TransformPointLocalToParent(FrontLeftCornerLoc);
        ChVector3d FrontRightCornerPos =
            m113.GetChassisBody()->TransformPointLocalToParent(FrontRightCornerLoc);
        ChVector3d RearLeftCornerPos = m113.GetChassisBody()->TransformPointLocalToParent(RearLeftCornerLoc);
        ChVector3d RearRightCornerPos =
            m113.GetChassisBody()->TransformPointLocalToParent(RearRightCornerLoc);

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector3d& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector3d& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            }

            render_frame++;
        }

        if (state_output) {
            if (controlSlip)
            {
                drawbarPull = lockJoint->GetReaction2().force.x();
                torque = motor->GetMotorTorque();
                torque2 = motor2->GetMotorTorque();
            }
            csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
            csv << vehicle.GetSpeed() << ", ";
            csv << acc_CG.x() << ", " << fwd_acc_CG << ", " << acc_CG.y() << ", " << lat_acc_CG << ", ";
            csv << acc_driver.x() << ", " << fwd_acc_driver << ", " << acc_driver.y() << ", " << lat_acc_driver << ", ";
            csv << acc_CG.z() << ", ";  // vertical acceleration
            csv << vel_CG.x() << ", " << vel_CG.y() << ", " << vel_CG.z() << ", ";
            csv << m113.GetChassis()->GetPos().x() << ", " << m113.GetChassis()->GetPos().y() << ", "
                << m113.GetChassis()->GetPos().z() << ", ";
            csv << 180.0 * theta / CH_PI << ", ";                                                    // ground angle
            csv << 180.0 * (theta - m113.GetChassisBody()->GetRotMat().Get_A_Rxyz().x()) / CH_PI << ", ";  // vehicle roll
            csv << 180.0 * (m113.GetChassisBody()->GetRotMat().Get_A_Rxyz().y()) / CH_PI << ", ";          // vehicle pitch
            csv << 180.0 * (m113.GetChassisBody()->GetRotMat().Get_A_Rxyz().z()) / CH_PI << ", ";          // vehicle yaw
            csv << FrontLeftCornerPos.x() << ", " << FrontLeftCornerPos.y() << ", " << FrontLeftCornerPos.z() << ", ";
            csv << FrontRightCornerPos.x() << ", " << FrontRightCornerPos.y() << ", " << FrontRightCornerPos.z()
                << ", ";
            csv << RearLeftCornerPos.x() << ", " << RearLeftCornerPos.y() << ", " << RearLeftCornerPos.z() << ", ";
            csv << RearRightCornerPos.x() << ", " << RearRightCornerPos.y() << ", " << RearRightCornerPos.z() << ", ";
            csv << m113.GetDriveline()->GetSprocketSpeed(chrono::vehicle::RIGHT) << ", ";
            csv << m113.GetDriveline()->GetSprocketSpeed(chrono::vehicle::LEFT) << ", ";
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
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

#else

    while (numPasses > 0) {
        if (controlSlip && time > tend)
            break;

        // Forward Mode
        if (m113.GetChassis()->GetPos().x() > traversalLength && !performBrakingManeuver) {
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
        if (m113.GetChassis()->GetPos().x() < -traversalLength && !performBrakingManeuver) {
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
        ChVector3d vel_CG = m113.GetChassisBody()->GetPosDt();
        ChVector3d acc_CG = m113.GetChassisBody()->GetPosDt2();
        acc_CG = m113.GetChassisBody()->TransformDirectionParentToLocal(acc_CG);
        ChVector3d acc_driver = vehicle.GetPointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        ChVector3d FrontLeftCornerPos = m113.GetChassisBody()->TransformPointLocalToParent(FrontLeftCornerLoc);
        ChVector3d FrontRightCornerPos = m113.GetChassisBody()->TransformPointLocalToParent(FrontRightCornerLoc);
        ChVector3d RearLeftCornerPos = m113.GetChassisBody()->TransformPointLocalToParent(RearLeftCornerLoc);
        ChVector3d RearRightCornerPos = m113.GetChassisBody()->TransformPointLocalToParent(RearRightCornerLoc);

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
            utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            std::cout << "Output frame:   " << render_frame << std::endl;
            std::cout << "Sim frame:      " << step_number << std::endl;
            std::cout << "Time:           " << time << "   # Passes Left = " << numPasses << std::endl;
            std::cout << "   throttle: " << driver.GetThrottle() << "   steering: " << driver.GetSteering()
                      << "   braking:  " << driver.GetBraking() << std::endl;
            std::cout << "   xPos: " << m113.GetChassis()->GetPos().x()
                      << "   yPos: " << m113.GetChassis()->GetPos().y()
                      << "   zPos:  " << m113.GetChassis()->GetPos().z() << std::endl;
            std::cout << std::endl;

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            }

            render_frame++;
        }

        if (state_output) {
            if (controlSlip)
            {
                drawbarPull = lockJoint->GetReaction2().force.x();
                torque = motor->GetMotorTorque();
                torque2 = motor2->GetMotorTorque();

                // sum up horizontal component of normal contact force acting on each tread (TODO: Currently Chrono hack)
                motionResistance = 0;
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(LEFT); ++ii)
                {
                   //motionResistance += ((SCMTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(LEFT, ii)->GetShoeBody()).fN.x();
                }
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(RIGHT); ++ii)
                {
                   //motionResistance += ((SCMTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(RIGHT, ii)->GetShoeBody()).fN.x();
                }

                // sum up horizontal component of friction contact force acting on each tread (TODO: Currently Chrono hack)
                tractiveEffort = 0;
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(LEFT); ++ii)
                {
                   //tractiveEffort += ((SCMTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(LEFT, ii)->GetShoeBody()).fT.x();
                }
                for (int ii = 0; ii < vehicle.GetNumTrackShoes(RIGHT); ++ii)
                {
                   //tractiveEffort += ((SCMTerrain*)terrain)->GetContactForce(vehicle.GetTrackShoe(RIGHT, ii)->GetShoeBody()).fT.x();
                }

            }
            csv << time << driver_inputs.m_steering << ", " << driver_inputs.m_throttle <<", " << driver_inputs.m_braking << ", ";
            csv << vehicle.GetSpeed() << ", ";
            csv << acc_CG.x() << ", " << fwd_acc_CG << ", " << acc_CG.y() << ", " << lat_acc_CG << ", ";
            csv << acc_driver.x() << ", " << fwd_acc_driver << ", " << acc_driver.y() << ", " << lat_acc_driver << ", ";
            csv << acc_CG.z() << ", ";  // vertical acceleration
            csv << vel_CG.x() << ", " << vel_CG.y() << ", " << vel_CG.z() << ", ";
            csv << m113.GetChassis()->GetPos().x() << ", " << m113.GetChassis()->GetPos().y() << ", "
                << m113.GetChassis()->GetPos().z() << ", ";
            csv << 180.0 * theta / CH_PI << ", ";  // ground angle
            csv << 180.0 * (theta - m113.GetChassisBody()->GetRotMat().Get_A_Rxyz().x()) / CH_PI
                << ", ";                                                                           // vehicle roll
            csv << 180.0 * (m113.GetChassisBody()->GetRotMat().Get_A_Rxyz().y()) / CH_PI << ", ";  // vehicle pitch
            csv << 180.0 * (m113.GetChassisBody()->GetRotMat().Get_A_Rxyz().z()) / CH_PI << ", ";  // vehicle yaw
            csv << FrontLeftCornerPos.x() << ", " << FrontLeftCornerPos.y() << ", " << FrontLeftCornerPos.z() << ", ";
            csv << FrontRightCornerPos.x() << ", " << FrontRightCornerPos.y() << ", " << FrontRightCornerPos.z()
                << ", ";
            csv << RearLeftCornerPos.x() << ", " << RearLeftCornerPos.y() << ", " << RearLeftCornerPos.z() << ", ";
            csv << RearRightCornerPos.x() << ", " << RearRightCornerPos.y() << ", " << RearRightCornerPos.z() << ", ";
            csv << m113.GetDriveline()->GetSprocketSpeed(chrono::vehicle::RIGHT) << ", ";
            csv << m113.GetDriveline()->GetSprocketSpeed(chrono::vehicle::LEFT) << ", ";
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
    //      csv.WriteToFile(outputFileStream.str());
    //  }

    return 0;
}
