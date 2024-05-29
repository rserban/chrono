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
// Test program for M113 grade climbing test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"

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
ChVector3d initLoc(-50, 0, 10.7);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
// double target_speed = 15.6464;  // 35 mph
// double target_speed = 11.176;  // 25 mph
// double target_speed = 6.7056;  // 15 mph
double target_speed = 4;
// double target_speed = 2.2352;  // 5 mph

// Grade of side slope
double slope = -6;       // % slope (positive for uphill, negative for downhill)
double gravity = -9.81;  // gravitational acceleration, m/s
bool useDefSoil = true;

// Chassis Corner Point Locations
ChVector3d FrontLeftCornerLoc(13.5 * 0.0254, 53.0 * 0.0254, 0.0);
ChVector3d FrontRightCornerLoc(13.5 * 0.0254, -53.0 * 0.0254, 0.0);
ChVector3d RearLeftCornerLoc(-178.2 * 0.0254, 53.0 * 0.0254, -5.9 * 0.0254);
ChVector3d RearRightCornerLoc(-178.2 * 0.0254, -53.0 * 0.0254, -5.9 * 0.0254);

// Input file names for the path-follower driver model
std::string path_file("M113a_benchmark/paths/obstacleAvoidance_3m.txt");
std::string steering_controller_file("M113a_benchmark/SteeringController_M113_sideSlopeStability.json");
std::string speed_controller_file("M113a_benchmark/SpeedController.json");

// Rigid terrain dimensions
double terrainLength = 60.0;  // size in X direction (TODO: This should be 30, per the NRMM requirements)
double terrainWidth = 15.0;   // size in Y direction
double rigidLength = 30.0;    // size of rigid portion in X direction

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 100;  // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector3d trackPoint(0.0, 0.0, 0.0);

// Simulation length (set to a negative value to disable for Irrlicht)
double tend = 50;

// Output directories (Povray only)
std::string out_dir = "../M113_GRADECLIMBING_DEFSOIL";
std::string pov_dir = out_dir + "/POVRAY";

// Visualization type for all vehicle subsystems (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::PRIMITIVES;

// POV-Ray output
bool povray_output = true;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = true;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    if (argc > 1) {
        slope = atof(argv[1]);
        target_speed = atof(argv[2]);

        std::stringstream dataFolderStream;
        dataFolderStream << "../M113_GRADECLIMBING_DEFSOIL_slope" << slope << "_speed" << target_speed << "/";
        out_dir = dataFolderStream.str();
        pov_dir = out_dir + "/POVRAY";
    }

    // ---------------
    // Create the M113
    // ---------------

    // Create the vehicle system
    double alpha = atan(slope / 100.0);
    initLoc = ChVector3d(-0.5 * (terrainLength + rigidLength * cos(alpha)), 0, 1 + 0.5 * rigidLength * sin(alpha));
    initRot = chrono::QuatFromAngleAxis(alpha, VECT_Y);

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

    // Set visualization type for subsystems
    m113.SetChassisVisualizationType(vis_type);
    m113.SetSprocketVisualizationType(vis_type);
    m113.SetIdlerVisualizationType(vis_type);
    m113.SetSuspensionVisualizationType(vis_type);
    m113.SetRoadWheelVisualizationType(vis_type);
    m113.SetTrackShoeVisualizationType(vis_type);

    // Control steering type (enable crossdrive capability).
    m113.GetDriveline()->SetGyrationMode(true);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->EnableWarmStart(true);
    solver->SetTolerance(1e-10);
    m113.GetSystem()->SetSolver(solver);

    m113.GetSystem()->SetGravitationalAcceleration(ChVector3d(gravity * sin(alpha), 0, gravity * cos(alpha)));

    // ------------------
    // Create the terrain
    // ------------------

    // Deformable terrain properties (LETE sand)
    double Kphi = 5301e3;
    double Kc = 102e3;
    double n = 0.793;
    double c = 1.3e3;
    double phi = 31.1;
    double K = 1.2e-2;
    double E_elastic = 2e8;
    double damping = 3e4;

    // Rigid terrain (SMC material)
    double depth = 10;
    double factor = 10;

    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    material->SetFriction(0.8f);
    material->SetRestitution(0.01f);
    material->SetYoungModulus(2e7f);
    material->SetPoissonRatio(0.3f);

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
        auto patch = terrain_R->AddPatch(material, CSYSNORM, terrainLength, terrainWidth);
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
    ground1->SetPos(ChVector3d(-0.5 * (terrainLength + rigidLength * cos(alpha)), 0, 0.5 * rigidLength * sin(alpha)));
    ground1->SetRot(chrono::QuatFromAngleAxis(alpha, VECT_Y));
    ground1->SetFixed(true);
    ground1->EnableCollision(true);
    m113.GetSystem()->AddBody(ground1);

    ground1->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeBox>(material, rigidLength, terrainWidth, depth),
        ChVector3d(0, 0, -0.5 * depth));

    auto box1 = chrono_types::make_shared<ChVisualShapeBox>(rigidLength, terrainWidth, depth);
    ground1->AddVisualShape(box1, ChFrame<>(ChVector3d(0, 0, -0.5 * depth)));

    auto ground2 = chrono_types::make_shared<ChBody>();
    ground2->SetName("ground1");
    ground2->SetPos(ChVector3d(0.5 * (terrainLength + rigidLength), 0, 0));
    ground2->SetFixed(true);
    ground2->EnableCollision(true);
    m113.GetSystem()->AddBody(ground2);

    ground2->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeBox>(material, rigidLength, terrainWidth, depth),
        ChVector3d(0, 0, -0.5 * depth));

    auto box2 = chrono_types::make_shared<ChVisualShapeBox>(rigidLength, terrainWidth, depth);
    ground2->AddVisualShape(box1, ChFrame<>(ChVector3d(0, 0, -0.5 * depth)));

    // -------------------
    // Create the obstacle
    // -------------------

    auto obstacle = chrono_types::make_shared<ChBody>();
    obstacle->SetName("obstacle");
    obstacle->SetPos(ChVector3d(0, 0, 0));
    obstacle->SetFixed(true);
    obstacle->EnableCollision(false);
    m113.GetSystem()->AddBody(obstacle);

    obstacle->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeSphere>(material, 3.0));
    auto ball = chrono_types::make_shared<ChVisualShapeSphere>(3.0);
    obstacle->AddVisualShape(ball);

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
    vis->SetWindowTitle("M113 grade climbing");
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

    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

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

#ifdef CHRONO_IRRLICHT

    while (vis->Run()) {
        time = vehicle.GetChTime();

        // End simulation
        if ((time > tend) && (tend > 0) || m113.GetChassisBody()->GetPos().x() > 0.5 * terrainLength + rigidLength)
            break;

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
                sprintf(filename, "%s/data_%05d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            }

            if (state_output) {
                auto chassis_angles = m113.GetChassisBody()->GetRotMat().GetCardanAnglesXYZ();
                csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
                csv << vehicle.GetSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << acc_CG.z();  // vertical acceleration
                csv << vel_CG;
                csv << m113.GetChassis()->GetPos();
                csv << CH_RAD_TO_DEG * theta;                         // ground angle
                csv << CH_RAD_TO_DEG * (theta - chassis_angles.x());  // vehicle roll
                csv << CH_RAD_TO_DEG * chassis_angles.y();            // vehicle pitch
                csv << CH_RAD_TO_DEG * chassis_angles.z();            // vehicle yaw
                csv << FrontLeftCornerPos << FrontRightCornerPos << RearLeftCornerPos << RearRightCornerPos;
                csv << std::endl;
            }

            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

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

    while (time <= tend) {
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

        if (step_number % render_steps == 0) {
            // Output render data
            // char filename[100];
            // sprintf(filename, "%s/data_%05d.dat", pov_dir.c_str(), render_frame + 1);
            // utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            std::cout << "Output frame:   " << render_frame << std::endl;
            std::cout << "Sim frame:      " << step_number << std::endl;
            std::cout << "Time:           " << time << "   % Complete = " << time / tend * 100.0 << std::endl;
            std::cout << "   throttle: " << driver.GetThrottle() << "   steering: " << driver.GetSteering()
                      << "   braking:  " << driver.GetBraking() << std::endl;
            std::cout << std::endl;

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%05d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            }

            if (state_output) {
                auto chassis_angles = m113.GetChassisBody()->GetRotMat().GetCardanAnglesXYZ();
                csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
                csv << vehicle.GetSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << acc_CG.z();  // vertical acceleration
                csv << vel_CG;
                csv << m113.GetChassis()->GetPos();
                csv << CH_RAD_TO_DEG * theta;                         // ground angle
                csv << CH_RAD_TO_DEG * (theta - chassis_angles.x());  // vehicle roll
                csv << CH_RAD_TO_DEG * chassis_angles.y();            // vehicle pitch
                csv << CH_RAD_TO_DEG * chassis_angles.z();            // vehicle yaw
                csv << FrontLeftCornerPos << FrontRightCornerPos << RearLeftCornerPos << RearRightCornerPos;
                csv << std::endl;
            }

            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle.Advance(step_size);

        // Increment frame number
        time += step_size;
        step_number++;
    }

#endif

    if (state_output) {
        std::stringstream outputFileStream;
        outputFileStream << out_dir << "/output_slope" << slope << "_speed" << target_speed << ".dat";
        csv.WriteToFile(outputFileStream.str());
    }

    return 0;
}
