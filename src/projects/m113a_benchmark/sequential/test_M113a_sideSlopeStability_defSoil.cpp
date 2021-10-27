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
// Test program for M113 side slope stability test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChStream.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"

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
ChVector<> initLoc(-50, 0, 0.75);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
// double target_speed = 15.6464;  // 35 mph
// double target_speed = 11.176;  // 25 mph
// double target_speed = 6.7056;  // 15 mph
// double target_speed = 5;
double target_speed = 2.2352;  // 5 mph

// Grade of side slope
double grade = -20;      // %
double gravity = -9.81;  // gravitational acceleration, m/s
bool useDefSoil = true;

// Chassis Corner Point Locations
ChVector<> FrontLeftCornerLoc(13.5 * 0.0254, 53.0 * 0.0254, 0.0);
ChVector<> FrontRightCornerLoc(13.5 * 0.0254, -53.0 * 0.0254, 0.0);
ChVector<> RearLeftCornerLoc(-178.2 * 0.0254, 53.0 * 0.0254, -5.9 * 0.0254);
ChVector<> RearRightCornerLoc(-178.2 * 0.0254, -53.0 * 0.0254, -5.9 * 0.0254);

// Input file names for the path-follower driver model
std::string path_file("M113a_benchmark/paths/obstacleAvoidance_3m.txt");
std::string steering_controller_file("M113a_benchmark/SteeringController_M113_sideSlopeStability.json");
std::string speed_controller_file("M113a_benchmark/SpeedController.json");

// Rigid terrain dimensions
double terrainLength = 50.0;  // size in X direction (TODO: This should be 30, per the NRMM requirements)
double terrainWidth = 15.0;   // size in Y direction
double rigidLength = 50;      // size of rigid portion in X direction

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 100;  // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Simulation length (set to a negative value to disable for Irrlicht)
double tend = 40;

// Output directories (Povray only)
std::string out_dir = "../M113_SIDESLOPE_DEFSOIL";
std::string pov_dir = out_dir + "/POVRAY";

// Visualization type for all vehicle subsystems (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::MESH;

// POV-Ray output
bool povray_output = true;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = true;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    if (argc > 1) {
        target_speed = atof(argv[1]);

        std::stringstream dataFolderStream;
        dataFolderStream << "../M113_SIDESLOPE_DEFSOIL_speed" << target_speed << "/";
        out_dir = dataFolderStream.str();
        pov_dir = out_dir + "/POVRAY";
    }

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

    // Control steering type (enable crossdrive capability).
    vehicle.GetDriveline()->SetGyrationMode(true);

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<M113a_SimplePowertrain>("Powertrain");
    vehicle.InitializePowertrain(powertrain);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->EnableWarmStart(true);
    solver->SetTolerance(1e-10);
    vehicle.GetSystem()->SetSolver(solver);

    double alpha = atan(grade / 100.0);
    vehicle.GetSystem()->Set_G_acc(ChVector<>(0, gravity * sin(alpha), gravity * cos(alpha)));

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

    double depth = 10;
    double factor = 10;

    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ground_mat->SetFriction(0.8f);
    ground_mat->SetRestitution(0.01f);
    ground_mat->SetYoungModulus(2e7f);
    ground_mat->SetPoissonRatio(0.3f);

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
    ground1->SetPos(ChVector<>(-0.5 * (terrainLength + rigidLength), 0, 0));
    ground1->SetBodyFixed(true);
    ground1->SetCollide(true);
    vehicle.GetSystem()->AddBody(ground1);

    ground1->GetCollisionModel()->ClearModel();
    ground1->GetCollisionModel()->AddBox(ground_mat, 0.5 * rigidLength, 0.5 * terrainWidth, 0.5 * depth,
                                         ChVector<>(0, 0, -0.5 * depth));

    auto box1 = chrono_types::make_shared<ChBoxShape>();
    box1->GetBoxGeometry().Size = ChVector<>(0.5 * rigidLength, 0.5 * terrainWidth, 0.5 * depth);
    box1->GetBoxGeometry().Pos = ChVector<>(0, 0, -0.5 * depth);
    ground1->AddAsset(box1);
    ground1->GetCollisionModel()->BuildModel();

    auto ground2 = std::shared_ptr<ChBody>(vehicle.GetSystem()->NewBody());
    ground2->SetIdentifier(-1);
    ground2->SetName("ground1");
    ground2->SetPos(ChVector<>(0.5 * (terrainLength + rigidLength), 0, 0));
    ground2->SetBodyFixed(true);
    ground2->SetCollide(true);
    vehicle.GetSystem()->AddBody(ground2);

    ground2->GetCollisionModel()->ClearModel();
    ground2->GetCollisionModel()->AddBox(ground_mat, 0.5 * rigidLength, 0.5 * terrainWidth, 0.5 * depth,
                                         ChVector<>(0, 0, -0.5 * depth));

    auto box2 = chrono_types::make_shared<ChBoxShape>();
    box2->GetBoxGeometry().Size = ChVector<>(0.5 * rigidLength, 0.5 * terrainWidth, 0.5 * depth);
    box2->GetBoxGeometry().Pos = ChVector<>(0, 0, -0.5 * depth);
    ground2->AddAsset(box1);
    ground2->GetCollisionModel()->BuildModel();

    // -------------------
    // Create the obstacle
    // -------------------

    auto obstacle = std::shared_ptr<ChBody>(vehicle.GetSystem()->NewBody());
    obstacle->SetIdentifier(-1);
    obstacle->SetName("obstacle");
    obstacle->SetPos(ChVector<>(0, 0, 0));
    obstacle->SetBodyFixed(true);
    obstacle->SetCollide(false);
    vehicle.GetSystem()->AddBody(obstacle);

    obstacle->GetCollisionModel()->ClearModel();
    obstacle->GetCollisionModel()->AddSphere(ground_mat, 3.0);
    auto ball = chrono_types::make_shared<ChSphereShape>();
    ball->GetSphereGeometry().rad = 3.0;
    obstacle->AddAsset(ball);
    obstacle->GetCollisionModel()->BuildModel();

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

    ChTrackedVehicleIrrApp app(&vehicle, L"M113 side slope stability");
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

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

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

#ifdef CHRONO_IRRLICHT

    while (app.GetDevice()->run()) {
        time = vehicle.GetChTime();

        // End simulation
        if ((time > tend) && (tend > 0))
            break;

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

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

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

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%05d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);

                if (useDefSoil) {
                    std::vector<ChVector<> >& vertices =
                        ((SCMDeformableTerrain*)terrain)->GetMesh()->GetMesh()->getCoordsVertices();
                    std::vector<ChVector<int> >& idx_vertices =
                        ((SCMDeformableTerrain*)terrain)->GetMesh()->GetMesh()->getIndicesVertexes();
                    std::vector<ChVector<float> >& colors =
                        ((SCMDeformableTerrain*)terrain)->GetMesh()->GetMesh()->getCoordsColors();

                    std::string delim = ",";
                    utils::CSV_writer csv(delim);
                    csv << idx_vertices.size() << std::endl;
                    for (int i = 0; i < idx_vertices.size(); i++) {
                        ChVector<> pos = (vertices[idx_vertices[i].x()] + vertices[idx_vertices[i].y()] +
                                          vertices[idx_vertices[i].z()]) /
                                         3.0;
                        csv << pos << vertices[idx_vertices[i].x()] << vertices[idx_vertices[i].y()]
                            << vertices[idx_vertices[i].z()] << (double)colors[idx_vertices[i].x()].x()
                            << (double)colors[idx_vertices[i].x()].y() << (double)colors[idx_vertices[i].x()].z()
                            << (double)colors[idx_vertices[i].y()].x() << (double)colors[idx_vertices[i].y()].y()
                            << (double)colors[idx_vertices[i].y()].z() << (double)colors[idx_vertices[i].z()].x()
                            << (double)colors[idx_vertices[i].z()].y() << (double)colors[idx_vertices[i].z()].z()
                            << std::endl;
                    }

                    sprintf(filename, "%s/terrain_%05d.dat", pov_dir.c_str(), render_frame + 1);
                    csv.write_to_file(filename);
                }
            }

            if (state_output) {
                csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
                csv << vehicle.GetVehicleSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << acc_CG.z();  // vertical acceleration
                csv << vel_CG.x() << vel_CG.y() << vel_CG.z();
                csv << vehicle.GetChassis()->GetPos().x() << vehicle.GetChassis()->GetPos().y()
                    << vehicle.GetChassis()->GetPos().z();
                csv << 180.0 * theta / CH_C_PI;                                                    // ground angle
                csv << 180.0 * (theta - vehicle.GetChassisBody()->GetA().Get_A_Rxyz().x()) / CH_C_PI;  // vehicle roll
                csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().y()) / CH_C_PI;          // vehicle pitch
                csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().z()) / CH_C_PI;          // vehicle yaw
                csv << FrontLeftCornerPos.x() << FrontLeftCornerPos.y() << FrontLeftCornerPos.z();
                csv << FrontRightCornerPos.x() << FrontRightCornerPos.y() << FrontRightCornerPos.z();
                csv << RearLeftCornerPos.x() << RearLeftCornerPos.y() << RearLeftCornerPos.z();
                csv << RearRightCornerPos.x() << RearRightCornerPos.y() << RearRightCornerPos.z();
                csv << std::endl;
            }

            render_frame++;
        }

        // Increment frame number
        step_number++;
    }

#else

    while (time <= tend) {
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

        if (step_number % render_steps == 0) {
            // Output render data
            std::cout << "Output frame:   " << render_frame << std::endl;
            std::cout << "Sim frame:      " << step_number << std::endl;
            std::cout << "Time:           " << time << "   % Complete = " << time / tend * 100.0 << std::endl;
            std::cout << "   throttle: " << driver.GetThrottle() << "   steering: " << driver.GetSteering()
                      << "   braking:  " << driver.GetBraking() << std::endl;
            std::cout << std::endl;

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%05d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);

                if (useDefSoil) {
                    std::vector<ChVector<> >& vertices =
                        ((SCMDeformableTerrain*)terrain)->GetMesh()->GetMesh()->getCoordsVertices();
                    std::vector<ChVector<int> >& idx_vertices =
                        ((SCMDeformableTerrain*)terrain)->GetMesh()->GetMesh()->getIndicesVertexes();
                    std::vector<ChVector<float> >& colors =
                        ((SCMDeformableTerrain*)terrain)->GetMesh()->GetMesh()->getCoordsColors();

                    std::string delim = ",";
                    utils::CSV_writer csv(delim);
                    csv << idx_vertices.size() << std::endl;
                    for (int i = 0; i < idx_vertices.size(); i++) {
                        ChVector<> pos = (vertices[idx_vertices[i].x()] + vertices[idx_vertices[i].y()] +
                                          vertices[idx_vertices[i].z()]) /
                                         3.0;
                        csv << pos << vertices[idx_vertices[i].x()] << vertices[idx_vertices[i].y()]
                            << vertices[idx_vertices[i].z()] << (double)colors[idx_vertices[i].x()].x()
                            << (double)colors[idx_vertices[i].x()].y() << (double)colors[idx_vertices[i].x()].z()
                            << (double)colors[idx_vertices[i].y()].x() << (double)colors[idx_vertices[i].y()].y()
                            << (double)colors[idx_vertices[i].y()].z() << (double)colors[idx_vertices[i].z()].x()
                            << (double)colors[idx_vertices[i].z()].y() << (double)colors[idx_vertices[i].z()].z()
                            << std::endl;
                    }
                    sprintf(filename, "%s/terrain_%05d.dat", pov_dir.c_str(), render_frame + 1);
                    csv.write_to_file(filename);
                }
            }

            if (state_output) {
                csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
                csv << vehicle.GetVehicleSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << acc_CG.z();  // vertical acceleration
                csv << vel_CG.x() << vel_CG.y() << vel_CG.z();
                csv << vehicle.GetChassisBody()->GetPos().x() << vehicle.GetChassisBody()->GetPos().y()
                    << vehicle.GetChassisBody()->GetPos().z();
                csv << 180.0 * theta / CH_C_PI;                                                        // ground angle
                csv << 180.0 * (theta - vehicle.GetChassisBody()->GetA().Get_A_Rxyz().x()) / CH_C_PI;  // vehicle roll
                csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().y()) / CH_C_PI;          // vehicle pitch
                csv << 180.0 * (vehicle.GetChassisBody()->GetA().Get_A_Rxyz().z()) / CH_C_PI;          // vehicle yaw
                csv << FrontLeftCornerPos.x() << FrontLeftCornerPos.y() << FrontLeftCornerPos.z();
                csv << FrontRightCornerPos.x() << FrontRightCornerPos.y() << FrontRightCornerPos.z();
                csv << RearLeftCornerPos.x() << RearLeftCornerPos.y() << RearLeftCornerPos.z();
                csv << RearRightCornerPos.x() << RearRightCornerPos.y() << RearRightCornerPos.z();
                csv << std::endl;
            }

            render_frame++;
        }

        // Increment frame number
        time += step_size;
        step_number++;
    }

#endif

    if (state_output) {
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
