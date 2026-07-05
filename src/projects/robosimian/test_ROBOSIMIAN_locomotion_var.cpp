// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Robosimian model (URDF)
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_parsers/urdf/ChParserURDF.h"

#include "chrono_models/robot/ChRobotActuation.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#ifdef CHRONO_FSI_SPH
    #include "chrono_vehicle/terrain/CRMTerrain.h"
#endif

#include "chrono_vsg/ChVisualSystemVSG.h"
#ifdef CHRONO_FSI_SPH
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::vsg3d;
#ifdef CHRONO_FSI_SPH
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
#endif

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// RoboSimian locomotion mode
enum class LocomotionMode { WALK, SCULL, INCHWORM, DRIVE };
LocomotionMode locomotion_mode = LocomotionMode::INCHWORM;

// Terrain type
enum class TerrainType { RIGID, SCM, CRM };
TerrainType terrain_type = TerrainType::RIGID;

// -----------------------------------------------------------------------------

std::string LocomotionModeString(LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::WALK:
            return "Walk";
        case LocomotionMode::SCULL:
            return "Scull";
        case LocomotionMode::INCHWORM:
            return "Inchworm";
        case LocomotionMode::DRIVE:
            return "Drive";
    }
    return "";
}

std::string TerrainTypeString(TerrainType type) {
    switch (type) {
        case TerrainType::RIGID:
            return "Rigid";
        case TerrainType::SCM:
            return "SCM";
        case TerrainType::CRM:
            return "CRM";
    }
    return "";
}

// -----------------------------------------------------------------------------

static std::shared_ptr<vehicle::RigidTerrain> CreateTerrainRigid(ChParserURDF& robot, double length, double width, double height, double offset) {
    ChContactMaterialData mat;
    mat.mu = 0.8f;
    mat.cr = 0.0f;
    mat.Y = 1e7f;
    auto cmat = mat.CreateMaterial(robot.GetChSystem().GetContactMethod());

    auto terrain = chrono_types::make_shared<vehicle::RigidTerrain>(&robot.GetChSystem());
    auto patch = terrain->AddPatch(cmat, ChCoordsysd(ChVector3d(offset, 0, height - 0.1), QUNIT), length, width, 0.2);
    patch->SetTexture(GetChronoDataFile("textures/checker2.png"), 4, 1);

    terrain->Initialize();

    return terrain;
}

static std::shared_ptr<vehicle::SCMTerrain> CreateTerrainSCM(ChParserURDF& robot, double length, double width, double height, double offset) {
    // Deformable terrain properties (LETE sand)
    ////double Kphi = 5301e3;    // Bekker Kphi
    ////double Kc = 102e3;       // Bekker Kc
    ////double n = 0.793;        // Bekker n exponent
    ////double coh = 1.3e3;      // Mohr cohesive limit (Pa)
    ////double phi = 31.1;       // Mohr friction limit (degrees)
    ////double K = 1.2e-2;       // Janosi shear coefficient (m)
    ////double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    ////double damping = 3e4;    // Damping coefficient (Pa*s/m)

    // SCM terrain spacing
    double spacing = 1.0 / 64;

    // Deformable terrain properties (CDT FGS dry - 6/29/2018)
    double Kphi = 6259.1e3;  // Bekker Kphi
    double Kc = 5085.6e3;    // Bekker Kc
    double n = 1.42;         // Bekker n exponent
    double coh = 1.58e3;     // Mohr cohesive limit (Pa)
    double phi = 34.1;       // Mohr friction limit (degrees)
    double K = 22.17e-3;     // Janosi shear coefficient (m)
    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&robot.GetChSystem());
    terrain->SetReferenceFrame(ChCoordsys<>(ChVector3d(length / 2 - offset, 0, height), QUNIT));
    terrain->SetSoilParameters(Kphi, Kc, n, coh, phi, K, E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);

    // Enable active domain feature
    auto torso = robot.GetChBody("torso");
    terrain->AddActiveDomain(torso, ChVector3d(0, 0, 0), ChVector3d(3.0, 2.0, 1.0));

    terrain->Initialize(length, width, spacing);

    return terrain;
}

#ifdef CHRONO_FSI_SPH

static std::shared_ptr<vehicle::CRMTerrain> CreateTerrainCRM(ChParserURDF& robot, double length, double width, double height, double offset) {
    bool verbose = true;
    double step_size = 1e-4;

    // CRM terrain spacing
    double spacing = 0.02;

    // CRM material properties
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    // SPH integration scheme
    IntegrationScheme integration_scheme = IntegrationScheme::RK2;

    auto terrain = chrono_types::make_shared<vehicle::CRMTerrain>(robot.GetChSystem(), spacing);
    terrain->SetVerbose(verbose);
    terrain->SetGravitationalAcceleration(robot.GetChSystem().GetGravitationalAcceleration());
    terrain->SetStepSizeCFD(step_size);

    // Set SPH parameters and soil material properties
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;
    terrain->SetElasticSPH(mat_props);

    // Set SPH solver parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = integration_scheme;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.0;
    sph_params.free_surface_threshold = 2.0;
    sph_params.artificial_viscosity = 0.5;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.num_proximity_search_steps = 4;
    terrain->SetSPHParameters(sph_params);

    // Construct the terrain
    terrain->Construct({length, width, height},                           // length X width X height
                       ChVector3d(length / 2 - offset, 0, -2 * spacing),  // patch center
                       BoxSide::ALL & ~BoxSide::Z_POS                     // all boundaries, except top
    );

    // Add robot bodies as FSI bodies
    auto sled = robot.GetChBody("sled");
    auto wheel1 = robot.GetChBody("limb1_link8");
    auto wheel2 = robot.GetChBody("limb2_link8");
    auto wheel3 = robot.GetChBody("limb3_link8");
    auto wheel4 = robot.GetChBody("limb4_link8");

    auto sled_geometry = robot.GetCollisionGeometry("sled");
    auto wheel1_geometry = robot.GetCollisionGeometry("limb1_link8");
    auto wheel2_geometry = robot.GetCollisionGeometry("limb2_link8");
    auto wheel3_geometry = robot.GetCollisionGeometry("limb3_link8");
    auto wheel4_geometry = robot.GetCollisionGeometry("limb4_link8");

    ChAssertAlways(sled_geometry);
    ChAssertAlways(wheel1_geometry);
    ChAssertAlways(wheel2_geometry);
    ChAssertAlways(wheel3_geometry);
    ChAssertAlways(wheel4_geometry);

    auto wheel_aabb = wheel1_geometry->CalculateAABB();

    terrain->AddRigidBody(sled, sled_geometry, false);
    terrain->AddRigidBody(wheel1, wheel1_geometry, false);
    terrain->AddRigidBody(wheel2, wheel2_geometry, false);
    terrain->AddRigidBody(wheel3, wheel3_geometry, false);
    terrain->AddRigidBody(wheel4, wheel4_geometry, false);

    terrain->SetActiveDomain(1.25 * wheel_aabb.Size());

    // Initialize the CRM terrain
    terrain->Initialize();

    auto terrain_aabb = terrain->GetSPHBoundingBox();
    cout << "CRM terrain AABB" << endl;
    cout << "  min:  " << terrain_aabb.min << endl;
    cout << "  max:  " << terrain_aabb.max << endl;

    return terrain;
}

void AttachCRMVisualization(std::shared_ptr<vehicle::CRMTerrain> terrain, std::shared_ptr<ChVisualSystemVSG> vis) {
    auto sysFSI = terrain->GetFsiSystemSPH();

    auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
    visFSI->EnableFluidMarkers(true);
    visFSI->EnableBoundaryMarkers(false);
    visFSI->EnableRigidBodyMarkers(true);
    visFSI->EnableColormapGUI(false);

    //// TODO - VSG color callback does not work for a plugin attached after VSG initialization!
    ////const auto& aabb = terrain->GetSPHBoundingBox();
    ////auto col_callback = chrono_types::make_shared<fsi::sph::ParticleHeightColorCallback>(aabb.min.z(), aabb.max.z());
    ////visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);

    vis->AttachPlugin(visFSI);
}

#endif

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a Chrono system and an associated collision detection system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create parser instance
    ChParserURDF robot(GetChronoDataFile("robot/robosimian/rs.urdf"));

    // Set root body pose
    robot.SetRootInitPose(ChFrame<>(ChVector3d(0, 0, 1.5), QUNIT));

    // Make all eligible joints as actuated (POSITION type) and
    // overwrite wheel motors with SPEED actuation.
    robot.SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION);
    robot.SetJointActuationType("limb1_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb2_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb3_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb4_joint8", ChParserURDF::ActuationType::SPEED);

    // Use convex hull for the sled collision shape
    robot.SetBodyMeshCollisionType("sled", ChParserURDF::MeshCollisionType::CONVEX_HULL);

    // Optional: visualize collision shapes
    ////robot.EnableCollisionVisualization();

    // Report parsed elements
    robot.PrintModelBodies();
    robot.PrintModelJoints();

    // Create the Chrono model
    robot.PopulateSystem(sys);

    // Get selected bodies of the robot
    auto root = robot.GetRootChBody();
    auto sled = robot.GetChBody("sled");
    auto limb1_wheel = robot.GetChBody("limb1_link8");
    auto limb2_wheel = robot.GetChBody("limb2_link8");
    auto limb3_wheel = robot.GetChBody("limb3_link8");
    auto limb4_wheel = robot.GetChBody("limb4_link8");

    auto wheel_geometry = robot.GetCollisionGeometry("limb1_link8");
    auto wheel_aabb = wheel_geometry->CalculateAABB();
    auto wheel_radius = wheel_aabb.Size().z();

    // Enable collision and set contact material for selected bodies of the robot
    sled->EnableCollision(true);
    limb1_wheel->EnableCollision(true);
    limb2_wheel->EnableCollision(true);
    limb3_wheel->EnableCollision(true);
    limb4_wheel->EnableCollision(true);

    ChContactMaterialData mat;
    mat.mu = 0.8f;
    mat.cr = 0.0f;
    mat.Y = 1e7f;
    auto cmat = mat.CreateMaterial(sys.GetContactMethod());
    sled->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb1_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb2_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb3_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb4_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);

    // Fix root body
    root->SetFixed(true);

    // Read the list of actuated motors, cache the motor links, and set their actuation function
    int num_motors = 32;
    std::ifstream ifs(GetChronoDataFile("robot/robosimian/actuation/motor_names.txt"));
    std::vector<std::shared_ptr<ChLinkMotor>> motors(num_motors);
    std::vector<std::shared_ptr<ChFunctionSetpoint>> motor_functions(num_motors);
    for (int i = 0; i < num_motors; i++) {
        std::string name;
        ifs >> name;
        motors[i] = robot.GetChMotor(name);
        motor_functions[i] = chrono_types::make_shared<ChFunctionSetpoint>();
        motors[i]->SetMotorFunction(motor_functions[i]);
    }

    // Actuation input files
    std::string start_filename;
    std::string cycle_filename;
    std::string stop_filename;

    switch (locomotion_mode) {
        case LocomotionMode::WALK:
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt");
            break;
        case LocomotionMode::SCULL:
            start_filename = GetChronoDataFile("robot/robosimian/actuation/sculling_start.txt");
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/sculling_cycle2.txt");
            stop_filename = GetChronoDataFile("robot/robosimian/actuation/sculling_stop.txt");
            break;
        case LocomotionMode::INCHWORM:
            start_filename = GetChronoDataFile("robot/robosimian/actuation/inchworming_start.txt");
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/inchworming_cycle.txt");
            stop_filename = GetChronoDataFile("robot/robosimian/actuation/inchworming_stop.txt");
            break;
        case LocomotionMode::DRIVE:
            start_filename = GetChronoDataFile("robot/robosimian/actuation/driving_start.txt");
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/driving_cycle.txt");
            stop_filename = GetChronoDataFile("robot/robosimian/actuation/driving_stop.txt");
            break;
    }

    // Create a robot motor actuation object
    models::ChRobotActuation actuator(32,              // number motors
                                      start_filename,  // start input file
                                      cycle_filename,  // cycle input file
                                      stop_filename,   // stop input file
                                      true             // repeat cycle
    );
    double duration_pose = 1.0;          // time interval to assume initial pose
    double duration_settle_robot = 0.5;  // time interval to allow robot settling on terrain
    actuator.SetTimeOffsets(duration_pose, duration_settle_robot);
    actuator.SetVerbose(true);

    // Create the visualization window
    auto camera_lookat = root->GetPos();
    auto camera_loc = camera_lookat + ChVector3d(3, 3, 0);

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowTitle("RoboSimian - " + LocomotionModeString(locomotion_mode) + " - " + TerrainTypeString(terrain_type));
    vis->AddCamera(camera_loc, camera_lookat);
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetBackgroundColor(ChColor(0.455f, 0.525f, 0.640f));
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows(false);
    vis->Initialize();

    // Solver settings
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(200);

// Terrain type
#ifndef CHRONO_FSI_SPH
    if (terrain_type == TerrainType::CRM) {
        cerr << "Chrono::FSI-SPH not enabled. Switch terrain type from CRM to SCM." << endl;
        terrain_type = TerrainType::SCM;
    }
#endif

    // Simulation loop
    double step_size = 5e-4;
    ChRealtimeStepTimer real_timer;
    bool terrain_created = false;

    std::shared_ptr<vehicle::ChTerrain> terrain;

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Create the terrain
        if (!terrain_created && sys.GetChTime() > duration_pose) {
            // Set terrain height
            double z = limb1_wheel->GetPos().z() - wheel_radius;
            cout << "Create terrain below z = " << z << endl;

            // Rigid terrain parameters
            double length = 8;
            double width = 2;

            // Create terrain
            switch (terrain_type) {
                case TerrainType::RIGID: {
                    auto terrain_rigid = CreateTerrainRigid(robot, length, width, z, length / 4);
                    auto ground = terrain_rigid->GetPatch(0)->GetGroundBody();
                    vis->BindItem(ground);
                    sys.GetCollisionSystem()->BindItem(ground);
                    terrain = terrain_rigid;
                    break;
                }
                case TerrainType::SCM: {
                    auto terrain_scm = CreateTerrainSCM(robot, length, width, z, length / 4);
                    vis->BindItem(terrain_scm->GetSCMLoader());
                    terrain = terrain_scm;
                    break;
                }
                case TerrainType::CRM: {
#ifdef CHRONO_FSI_SPH
                    auto terrain_crm = CreateTerrainCRM(robot, length, width, z, length / 4);
                    AttachCRMVisualization(terrain_crm, vis);
                    terrain = terrain_crm;
#endif
                    break;
                }
            }
            // Release robot
            robot.GetRootChBody()->SetFixed(false);

            terrain_created = true;
        }

        // Update camera location
        camera_lookat = ChVector3d(root->GetPos().x(), root->GetPos().y(), camera_lookat.z());
        camera_loc = camera_lookat + ChVector3d(3, 3, 0);

        ////vis->UpdateCamera(camera_loc, camera_lookat);
        vis->Render();

        // Update actuator and get current actuations
        actuator.Update(time);
        const auto& actuations = actuator.GetActuation();

        ////cout << time << "   " << actuator.GetCurrentPhase() << endl;
        ////int k = 0;
        ////for (int i = 0; i < 4; i++) {
        ////    for (int j = 0; j < 8; j++) {
        ////        cout << actuations[k++] << " ";
        ////    }
        ////    cout << endl;
        ////}
        ////cout << endl;

        // Apply motor actuations
        for (int i = 0; i < num_motors; i++)
            motor_functions[i]->SetSetpoint(-actuations[i], time);

        // Advance system dynamics
        if (terrain_type == TerrainType::CRM)
#ifdef CHRONO_FSI_SPH
            if (terrain)
                terrain->Advance(step_size);
            else
                sys.DoStepDynamics(step_size);
#endif
        else
            sys.DoStepDynamics(step_size);

        real_timer.Spin(step_size);
    }

    return 0;
}
