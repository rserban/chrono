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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#ifdef CHRONO_FSI_SPH
    #include "chrono_vehicle/terrain/CRMTerrain.h"
#endif

#ifdef CHRONO_FSI_SPH
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_models/robot/robosimian/RoboSimianURDF.h"

using namespace chrono;
using namespace chrono::robosimian;
#ifdef CHRONO_FSI_SPH
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
#endif

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Terrain type
enum class TerrainType { RIGID, SCM, CRM };

static std::string TerrainTypeString(TerrainType type) {
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

static std::shared_ptr<vehicle::RigidTerrain> CreateTerrainRigid(ChSystem& sys, RoboSimianURDF& rs, double length, double width, double height, double offset) {
    ChContactMaterialData mat;
    mat.mu = 0.8f;
    mat.cr = 0.0f;
    mat.Y = 1e7f;
    auto cmat = mat.CreateMaterial(sys.GetContactMethod());

    auto terrain = chrono_types::make_shared<vehicle::RigidTerrain>(&sys);
    auto patch = terrain->AddPatch(cmat, ChCoordsysd(ChVector3d(offset, 0, height - 0.1), QUNIT), length, width, 0.2);
    patch->SetTexture(GetChronoDataFile("textures/checker2.png"), 4, 1);

    terrain->Initialize();

    return terrain;
}

static std::shared_ptr<vehicle::SCMTerrain> CreateTerrainSCM(ChSystem& sys, RoboSimianURDF& rs, double length, double width, double height, double offset) {
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

    auto terrain = chrono_types::make_shared<vehicle::SCMTerrain>(&sys);
    terrain->SetReferenceFrame(ChCoordsys<>(ChVector3d(length / 2 - offset, 0, height), QUNIT));
    terrain->SetSoilParameters(Kphi, Kc, n, coh, phi, K, E_elastic, damping);
    terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);

    // Enable active domain feature
    auto torso = rs.GetTorsoBody();
    terrain->AddActiveDomain(torso, ChVector3d(0, 0, 0), ChVector3d(3.0, 2.0, 1.0));

    terrain->Initialize(length, width, spacing);

    return terrain;
}

#ifdef CHRONO_FSI_SPH

static std::shared_ptr<vehicle::CRMTerrain> CreateTerrainCRM(ChSystem& sys, RoboSimianURDF& rs, double length, double width, double height, double offset, double step_size) {
    bool verbose = true;

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

    auto terrain = chrono_types::make_shared<vehicle::CRMTerrain>(sys, spacing);
    terrain->SetVerbose(verbose);
    terrain->SetGravitationalAcceleration(sys.GetGravitationalAcceleration());
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
    ChAssertAlways(rs.GetSledGeometry());
    ChAssertAlways(rs.GetWheelGeometry());

    //// TODO - need a closed mesh for the sled. Or else, explicitly provide BCE markers

    ////terrain->AddRigidBody(rs.GetSledBody(), rs.GetSledGeometry(), false);
    for (const auto& wheel : rs.GetWheelBodies())
        terrain->AddRigidBody(wheel, rs.GetWheelGeometry(), false);

    terrain->SetActiveDomain(1.25 * ChVector3d(rs.GetWheelRadius()));

    // Initialize the CRM terrain
    terrain->Initialize();

    if (verbose) {
        const auto& terrain_aabb = terrain->GetSPHBoundingBox();
        cout << "CRM terrain AABB" << endl;
        cout << "  min:  " << terrain_aabb.min << endl;
        cout << "  max:  " << terrain_aabb.max << endl;
    }

    return terrain;
}

static void AttachCRMVisualization(std::shared_ptr<vehicle::CRMTerrain> terrain, std::shared_ptr<vsg3d::ChVisualSystemVSG> vis) {
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

static void SimulateSimulateToStartPose(ChSystem& sys, RoboSimianURDF& rs) {
    // Create the visualization system
    auto camera_lookat = rs.GetTorsoBody()->GetPos();
    auto camera_loc = camera_lookat + ChVector3d(3, 3, 0);

    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowTitle("RoboSimian - " + LocomotionModeAsString(rs.GetLocomotionMode()));
    vis->AddCamera(camera_loc, camera_lookat);
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetBackgroundColor(ChColor(0.455f, 0.525f, 0.640f));
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows(false);
    vis->Initialize();

    // Simulate to the start pose and save checkpoint
    rs.SimulateToStartPose(1e-3, vis);
}

// -----------------------------------------------------------------------------

static void Simulate(ChSystem& sys, RoboSimianURDF& rs, TerrainType terrain_type) {
    // Read system state from checkpoint file
    rs.LoadCheckpoint();

    // Force RoboSimian into HOLD phase
    rs.SetLocomotionPhase(models::ChRobotActuation::Phase::HOLD);

    // Create the visualization system
    auto camera_lookat = rs.GetTorsoBody()->GetPos();
    auto camera_loc = camera_lookat + ChVector3d(3, 3, 0);

    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowTitle("RoboSimian - " + LocomotionModeAsString(rs.GetLocomotionMode()) + " - " + TerrainTypeString(terrain_type));
    vis->AddCamera(camera_loc, camera_lookat);
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetBackgroundColor(ChColor(0.455f, 0.525f, 0.640f));
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows(false);

    // Integration step size
    double step_size = 5e-4;

    // Create the terrain
    double length = 8;
    double width = 2;
    double height = rs.GetWheelPos(0).z() - rs.GetWheelRadius();
    cout << "RoboSimian wheel height: " << rs.GetWheelPos(0).z() << endl;
    cout << "           wheel radius: " << rs.GetWheelRadius() << endl;
    cout << "Terrain surface height:  " << height << endl;

    std::shared_ptr<vehicle::ChTerrain> terrain;
#ifndef CHRONO_FSI_SPH
    if (terrain_type == TerrainType::CRM) {
        cerr << "Chrono::FSI-SPH not enabled. Switch terrain type from CRM to SCM." << endl;
        terrain_type = TerrainType::SCM;
    }
#endif

    switch (terrain_type) {
        case TerrainType::RIGID: {
            vis->Initialize();
            auto terrain_rigid = CreateTerrainRigid(sys, rs, length, width, height, length / 4);
            auto ground = terrain_rigid->GetPatch(0)->GetGroundBody();
            vis->BindItem(ground);
            sys.GetCollisionSystem()->BindItem(ground);
            terrain = terrain_rigid;
            break;
        }
        case TerrainType::SCM: {
            vis->Initialize();
            auto terrain_scm = CreateTerrainSCM(sys, rs, length, width, height, length / 4);
            vis->BindItem(terrain_scm->GetSCMLoader());
            terrain = terrain_scm;
            break;
        }
        case TerrainType::CRM: {
#ifdef CHRONO_FSI_SPH
            auto terrain_crm = CreateTerrainCRM(sys, rs, length, width, height, length / 4, step_size);
            AttachCRMVisualization(terrain_crm, vis);
            vis->Initialize();
            terrain = terrain_crm;
#endif
            break;
        }
    }

    // Release robot
    rs.FixTorso(false);

    // Simulation loop
    ChRealtimeStepTimer real_timer;

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Update camera location
        const auto& torso_pos = rs.GetTorsoBody()->GetPos();
        camera_lookat = ChVector3d(torso_pos.x(), torso_pos.y(), camera_lookat.z());
        camera_loc = camera_lookat + ChVector3d(3, 3, 0);

        ////vis->UpdateCamera(camera_loc, camera_lookat);
        vis->Render();

        // Update actuation
        rs.UpdateActuation(time);

        // Advance system dynamics
        if (terrain_type == TerrainType::CRM)
#ifdef CHRONO_FSI_SPH
            terrain->Advance(step_size);
#endif
        else
            sys.DoStepDynamics(step_size);

        real_timer.Spin(step_size);
    }
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "ROBOSIMIAN_URDF";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono system and an associated collision detection system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Solver settings
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    // Create the RoboSimian robot, set parameters, and construct it
    RoboSimianURDF rs;
    rs.SetVerbose(true);
    rs.SetOutputDirectory(out_dir);
    rs.SetInitialPose(ChFramed(ChVector3d(0, 0, 1.5), QUNIT));
    rs.SetLocomotionMode(LocomotionMode::INCHWORM);
    rs.SetDurationStartPose(1.0);
    rs.SetDurationHold(0.5);
    rs.Construct(sys);

    // Generate checkpoint in start pose
    ////SimulateSimulateToStartPose(sys, rs);

    // Simulate from checkpoint on specified terrain type
    Simulate(sys, rs, TerrainType::CRM);

    return 0;
}
