// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Main driver function for a vehicle specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Vehicle JSON specification
////std::string vehicle_json = "hmmwv/vehicle/HMMWV_Vehicle.json";
std::string vehicle_json = "generic/vehicle/Vehicle_RigidRigidSuspension.json";

// Powertrain JSON specification
bool add_powertrain = false;
std::string engine_json = "hmmwv/powertrain/HMMWV_EngineShafts.json";
std::string transmission_json = "hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json";

// Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89, PAC02, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

enum class TerrainType { FLAT, RIGID };
TerrainType terrain_type = TerrainType::RIGID;

// Terrain length (X direction)
double terrainLength = 200.0;

// Lane direction
double yaw_angle = 0 * CH_DEG_TO_RAD;

// Initial chassis velocity if driveline disconnected
bool disconnect_driveline = true;
double init_vel = 10;

// Contact formulation (when using rigid tires)
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step size
double step_size_NSC = 1e-3;
double step_size_SMC = 5e-4;

double tire_step_size = 1e-3;

// Solver and integrator types
////ChSolver::Type slvr_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type slvr_type = ChSolver::Type::PSOR;
////ChSolver::Type slvr_type = ChSolver::Type::MINRES;
////ChSolver::Type slvr_type = ChSolver::Type::GMRES;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_LU;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_QR;
ChSolver::Type slvr_type = ChSolver::Type::PARDISO_MKL;
////ChSolver::Type slvr_type = ChSolver::Type::MUMPS;

////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::HHT;

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;

// =============================================================================

// Forward declarations
void CreateTires(WheeledVehicle& vehicle, VisualizationType tire_vis);
void SelectSolver(ChSystem& sys, ChSolver::Type& solver_type, ChTimestepper::Type& integrator_type);
void ReportTiming(ChSystem& sys);

// =============================================================================

class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter() {}

  private:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector3d& cforce,
                                 const ChVector3d& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        const ChVector3d& nrm = plane_coord.GetAxisX();
        std::cout << "   " << ((ChBody*)modA)->GetName() << "  -  " << ((ChBody*)modB)->GetName()
                  << std::endl;
        std::cout << "      pt A:  " << pA << std::endl;
        std::cout << "      pt B:  " << pB << std::endl;
        std::cout << "      nrm:   " << nrm << std::endl;
        std::cout << "      frc:   " << plane_coord * cforce << std::endl;
        std::cout << "      delta: " << distance << std::endl;
        std::cout << "      reff:  " << eff_radius << std::endl;

        if (nrm.z() < 0.5) {
            std::cout << "\n\n\n*************\n\n\n" << std::endl;
            exit(1);
        }

        return true;
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    if (terrain_type == TerrainType::FLAT &&
        (tire_model == TireModelType::RIGID || tire_model == TireModelType::RIGID_MESH)) {
        std::cout << "Flat terrain incompatible with a rigid tire model!" << std::endl;
        return 1;
    }

    ChQuaternion<> yaw_rot = QuatFromAngleZ(yaw_angle);
    ChCoordsys<> patch_sys(VNULL, yaw_rot);
    ChVector3d init_loc = patch_sys.TransformPointLocalToParent(ChVector3d(-terrainLength / 2 + 5, 0, 0.7));
    ChVector3d path_start = patch_sys.TransformPointLocalToParent(ChVector3d(-terrainLength / 2, 0, 0.5));
    ChVector3d path_end = patch_sys.TransformPointLocalToParent(ChVector3d(+terrainLength / 2, 0, 0.5));

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_json), contact_method);
    if (disconnect_driveline) {
        vehicle.Initialize(ChCoordsys<>(init_loc, yaw_rot), init_vel);
        vehicle.DisconnectDriveline();
    } else {
        vehicle.Initialize(ChCoordsys<>(init_loc, yaw_rot));
    }
    vehicle.DisconnectDriveline();
    vehicle.GetChassis()->SetFixed(false);

    vehicle.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create and initialize the powertrain system
    if (add_powertrain) {
        auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
        auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        vehicle.InitializePowertrain(powertrain);
    }

    // Create and initialize the tires
    auto tire_vis = (tire_model == TireModelType::RIGID_MESH ? VisualizationType::MESH : VisualizationType::PRIMITIVES);
    CreateTires(vehicle, tire_vis);

    // Containing system
    auto system = vehicle.GetSystem();

    // Create the terrain
    std::shared_ptr<ChTerrain> terrain;
    switch (terrain_type) {
        case TerrainType::RIGID:
        default: {
            ChContactMaterialData minfo;
            minfo.mu = 0.9f;
            minfo.cr = 0.1f;
            minfo.Y = 2e7f;
            minfo.nu = 0.3f;
            auto patch_mat = minfo.CreateMaterial(contact_method);

            auto rigid_terrain = chrono_types::make_shared<RigidTerrain>(system);
            auto patch = rigid_terrain->AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0), yaw_rot), terrainLength, 10);
            patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);

            rigid_terrain->Initialize();
            terrain = rigid_terrain;
            break;
        }
        case TerrainType::FLAT: {
            auto flat_terrain = chrono_types::make_shared<FlatTerrain>(0, 0.9f);
            terrain = flat_terrain;
            break;
        }
    }

    // Create the straight path and the driver system
    auto path = StraightLinePath(path_start, path_end, 1);
    ChPathFollowerDriver driver(vehicle, path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create Irrlicht visualization
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Vehicle acceleration - JSON specification");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // Solver and integrator settings
    double step_size = 1e-3;
    switch (contact_method) {
        case ChContactMethod::NSC:
            std::cout << "Use NSC" << std::endl;
            step_size = step_size_NSC;
            break;
        case ChContactMethod::SMC:
            std::cout << "Use SMC" << std::endl;
            step_size = step_size_SMC;
            break;
    }

    SelectSolver(*vehicle.GetSystem(), slvr_type, intgr_type);
    vehicle.GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    vehicle.GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    std::cout << "SOLVER TYPE:     " << (int)slvr_type << std::endl;
    std::cout << "INTEGRATOR TYPE: " << (int)intgr_type << std::endl;

    // Set tire integration step-size
    tire_step_size = std::min(tire_step_size, step_size);

    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);
            if (tire_step_size > 0)
                wheel->GetTire()->SetStepsize(tire_step_size);
        }
    }

    // User-defined callback for contact reporting
    auto creporter = chrono_types::make_shared<ContactReporter>();

    vehicle.LogSubsystemTypes();

    // Simulation loop
    int step_number = 0;
    double time = 0;
    double time_end = 6;

    ChTimer timer;
    timer.start();
    while (vis->Run() && time < time_end) {
        time = system->GetChTime();

        // Render scene and draw a coordinate system aligned with the world frame
        vis->BeginScene();
        vis->Render();
        irrlicht::tools::drawCoordsys(vis.get(), ChCoordsys<>(vehicle.GetPos(), QUNIT), 2);
        vis->EndScene();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, *terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        ////ReportTiming(*system);

        /*
        // Output contact information
        if (terrain_type == TerrainType::RIGID && time > 1) {
            std::cout << "--------------------------------------" << std::endl;
            std::cout << "time: " << time << std::endl;
            system->GetContactContainer()->ReportAllContacts(creporter);
            auto rterrain = std::static_pointer_cast<RigidTerrain>(terrain);
            std::cout << "Ground frc: " << rterrain->GetPatches()[0]->GetGroundBody()->GetContactForce() << std::endl;
        }
        */

        // Increment frame number
        step_number++;
    }

    std::cout << "\n\nFinal vehicle speed: " << vehicle.GetSpeed() << std::endl;

    return 0;
}

// =============================================================================

void CreateTires(WheeledVehicle& vehicle, VisualizationType tire_vis) {
    switch (tire_model) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (tire_model == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<HMMWV_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<HMMWV_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<HMMWV_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<HMMWV_RigidTire>("RR", use_mesh);

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::FIALA: {
            auto tire_FL = chrono_types::make_shared<HMMWV_FialaTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_FialaTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_FialaTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_FialaTire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<HMMWV_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_TMeasyTire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::PAC89: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac89Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac89Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac89Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac89Tire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac02Tire>("RR");

            vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[RIGHT], tire_vis);
            vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[LEFT], tire_vis);
            vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[RIGHT], tire_vis);

            break;
        }
        default:
            break;
    }
}

void SelectSolver(ChSystem& sys, ChSolver::Type& solver_type, ChTimestepper::Type& integrator_type) {
    // For NSC systems, use implicit linearized Euler and an iterative VI solver
    if (sys.GetContactMethod() == ChContactMethod::NSC) {
        integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
        if (solver_type != ChSolver::Type::BARZILAIBORWEIN && solver_type != ChSolver::Type::APGD &&
            solver_type != ChSolver::Type::PSOR && solver_type != ChSolver::Type::PSSOR) {
            solver_type = ChSolver::Type::BARZILAIBORWEIN;
        }
    }

    // If none of the direct sparse solver modules is enabled, default to SPARSE_QR
    if (solver_type == ChSolver::Type::PARDISO_MKL) {
#ifndef CHRONO_PARDISO_MKL
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
    } else if (solver_type == ChSolver::Type::MUMPS) {
#ifndef CHRONO_MUMPS
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
    }

    if (solver_type == ChSolver::Type::PARDISO_MKL) {
#ifdef CHRONO_PARDISO_MKL
        auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        solver->LockSparsityPattern(true);
        sys.SetSolver(solver);
#endif
    } else if (solver_type == ChSolver::Type::MUMPS) {
#ifdef CHRONO_MUMPS
        auto solver = chrono_types::make_shared<ChSolverMumps>();
        solver->LockSparsityPattern(true);
        solver->EnableNullPivotDetection(true);
        solver->GetMumpsEngine().SetICNTL(14, 50);
        sys.SetSolver(solver);
#endif
    } else {
        sys.SetSolverType(solver_type);
        switch (solver_type) {
            case ChSolver::Type::SPARSE_LU:
            case ChSolver::Type::SPARSE_QR: {
                auto solver = std::static_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
                solver->LockSparsityPattern(false);
                solver->UseSparsityPatternLearner(false);
                break;
            }
            case ChSolver::Type::BARZILAIBORWEIN:
            case ChSolver::Type::APGD:
            case ChSolver::Type::PSOR: {
                auto solver = std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver());
                solver->SetMaxIterations(100);
                solver->SetOmega(0.8);
                solver->SetSharpnessLambda(1.0);

                ////sys.SetMaxPenetrationRecoverySpeed(1.5);
                break;
            }
            case ChSolver::Type::BICGSTAB:
            case ChSolver::Type::MINRES:
            case ChSolver::Type::GMRES: {
                auto solver = std::static_pointer_cast<ChIterativeSolverLS>(sys.GetSolver());
                solver->SetMaxIterations(200);
                solver->SetTolerance(1e-10);
                solver->EnableDiagonalPreconditioner(true);
                break;
            }
        }
    }

    sys.SetTimestepperType(integrator_type);
    switch (integrator_type) {
        case ChTimestepper::Type::HHT: {
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxIters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            integrator->SetStepControl(false);
            integrator->SetModifiedNewton(false);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT: {
            auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
            integrator->SetMaxIters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
            break;
    }
}

void ReportTiming(ChSystem& sys) {
    std::stringstream ss;
    ss.precision(4);
    ss << std::fixed << sys.GetChTime() << " | ";
    ss << sys.GetTimerStep() << " " << sys.GetTimerAdvance() << " " << sys.GetTimerUpdate() << " | ";
    ss << sys.GetTimerJacobian() << " " << sys.GetTimerLSsetup() << " " << sys.GetTimerLSsolve() << " | ";
    ss << sys.GetTimerCollision() << " " << sys.GetTimerCollisionBroad() << " " << sys.GetTimerCollisionNarrow();

    auto LS = std::dynamic_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
    if (LS) {
        ss << " | ";
        ss << LS->GetTimeSetup_Assembly() << " " << LS->GetTimeSetup_SolverCall() << " ";
        ss << LS->GetTimeSolve_Assembly() << " " << LS->GetTimeSolve_SolverCall();
        LS->ResetTimers();
    }
    std::cout << ss.str() << std::endl;
}
