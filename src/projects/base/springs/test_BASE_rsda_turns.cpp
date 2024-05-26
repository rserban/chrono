#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif
#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    double step_size = 5e-4;
    double g = 0;

    double length = 1;
    double init_angle = 30 * CH_DEG_TO_RAD;

    double rest_angle = 0 * CH_DEG_TO_RAD;
    double num_init_revs = 0;
    double rsda_K = 10;
    double rsda_D = 2;
    double rsda_T = rsda_K * (720 + 30) * CH_DEG_TO_RAD;

    ////ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
    ////ChSolver::Type solver_type = ChSolver::Type::MINRES;
    ////ChSolver::Type solver_type = ChSolver::Type::GMRES;
    ////ChSolver::Type solver_type = ChSolver::Type::SPARSE_LU;
    ////ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;
    ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;
    ////ChSolver::Type solver_type = ChSolver::Type::MUMPS;

    ////ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT;
    ////ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
    ChTimestepper::Type integrator_type = ChTimestepper::Type::HHT;

    bool verbose_solver = false;
    bool verbose_integrator = true;

    // ------------------------------------------------

    // System, solver and integrator
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, g));

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
        std::cout << "Using Pardiso MKL solver" << std::endl;
        auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        solver->LockSparsityPattern(true);
        sys.SetSolver(solver);
#endif
    } else if (solver_type == ChSolver::Type::MUMPS) {
#ifdef CHRONO_MUMPS
        std::cout << "Using MUMPS solver" << std::endl;
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
                std::cout << "Using a direct sparse LS solver" << std::endl;
                auto solver = std::static_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
                solver->LockSparsityPattern(false);
                solver->UseSparsityPatternLearner(false);
                break;
            }
            case ChSolver::Type::BARZILAIBORWEIN:
            case ChSolver::Type::APGD:
            case ChSolver::Type::PSOR: {
                std::cout << "Using an iterative VI solver" << std::endl;
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
                std::cout << "Using an iterative LS solver" << std::endl;
                auto solver = std::static_pointer_cast<ChIterativeSolverLS>(sys.GetSolver());
                solver->SetMaxIterations(200);
                solver->SetTolerance(1e-10);
                solver->EnableDiagonalPreconditioner(true);
                break;
            }
        }
    }
    sys.GetSolver()->SetVerbose(verbose_solver);

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
    }
    sys.GetTimestepper()->SetVerbose(verbose_integrator);

    // ------------------------------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->SetName("ground");
    sys.AddBody(ground);

    ChQuaternion<> rot = QuatFromAngleY(-init_angle);
    ChVector3d loc = rot.Rotate(ChVector3d(length / 2, 0, 0));

    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("body");
    body->SetPos(loc);
    body->SetRot(rot);
    auto box = chrono_types::make_shared<ChVisualShapeBox>(2 * length, length / 4, length / 4);
    box->SetColor(ChColor(0.0f, 0.0f, 1.0f));
    body->AddVisualShape(box);
    sys.AddBody(body);

    ChQuaternion<> y2z = QuatFromAngleX(CH_PI_2);

    auto rev = chrono_types::make_shared<ChLinkRevolute>();
    rev->SetName("rev");
    rev->Initialize(ground, body, ChFrame<>(ChVector3d(0, 0, 0), y2z));
    sys.AddLink(rev);

    auto rsda = chrono_types::make_shared<ChLinkRSDA>();
    rsda->SetName("rsda");
    rsda->SetSpringCoefficient(rsda_K);
    rsda->SetDampingCoefficient(rsda_D);
    rsda->SetActuatorTorque(rsda_T);
    rsda->SetRestAngle(rest_angle);
    rsda->SetNumInitRevolutions(num_init_revs);
    rsda->Initialize(ground, body,                                    //
                     true,                                            //
                     ChFrame<>(ChVector3d(0, 0, 0), y2z),             //
                     ChFrame<>(ChVector3d(-length / 2, 0, 0), y2z));  //
    sys.AddLink(rsda);

    rsda->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(length / 4, 100));

    // Create the Irrlicht application
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Track RSDA test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 2, 0));
    vis->AttachSystem(&sys);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        ////tools::drawAllCOGs(vis.get(), 1.0);
        tools::drawAllLinkframes(vis.get(), 1.5);
        vis->EndScene();

        sys.DoStepDynamics(step_size);
        std::cout << rsda->GetAngle() * CH_RAD_TO_DEG << "   " << rsda->GetVelocity() << "   " << rsda->GetTorque()
                  << std::endl;
    }
}
