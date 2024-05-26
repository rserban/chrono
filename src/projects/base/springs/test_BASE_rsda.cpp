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
    // Number of bodies and links/RSDAs
    int n = 3;

    // Fix one of the bodies (0, 1, ..., n-1)
    int fixed_body = 0;

    // Break one of the links (0, 1, ..., n-1)
    int broken_link = 0;

    double step_size = 5e-4;
    double g = 0;

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

    double length = 1;
    double del_angle = CH_2PI / n;
    double radius = (length / 2) / std::tan(del_angle / 2);

    std::vector<std::shared_ptr<ChBody>> bodies;
    ChVector3d loc;
    ChQuaternion<> rot;
    double angle = 0;
    for (int i = 0; i < n; i++) {
        loc.x() = radius * std::sin(angle);
        loc.z() = radius * std::cos(angle);
        rot = QuatFromAngleY(angle);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("body_" + std::to_string(i));
        body->SetPos(loc);
        body->SetRot(rot);
        if (i == fixed_body)
            body->SetFixed(true);
        auto box = chrono_types::make_shared<ChVisualShapeBox>(2 * length, length / 3, length / 3);
        box->SetColor(ChColor(0.0f, 0.0f, (i * 1.0f) / (n - 1)));
        body->AddVisualShape(box);
        sys.AddBody(body);
        bodies.push_back(body);

        angle += del_angle;
    }

    ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

    for (int i = 0; i < n; i++) {
        if (i == broken_link)
            continue;

        int j = (i == n - 1) ? 0 : i + 1;

        auto rev = chrono_types::make_shared<ChLinkRevolute>();
        rev->SetName("rev" + std::to_string(i));
        ChVector3d rloc = bodies[i]->TransformPointLocalToParent(ChVector3d(length/2, 0, 0));
        rev->Initialize(bodies[i], bodies[j], ChFrame<>(rloc, bodies[i]->GetRot() * z2y));
        sys.AddLink(rev);

        auto rsda = chrono_types::make_shared<ChLinkRSDA>();
        rsda->SetName("rsda" + std::to_string(i));
        rsda->SetSpringCoefficient(10);
        rsda->SetDampingCoefficient(3);
        rsda->SetRestAngle(0);
        rsda->Initialize(bodies[i], bodies[j],                            //
                         true,                                            //
                         ChFrame<>(ChVector3d(+length / 2, 0, 0), z2y),   //
                         ChFrame<>(ChVector3d(-length / 2, 0, 0), z2y));  //
        sys.AddLink(rsda);

        rsda->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(length / 4, 100));
    }

    // Create the Irrlicht application
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowTitle("RSDA test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 0, 6));
    vis->AttachSystem(&sys);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawAllLinkframes(vis.get(), 1.5);
        vis->EndScene();

        sys.DoStepDynamics(step_size);
    }
}
