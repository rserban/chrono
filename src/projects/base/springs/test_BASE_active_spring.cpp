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
// Simple example demonstrating the use of a spring with force defined through
// a functor class.
//
// The spring also carries additional internal dynamics.  In this demo, these
// are completely coupled from the connected bodies.
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================

// Solver type (PARDISO, MINRES, GMRES)
ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;

// Integrator type (HHT, EULER_IMPLICIT, EULER_IMPLICIT_LINEARIZED)
ChTimestepper::Type integrator_type = ChTimestepper::Type::HHT;

bool use_jacobians = true;
int num_nonlin_iterations = 20;
double abs_tol = 1e-5;
bool step_control = true;

// -------------------------------------------------

double step_size = 1e-4;
bool verbose_integrator = false;
bool verbose_solver = false;

// =============================================================================

double mass = 1;
double rest_length = 1.5;
double spring_coef = 1e3;
double critical_damping = 2 * std::sqrt(mass * spring_coef);
double damping_coef = 0.5 * critical_damping;

// =============================================================================

// Functor class implementing the force for a ChLinkTSDA.
class MySpringForce : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double length0,         // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
                            ) override {
        // Access current states.
        ////ChVectorDynamic<> states = link->GetStates();
        ////std::cout << "t = " << time << "  " << states(0) << " " << states(1) << std::endl;

        double force = -spring_coef * (length - length0) - damping_coef * vel;
        return force;
    }
};

// Functor class implementing the ODE right-hand side for a ChLinkTSDA.
class MySpringRHS : public ChLinkTSDA::ODE {
    virtual unsigned int GetNumStates() const override { return 2; }
    virtual void SetInitialConditions(ChVectorDynamic<>& states,  // output vector containig initial conditions
                                      const ChLinkTSDA& link      // associated link
                                      ) override {
        states(0) = 1;
        states(1) = 0;
    }
    virtual void CalculateRHS(double time,
                              const ChVectorDynamic<>& states,  // current states
                              ChVectorDynamic<>& rhs,           // output vector containing the ODE right-hand side
                              const ChLinkTSDA& link            // associated link
                              ) override {
        rhs(0) = states(0);
        rhs(1) = std::cos(time);
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create the ground body with two visualization spheres
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    ground->AddVisualShape(sph);

    // Create a body suspended through a ChLinkTSDA
    auto body = chrono_types::make_shared<ChBody>();
    system.AddBody(body);
    body->SetPos(ChVector3d(0, -3, 0));
    body->SetFixed(false);
    body->EnableCollision(false);
    body->SetMass(mass);
    body->SetInertiaXX(ChVector3d(1, 1, 1));

    auto box = chrono_types::make_shared<ChVisualShapeBox>(2, 2, 2);
    box->SetColor(ChColor(0.6f, 0, 0));
    body->AddVisualShape(box);

    // Create the spring between body and ground. The spring end points are specified in the body relative frames.
    auto force = std::make_shared<MySpringForce>();
    MySpringRHS rhs;

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body, ground, true, ChVector3d(0, 0, 0), ChVector3d(0, 0, 0));
    spring->SetRestLength(rest_length);
    spring->IsStiff(use_jacobians);
    spring->RegisterForceFunctor(force);
    spring->RegisterODE(&rhs);
    system.AddLink(spring);
    auto spring_shape = chrono_types::make_shared<ChVisualShapeSpring>(0.05, 80, 15);
    spring_shape->SetColor(ChColor(0.5f, 0.5f, 0.5f));
    spring->AddVisualShape(spring_shape);

    // Create the Irrlicht application
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Active spring test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 0, 6));
    vis->AttachSystem(&system);

    // Create output directory and log file
    const std::string out_dir = GetChronoOutputPath() + "DEMO_ACTIVE_SPRING";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string logfile = out_dir + "/log.dat";
    std::ofstream log(logfile);

    std::string btitle("Body - ");

    // Modify integrator
    switch (integrator_type) {
        case ChTimestepper::Type::HHT: {
            btitle += "HHT - ";
            system.SetTimestepperType(ChTimestepper::Type::HHT);
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxIters(num_nonlin_iterations);
            integrator->SetAbsTolerances(abs_tol);
            integrator->SetStepControl(step_control);
            integrator->SetVerbose(verbose_integrator);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT: {
            btitle += "EI - ";
            system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
            auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(system.GetTimestepper());
            integrator->SetMaxIters(num_nonlin_iterations);
            integrator->SetAbsTolerances(1e-6);
            integrator->SetVerbose(verbose_integrator);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED: {
            btitle += "EIL - ";
            break;
        }
    }

    // Modify solver
    switch (solver_type) {
        case ChSolver::Type::GMRES: {
            btitle += "GMRES - ";
            auto solver = chrono_types::make_shared<ChSolverGMRES>();
            system.SetSolver(solver);
            solver->SetMaxIterations(100);
            solver->SetVerbose(verbose_solver);
            break;
        }
        case ChSolver::Type::MINRES: {
            btitle += "MINRES - ";
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            solver->SetMaxIterations(200);
            solver->SetTolerance(1e-15);
            solver->EnableWarmStart(true);
            solver->EnableDiagonalPreconditioner(true);
            solver->SetVerbose(verbose_solver);
            system.SetSolver(solver);
            break;
        }
        case ChSolver::Type::PARDISO_MKL: {
            btitle += "PARDISO - ";
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->LockSparsityPattern(false);
            solver->SetVerbose(verbose_solver);
            system.SetSolver(solver);
            break;
        }
    }

    btitle += use_jacobians ? " Jac YES" : "Jac NO";

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        system.DoStepDynamics(step_size);

        ChVectorDynamic<> state = spring->GetStates();
        log << system.GetChTime() << ", " << state(0) << ", " << state(1) << ", ";
        log << body->GetPos().y() << ", " << body->GetPosDt().y();
        log << "\n";

        if (system.GetChTime() >= 0.4)
            break;
    }

#ifdef CHRONO_POSTPROCESS
    // Plot results
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    mplot.OutputWindow(0);
    mplot.SetGrid();
    mplot.SetTitle("ODE solution");
    mplot.SetLabelX("t");
    mplot.SetLabelY("y");
    mplot.SetCommand("set ytics nomirror");
    mplot.SetCommand("set y2range [-1:1]");
    mplot.SetCommand("set y2tics -1, 0.25");
    mplot.Plot(logfile.c_str(), 1, 2, "state 1", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 3, "state 2", " with lines lw 2 axis x1y2");

    mplot.OutputWindow(1);
    mplot.SetGrid();
    mplot.SetTitle(btitle.c_str());
    mplot.SetLabelX("t");
    mplot.SetLabelY("y");
    mplot.SetCommand("set ytics nomirror");
    mplot.SetCommand("set y2range [-20:20]");
    mplot.SetCommand("set y2tics -20, 5");
    mplot.Plot(logfile.c_str(), 1, 4, "y", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 5, "y'", " with lines lw 2 axis x1y2");

#endif

    return 0;
}
