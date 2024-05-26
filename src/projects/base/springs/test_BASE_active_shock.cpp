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
// Simple example demonstrating the use of a stiff element with force controlled
// by internal dynamics.
//
// =============================================================================

////#include <cfloat>
////unsigned int fp_control_state = _controlfp(_EM_DENORMAL, _MCW_EM);


#include <cstdio>

#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// =============================================================================

//// TODO
////   (1) Check HHT when setting abs_tol too tight (e.g., 1e-5) - Solution explodes!
////       Works fine with looser tolerance (e.g., 1e-4).  
////       Check number of iterations in each case. Check ewt vectors.
////
////   (2) When using tables, check discontinuity between HF <-> HF+LF
////       Unlike the NO tables case, here we use vel_min, which introduces a jump. 
////       This results in solution oscillations with implicit integrators (HT and EI)
////       [SOLVED] by fixing ODE rhs so that there is no discontinuity
////
////   (3) When using step actuation, HHT leads to some (small) spikes in velocity.
////       I would have expected they would smoothen the sharp fronts...
////       Euler implicit behaves better?
////       
////   (4) HHT step size control out of whack - ensure it falls *precisely* on what
////       the user requests

// --------------------------------------------------

// Solver type (PARDISO, MINRES, GMRES)
ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;

// Integrator type (HHT, EULER_IMPLICIT, EULER_IMPLICIT_LINEARIZED)
ChTimestepper::Type integrator_type = ChTimestepper::Type::HHT;

bool use_jacobians = true;
int num_nonlin_iterations = 20;
double abs_tol = 1e-3;
bool step_control = true;

// -------------------------------------------------

bool smooth_actuation = true;
bool use_tables = true;

// -------------------------------------------------

double step_size = 1e-3;
bool verbose_integrator = true;
bool verbose_solver = true;

// =============================================================================

class ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    virtual double evaluate(double time,            // current time
                            double rest_length,     // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
                            ) override {
        // Damper force is the value oif the 2nd state.
        // Flip the sign to make the force acting against velocity
        return -link.GetStates()(1);
    }
};

class ShockODE : public ChLinkTSDA::ODE {
  public:
    ShockODE(bool use_tables) : m_use_damper_tables(use_tables) {
        // Setup damper tables for high frequency signals
        m_hf_damper_table.AddPoint(-5, -11936.925);
        m_hf_damper_table.AddPoint(-3, -11368.5);
        m_hf_damper_table.AddPoint(-0.33, -10335);
        m_hf_damper_table.AddPoint(-0.22, -8376);
        m_hf_damper_table.AddPoint(-0.13, -5789);
        m_hf_damper_table.AddPoint(-0.05, -2672);
        m_hf_damper_table.AddPoint(-0.02, -654);
        m_hf_damper_table.AddPoint(0, 0);
        m_hf_damper_table.AddPoint(0.02, 652);
        m_hf_damper_table.AddPoint(0.05, 1649);
        m_hf_damper_table.AddPoint(0.13, 2975);
        m_hf_damper_table.AddPoint(0.22, 4718);
        m_hf_damper_table.AddPoint(0.33, 7496);
        m_hf_damper_table.AddPoint(3, 8245.6);
        m_hf_damper_table.AddPoint(5, 8657.88);

        // Setup damper tables for low frequency signals
        m_lf_damper_table.AddPoint(-5, -12183.06833);
        m_lf_damper_table.AddPoint(-3, -11602.92222);
        m_lf_damper_table.AddPoint(-0.33, -10548);
        m_lf_damper_table.AddPoint(-0.22, -8620);
        m_lf_damper_table.AddPoint(-0.13, -6669);
        m_lf_damper_table.AddPoint(-0.05, -2935);
        m_lf_damper_table.AddPoint(-0.02, -986);
        m_lf_damper_table.AddPoint(0, 0);
        m_lf_damper_table.AddPoint(0.02, 766);
        m_lf_damper_table.AddPoint(0.05, 3621);
        m_lf_damper_table.AddPoint(0.13, 12628);
        m_lf_damper_table.AddPoint(0.22, 14045);
        m_lf_damper_table.AddPoint(0.33, 15444);
        m_lf_damper_table.AddPoint(3, 16988.27778);
        m_lf_damper_table.AddPoint(5, 17837.69167);
    }

    virtual unsigned int GetNumStates() const override { return 2; }
    virtual void SetInitialConditions(ChVectorDynamic<>& states,  // output vector containig initial conditions
                                      const ChLinkTSDA& link      // associated link
                                      ) override {
        // we start with zero velocity and zero force
        states(0) = 0;
        states(1) = 0;
    }

    virtual void CalculateRHS(double time,                      // current time
                              const ChVectorDynamic<>& states,  // current states
                              ChVectorDynamic<>& rhs,           // output vector containing the ODE right-hand side
                              const ChLinkTSDA& link            // associated link
                              ) override {
        // ODE1
        // y_dot0 = (u0 - y0)/T0;
        // u0 is damper velocity vel = input
        // y0 = delayed damper velocity v_del = stage(0) = output
        const double T0 = 0.04;
        const double T1 = 1.02e-3;
        double vel = link.GetVelocity();
        rhs(0) = (vel - states(0)) / T0;
        double vel_delayed = states(0);
        double vel_min = std::min(vel, vel_delayed);
        double force_hf, force_lf;
        if (m_use_damper_tables) {
            // use lookup tables
            force_hf = m_hf_damper_table.GetVal(vel);
            force_lf = m_lf_damper_table.GetVal(vel_min);
        } else {
            // use continuous funktions (derived from the tables)
            force_hf = HF_DamperForce(vel);
            force_lf = LF_DamperForce(vel_min);
        }
        double force1 = 0.0;
        if (vel_min > 0.0) {
            force1 = force_hf + force_lf;
            //std::cout << time << "  " << vel << "  " << force1 << "   HF + LF" << std::endl;
        } else {
            force1 = force_hf;
            //std::cout << time << "  " << vel << "  " << force1 << "   HF" << std::endl;
        }

        // ODE2
        // y_dot1 = (u1 - y1)/T1;
        // u1 = damper force1 = input
        // y1 = delayed damper force = stage(1) = output
        rhs(1) = (force1 - states(1)) / T1;
    }

    virtual bool CalculateJac(double time,                      // current time
                              const ChVectorDynamic<>& states,  // current ODE states
                              const ChVectorDynamic<>& rhs,     // current ODE right-hand side vector
                              ChMatrixDynamic<>& jac,           // output Jacobian matrix
                              ChLinkTSDA* link                  // back-pointer to associated link
    ) {
        const double T0 = 0.04;
        const double T1 = 1.02e-3;

        jac(0, 0) = -1 / T0;
        jac(0, 1) = 0;
        jac(1, 0) = 0;
        jac(1, 1) = -1 / T1;

        return true;
    }

  private:
    double HF_DamperForce(double vel) {
        const double p1 = 38097.1;
        const double p2 = 2.83566;
        const double p4 = 2.45786;
        if (vel >= 0.0) {
            return p1 * vel / (1.0 + p2 * vel);
        } else {
            return p1 * vel / (1.0 - p4 * vel);
        }
    }
    double LF_DamperForce(double vel) {
        const double q1 = 160650;
        const double q2 = 7.46883;
        const double q4 = 11.579;
        if (vel >= 0.0) {
            return q1 * vel / (1.0 + q2 * vel);
        } else {
            return q1 * vel / (1.0 - q4 * vel);
        }
    }

    ChFunctionInterp m_hf_damper_table;
    ChFunctionInterp m_lf_damper_table;
    bool m_use_damper_tables;
};

// =============================================================================

class Actuation : public ChFunction {
  public:
    Actuation(bool smooth) : m_smooth(smooth), m_period(0.5), m_width(0.05), m_start(0.1) {}
    virtual Actuation* Clone() const override { return new Actuation(*this); }

    virtual void Update(double x) override {
        if (x > m_start + m_period) {
            m_start += m_period;
        }
    }

    virtual double GetVal(double x) const override {
        if (x > m_start && x < m_start + m_width) {
            if (m_smooth)
                return 1 + std::cos(2 * CH_PI * ((x - m_start) / m_width - 0.5));
            else
                return 2;
        }

        return 0;
    }

  private:
    bool m_smooth;
    double m_period;
    double m_width;
    double m_start;
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
    body->SetMass(500);
    body->SetInertiaXX(ChVector3d(1, 1, 1));

    auto box = chrono_types::make_shared<ChVisualShapeBox>(2, 2, 2);
    box->SetColor(ChColor(0.6f, 0, 0));
    body->AddVisualShape(box);

    // Create an actuator to control body speed (along Y direction)
    auto x2y = QuatFromAngleZ(-CH_PI_2);
    auto speed_fun = chrono_types::make_shared<Actuation>(smooth_actuation);
    auto motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    system.AddLink(motor);
    motor->Initialize(ground, body, ChFrame<>(body->GetPos(), x2y));
    motor->SetSpeedFunction(speed_fun);

    // Create the shock between body and ground. The end points are specified in the body relative frames.
    auto force = chrono_types::make_shared<ShockForce>();
    ShockODE rhs(use_tables);

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body, ground, true, ChVector3d(0, 0, 0), ChVector3d(0, 0, 0));
    spring->IsStiff(use_jacobians);
    spring->RegisterForceFunctor(force);
    spring->RegisterODE(&rhs);
    system.AddLink(spring);
    auto spring_shape = chrono_types::make_shared<ChSegmentShape>();
    spring_shape->SetColor(ChColor(0.5f, 0.5f, 0.5f));
    spring->AddVisualShape(spring_shape);

    // Create the Irrlicht application
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Active shock test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 0, 6));
    vis->AttachSystem(&system);

    // Create output directory and log file
    const std::string out_dir = GetChronoOutputPath() + "DEMO_ACTIVE_SHOCK";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string logfile = out_dir + "/log.dat";
    ChStreamOutAsciiFile log(logfile.c_str());

    std::string title("");

    // Modify integrator
    switch (integrator_type) {
        case ChTimestepper::Type::HHT: {
            title += "HHT - ";
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
            title += "EI - ";
            system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
            auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(system.GetTimestepper());
            integrator->SetMaxIters(num_nonlin_iterations);
            integrator->SetAbsTolerances(1e-6);
            integrator->SetVerbose(verbose_integrator);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED: {
            title += "EIL - ";
            break;
        }
    }

    // Modify solver
    switch (solver_type) {
        case ChSolver::Type::GMRES: {
            title += "GMRES - ";
            auto solver = chrono_types::make_shared<ChSolverGMRES>();
            system.SetSolver(solver);
            solver->SetMaxIterations(100);
            solver->SetVerbose(verbose_solver);
            break;
        }
        case ChSolver::Type::MINRES: {
            title += "MINRES - ";
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            solver->EnableWarmStart(true);
            solver->EnableDiagonalPreconditioner(true);
            solver->SetMaxIterations(200);
            solver->SetTolerance(1e-10);
            solver->SetVerbose(verbose_solver);
            system.SetSolver(solver);
            break;
        }
        case ChSolver::Type::PARDISO_MKL: {
            title += "PARDISO - ";
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->LockSparsityPattern(false);
            solver->SetVerbose(verbose_solver);
            system.SetSolver(solver);
            break;
        }
    }

    title += use_jacobians ? " Jac YES" : "Jac NO";

    // Simulation loop
    while (vis->Run()) {
        
        double time = system.GetChTime();

        if (verbose_integrator || verbose_solver) {
            std::cout << time << " ---------------- " << std::endl;
        }
        
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        system.DoStepDynamics(step_size);

        double spring_vel = spring->GetVelocity();
        double pos = body->GetPos().y();
        double vel = body->GetPosDt().y();
        ChVectorDynamic<> state = spring->GetStates();
        log << time << ", " << spring_vel << ", " << state(0) << ", " << state(1) << ", ";
        log << pos << ", " << vel;
        log << "\n";

        if (system.GetChTime() >= 0.4)
            break;
    }

#ifdef CHRONO_POSTPROCESS
    // Plot results
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    std::string otitle = "ODE solution - " + title;
    mplot.OutputWindow(0);
    mplot.SetTitle(otitle.c_str());
    mplot.SetLabelX("t");
    mplot.SetLabelY("velocity / state 1");
    mplot.SetCommand("set y2label 'state 2'");
    // mplot.SetCommand("set ytics nomirror");
    // mplot.SetCommand("set y2range [-1:1]");
    // mplot.SetCommand("set y2tics -1, 0.25");
    mplot.Plot(logfile.c_str(), 1, 2, "velocity", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 3, "state 1", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 4, "state 2", " with lines lw 2 axis x1y2");

    std::string btitle = "BODY states - " + title;
    mplot.OutputWindow(1);
    //mplot.SetGrid();
    mplot.SetTitle(btitle.c_str());
    mplot.SetLabelX("t");
    mplot.SetLabelY("y pos");
    mplot.SetCommand("set y2label 'y vel'");
    mplot.SetCommand("set ytics nomirror");
    mplot.SetCommand("set y2range [-1.5:2.5]");
    mplot.SetCommand("set y2tics -1.5, 0.5");
    mplot.Plot(logfile.c_str(), 1, 5, "y pos", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 6, "y vel", " with lines lw 2 axis x1y2");
#endif

    return 0;
}
