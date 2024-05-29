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
// Simple example demonstrating the use of a ChLinkTSDA with states.
//
// Currently not working (states of ChLinkTSDA not yet included in integration)
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/physics/ChBody.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// =============================================================================

double rest_length = 1.5;
double spring_coef = 50;
double damping_coef = 1;

// =============================================================================

// Functor class implementing the force for a ChLinkTSDA.
class MySpringForce : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double rest_length,     // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
                            ) override {
        // Access current states.
        ////ChVectorDynamic<> states = link->GetStates();
        ////std::cout << "t = " << time << "  " << states(0) << " " << states(1) << std::endl;

        double force = -spring_coef * (length - rest_length) - damping_coef * vel;
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

    ChSystemMulticoreNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create the ground body with two visualization spheres
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    ground->AddVisualShape(sph, ChFrame<>(ChVector3d(0, 0, 0)));

    // Create a body suspended through a ChLinkTSDA
    auto body = chrono_types::make_shared<ChBody>();
    system.AddBody(body);
    body->SetPos(ChVector3d(0, -3, 0));
    body->SetFixed(false);
    body->EnableCollision(false);
    body->SetMass(1);
    body->SetInertiaXX(ChVector3d(1, 1, 1));

    auto box = chrono_types::make_shared<ChVisualShapeBox>(2, 2, 2);
    box->SetColor(ChColor(0.6f, 0, 0));
    body->AddVisualShape(box);

    // Create the spring between body and ground. The spring end points are specified in the body relative frames.
    auto force = chrono_types::make_shared<MySpringForce>();
    MySpringRHS rhs;

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body, ground, true, ChVector3d(0, 0, 0), ChVector3d(0, 0, 0));
    spring->SetRestLength(rest_length);
    spring->RegisterForceFunctor(force);
    spring->RegisterODE(&rhs);
    system.AddLink(spring);
    auto spring_shape = chrono_types::make_shared<ChVisualShapeSpring>(0.05, 80, 15);
    spring_shape->SetColor(ChColor(0.5f, 0.5f, 0.5f));
    spring->AddVisualShape(spring_shape);

    // Create the visualization window
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&system);
    vis.SetWindowTitle("Active spring");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, 0, 5), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Y);

    // Create output directory and log file
    const std::string out_dir = GetChronoOutputPath() + "DEMO_ACTIVE_SPRING";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string logfile = out_dir + "/log_par.dat";
    std::ofstream log(logfile);

    // Simulation loop
    while (vis.Run()) {
        ChVectorDynamic<> state = spring->GetStates();
        log << system.GetChTime() << ", " << state(0) << ", " << state(1) << "\n";
        system.DoStepDynamics(0.001);
        vis.Render();
    }

#ifdef CHRONO_POSTPROCESS
    // Plot results
    std::string gplfile = out_dir + "/tmp_par.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    mplot.SetGrid();
    mplot.SetTitle("ODE solution");
    mplot.SetLabelX("t");
    mplot.SetLabelY("y");
    mplot.SetCommand("set ytics nomirror");
    mplot.SetCommand("set y2range [-1:1]");
    mplot.SetCommand("set y2tics -1, 0.25");
    mplot.Plot(logfile.c_str(), 1, 2, "state 1", " with lines lw 2 axis x1y1");
    mplot.Plot(logfile.c_str(), 1, 3, "state 2", " with lines lw 2 axis x1y2");
#endif

    return 0;
}
