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
// Simple example demonstrating equilibrium analyis.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create bodies.
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto body1 = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.1, 1.0, 1000);
    body1->SetPos(ChVector3d(0.5, 0, 0));
    body1->SetRot(Q_ROTATE_Y_TO_X);
    body1->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    system.AddBody(body1);

    auto body2 = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.1, 1.0, 1000);
    body2->SetPos(ChVector3d(2.5, 0, 0));
    body2->SetRot(Q_ROTATE_Y_TO_X);
    body2->GetVisualShape(0)->SetColor(ChColor(0.0f, 1.0f, 0.0f));
    system.AddBody(body2);

    auto body3 = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.1, 2.0, 1000);
    body3->SetPos(ChVector3d(2.0, 0, 0));
    body3->SetRot(Q_ROTATE_Y_TO_X);
    body3->GetVisualShape(0)->SetColor(ChColor(0.0f, 0.0f, 1.0f));
    system.AddBody(body3);

    // Add joints.
    auto rev1 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev1->Initialize(ground, body1, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    system.AddLink(rev1);

    auto rev2 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev2->Initialize(ground, body2, ChFrame<>(ChVector3d(2, 0, 0), QUNIT));
    system.AddLink(rev2);

    auto rev13 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev13->Initialize(body1, body3, ChFrame<>(ChVector3d(1, 0, 0), QUNIT));
    system.AddLink(rev13);

    auto rev23 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev23->Initialize(body2, body3, ChFrame<>(ChVector3d(3, 0, 0), QUNIT));
    system.AddLink(rev23);

    // Create the spring between body1 and ground.
    // The spring end points are specified in the body relative frames.
    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(ground, body1, true, ChVector3d(1.5, 1, 0), ChVector3d(0, 0.5, 0));
    spring->SetRestLength(0.5);
    spring->SetSpringCoefficient(2e2);
    spring->SetDampingCoefficient(1e2);
    system.AddLink(spring);

    auto spring_shape = chrono_types::make_shared<ChVisualShapeSpring>(0.05, 80, 15);
    spring_shape->SetColor(ChColor(0.0f, 0.0f, 0.0f));
    spring->AddVisualShape(spring_shape);

    // Perform equilibrium analysis.
    std::cout << "Body1 pos: " << body1->GetPos() << std::endl;
    ChStaticNonLinearAnalysis analysis;
    analysis.SetMaxIterations(150);
    analysis.SetCorrectionTolerance(1e-4, 1e-8);
    ////analysis.SetResidualTolerance(1e-8);
    analysis.SetIncrementalSteps(6);
    analysis.SetVerbose(true);
    system.DoStaticAnalysis(analysis);
    std::cout << "Body1 pos: " << body1->GetPos() << std::endl;
    std::cout << "Reaction force rev1: " << rev1->GetReaction2().force << std::endl;
    std::cout << std::endl;

    // Create Irrlicht window.
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Equilibrium demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 0, 6));
    vis->AttachSystem(&system);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        system.DoStepDynamics(1e-3);
    }

    std::cout << "\nReaction force rev1: " << rev1->GetReaction2().force << "\n" << std::endl;

    return 0;
}
