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
// Authors: Radu Serban
// =============================================================================
//
// Chrono test for using collision families
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;

bool ball_collision = false;

int main(int argc, char* argv[]) {
    // Create system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -1, 0));

    // Shared contact material (default properties)
    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);

    // Lower ball
    auto ball_lower = chrono_types::make_shared<ChBody>();
    ball_lower->SetTag(1);
    ball_lower->SetMass(mass);
    ball_lower->SetInertiaXX(inertia);
    ball_lower->SetPos(ChVector3d(0, 2, 0));
    ball_lower->SetFixed(false);
    ball_lower->EnableCollision(true);

    ball_lower->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.0f, 0.0f));

    ball_lower->GetCollisionModel()->SetFamily(3);
    if (!ball_collision) {
        ball_lower->GetCollisionModel()->DisallowCollisionsWith(3);
    }
    utils::AddSphereGeometry(ball_lower.get(), contact_mat, radius);

    sys.AddBody(ball_lower);

    // Upper ball
    auto ball_upper = chrono_types::make_shared<ChBody>();
    ball_upper->SetTag(2);
    ball_upper->SetMass(mass);
    ball_upper->SetInertiaXX(inertia);
    ball_upper->SetPos(ChVector3d(radius, 3, 0));
    ball_upper->SetPosDt(ChVector3d(0, -1, 0));
    ball_upper->SetFixed(false);
    ball_upper->EnableCollision(true);

    ball_upper->GetVisualShape(0)->SetColor(ChColor(0.0f, 1.0f, 0.0f));

    ball_upper->GetCollisionModel()->SetFamily(3);
    if (!ball_collision) {
        ball_upper->GetCollisionModel()->DisallowCollisionsWith(3);
    }
    utils::AddSphereGeometry(ball_upper.get(), contact_mat, radius);

    sys.AddBody(ball_upper);

    // Plate
    auto plate = chrono_types::make_shared<ChBody>();
    plate->SetTag(0);
    plate->SetPos(ChVector3d(0, 0, 0));
    plate->SetFixed(true);
    plate->EnableCollision(true);

    plate->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.5f));

    utils::AddBoxGeometry(plate.get(), contact_mat, ChVector3d(10 * radius, radius, 10 * radius));

    sys.AddBody(plate);

    // Create the visualization window
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Collision family");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 2, -4));
    vis->AttachSystem(&sys);

    for (const auto& body : sys.GetBodies()) {
        std::cout << "Body " << body->GetTag() << "  family: " << body->GetCollisionModel()->GetFamily()
                  << "  family mask: " << body->GetCollisionModel()->GetFamilyMask() << std::endl;
    }

    // Run simulation for specified time
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(1e-3);
    }

    return 0;
}
