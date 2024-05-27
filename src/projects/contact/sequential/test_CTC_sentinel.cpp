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
// Test using a collision shape as a "sentinel" body
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// ====================================================================================

class Monitor : public ChCollisionSystem::NarrowphaseCallback {
public:
    Monitor(std::shared_ptr<ChBody> sentinel) : m_sentinel(sentinel) {}
    virtual bool OnNarrowphase(ChCollisionInfo& contactinfo) {
        auto c1 = contactinfo.modelA->GetContactable();
        auto c2 = contactinfo.modelB->GetContactable();
        if (c1 == m_sentinel.get() || c2 == m_sentinel.get()) {
            std::cout << "Collision" << std::endl;
            return false;
        }
        return true;
    }
private:
    std::shared_ptr<ChBody> m_sentinel;
};

// ====================================================================================

int main(int argc, char* argv[]) {
    // Create the system
    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0, -10, 0));

    // Shared contact material
    auto material = chrono_types::make_shared<ChContactMaterialSMC>();

    // Create the sentinel sphere
    auto sentinel = chrono_types::make_shared<ChBody>();

    sentinel->SetTag(0);
    sentinel->SetMass(1);
    sentinel->SetPos(ChVector3d(0, 4, 0));
    sentinel->EnableCollision(true);
    sentinel->SetFixed(true);

    sentinel->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeSphere>(material, 0.2));

    auto sphereS = chrono_types::make_shared<ChVisualShapeSphere>(0.2);
    sentinel->AddVisualShape(sphereS);

    system.AddBody(sentinel);

    // Create a falling ball
    double mass = 100;
    double radius = 1;

    auto ball = chrono_types::make_shared<ChBody>();

    ball->SetTag(1);
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
    ball->SetPos(ChVector3d(0.9, 1.1, 0));
    ball->EnableCollision(true);
    ball->SetFixed(false);

    ball->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeSphere>(material, radius));

    auto sphereB = chrono_types::make_shared<ChVisualShapeSphere>(radius);
    sphereB->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddVisualShape(sphereB);

    system.AddBody(ball);

    // Create ground
    double width = 4;
    double length = 4;
    double thickness = 0.2;

    auto ground = chrono_types::make_shared<ChBody>();

    ground->SetTag(-1);
    ground->SetMass(1);
    ground->SetPos(ChVector3d(0, 0, 0));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->EnableCollision(true);
    ground->SetFixed(true);

    ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(material, width, thickness, length),
                              ChVector3d(0, -thickness, 0));

    auto box = chrono_types::make_shared<ChVisualShapeBox>(width, thickness, length);
    ground->AddVisualShape(box, ChFrame<>(ChVector3d(0, -thickness, 0)));

    system.AddBody(ground);

    // Create the Irrlicht visualization
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("DEM demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 3, -6));
    vis->AttachSystem(&system);

    // Custom callback
    system.GetCollisionSystem()->RegisterNarrowphaseCallback(chrono_types::make_shared<Monitor>(sentinel));

    // Simulation loop
    double time_step = 1e-3;
    double speed = -2;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        auto crt_pos = sentinel->GetPos();
        crt_pos.y() += speed * time_step;
        sentinel->SetPos(crt_pos);

        system.DoStepDynamics(time_step);
    }

    return 0;
}
