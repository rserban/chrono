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
// Chrono test for box-box collision
//
// The global reference frame has Y up.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/physics/ChSystemNSC.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

using namespace chrono;

// -----------------------------------------------------------------------------
// Callback functor for contact reporting
// -----------------------------------------------------------------------------
class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager(std::shared_ptr<ChBody> box) : m_box(box) {}

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
        if (modA == m_box.get()) {
            printf("  %6.3f  %6.3f  %6.3f\n", pA.x(), pA.y(), pA.z());
        } else if (modB == m_box.get()) {
            printf("  %6.3f  %6.3f  %6.3f\n", pB.x(), pB.y(), pB.z());
        }
        return true;
    }

    std::shared_ptr<ChBody> m_box;
};

// --------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // ----------------
    // Parameters
    // ----------------

    double time_end = 4;
    double time_step = 1e-3;

    double tolerance = 0;
    double contact_recovery_speed = 1e8;
    double collision_envelope = .001;

    // -----------------
    // Create the system
    // -----------------

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, -10, 0));

    // Set solver settings
    system.SetSolverType(ChSolver::Type::APGD);
    system.GetSolver()->AsIterative()->SetMaxIterations(100);
    system.GetSolver()->AsIterative()->SetTolerance(tolerance);
    system.SetMaxPenetrationRecoverySpeed(contact_recovery_speed);

    // Shared contact material
    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    contact_mat->SetFriction(0.4f);

    // ----------
    // Add bodies
    // ----------

    auto container = chrono_types::make_shared<ChBody>();
    system.Add(container);
    container->SetPos(ChVector3d(0, 0, 0));
    container->SetFixed(true);
    container->SetTag(-1);

    container->EnableCollision(true);
    container->GetCollisionModel()->SetEnvelope(collision_envelope);
    utils::AddBoxGeometry(container.get(), contact_mat, ChVector3d(4, .5, 4), ChVector3d(0, -.5, 0));

    container->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.2f));

    auto box = chrono_types::make_shared<ChBody>();
    box->SetMass(10);
    box->SetPos(ChVector3d(1, 2, 1));
    box->SetInertiaXX(ChVector3d(1, 1, 1));

    box->EnableCollision(true);
    box->GetCollisionModel()->SetEnvelope(collision_envelope);
    utils::AddBoxGeometry(box.get(), contact_mat, ChVector3d(0.4, 0.2, 0.1));

    box->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.4f));

    system.AddBody(box);

#ifdef CHRONO_IRRLICHT
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Rolling test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(6, 6, -10));
    vis->AttachSystem(&system);

#endif

    // ---------------
    // Simulate system
    // ---------------

    auto cmanager = chrono_types::make_shared<ContactManager>(box);

    while (system.GetChTime() < time_end) {
        // Process contacts
        std::cout << system.GetChTime() << "  " << system.GetNumContacts() << std::endl;
        system.GetContactContainer()->ReportAllContacts(cmanager);

        // Advance dynamics
        system.DoStepDynamics(time_step);

#ifdef CHRONO_IRRLICHT
        // Render scene
        if (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        } else {
            return 1;
        }
#endif
    }

    return 0;
}
