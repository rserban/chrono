
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
// Chrono::Multicore test for box-box collision
//
// The global reference frame has Y up.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;

ChContactMethod method = ChContactMethod::SMC;
bool report_contacts = false;
bool report_aabb = true;

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
        printf("  ------------\n");
        if (modA == m_box.get()) {
            printf("  box:    %6.3f  %6.3f  %6.3f\n", pA.x(), pA.y(), pA.z());
            printf("  other:  %6.3f  %6.3f  %6.3f\n", pB.x(), pB.y(), pB.z());
        } else if (modB == m_box.get()) {
            printf("  other:  %6.3f  %6.3f  %6.3f\n", pA.x(), pA.y(), pA.z());
            printf("  box:    %6.3f  %6.3f  %6.3f\n", pB.x(), pB.y(), pB.z());
        }
        ChVector3d nrm = plane_coord.GetAxisX();
        printf("  dist:   %6.4f\n", distance);
        printf("  normal: %6.3f  %6.3f  %6.3f\n", nrm.x(), nrm.y(), nrm.z());

        return true;
    }

    std::shared_ptr<ChBody> m_box;
};

// -------------------------------------------------------
// Access contact information directly in the data manager
// -------------------------------------------------------
void ReportContacts(ChSystemMulticore& system, unsigned int id) {
    printf("  ------------\n");
    printf("  Total number contacts: %d\n", system.GetNumContacts());

    auto& bb = system.data_manager->cd_data->bids_rigid_rigid;
    auto& p1 = system.data_manager->cd_data->cpta_rigid_rigid;
    auto& p2 = system.data_manager->cd_data->cptb_rigid_rigid;

    for (uint i = 0; i < system.data_manager->cd_data->num_rigid_contacts; i++) {
        // IDs of bodies in contact
        int b1 = bb[i].x;
        int b2 = bb[i].y;

        if (id == b1) {
            printf("  %6.3f  %6.3f  %6.3f\n", p1[i].x, p1[i].y, p1[i].z);
        } else if (id == b2) {
            printf("   %6.3f  %6.3f  %6.3f\n", p2[i].x, p2[i].y, p2[i].z);
        }
    }
}

// -------------------------------------------------------
// Access AABB information directly in the data manager
// -------------------------------------------------------
void ReportShapeAABB(ChSystemMulticore& system) {
    printf("  ------------\n");

    auto& aabb_min = system.data_manager->cd_data->aabb_min;
    auto& aabb_max = system.data_manager->cd_data->aabb_max;
    auto& id_rigid = system.data_manager->cd_data->shape_data.id_rigid;
    auto& offset = system.data_manager->measures.collision.global_origin;

    printf("  AABB offset: %6.3f %6.3f %6.3f\n", offset.x, offset.y, offset.z);
    printf("  Number rigid shapes: %d\n", system.data_manager->cd_data->num_rigid_shapes);

    for (uint i = 0; i < system.data_manager->cd_data->num_rigid_shapes; i++) {
        auto min = aabb_min[i] + offset;
        auto max = aabb_max[i] + offset;
        printf("  shape %d (on body %d)   AABB (%6.3f %6.3f %6.3f) - (%6.3f %6.3f %6.3f)\n",  //
               i, id_rigid[i], min.x, min.y, min.z, max.x, max.y, max.z);
    }
}

// -------------------------------------------------------
// Report body AABB
// -------------------------------------------------------
void ReportBodyAABB(ChSystemMulticore& system) {
    printf("  ------------\n");
    printf("  Number rigid bodies: %d\n", system.GetNumBodies());

    for (auto b : system.GetBodies()) {
        auto aabb = b->GetCollisionModel()->GetBoundingBox();
        printf("  body %d   AABB (%6.3f %6.3f %6.3f) - (%6.3f %6.3f %6.3f)\n",  //
               b->GetIdentifier(), aabb.min.x(), aabb.min.y(), aabb.min.z(), aabb.max.x(), aabb.max.y(), aabb.max.z());
    }
}

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
    // ----------------
    // Parameters
    // ----------------

    double time_end = 400;
    double time_step = 1e-3;

    double tolerance = 0;
    double contact_recovery_speed = 1e8;
    double collision_envelope = .001;

    uint max_iteration_normal = 0;
    uint max_iteration_sliding = 0;
    uint max_iteration_spinning = 100;
    uint max_iteration_bilateral = 0;

    // ---------------------------
    // Create the multicore system
    // ---------------------------

    ChSystemMulticore* sys = nullptr;
    std::shared_ptr<ChContactMaterial> contact_mat;

    switch (method) {
        case chrono::ChContactMethod::NSC: {
            auto sysNSC = new ChSystemMulticoreNSC;
            sysNSC->ChangeSolverType(SolverType::APGD);
            sysNSC->GetSettings()->solver.solver_mode = SolverMode::SPINNING;
            sysNSC->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
            sysNSC->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
            sysNSC->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
            sysNSC->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
            sysNSC->GetSettings()->solver.alpha = 0;
            sysNSC->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
            sys = sysNSC;
            auto materialNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            materialNSC->SetFriction(0.4f);
            contact_mat = materialNSC;
            time_step = 1e-3;
            collision_envelope = 0.001;
            break;
        }
        case chrono::ChContactMethod::SMC: {
            auto sysSMC = new ChSystemMulticoreSMC;
            sysSMC->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
            sysSMC->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
            sys = sysSMC;
            auto materialSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            materialSMC->SetFriction(0.4f);
            materialSMC->SetYoungModulus(1e7f);
            materialSMC->SetRestitution(0.1f);
            contact_mat = materialSMC;
            time_step = 1e-4;
            collision_envelope = 0.0;
            break;
        }
    }

    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -10));
    sys->SetNumThreads(1);
    sys->GetSettings()->collision.collision_envelope = collision_envelope;
    sys->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    sys->GetSettings()->collision.bins_per_axis = vec3(1, 1, 1);
    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->solver.tolerance = tolerance;

    // Add plate
    auto plate = chrono_types::make_shared<ChBody>();
    sys->Add(plate);
    plate->SetPos(ChVector3d(0, 0, 0));
    plate->SetFixed(true);
    plate->SetTag(-1);

    plate->EnableCollision(true);
    plate->GetCollisionModel()->SetEnvelope(collision_envelope);
    utils::AddBoxGeometry(plate.get(), contact_mat, ChVector3d(10, 10, 1), ChVector3d(0, 0, -5));

    plate->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.2f));

    // -----------------------------------
    // Add boxes (1: bottom fixed, 2: top)
    // -----------------------------------

    // Corner on face

    ChVector3d hdims1(1.0, 1.0, 1.0);
    ChVector3d pos1(0.0, 0.0, 0.0);
    ChQuaternion<> rot1(1, 0, 0, 0);

    ChVector3d hdims2(1.0, 1.0, 1.0);
    ChVector3d pos2(0, 0, 1.0 + sqrt(3.0));
    ChQuaternion<> rot2 = QuatFromAngleAxis(atan(sqrt(2.0)), ChVector3d(1, 1, 0).GetNormalized());

    // Corner on corner

    ////ChVector3d hdims1(1.0, 1.0, 1.0);
    ////ChVector3d pos1(0.0, 0.0, 0.0);
    ////ChQuaternion<> rot1 = QuatFromAngleAxis(atan(sqrt(2.0)), ChVector3d(1, 1, 0).GetNormalized());

    ////ChVector3d hdims2(1.0, 1.0, 1.0);
    ////ChVector3d pos2(0, 0, sqrt(3.0) + sqrt(3.0));
    ////ChQuaternion<> rot2 = QuatFromAngleAxis(atan(sqrt(2.0)), ChVector3d(1, 1, 0).GetNormalized());



    auto box1 = chrono_types::make_shared<ChBody>();
    box1->SetMass(10);
    box1->SetPos(pos1);
    box1->SetRot(rot1);
    box1->SetInertiaXX(ChVector3d(1, 1, 1));
    box1->SetFixed(true);
    box1->EnableCollision(true);
    box1->GetCollisionModel()->SetEnvelope(collision_envelope);
    utils::AddBoxGeometry(box1.get(), contact_mat, hdims1);
    box1->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.4f));
    sys->AddBody(box1);

    auto box2 = chrono_types::make_shared<ChBody>();
    box2->SetMass(10);
    box2->SetPos(pos2);
    box2->SetRot(rot2);
    box2->SetInertiaXX(ChVector3d(1, 1, 1));
    box2->SetFixed(false);
    box2->EnableCollision(true);
    box2->GetCollisionModel()->SetEnvelope(collision_envelope);
    utils::AddBoxGeometry(box2.get(), contact_mat, hdims2);
    box2->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.4f));
    sys->AddBody(box2);

#ifdef CHRONO_OPENGL
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(sys);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(6, 6, 1), ChVector3d(0, 0, 1));
    vis.SetCameraVertical(CameraVerticalDir::Z);

#endif

    // ---------------
    // Simulate system
    // ---------------

    auto cmanager = chrono_types::make_shared<ContactManager>(box2);

    while (sys->GetChTime() < time_end) {
        // Advance dynamics
        sys->DoStepDynamics(time_step);

        std::cout << "\nTime: " << sys->GetChTime() << std::endl;

        // Process contacts
        if (report_contacts && sys->GetNumContacts() > 0) {
            ReportContacts(*sys, box2->GetIdentifier());
            sys->GetContactContainer()->ReportAllContacts(cmanager);
        }

        // Display AABB information
        if (report_aabb) {
            ReportShapeAABB(*sys);
            ReportBodyAABB(*sys);
        }

#ifdef CHRONO_OPENGL
        if (vis.Run()) {
            vis.Render();
        } else {
            return 1;
        }
#endif
    }

    return 0;
}