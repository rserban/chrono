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
// Collision test
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// ====================================================================================

class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager() {}
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override;
};

// ====================================================================================

std::shared_ptr<geometry::ChTriangleMeshConnected> GroundMesh(double hx, double hy);

// ====================================================================================

int main(int argc, char* argv[]) {
    // ------------------
    // Collision settings
    // ------------------

    collision::ChCollisionSystemType collsys_type = collision::ChCollisionSystemType::CHRONO;
    double mesh_swept_sphere_radius = 0.005;

    // ---------------------------
    // Contact material properties
    // ---------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    bool use_mat_properties = true;

    float object_friction = 0.9f;
    float object_restitution = 0.1f;
    float object_young_modulus = 2e7f;
    float object_poisson_ratio = 0.3f;
    float object_adhesion = 0.0f;
    float object_kn = 2e5;
    float object_gn = 40;
    float object_kt = 2e5;
    float object_gt = 20;

    float ground_friction = 0.9f;
    float ground_restitution = 0.01f;
    float ground_young_modulus = 1e6f;
    float ground_poisson_ratio = 0.3f;
    float ground_adhesion = 0.0f;
    float ground_kn = 2e5;
    float ground_gn = 40;
    float ground_kt = 2e5;
    float ground_gt = 20;

    // ---------------------------------
    // Parameters for the falling object
    // ---------------------------------

    double init_height = 0.75;
    double init_roll = 0 * CH_C_DEG_TO_RAD;

    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 0);


    // ---------
    // Step size
    // ---------

    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-4;

    // --------------
    // Print settings
    // --------------

    std::cout << "-----------------------" << std::endl;
    std::cout << "Contact method:        " << (contact_method == ChContactMethod::SMC ? "SMC" : "NSC") << std::endl;
    std::cout << "Step size:             " << time_step << std::endl;
    std::cout << "-----------------------" << std::endl;

    // -----------------
    // Create the system
    // -----------------

    ChSystem* system;

    switch (contact_method) {
        case ChContactMethod::NSC:
            system = new ChSystemNSC();
            break;
        case ChContactMethod::SMC:
            system = new ChSystemSMC(use_mat_properties);
            break;
    }

    system->Set_G_acc(ChVector<>(0, -9.81, 0));
    system->SetCollisionSystemType(collsys_type);

    // Create the Irrlicht visualization
    ChIrrApp application(system, L"Collision test", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0.5, 1, -2), irr::core::vector3df(0, 0, 0));

    // Render contact forces or normals
    application.SetSymbolscale(5e-3);
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_FORCES);
    ////application.SetSymbolscale(1);
    ////application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_NORMALS);

    // Rotation Z->Y (because meshes used here assume Z up)
    ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

    // Create the falling object
    auto object = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(object);

    object->SetName("object");
    object->SetMass(200);
    object->SetInertiaXX(40.0 * ChVector<>(1, 1, 0.2));
    object->SetPos(ChVector<>(0, init_height, 0));
    object->SetRot(z2y * Q_from_AngX(init_roll));
    object->SetPos_dt(init_vel);
    object->SetWvel_par(init_omg);
    object->SetCollide(true);
    object->SetBodyFixed(false);

    auto object_mat = ChMaterialSurface::DefaultMaterial(contact_method);
    object_mat->SetFriction(object_friction);
    object_mat->SetRestitution(object_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChMaterialSurfaceSMC>(object_mat);
        matSMC->SetYoungModulus(object_young_modulus);
        matSMC->SetPoissonRatio(object_poisson_ratio);
        matSMC->SetKn(object_kn);
        matSMC->SetGn(object_gn);
        matSMC->SetKt(object_kt);
        matSMC->SetGt(object_gt);
    }

    ChVector<> box_size(1.0, 0.5, 0.5);
    ChVector<> box_pos(0, 0, 0);
    ChQuaternion<> box_rot(1, 0, 0, 0);

            object->GetCollisionModel()->ClearModel();
            utils::AddBoxGeometry(object.get(), object_mat, box_size, box_pos, box_rot);
            object->GetCollisionModel()->BuildModel();

    object->AddAsset(chrono_types::make_shared<ChColorAsset>(0.3f, 0.3f, 0.3f));

    // Create ground body
    auto ground = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(ground);

    ground->SetName("ground");
    ground->SetMass(1);
    ground->SetPos(ChVector<>(0, 0, 0));
    ground->SetRot(z2y);
    ground->SetCollide(true);
    ground->SetBodyFixed(true);

    auto ground_mat = ChMaterialSurface::DefaultMaterial(contact_method);
    ground_mat->SetFriction(ground_friction);
    ground_mat->SetRestitution(ground_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat);
        matSMC->SetYoungModulus(ground_young_modulus);
        matSMC->SetPoissonRatio(ground_poisson_ratio);
        matSMC->SetKn(ground_kn);
        matSMC->SetGn(ground_gn);
        matSMC->SetKt(ground_kt);
        matSMC->SetGt(ground_gt);
    }

    double hx = 4;
    double hy = 4;

            auto trimesh = GroundMesh(hx, hy);

            ground->GetCollisionModel()->ClearModel();
            ground->GetCollisionModel()->AddTriangleMesh(ground_mat, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                         mesh_swept_sphere_radius);
            ground->GetCollisionModel()->BuildModel();

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            ground->AddAsset(trimesh_shape);

            auto texture = chrono_types::make_shared<ChTexture>();
            texture->SetTextureFilename(GetChronoDataFile("textures/checker2.png"));
            texture->SetTextureScale(1, 1);
            ground->AddAsset(texture);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    auto cmanager = chrono_types::make_shared<ContactManager>();

    // ---------------
    // Simulation loop
    // ---------------
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.EndScene();

        system->DoStepDynamics(time_step);

        ////std::cout << system->GetChTime() << "  " << system->GetNcontacts() << std::endl;
        ////system->GetContactContainer()->ReportAllContacts(cmanager);
    }

    return 0;
}

// ====================================================================================

std::shared_ptr<geometry::ChTriangleMeshConnected> GroundMesh(double hx, double hy) {
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    std::vector<ChVector<>>& v = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& n = trimesh->getCoordsNormals();
    std::vector<ChVector<>>& uv = trimesh->getCoordsUV();
    std::vector<ChVector<int>>& iv = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& in = trimesh->getIndicesNormals();

    v.resize(4);
    n.resize(4);
    uv.resize(4);

    iv.resize(2);
    in.resize(2);

    v[0] = ChVector<>(+hx, +hy, 0);
    v[1] = ChVector<>(-hx, +hy, 0);
    v[2] = ChVector<>(-hx, -hy, 0);
    v[3] = ChVector<>(+hx, -hy, 0);

    n[0] = ChVector<>(0, 0, 1);
    n[1] = ChVector<>(0, 0, 1);
    n[2] = ChVector<>(0, 0, 1);
    n[3] = ChVector<>(0, 0, 1);

    uv[0] = ChVector<>(1, 1, 0);
    uv[1] = ChVector<>(0, 1, 0);
    uv[2] = ChVector<>(0, 0, 0);
    uv[3] = ChVector<>(1, 0, 0);

    iv[0] = ChVector<int>(0, 1, 2);
    iv[1] = ChVector<int>(0, 2, 3);

    in[0] = ChVector<int>(0, 1, 2);
    in[1] = ChVector<int>(0, 2, 3);

    return trimesh;
}

// ====================================================================================

bool ContactManager::OnReportContact(const ChVector<>& pA,
                                     const ChVector<>& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const double& eff_radius,
                                     const ChVector<>& cforce,
                                     const ChVector<>& ctorque,
                                     ChContactable* modA,
                                     ChContactable* modB) {
    auto bodyA = static_cast<ChBody*>(modA);
    auto bodyB = static_cast<ChBody*>(modB);

    std::cout << "  " << bodyA->GetName() << "  " << bodyB->GetName() << std::endl;
    std::cout << "  " << pA << "    " << pB << std::endl;
    std::cout << "  " << plane_coord.Get_A_Xaxis() << std::endl;
    std::cout << std::endl;

    return true;
}
