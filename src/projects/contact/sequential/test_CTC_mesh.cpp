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
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// ====================================================================================

class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager() {}
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector3d& cforce,
                                 const ChVector3d& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override;
};

// ====================================================================================

class CustomCollision : public ChSystem::CustomCollisionCallback {
  public:
    CustomCollision(std::shared_ptr<ChBody> ground,
                    std::shared_ptr<ChBody> object,
                    std::shared_ptr<ChContactMaterial> ground_mat,
                    std::shared_ptr<ChContactMaterial> object_mat,
                    double radius,
                    double len);
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    std::shared_ptr<ChBody> m_ground;
    std::shared_ptr<ChBody> m_object;
    std::shared_ptr<ChContactMaterial> m_ground_mat;
    std::shared_ptr<ChContactMaterial> m_object_mat;
    double m_radius;
    double m_len;
};

// ====================================================================================

std::shared_ptr<ChTriangleMeshConnected> GroundMesh(double hx, double hy);

// ====================================================================================

int main(int argc, char* argv[]) {
    // ------------------
    // Collision settings
    // ------------------

    enum class CollisionType { PRIMITIVE, MESH };
    CollisionType object_model = CollisionType::MESH;
    CollisionType ground_model = CollisionType::PRIMITIVE;

    std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_fine.obj");
    ////std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_coarse.obj");
    double mesh_swept_sphere_radius = 0.005;

    bool use_custom_collision = false;
    if (use_custom_collision)
        object_model = CollisionType::PRIMITIVE;

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
    double init_roll = 0 * CH_DEG_TO_RAD;

    ChVector3d init_vel(0, 0, 0);
    ChVector3d init_omg(0, 0, 0);

    double radius = 0.5;  // cylinder radius
    double len = 0.4;     // cylinder length

    // ---------
    // Step size
    // ---------

    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-4;

    // --------------
    // Print settings
    // --------------

    std::cout << "-----------------------" << std::endl;
    std::cout << "Ground collision type: " << (ground_model == CollisionType::PRIMITIVE ? "BOX" : "MESH") << std::endl;
    std::cout << "Object collision type: " << (object_model == CollisionType::PRIMITIVE ? "CYL" : "MESH") << std::endl;
    std::cout << "Contact method:        " << (contact_method == ChContactMethod::SMC ? "SMC" : "NSC") << std::endl;
    std::cout << "Collision system:      " << (use_custom_collision ? "Custom" : "Bullet") << std::endl;
    std::cout << "Step size:             " << time_step << std::endl;
    std::cout << "-----------------------" << std::endl;

    // -----------------
    // Create the system
    // -----------------

    ChSystem* system;

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto systemNSC = new ChSystemNSC;
            system = systemNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto systemSMC = new ChSystemSMC;
            systemSMC->UseMaterialProperties(use_mat_properties);
            system = systemSMC;
            break;
        }
    }

    system->SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));


    // Rotation Z->Y (because meshes used here assume Z up)
    ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

    // Create the falling object
    auto object = chrono_types::make_shared<ChBody>();
    system->AddBody(object);

    object->SetName("object");
    object->SetMass(200);
    object->SetInertiaXX(40.0 * ChVector3d(1, 1, 0.2));
    object->SetPos(ChVector3d(0, init_height, 0));
    object->SetRot(z2y * QuatFromAngleX(init_roll));
    object->SetPosDt(init_vel);
    object->SetAngVelParent(init_omg);
    object->EnableCollision(true);
    object->SetFixed(false);

    auto object_mat = ChContactMaterial::DefaultMaterial(contact_method);
    object_mat->SetFriction(object_friction);
    object_mat->SetRestitution(object_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChContactMaterialSMC>(object_mat);
        matSMC->SetYoungModulus(object_young_modulus);
        matSMC->SetPoissonRatio(object_poisson_ratio);
        matSMC->SetKn(object_kn);
        matSMC->SetGn(object_gn);
        matSMC->SetKt(object_kt);
        matSMC->SetGt(object_gt);
    }

    switch (object_model) {
        case CollisionType::PRIMITIVE: {
            object->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCylinder>(object_mat, radius, len),
                                      ChFramed(ChVector3d(0), ChMatrix33<>(1)));

            auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(radius, len);
            cyl->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            object->AddVisualShape(cyl, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

            break;
        }
        case CollisionType::MESH: {
            auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            if (!trimesh->LoadWavefrontMesh(tire_mesh_file, true, false))
                return 1;

            object->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                object_mat, trimesh, false, false, mesh_swept_sphere_radius));

            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetWireframe(true);
            trimesh_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            object->AddVisualShape(trimesh_shape);

            break;
        }
    }

    // Create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    system->AddBody(ground);

    ground->SetName("ground");
    ground->SetMass(1);
    ground->SetPos(ChVector3d(0, 0, 0));
    ground->SetRot(z2y);
    ground->EnableCollision(true);
    ground->SetFixed(true);

    auto ground_mat = ChContactMaterial::DefaultMaterial(contact_method);
    ground_mat->SetFriction(ground_friction);
    ground_mat->SetRestitution(ground_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChContactMaterialSMC>(ground_mat);
        matSMC->SetYoungModulus(ground_young_modulus);
        matSMC->SetPoissonRatio(ground_poisson_ratio);
        matSMC->SetKn(ground_kn);
        matSMC->SetGn(ground_gn);
        matSMC->SetKt(ground_kt);
        matSMC->SetGt(ground_gt);
    }

    double hx = 8;
    double hy = 8;
    double hz = 0.2;

    switch (ground_model) {
        case CollisionType::PRIMITIVE: {
            ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(ground_mat, hx, hy, hz),
                                      ChVector3d(0, 0, -hz));

            auto box = chrono_types::make_shared<ChVisualShapeBox>(hx, hy, hz);
            box->SetTexture(GetChronoDataFile("textures/checker1.png"), 4, 3);
            ground->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -hz)));

            break;
        }
        case CollisionType::MESH: {
            auto trimesh = GroundMesh(hx, hy);

            ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                ground_mat, trimesh, false, false, mesh_swept_sphere_radius));

            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetTexture(GetChronoDataFile("textures/checker2.png"), 1, 1);
            ground->AddVisualShape(trimesh_shape);

            break;
        }
    }

    // Create the Irrlicht visualization
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Collision test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0.5, 1, -2));
    vis->SetSymbolScale(5e-3);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    vis->AttachSystem(system);

    auto cmanager = chrono_types::make_shared<ContactManager>();

    if (use_custom_collision) {
        auto ccollision =
            chrono_types::make_shared<CustomCollision>(ground, object, ground_mat, object_mat, radius, len);
        system->RegisterCustomCollisionCallback(ccollision);
    }

    // ---------------
    // Simulation loop
    // ---------------
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        system->DoStepDynamics(time_step);

        ////std::cout << system->GetChTime() << "  " << system->GetNumContacts() << std::endl;
        ////system->GetContactContainer()->ReportAllContacts(cmanager);
    }

    return 0;
}

// ====================================================================================

std::shared_ptr<ChTriangleMeshConnected> GroundMesh(double hx, double hy) {
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    std::vector<ChVector3d>& v = trimesh->GetCoordsVertices();
    std::vector<ChVector3d>& n = trimesh->GetCoordsNormals();
    std::vector<ChVector2<>>& uv = trimesh->GetCoordsUV();
    std::vector<ChVector3<int>>& iv = trimesh->GetIndicesVertexes();
    std::vector<ChVector3<int>>& in = trimesh->GetIndicesNormals();

    v.resize(4);
    n.resize(4);
    uv.resize(4);

    iv.resize(2);
    in.resize(2);

    v[0] = ChVector3d(+hx, +hy, 0);
    v[1] = ChVector3d(-hx, +hy, 0);
    v[2] = ChVector3d(-hx, -hy, 0);
    v[3] = ChVector3d(+hx, -hy, 0);

    n[0] = ChVector3d(0, 0, 1);
    n[1] = ChVector3d(0, 0, 1);
    n[2] = ChVector3d(0, 0, 1);
    n[3] = ChVector3d(0, 0, 1);

    uv[0] = ChVector2<>(1, 1);
    uv[1] = ChVector2<>(0, 1);
    uv[2] = ChVector2<>(0, 0);
    uv[3] = ChVector2<>(1, 0);

    iv[0] = ChVector3<int>(0, 1, 2);
    iv[1] = ChVector3<int>(0, 2, 3);

    in[0] = ChVector3<int>(0, 1, 2);
    in[1] = ChVector3<int>(0, 2, 3);

    return trimesh;
}

// ====================================================================================

bool ContactManager::OnReportContact(const ChVector3d& pA,
                                     const ChVector3d& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const double& eff_radius,
                                     const ChVector3d& cforce,
                                     const ChVector3d& ctorque,
                                     ChContactable* modA,
                                     ChContactable* modB) {
    auto bodyA = static_cast<ChBody*>(modA);
    auto bodyB = static_cast<ChBody*>(modB);

    std::cout << "  " << bodyA->GetName() << "  " << bodyB->GetName() << std::endl;
    std::cout << "  " << pA << "    " << pB << std::endl;
    std::cout << "  " << plane_coord.GetAxisX() << std::endl;
    std::cout << std::endl;

    return true;
}

CustomCollision::CustomCollision(std::shared_ptr<ChBody> ground,
                                 std::shared_ptr<ChBody> object,
                                 std::shared_ptr<ChContactMaterial> ground_mat,
                                 std::shared_ptr<ChContactMaterial> object_mat,
                                 double radius,
                                 double len)
    : m_ground(ground),
      m_object(object),
      m_ground_mat(ground_mat),
      m_object_mat(object_mat),
      m_radius(radius),
      m_len(len) {
    // Disable Chrono collision detection for ground and object
    m_ground->EnableCollision(false);
    m_object->EnableCollision(false);
}

// Assumptions:
// - object with cylindrical shape
// - ground height at 0
// - collision with cylinder circumference
void CustomCollision::OnCustomCollision(ChSystem* system) {
    // Lowest point on central cylinder plane (in absolute frame)
    ChVector3d A = m_object->GetPos() - m_radius * ChVector3d(0, 1, 0);
    // Express in local frame
    ChVector3d A_loc = m_object->TransformPointParentToLocal(A);
    // Points on cylinder edges (local frame)
    ChVector3d B1_loc = A_loc + ChVector3d(0, -m_len / 2, 0);
    ChVector3d B2_loc = A_loc + ChVector3d(0, +m_len / 2, 0);
    // Express in absolute frame
    ChVector3d B1 = m_object->TransformPointLocalToParent(B1_loc);
    ChVector3d B2 = m_object->TransformPointLocalToParent(B2_loc);

    // If B1 below ground surface (assumed at 0), add contact
    if (B1.y() < 0) {
        ChCollisionInfo contact;
        contact.modelA = m_ground->GetCollisionModel().get();
        contact.modelB = m_object->GetCollisionModel().get();
        contact.shapeA = nullptr;
        contact.shapeB = nullptr;
        contact.vN = ChVector3d(0, 1, 0);
        contact.vpA = B1;
        contact.vpB = ChVector3d(B1.x(), 0, B1.z());
        contact.distance = B1.y();
        system->GetContactContainer()->AddContact(contact, m_ground_mat, m_object_mat);
    }

    // If B2 below ground surface (assumed at 0), add contact
    if (B2.y() < 0) {
        ChCollisionInfo contact;
        contact.modelA = m_ground->GetCollisionModel().get();
        contact.modelB = m_object->GetCollisionModel().get();
        contact.shapeA = nullptr;
        contact.shapeB = nullptr;
        contact.vN = ChVector3d(0, 1, 0);
        contact.vpA = B2;
        contact.vpB = ChVector3d(B2.x(), 0, B2.z());
        contact.distance = B2.y();
        system->GetContactContainer()->AddContact(contact, m_ground_mat, m_object_mat);
    }
}
