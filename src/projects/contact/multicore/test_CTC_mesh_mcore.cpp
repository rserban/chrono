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
// Test mesh collision
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;

// ====================================================================================

int main(int argc, char* argv[]) {
    // ---------------------
    // Simulation parameters
    // ---------------------

    double gravity = 9.81;    // gravitational acceleration
    double time_step = 1e-4;  // integration step size

    double tolerance = 0;
    double contact_recovery_speed = 10;
    double collision_envelope = .005;

    uint max_iteration_normal = 0;
    uint max_iteration_sliding = 0;
    uint max_iteration_spinning = 100;
    uint max_iteration_bilateral = 0;
    
    enum class GeometryType { PRIMITIVE, MESH };
    GeometryType ground_geometry = GeometryType::PRIMITIVE;  // box or ramp mesh
    GeometryType object_geometry = GeometryType::MESH;       // sphere or tire mesh

    double mesh_swept_sphere_radius = 0.005;

    // ---------------------------
    // Contact material properties
    // ---------------------------

    ChContactMethod contact_method = ChContactMethod::NSC;
    bool use_mat_properties = true;

    float object_friction = 0.9f;
    float object_restitution = 0.0f;
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

    ChVector3d pos(0.2, 0.55, 0.2);
    ChVector3d init_vel(0, 0, 0);
    ChVector3d init_omg(0, 0, 0);

    // ---------------------------------
    // Parameters for the containing bin
    // ---------------------------------

    double width = 4;
    double length = 2;
    double thickness = 0.2;

    // -----------------
    // Create the system
    // -----------------

    ChSystemMulticore* system;

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto my_sys = new ChSystemMulticoreNSC();
            my_sys->ChangeSolverType(SolverType::APGD);
            my_sys->GetSettings()->solver.solver_mode = SolverMode::SPINNING;
            my_sys->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
            my_sys->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
            my_sys->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
            my_sys->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
            my_sys->GetSettings()->solver.alpha = 0;
            my_sys->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;

            system = my_sys;
            break;
        }
        case ChContactMethod::SMC: {
            auto my_sys = new ChSystemMulticoreSMC();
            my_sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            my_sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::OneStep;

            system = my_sys;
            break;
        }
    }

    system->SetGravitationalAcceleration(ChVector3d(0, -gravity, 0));

    int nthreads = 2;
    int max_threads = omp_get_num_procs();
    if (nthreads > max_threads)
        nthreads = max_threads;
    system->SetNumThreads(nthreads);

    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = tolerance;

    system->GetSettings()->collision.collision_envelope = collision_envelope;
    system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Rotation Z->Y (because meshes used here assume Z up)
    ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

    // Create the falling object
    auto object = chrono_types::make_shared<ChBody>();
    system->AddBody(object);

    object->SetMass(200);
    object->SetInertiaXX(40.0 * ChVector3d(1, 1, 0.2));
    object->SetPos(pos);
    object->SetRot(z2y);
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

    switch (object_geometry) {
        case GeometryType::MESH: {
            auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            trimesh->LoadWavefrontMesh(GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_coarse.obj"), true, false);

            object->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                object_mat, trimesh, false, false, mesh_swept_sphere_radius));

            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName("tire");
            trimesh_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            object->AddVisualShape(trimesh_shape);

            break;
        }
        case GeometryType::PRIMITIVE: {
            object->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeSphere>(object_mat, 0.2),
                                      ChVector3d(0, 0, 0));

            auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(0.2);
            object->AddVisualShape(sphere);

            break;
        }
    }

    // Create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    system->AddBody(ground);

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

    switch (ground_geometry) {
        case GeometryType::PRIMITIVE: {
            ground->AddCollisionShape(
                chrono_types::make_shared<ChCollisionShapeBox>(ground_mat, width, length, thickness),
                ChVector3d(0, 0, -thickness));

            auto box = chrono_types::make_shared<ChVisualShapeBox>(width, length, thickness);
            ground->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -thickness)));

            break;
        }
        case GeometryType::MESH: {
            auto trimesh_ground = chrono_types::make_shared<ChTriangleMeshConnected>();
            trimesh_ground->LoadWavefrontMesh(GetChronoDataFile("vehicle/terrain/meshes/ramp_10x1.obj"), true, false);

            ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                ground_mat, trimesh_ground, false, false, mesh_swept_sphere_radius));

            auto trimesh_ground_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_ground_shape->SetMesh(trimesh_ground);
            trimesh_ground_shape->SetName("ground");
            ground->AddVisualShape(trimesh_ground_shape);

            break;
        }
    }

#ifdef CHRONO_OPENGL
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(system);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(2, 1, 2), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

#endif

    // ---------------
    // Simulation loop
    // ---------------

    double time_end = 4;
    while (system->GetChTime() < time_end) {
        // Advance dynamics
        system->DoStepDynamics(time_step);

        std::cout << "\nTime: " << system->GetChTime() << std::endl;

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
