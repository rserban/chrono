
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
// Authors: Hammad Mazhar
// =============================================================================
//
// Chrono::Multicore test program for rolling friction
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

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
    // ----------------
    // Parameters
    // ----------------
    double radius = 0.5;
    double density = 1000;
    double mass = density * (4.0 / 3.0) * CH_PI * pow(radius, 3);
    double inertia = (2.0 / 5.0) * mass * pow(radius, 2);
    double initial_angspeed = 10;
    double initial_linspeed = initial_angspeed * radius;

    float sliding_friction = 0.1f;
    float rolling_friction = 0.1f;

    double time_step = 1e-3;

    double tolerance = 0;
    double contact_recovery_speed = 1e8;
    double collision_envelope = .05 * radius;

    uint max_iteration_normal = 0;
    uint max_iteration_sliding = 0;
    uint max_iteration_spinning = 100;
    uint max_iteration_bilateral = 0;
    
    // -------------------------
    // Create the multicore system
    // ---------------------------

    ChSystemMulticoreNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, -10, 0));

    // Set number of threads
    system.SetNumThreads(1);

    // Set solver settings
    system.ChangeSolverType(SolverType::APGD);

    system.GetSettings()->solver.solver_mode = SolverMode::SPINNING;
    system.GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system.GetSettings()->solver.alpha = 0;
    system.GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system.GetSettings()->solver.use_full_inertia_tensor = false;
    system.GetSettings()->solver.tolerance = tolerance;

    system.GetSettings()->collision.collision_envelope = collision_envelope;
    system.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::PRIMS;
    system.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // ----------
    // Add bodies
    // ----------

    // Shared contact material
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction(sliding_friction);
    material->SetRollingFriction(rolling_friction);

    auto container = chrono_types::make_shared<ChBody>();
    system.Add(container);
    container->SetPos(ChVector3d(0, 0, 0));
    container->SetFixed(true);
    container->SetTag(-1);

    container->EnableCollision(true);
    utils::AddBoxGeometry(container.get(), material, ChVector3d(20, .5, 20), ChVector3d(0, -.5, 0));

    container->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.2f));

    auto ball = chrono_types::make_shared<ChBody>();
    ChVector3d pos = ChVector3d(0, radius, 0);
    ChVector3d vel = ChVector3d(initial_linspeed, 0, 0);
    ChVector3d wvel = ChVector3d(0, 0, -initial_angspeed);
    ball->SetMass(mass);
    ball->SetPos(pos);
    ball->SetPosDt(vel);
    ball->SetAngVelParent(wvel);
    ball->SetInertiaXX(ChVector3d(inertia));

    ball->EnableCollision(true);
    utils::AddSphereGeometry(ball.get(), material, radius);

    ball->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.4f));

    system.AddBody(ball);

#ifdef CHRONO_OPENGL
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&system);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(10, 10, 20), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

#endif

    // ---------------
    // Simulate system
    // ---------------

    double time_end = 20.0;
    double time_out = 2.5;
    bool output = false;

    while (system.GetChTime() < time_end) {
        system.DoStepDynamics(time_step);

        auto pos = ball->GetPos();
        printf("T: %f  Pos: %f %f %f\n", system.GetChTime(), pos.x(), pos.y(), pos.z());

        //if (!output && system.GetChTime() >= time_out) {
        //    for (int i = 1; i <= 10; i++) {
        //        auto pos = system.GetBodies()->at(i)->GetPos();
        //        std::cout << pos.x() << std::endl;
        //    }
        //    output = true;
        //}
		
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