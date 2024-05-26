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
// Test program for Chrono::Multicore with Irrlicht visualization
//
// =============================================================================

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------
// Create the container (fixed to ground).
// -----------------------------------------------------------------------------
void AddContainer(ChSystemMulticoreNSC* sys) {
    // Create a common material
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);

    // Create the containing bin (4 x 4 x 1), tilted about Y
    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetMass(1);
    bin->SetPos(ChVector3d(0, 0, 0));
    bin->SetRot(QuatFromAngleY(CH_PI / 20));
    bin->EnableCollision(true);
    bin->SetFixed(true);

    ChVector3d hdim(2, 2, 0.5);
    double hthick = 0.1;

    utils::AddBoxGeometry(bin.get(), mat, ChVector3d(hdim.x(), hdim.y(), hthick), ChVector3d(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), mat, ChVector3d(hthick, hdim.y(), hdim.z()),
                          ChVector3d(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector3d(hthick, hdim.y(), hdim.z()),
                          ChVector3d(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector3d(hdim.x(), hthick, hdim.z()),
                          ChVector3d(0, -hdim.y() - hthick, hdim.z()), QUNIT, false);
    utils::AddBoxGeometry(bin.get(), mat, ChVector3d(hdim.x(), hthick, hdim.z()),
                          ChVector3d(0, +hdim.y() + hthick, hdim.z()), QUNIT, false);

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChContactMaterialNSC>();
    ballMat->SetFriction(0.4f);

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);

    for (int ix = -2; ix <= 2; ix++) {
        for (int iy = -2; iy <= 2; iy++) {
            ChVector3d pos(0.4 * ix, 0.4 * iy, 1);

            auto ball = chrono_types::make_shared<ChBody>();

            ball->SetMass(mass);
            ball->SetInertiaXX(inertia);
            ball->SetPos(pos);
            ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
            ball->SetFixed(false);
            ball->EnableCollision(true);

            utils::AddSphereGeometry(ball.get(), ballMat, radius);

            sys->AddBody(ball);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create system
    // -------------

    ChSystemMulticoreNSC sys;

    // Set number of threads
    int threads = 8;
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    sys.SetNumThreads(threads);

    // Set gravitational acceleration
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set solver parameters
    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_normal = 0;
    sys.GetSettings()->solver.max_iteration_sliding = 100;
    sys.GetSettings()->solver.max_iteration_spinning = 0;
    sys.GetSettings()->solver.max_iteration_bilateral = 0;
    sys.GetSettings()->solver.tolerance = 1e-3;
    sys.GetSettings()->solver.alpha = 0;
    sys.GetSettings()->solver.contact_recovery_speed = 10000;
    sys.ChangeSolverType(SolverType::APGD);
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    sys.GetSettings()->collision.collision_envelope = 0.01;
    sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Create the fixed and moving bodies
    // ----------------------------------

    AddContainer(&sys);
    AddFallingBalls(&sys);

    // Create Irrlicht application
    // ---------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Multicore + Irrlicht");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, -5, 1));
    vis->AttachSystem(&sys);

    // Perform the simulation
    // ----------------------

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(5e-3);
    }

    return 0;
}
