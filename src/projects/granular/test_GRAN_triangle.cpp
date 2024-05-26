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
//
// Test triangle collision shape in Chrono::Multicore
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

// Note: CHRONO_OPENGL is defined in ChConfig.h
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Problem setup
// -----------------------------------------------------------------------------

// Comment the following line to use NSC contact
#define USE_SMC

ChVector3d initPos(0.1, 0.1, 0.05);
ChQuaternion<> initRot(1.0, 0.0, 0.0, 0.0);
//ChQuaternion<> initRot = QuatFromAngleAxis(CH_PI / 3, ChVector3d(1, 0, 0));

ChVector3d initLinVel(0.0, 0.0, 0.0);
ChVector3d initAngVel(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 10;

// Perform dynamic tuning of number of threads?
bool thread_tuning = true;

// Simulation duration.
double time_end = 10;

// Solver parameters
#ifdef USE_SMC
double time_step = 1e-4;
int max_iteration = 20;
#else
double time_step = 1e-3;
int max_iteration_normal = 30;
int max_iteration_sliding = 20;
int max_iteration_spinning = 0;
float contact_recovery_speed = 0.1;
#endif

// Output
int out_fps = 60;

// =============================================================================
// Create ground body
// =============================================================================
void CreateGround(ChSystemMulticore* system) {

    auto ground = chrono_types::make_shared<ChBody>();

#ifdef USE_SMC
    auto mat_g = chrono_types::make_shared<ChContactMaterialSMC>();
    mat_g->SetYoungModulus(1e7f);
    mat_g->SetFriction(0.7f);
    mat_g->SetRestitution(0.01f);
#else
    auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
    mat_g->SetFriction(0.7f);
#endif

    ground->SetTag(-1);
    ground->SetMass(1);
    ground->SetPos(ChVector3d(0, 0, 0));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetFixed(true);
    ground->EnableCollision(true);

    // Set fixed contact shapes (grid of 10x10 spheres)
    double spacing = 0.6;
    double bigR = 1;
    for (int ix = -5; ix < 5; ix++) {
        for (int iy = -5; iy < 5; iy++) {
            ChVector3d pos(ix * spacing, iy * spacing, -bigR);
            utils::AddSphereGeometry(ground.get(), mat_g, bigR, pos);
        }
    }

    system->AddBody(ground);
}

// =============================================================================
// Create falling object
// =============================================================================
std::shared_ptr<ChBody> CreateObject(ChSystemMulticore* system) {
    double rho_o = 2000.0;

    auto obj = chrono_types::make_shared<ChBody>();

#ifdef USE_SMC
    auto mat_o = chrono_types::make_shared<ChContactMaterialSMC>();
    mat_o->SetYoungModulus(1e7f);
    mat_o->SetFriction(0.7f);
    mat_o->SetRestitution(0.01f);
#else
    auto mat_o = chrono_types::make_shared<ChContactMaterialNSC>();
    mat_o->SetFriction(0.7f);
#endif

    obj->SetTag(1);
    obj->EnableCollision(true);
    obj->SetFixed(false);

    // Mass and inertia
    double mass = 1;
    ChVector3d inertia = 1e-3 * mass * ChVector3d(0.1, 0.1, 0.1);
    obj->SetMass(mass);
    obj->SetInertia(inertia);

    // Set contact and visualization shape
    double len = 1;
    ChVector3d A(len, -len, 0);
    ChVector3d B(-len, -len, 0);
    ChVector3d C(0, len, 0);	
    utils::AddTriangleGeometry(obj.get(), mat_o, A, B, C, "triangle");

    // Set initial state.
    obj->SetPos(initPos);
    obj->SetRot(initRot);
    obj->SetPosDt(initLinVel);
    obj->SetAngVelLocal(initAngVel);

    // Add object to system.
    system->AddBody(obj);

    return obj;
}

// =============================================================================
// =============================================================================
int main(int argc, char* argv[]) {
    // Create system.
    char title[100];
#ifdef USE_SMC
    sprintf(title, "Object Drop >> SMC");
    cout << "Create SMC system" << endl;
    ChSystemMulticoreSMC* msystem = new ChSystemMulticoreSMC();
#else
    sprintf(title, "Object Drop >> NSC");
    cout << "Create NSC system" << endl;
    ChSystemMulticoreNSC* msystem = new ChSystemMulticoreNSC();
#endif

    msystem->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    msystem->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // ----------------------
    // Set number of threads.
    // ----------------------

    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    if (thread_tuning)
        msystem->SetNumThreads(1, 1, threads);
    else
        msystem->SetNumThreads(threads);

    // ---------------------
    // Edit system settings.
    // ---------------------

    msystem->GetSettings()->solver.tolerance = 1e-3;

#ifdef USE_SMC
    msystem->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
#else
    msystem->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    msystem->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    msystem->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    msystem->GetSettings()->solver.alpha = 0;
    msystem->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    msystem->ChangeSolverType(SolverType::APGDREF);

    msystem->GetSettings()->solver.contact_recovery_speed = 1;
#endif

    msystem->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Create bodies.
    CreateGround(msystem);
    auto obj = CreateObject(msystem);

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(msystem);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, -5, 2), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

#endif

    // Run simulation for specified time.
    int out_steps = (int)std::ceil((1.0 / time_step) / out_fps);

    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;

    while (time < time_end) {
        if (sim_frame == next_out_frame) {
            cout << "------------ Output frame:   " << out_frame << endl;
            cout << "             Sim frame:      " << sim_frame << endl;
            cout << "             Time:           " << time << endl;
            cout << "             Avg. contacts:  " << num_contacts / out_steps << endl;
            cout << "             Execution time: " << exec_time << endl;
            cout << endl;
            cout << obj->GetPos().z() << endl;
            out_frame++;
            next_out_frame += out_steps;
            num_contacts = 0;
        }

        obj->SetRot(QUNIT);

#ifdef CHRONO_OPENGL
        // OpenGL simulation step
        if (vis.Run()) {
            msystem->DoStepDynamics(time_step);
            vis.Render();
        } else
            break;
#else
        // Advance dynamics.
        msystem->DoStepDynamics(time_step);
#endif

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += msystem->GetTimerStep();
        num_contacts += msystem->GetNumContacts();
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;
    cout << "Number of threads: " << threads << endl;

    return 0;
}
