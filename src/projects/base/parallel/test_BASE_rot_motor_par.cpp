//
// Test rotational motor speed
//

#include <omp.h>
#include <cmath>
#include <cstdio>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;
using namespace chrono::collision;

#define PARALLEL_SYS
//#define MOTOR_SPEED

int main(int argc, char* argv[]) {
    double step_size = 1e-3;

#ifdef PARALLEL_SYS
    // Create parallel system
    ChSystemParallelNSC my_sys;
    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    my_sys.GetSettings()->solver.tolerance = 1e-5;
    my_sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    my_sys.GetSettings()->solver.max_iteration_normal = 0;
    my_sys.GetSettings()->solver.max_iteration_sliding = 300;
    my_sys.GetSettings()->solver.max_iteration_spinning = 0;
    my_sys.GetSettings()->solver.max_iteration_bilateral = 100;
    my_sys.GetSettings()->solver.compute_N = false;
    my_sys.GetSettings()->solver.alpha = 0;
    my_sys.GetSettings()->solver.cache_step_length = true;
    my_sys.GetSettings()->solver.use_full_inertia_tensor = false;
    my_sys.GetSettings()->solver.contact_recovery_speed = 1000;
    my_sys.GetSettings()->solver.bilateral_clamp_speed = 1e8;
    my_sys.GetSettings()->min_threads = 2;

    my_sys.GetSettings()->collision.collision_envelope = 0.01;
    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    my_sys.ChangeSolverType(SolverType::BB);

#else
    // Create sequential system
    ChSystemNSC my_sys;

    my_sys.SetMaxItersSolverSpeed(200);
    my_sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

#endif

    // Create ground body
    auto ground = std::shared_ptr<ChBody>(my_sys.NewBody());
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    my_sys.AddBody(ground);

    // Create rotating body
    auto body = std::shared_ptr<ChBody>(my_sys.NewBody());
    body->SetBodyFixed(false);
    body->SetCollide(false);
    body->SetMass(1);
    body->SetPos(ChVector<>(0, 0, 0));
    my_sys.AddBody(body);

    auto box_shape = chrono_types::make_shared<ChBoxShape>();
    box_shape->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 0.2));
    box_shape->Pos = ChVector<>(0, 0, 0);
    box_shape->Rot = QUNIT;
    body->AddAsset(box_shape);

    // Create rotational motor
#ifdef MOTOR_SPEED
    auto motor_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    auto joint = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    joint->Initialize(ground, body, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    joint->SetSpeedFunction(motor_fun);
#else
    auto motor_fun = chrono_types::make_shared<ChFunction_Setpoint>();
    auto joint = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    joint->Initialize(ground, body, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    joint->SetAngleFunction(motor_fun);
#endif
    my_sys.AddLink(joint);

    // Create the visualization window
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Rotational motor", &my_sys);
    gl_window.SetCamera(ChVector<>(0, -4, 1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Run simulation
    while (true) {
        if (!gl_window.Active())
            break;

#ifdef MOTOR_SPEED
        motor_fun->SetSetpoint(0.5, my_sys.GetChTime());
#else
        motor_fun->SetSetpoint(0.5 * my_sys.GetChTime(), my_sys.GetChTime());
#endif

        my_sys.DoStepDynamics(step_size);
        gl_window.Render();
    }

    return 0;
}