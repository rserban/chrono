#include <omp.h>
#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    // BASIC SETUP
    double time_step = 1e-3;
    double time_end = 25;
    double time_start = 1;

    double out_fps = 50;

    double tolerance = 1e-3;
    unsigned int max_iteration = 100;

    enum ForceType { BODY_FORCE, ACCUMULATOR_FORCE };
    ForceType force_type = BODY_FORCE;

    bool floating = false;

    float friction = 0.5f;

    // Create system
    ////ChSystemMulticoreSMC my_sys;
    ChSystemMulticoreNSC my_sys;

    if (floating)
        my_sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    else
        my_sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    int num_threads = 1;
    my_sys.SetNumThreads(num_threads);

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::PRIMS;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    // Create the plate
    auto plate = chrono_types::make_shared<ChBody>();

    plate->SetTag(0);
    plate->SetPos(ChVector3d(0, 0, 0));
    plate->SetFixed(true);
    plate->EnableCollision(true);

    utils::AddBoxGeometry(plate.get(), ChContactMaterial::DefaultMaterial(my_sys.GetContactMethod()),
                          ChVector3d(4, 0.5, 0.1));

    my_sys.AddBody(plate);

    // Create the ball
    double mass = 1;
    double radius = 0.15;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetTag(2);
    ball->SetMass(mass);
    ball->SetInertiaXX(inertia);
    ball->SetPos(ChVector3d(-3, 0, 1));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetFixed(false);
    ball->EnableCollision(true);
  
    auto ball_mat = ChContactMaterial::DefaultMaterial(my_sys.GetContactMethod());
    ball_mat->SetFriction(friction);
    ball_mat->SetRestitution(0);

    utils::AddSphereGeometry(ball.get(), ball_mat, radius);

    my_sys.AddBody(ball);

    // Force (ramp to maximum and then back to 0)
    double fullForce = 1;
    ChFunctionInterp forceFunct;
    forceFunct.AddPoint(0, 0);
    forceFunct.AddPoint(time_start, 0);
    forceFunct.AddPoint(time_start + 1, fullForce);
    forceFunct.AddPoint(time_start + 2, fullForce);
    forceFunct.AddPoint(time_start + 3, 0);
    forceFunct.AddPoint(time_end, 0);

    if (force_type == BODY_FORCE) {
        auto drawForce = chrono_types::make_shared<ChForce>();
        ball->AddForce(drawForce);
        drawForce->SetMode(ChForce::ForceType::FORCE); // force or torque
        drawForce->SetFrame(ChForce::ReferenceFrame::BODY);
        drawForce->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
        drawForce->SetVrelpoint(ChVector3d(0, 0, 0));
        drawForce->SetF_x(chrono_types::make_shared<ChFunctionInterp>(forceFunct));
        drawForce->SetF_y(chrono_types::make_shared<ChFunctionConst>(0));
        drawForce->SetF_z(chrono_types::make_shared<ChFunctionConst>(0));
    }

    // Create the visualization window
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&my_sys);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, -4, 0), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    // Run simulation for specified time
    int out_steps = static_cast<int>(std::ceil((1 / time_step) / out_fps));
    int sim_frame = 0;

    while (my_sys.GetChTime() < time_end) {
        if (force_type == ACCUMULATOR_FORCE) {
            double Fx = forceFunct.GetVal(my_sys.GetChTime());
            ball->EmptyAccumulators();
            ball->AccumulateForce(ChVector3d(Fx, 0, 0), ball->GetPos(), false);
        }
        my_sys.DoStepDynamics(time_step);
        sim_frame++;

        std::cout << "Vx: " << ball->GetPosDt().x() << std::endl;

        if (vis.Run()) {
            vis.Render();
        } else {
            return 1;
        }
    }

    return 0;
}
