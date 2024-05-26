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
    double time_end = 5;

    double out_fps = 50;

    double tolerance = 1e-3;
    unsigned int max_iteration = 100;

    // Create DEM system
    ChSystemMulticoreSMC my_sys;

    int num_threads = 1;
    my_sys.SetNumThreads(num_threads);

    my_sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    my_sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::PRIMS;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    // Common material
    float Y = 2e6f;
    float mu = 0.4f;
    float cr = 0.4f;
    auto contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    contact_mat->SetYoungModulus(Y);
    contact_mat->SetFriction(mu);
    contact_mat->SetRestitution(cr);
    contact_mat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);

    // Lower ball
    auto ball_lower = chrono_types::make_shared<ChBody>();

    ball_lower->SetTag(1);
    ball_lower->SetMass(mass);
    ball_lower->SetInertiaXX(inertia);
    ball_lower->SetPos(ChVector3d(0, 0, 10));
    ball_lower->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball_lower->SetFixed(false);
    ball_lower->EnableCollision(true);

    utils::AddSphereGeometry(ball_lower.get(), contact_mat, radius);

    my_sys.AddBody(ball_lower);

    // Upper ball
    auto ball_upper = chrono_types::make_shared<ChBody>();

    ball_upper->SetTag(2);
    ball_upper->SetMass(mass);
    ball_upper->SetInertiaXX(inertia);
    ball_upper->SetPos(ChVector3d(0, 0, 11));
    ball_upper->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball_upper->SetFixed(false);
    ball_upper->EnableCollision(true);

    utils::AddSphereGeometry(ball_upper.get(), contact_mat, radius);

    my_sys.AddBody(ball_upper);

    // Plate
    auto plate = chrono_types::make_shared<ChBody>();

    plate->SetTag(0);
    plate->SetPos(ChVector3d(0, 0, 8));
    plate->SetFixed(true);
    plate->EnableCollision(true);

    utils::AddBoxGeometry(plate.get(), contact_mat, ChVector3d(4 * radius, 4 * radius, radius));

    my_sys.AddBody(plate);

    // Create the visualization window
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&my_sys);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, -4, 10), ChVector3d(0, 0, 10));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    // Run simulation for specified time
    int out_steps = static_cast<int>(std::ceil((1 / time_step) / out_fps));
    double time = 0;
    int sim_frame = 0;
    while (my_sys.GetChTime() < time_end) {
        double z_lower = ball_lower->GetPos().z();
        double z_upper = ball_upper->GetPos().z();
        double vz_upper = ball_upper->GetPosDt().z();

        if (sim_frame % out_steps == 0) {
            std::cout << "t: " << my_sys.GetChTime() << "  z_lower: " << z_lower << "  z_upper: " << z_upper
                      << std::endl;
        }

        my_sys.DoStepDynamics(time_step);
        sim_frame++;

        // Fix lower ball to ground
        if (sim_frame == 200) {
            std::cout << "------- Setting lower body to fixed" << std::endl;
            ball_lower->SetFixed(true);
        }

        // After first interaction, disable contact on lower ball
        if (ball_lower->IsCollisionEnabled() && vz_upper > 0 && z_upper > z_lower + 2 * radius) {
            std::cout << "------- Setting lower body to not collide" << std::endl;
            ball_lower->EnableCollision(false);
        }

        if (vis.Run()) {
            vis.Render();
        } else {
            return 1;
        }
    }

    return 0;
}
