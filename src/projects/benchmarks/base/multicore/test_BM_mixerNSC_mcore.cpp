// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Benchmark test for multicore simulation using NSC contact.
//
// =============================================================================

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChBenchmark.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

using namespace chrono;

// =============================================================================

template <int N>
class MixerTestNSC : public utils::ChBenchmarkTest {
  public:
    MixerTestNSC();
    ~MixerTestNSC() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

    void SimulateVis();

  private:
    ChSystemMulticoreNSC* m_system;
    double m_step;
};

template <int N>
MixerTestNSC<N>::MixerTestNSC() : m_system(new ChSystemMulticoreNSC()), m_step(1e-3) {
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Set number of threads.
    m_system->SetNumThreads(omp_get_num_procs());

    // Set solver parameters
    m_system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    m_system->GetSettings()->solver.max_iteration_normal = 0;
    m_system->GetSettings()->solver.max_iteration_sliding = 50;
    m_system->GetSettings()->solver.max_iteration_spinning = 0;
    m_system->GetSettings()->solver.max_iteration_bilateral = 10;
    m_system->GetSettings()->solver.tolerance = 1e-3;
    m_system->GetSettings()->solver.alpha = 0;
    m_system->GetSettings()->solver.contact_recovery_speed = 10000;
    m_system->ChangeSolverType(SolverType::APGD);
    m_system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    m_system->GetSettings()->collision.collision_envelope = 0.01;
    m_system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Balls created in layesr of 25 = 5x5
    double radius = 0.075;
    int num_layers = (N + 24) / 25;

    // Create a common material
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);

    // Create container bin
    auto bin = utils::CreateBoxContainer(m_system, mat, ChVector3d(1, 1, 0.1 + 0.4 * num_layers * radius), 0.1);

    // The rotating mixer body (1.6 x 0.2 x 0.4)
    auto mixer = chrono_types::make_shared<ChBody>();
    mixer->SetTag(-200);
    mixer->SetMass(10.0);
    mixer->SetInertiaXX(ChVector3d(50, 50, 50));
    mixer->SetPos(ChVector3d(0, 0, 0.205));
    mixer->SetFixed(false);
    mixer->EnableCollision(true);
    utils::AddBoxGeometry(mixer.get(), mat, ChVector3d(0.8, 0.1, 0.2));
    m_system->AddBody(mixer);

    // Create a motor between the two bodies, constrained to rotate at 90 deg/s
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(mixer, bin, ChFrame<>(ChVector3d(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, CH_PI / 2));
    m_system->AddLink(motor);

    // Create the balls
    double mass = 1;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);
    int num_balls = 0;
    for (int il = 0; il < num_layers; il++) {
        double height = 1 + 2.01 * radius * il;
        for (int ix = -2; ix < 3; ix++) {
            for (int iy = -2; iy < 3; iy++) {
                auto ball = chrono_types::make_shared<ChBody>();
                ball->SetTag(num_balls);
                ball->SetMass(mass);
                ball->SetInertiaXX(inertia);
                ball->SetPos(ChVector3d(0.4 * ix + 0.01 * (il % 2), 0.4 * iy + 0.01 * (il % 2), height));
                ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
                ball->SetFixed(false);
                ball->EnableCollision(true);
                utils::AddSphereGeometry(ball.get(), mat, radius);
                m_system->AddBody(ball);
                num_balls++;
                if (num_balls == N)
                    return;
            }
        }
    }
}

template <int N>
void MixerTestNSC<N>::SimulateVis() {
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(m_system);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, -2, 3), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    while (vis.Run()) {
        ExecuteStep();
        vis.Render();
    }
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(MixerNSC032, MixerTestNSC<32>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(MixerNSC064, MixerTestNSC<64>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(MixerNSC128, MixerTestNSC<128>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(MixerNSC256, MixerTestNSC<256>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(MixerNSC512, MixerTestNSC<512>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        MixerTestNSC<256> test;
        test.SimulateVis();
        return 0;
    }

    ::benchmark::RunSpecifiedBenchmarks();
}
