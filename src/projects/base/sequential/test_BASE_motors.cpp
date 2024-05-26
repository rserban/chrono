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
// LinkMotor test
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChSystemNSC.h"

using namespace chrono;

// --------------------------------------------------------------------------

void test1() {
    // Create the system
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, -10));

    // Set solver settings
    system.GetSolver()->AsIterative()->SetMaxIterations(200);
    system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Add bodies
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0, 0, 0));
    ground->SetFixed(true);
    system.Add(ground);

    ChVector3d pos1(1, 0, 0);
    auto body1 = chrono_types::make_shared<ChBody>();
    body1->SetMass(1);
    body1->SetPos(pos1);
    body1->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    system.AddBody(body1);

    ChVector3d pos2(2, 0, 0);
    auto body2 = chrono_types::make_shared<ChBody>();
    body2->SetMass(1);
    body2->SetPos(pos2);
    body2->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    system.AddBody(body2);
    
    // Arbitrary direction
    ChQuaternion<> q(1, 2, 3, 4);
    q.Normalize();

    // Add angle motor to body1
    auto fun1 = chrono_types::make_shared<ChFunctionSetpoint>();
    auto joint1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    joint1->Initialize(ground, body1, ChFrame<>(pos1, q));
    joint1->SetAngleFunction(fun1);
    system.AddLink(joint1);

    // Add speed motor to body2
    auto fun2 = chrono_types::make_shared<ChFunctionSetpoint>();
    auto joint2 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    joint2->Initialize(ground, body2, ChFrame<>(pos2, q));
    joint2->SetSpeedFunction(fun2);
    system.AddLink(joint2);

    // Simulate system
    double alpha = 2.0;
    double time = 0;
    double step = 1e-4;
    while (time < 10) {
        fun1->SetSetpoint(0.5 * alpha * time * time, time);  // angle(t) = 0.5 * alpha * t^2 => torque = J * alpha
        fun2->SetSetpoint(alpha * time, time);               // speed(t) = alpha * t         => torque = J * alpha

        system.DoStepDynamics(step);

        std::cout << time << "  ";
        std::cout << joint1->GetMotorAngle() << "  " << joint1->GetMotorTorque() << "     ";
        std::cout << joint2->GetMotorAngle() << "  " << joint2->GetMotorTorque() << "     ";

        time += step;
    }
}

// --------------------------------------------------------------------------

void test2() {
    // Create the system
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Set solver settings
    system.GetSolver()->AsIterative()->SetMaxIterations(200);
    system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Add bodies
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0, 0, 0));
    ground->SetFixed(true);
    system.Add(ground);

    ChVector3d pos(3, 0, 0);
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1);
    body->SetPos(pos);
    body->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    system.AddBody(body);

    // Motor orientation
    auto q = QuatFromAngleY(0.25 * CH_PI);
    ////ChQuaternion<> q(1, 2, 3, 4);
    ////q.Normalize();
    auto frame = ChFrame<>(pos, q);

    // Add angle motor
    auto fun3 = chrono_types::make_shared<ChFunctionSetpoint>();
    auto joint = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    joint->Initialize(ground, body, frame);
    joint->SetAngleFunction(fun3);
    system.AddLink(joint);

    // Add off-center force to body
    auto F_abs = ChVector3d(0, 5, 0);
    auto d_loc = ChVector3d(1, 0, 0);

    auto container = chrono_types::make_shared<ChLoadContainer>();
    system.Add(container);
    auto force = chrono_types::make_shared<ChLoadBodyForce>(body, F_abs, false, d_loc, true);
    container->Add(force);

    auto F_loc = frame.TransformDirectionParentToLocal(F_abs);   
    auto a = Vcross(d_loc, F_loc);
    std::cout << a.x() << "  " << a.y() << "  " << a.z() << std::endl;

    // Simulate system
    double time = 0;
    double step = 1e-4;
    while (time < 10) {
        fun3->SetSetpoint(0.0, time);

        system.DoStepDynamics(step);

        auto omega = body->GetAngVelLocal();
        std::cout << time << "    ";
        std::cout << joint->GetMotorAngle() << "  " << joint->GetMotorTorque() << "    ";
        std::cout << omega.x() << "  " << omega.y() << "  " << omega.z() << "    ";
        std::cout << std::endl;

        time += step;
    }
}

int main(int argc, char* argv[]) {
    ////test1();
    test2();
}
