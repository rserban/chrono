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
// Test different ways of driving a steering mechanism.
//
// Use a 4-bar mechanism modeled after the Chrono::Vehicle Pitman Arm steering.
// Use numerical values from the HMMWV steering mechanism.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsMotorPosition.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleGeometry.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace filesystem;

using std::cout;
using std::endl;

// =============================================================================

enum Model {
    LINK_MOTOR,  // revolute between chassis and arm
    SHAFT_MOTOR  // shaft attached to arm and shaft attached to chassis
};

Model model = SHAFT_MOTOR;

// =============================================================================

class SteeringFunction : public ChFunction {
  public:
    SteeringFunction() : m_max_angle(0), m_val(0), m_der(0), m_val_prev(0), m_time_prev(0) {}
    SteeringFunction(double max_angle) : m_max_angle(max_angle), m_val(0), m_der(0), m_val_prev(0), m_time_prev(0) {}

    void Update(double time, double steering) {
        if (time <= m_time_prev)
            return;

        m_val = m_max_angle * steering;
        m_der = (m_val - m_val_prev) / (time - m_time_prev);

        m_val_prev = m_val;
        m_time_prev = time;
    }

    virtual SteeringFunction* Clone() const override { return new SteeringFunction(); }

    virtual double GetVal(double x) const override { return m_val; }
    virtual double GetDer(double x) const override { return m_der; }

  private:
    double m_max_angle;
    double m_val;
    double m_der;
    double m_time_prev;
    double m_val_prev;
};

// =============================================================================

double GetSteering(double time) {
    double freq = 0.5;
    return std::sin(2 * CH_PI * freq * time);
}

// Steering input applied to the LinkMotor
void ApplySteering(double time, double steering, std::shared_ptr<ChLinkMotorRotationAngle> element) {
    double max_angle = 50.0 * (CH_PI / 180);
    auto fun = std::static_pointer_cast<ChFunctionSetpoint>(element->GetAngleFunction());
    fun->SetSetpoint(max_angle * steering, time);
}

// Steering input applied to shaft
void ApplySteering(double time, double steering, std::shared_ptr<ChShaftsMotorPosition> element) {
    double max_angle = 50.0 * (CH_PI / 180);
    auto fun = std::static_pointer_cast<ChFunctionSetpoint>(element->GetPositionFunction());
    fun->SetSetpoint(max_angle * steering, time);
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // -------------
    // Create bodies
    // -------------

    // Chassis (fixed to ground)
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetFixed(true);
    sys.AddBody(chassis);

    // Steering link
    auto link = chrono_types::make_shared<ChBody>();
    link->SetPos(ChVector3d(0.129, 0, 0));
    link->SetRot(QUNIT);
    link->SetMass(3.68);
    link->SetInertiaXX(ChVector3d(0.252, 0.00233, 0.254));
    link->SetInertiaXY(ChVector3d(0.0, 0.0, 0.0));
    sys.AddBody(link);

    // Visualization of the steering link
    double link_radius = 0.03;
    auto pP = link->TransformPointParentToLocal(ChVector3d(0.129, 0.249, 0));        // univ joint
    auto pI = link->TransformPointParentToLocal(ChVector3d(0.129, -0.325, 0));       // S location of revsph
    auto pTP = link->TransformPointParentToLocal(ChVector3d(0.195, 0.448, 0.035));   // tierod loc (Pitman arm side)
    auto pTI = link->TransformPointParentToLocal(ChVector3d(0.195, -0.448, 0.035));  // tierod loc (idler side)
    vehicle::ChVehicleGeometry::AddVisualizationCylinder(link, pP, pI, link_radius);
    vehicle::ChVehicleGeometry::AddVisualizationCylinder(link, pP, pTP, link_radius);
    vehicle::ChVehicleGeometry::AddVisualizationCylinder(link, pI, pTI, link_radius);

    // Markers on steering link (at tie-rod connections)
    auto markerP = chrono_types::make_shared<ChMarker>();
    markerP->ImposeRelativeTransform(ChFrame<>(pTP));
    link->AddMarker(markerP);

    auto markerI = chrono_types::make_shared<ChMarker>();
    markerI->ImposeRelativeTransform(ChFrame<>(pTI));
    link->AddMarker(markerI);

    // Pitman arm body
    auto arm = chrono_types::make_shared<ChBody>();
    arm->SetPos(ChVector3d(0.064, 0.249, 0));
    arm->SetRot(QUNIT);
    arm->SetMass(1.605);
    arm->SetInertiaXX(ChVector3d(0.00638, 0.00756, 0.00150));
    arm->SetInertiaXY(ChVector3d(0.0, 0.0, 0.0));
    sys.AddBody(arm);

    double arm_radius = 0.02;

    auto pC = arm->TransformPointParentToLocal(ChVector3d(0, 0.249, 0));      // rev joint loc
    auto pL = arm->TransformPointParentToLocal(ChVector3d(0.129, 0.249, 0));  // univ joint loc
    vehicle::ChVehicleGeometry::AddVisualizationCylinder(arm, pC, pL, arm_radius);

    // -------------
    // Create joints
    // -------------

    double max_angle = 50.0 * (CH_PI / 180);

    auto revolute_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    revolute_motor->Initialize(chassis, arm, ChFrame<>(ChVector3d(0, 0.249, 0), QUNIT));
    auto motor_fun = chrono_types::make_shared<ChFunctionSetpoint>();
    revolute_motor->SetAngleFunction(motor_fun);

    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(chassis, arm, ChFrame<>(ChVector3d(0, 0.249, 0), QUNIT));
    auto shaftA = chrono_types::make_shared<ChShaft>();
    shaftA->SetInertia(0.01);
    auto shaft_arm = chrono_types::make_shared<ChShaftBodyRotation>();
    shaft_arm->Initialize(shaftA, arm, ChVector3d(0, 0, 1));
    auto shaftC = chrono_types::make_shared<ChShaft>();
    shaftC->SetInertia(0.01);
    auto shaft_chassis = chrono_types::make_shared<ChShaftBodyRotation>();
    shaft_chassis->Initialize(shaftC, chassis, ChVector3d(0, 0, 1));
    auto shaft_motor = chrono_types::make_shared<ChShaftsMotorPosition>();
    shaft_motor->Initialize(shaftA, shaftC);
    shaft_motor->SetPositionFunction(motor_fun);

    switch (model) {
        case LINK_MOTOR:
            sys.AddLink(revolute_motor);
            break;
        case SHAFT_MOTOR: {
            sys.AddLink(revolute);
            sys.Add(shaftA);
            sys.Add(shaftC);
            sys.Add(shaft_arm);
            sys.Add(shaft_chassis);
            sys.Add(shaft_motor);
            break;
        }
    }

    ChVector3d u(0, 0, 1);  // univ axis on arm
    ChVector3d v(1, 0, 0);  // univ axis on link
    ChVector3d w(0, 1, 0);  // w = u x v
    ChMatrix33<> rot(u, v, w);
    auto universal = chrono_types::make_shared<ChLinkUniversal>();
    universal->Initialize(arm, link, ChFrame<>(ChVector3d(0.129, 0.249, 0), rot.GetQuaternion()));
    sys.AddLink(universal);

    double distance = (ChVector3d(0.129, -0.325, 0) - ChVector3d(0, -0.325, 0)).Length();
    auto revsph = chrono_types::make_shared<ChLinkRevoluteSpherical>();
    revsph->Initialize(chassis, link, ChCoordsys<>(ChVector3d(0, -0.325, 0), QUNIT), distance);
    revsph->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    sys.AddLink(revsph);

    // ---------------------------
    // Create Irrlicht application
    // ---------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Compliant steering");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 0, -2));
    vis->AddTypicalLights();
    vis->AttachSystem(&sys);

    // -------------
    // Set up output
    // -------------

    std::string out_dir = "../TEST_steering_input";
    std::string out_file;
    switch (model) {
        case LINK_MOTOR:
            out_file = out_dir + "/out_LinkMotor.txt";
            break;
        case SHAFT_MOTOR:
            out_file = out_dir + "/out_ShaftMotor.txt";
            break;
    }

    bool out_dir_exists = path(out_dir).exists();
    if (out_dir_exists) {
        cout << "Output directory already exists" << endl;
    } else if (create_directory(path(out_dir))) {
        cout << "Create directory = " << path(out_dir).make_absolute() << endl;
    } else {
        cout << "Error creating output directory" << endl;
        return 1;
    }

    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    // ---------------
    // Simulation loop
    // ---------------

    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Update steering input
        double time = sys.GetChTime();
        double steering = GetSteering(time);
        switch (model) {
            case LINK_MOTOR:
                ApplySteering(time, steering, revolute_motor);
                break;
            case SHAFT_MOTOR:
                ApplySteering(time, steering, shaft_motor);
                break;
        }

        // Output
        auto aPos = arm->GetPos();
        auto aRot = arm->GetRot();
        auto aAngles = aRot.GetCardanAnglesXYZ();
        auto lPos = link->GetPos();
        auto mP = markerP->GetAbsFrame().GetPos();
        auto mI = markerI->GetAbsFrame().GetPos();
        csv << time << steering << aPos << aAngles << lPos << mP << mI << endl;

        // Advance dynamics
        sys.DoStepDynamics(1e-3);
    }

    csv.WriteToFile(out_file);

    return 0;
}
