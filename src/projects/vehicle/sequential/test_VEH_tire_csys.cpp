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
// Test tire models outside Chrono::Vehicle AND with a different frame convention.
// Compare a right frame with X forward and Z up (the ISO frame in Chrono::Vehicle)
// and a setup with X forward and Y up.
//
// Create two mechanisms (in disjunct Chrono systems) and simulate them concurrently.
// Each mechanism has a HMMWV TMeasy tire attached to a spindle body which is
// constrained to move in a vertical plane.  The spindle body is given an initial
// angular velocity to initiate motion in the forward direction.
//
// In both cases, the tire force calculation is done in an ISO frame.
//
// The ISO mechanism mimics exactly what is done within a Chrono::Vehicle wheeled
// vehicle.
//
// In the case of the YUP mechanism, wheel states expressed in an ISO frame are
// made available to the tire calculation through a "dummy" spindle body.
// At construction, the wheel object is associated with the "dummy" spindle body.
// At each simulation step (see MechanismYUP::Advance), a WheelState (expressed
// in the YUP frame) is first calculated from the state of the "real" spindle body
// (the one being actually simulated).  This WheelState is converted to ISO frame
// and used to overrite the state of the "dummy" spindle body.  The tire then
// performs its force calculation in an ISO frame (and internally requests a
// WheelState from the associated wheel object; the wheel calculates this based
// on the "dummy" spindle body thus passing an ISO WheelState to the tire).
// The tire forces, reported back in an ISO frame, are converted to the YUP frame
// and applied, as external forces, to the "real" spindle object.
//
// Note that, to exactly match simulations between the two systems, the mass and
// inertia of the spindle body in the YUP system must be overwritten to include
// the (properly transformed) masses and inertias of the spindle, the wheel, and
// the tire.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;

// =============================================================================

double grav = 9.8;

double step_size = 1e-3;
double tire_step_size = 1e-3;

double spindle_mass = 500;  // spindle mass (large to account for chassis load)
double spindle_I1 = 0.05;   // moment of inertia about rotation axis
double spindle_I2 = 0.1;    // moments of inertia about other 2 axes

double wheel_init_height = 0.6;
double wheel_init_omega = 10;

// =============================================================================
// Tire-wheel-spindle mechanism in ISO frame

class MechanismISO {
  public:
    MechanismISO(ChSystem* sys);
    void Advance(double step);
    const WheelState& GetWheelState() const { return m_wheelstate; }
    const TerrainForce& GetTireForce() const { return m_tireforce; }

  private:
    class Terrain : public ChTerrain {
      public:
        Terrain() {}
        virtual double GetHeight(const ChVector3d& loc) const override { return 0; }
        virtual ChVector3d GetNormal(const ChVector3d& loc) const override { return ChVector3d(0, 0, 1); }
        virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return 0.8f; }
    };

    ChSystem* m_sys;
    std::shared_ptr<ChBody> m_spindle;
    std::shared_ptr<ChWheel> m_wheel;
    std::shared_ptr<ChTire> m_tire;
    WheelState m_wheelstate;
    TerrainForce m_tireforce;
    Terrain m_terrain;
};

MechanismISO::MechanismISO(ChSystem* sys) : m_sys(sys) {
    auto ground = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(ground);
    ground->SetFixed(true);
    {
        auto box = chrono_types::make_shared<ChVisualShapeBox>(20, 4, 0.4);
        ground->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -0.1)));
    }

    // Create the spindle body (the wheel and tire objects will add to its mass and inertia).
    m_spindle = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(m_spindle);
    m_spindle->SetMass(spindle_mass);
    m_spindle->SetInertiaXX(ChVector3d(spindle_I2, spindle_I1, spindle_I2));
    m_spindle->SetPos(ChVector3d(0, 0, wheel_init_height));
    m_spindle->SetAngVelLocal(ChVector3d(0, +wheel_init_omega, 0));  // for the wheel to move in positive X
    {
        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.1, 0.2);
        m_spindle->AddVisualShape(cyl, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
    }

    // Connect spindle to ground.
    auto link = chrono_types::make_shared<ChLinkLockPlanePlane>();
    link->Initialize(ground, m_spindle, ChCoordsys<>(VNULL, QuatFromAngleX(CH_PI_2)));
    m_sys->AddLink(link);

    // Create a wheel object and associate it with the spindle body.
    m_wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");
    m_wheel->Initialize(m_spindle, LEFT);
    m_wheel->SetVisualizationType(VisualizationType::NONE);

    // Create the tire object and associate it with the wheel body.
    m_tire = chrono_types::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");
    m_tire->SetStepsize(tire_step_size);
    m_tire->Initialize(m_wheel);
    m_tire->SetVisualizationType(VisualizationType::MESH);

    // This is only needed if we want to call m_wheel->Synchronize in MechanismISO::Advance
    // (in which case the wheel must be able to obtain the tire forces).
    m_wheel->SetTire(m_tire);
}

void MechanismISO::Advance(double step) {
    double time = m_sys->GetChTime();

    // Get current wheel state
    m_wheelstate = m_wheel->GetState();

    // Get tire forces
    m_tireforce = m_tire->ReportTireForce(&m_terrain);

    // Synchronize and advance subsystems
    m_tire->Synchronize(time, m_terrain);

    // Apply tire forces to spindle body
    // (we could simply call m_wheel->Synchronize which does precisely this)
    m_spindle->EmptyAccumulators();
    m_spindle->AccumulateForce(m_tireforce.force, m_tireforce.point, false);
    m_spindle->AccumulateTorque(m_tireforce.moment, false);

    m_tire->Advance(step);
    m_sys->DoStepDynamics(step);
}

// =============================================================================

WheelState ConvertState_YUP_to_ISO(const WheelState& ws_YUP) {
    // Coordinate transforms
    ChFrame<> iso_X_yup(VNULL, QuatFromAngleX(CH_PI_2));          // YUP -> ISO
    ChFrame<> wiso_X_wyup = iso_X_yup;                            // Wheel_YUP -> Wheel_ISO
    ChFrame<> wyup_X_wiso = wiso_X_wyup.GetInverse();             // Wheel_ISO -> Wheel_YUP
    ChFrame<> yup_X_wyup(ws_YUP.pos, ws_YUP.rot);                 // Wheel_YUP -> YUP
    ChFrame<> iso_X_wiso = iso_X_yup * yup_X_wyup * wyup_X_wiso;  // Wheel_ISO -> ISO

    // Wheel body state in ISO frame
    WheelState ws_ISO;
    ws_ISO.pos = iso_X_wiso.GetPos();
    ws_ISO.rot = iso_X_wiso.GetRot();
    ws_ISO.lin_vel = iso_X_yup.TransformDirectionLocalToParent(ws_YUP.lin_vel);
    ws_ISO.ang_vel = iso_X_yup.TransformDirectionLocalToParent(ws_YUP.ang_vel);
    auto ang_vel_loc = ws_ISO.rot.RotateBack(ws_ISO.ang_vel);
    ws_ISO.omega = ang_vel_loc.y();

    return ws_ISO;
}

TerrainForce ConvertForce_ISO_to_YUP(const TerrainForce& tf_ISO) {
    // Coordinate transforms
    ChFrame<> yup_X_iso(VNULL, QuatFromAngleX(-CH_PI_2));  // ISO -> YUP

    // Terrain force in YUP frame
    TerrainForce tf_YUP;
    tf_YUP.point = yup_X_iso.TransformPointLocalToParent(tf_ISO.point);
    tf_YUP.force = yup_X_iso.TransformDirectionLocalToParent(tf_ISO.force);
    tf_YUP.moment = yup_X_iso.TransformDirectionLocalToParent(tf_ISO.moment);

    return tf_YUP;

    /*
    // Alternatively, using the iso_X_yup transform...
    ChFrame<> iso_X_yup(VNULL, QuatFromAngleX(CH_PI_2));
    TerrainForce tf_YUP;
    tf_YUP.point = iso_X_yup.TransformPointParentToLocal(tf_ISO.point);
    tf_YUP.force = iso_X_yup.TransformDirectionParentToLocal(tf_ISO.force);
    tf_YUP.moment = iso_X_yup.TransformDirectionParentToLocal(tf_ISO.moment);

    return tf_YUP;
    */
}

ChVector3d ConvertInertia_ISO_to_YUP(const ChVector3d& inertia_ISO) {
    // Coordinate transforms
    ChMatrix33<> iso_R_yup(QuatFromAngleX(+CH_PI_2));
    ChMatrix33<> yup_R_iso(QuatFromAngleX(-CH_PI_2));

    // Moments of inertia in YUP frame
    ChMatrix33<> J_ISO(inertia_ISO);
    ChMatrix33<> J_YUP = yup_R_iso * J_ISO * iso_R_yup;
    ChVector3d inertia_YUP = J_YUP.diagonal();

    /*
    // Alternatively, using the iso_X_yup transform...
    ChFrame<> iso_X_yup(VNULL, QuatFromAngleX(CH_PI_2));
    ChFrame<> yup_X_iso = iso_X_yup.inverse();
    ChMatrix33<> J_ISO(inertia_ISO);
    ChMatrix33<> J_YUP = (yup_X_iso.GetRotMat() * J_ISO) * iso_X_yup.GetRotMat();
    ChVector3d inertia_YUP = J_YUP.digonal();
    */

    /*
    // Alternatively, just flip elements (for the particular case of YUP frame)
    ChVector3d inertia_YUP(inertia_ISO.x(), inertia_ISO.z(), inertia_ISO.y());
    */

    return inertia_YUP;
}

// =============================================================================
// Tire-wheel-spindle mechanism in YUP frame

class MechanismYUP {
  public:
    MechanismYUP(ChSystem* sys);
    void Advance(double step);
    void Advance(double step, const WheelState& ws_ISO);
    const WheelState& GetWheelState() const { return m_wheelstate; }
    const TerrainForce& GetTireForce() const { return m_tireforce; }

  private:
    class Terrain : public ChTerrain {
      public:
        Terrain() {}
        virtual double GetHeight(const ChVector3d& loc) const override { return 0; }
        virtual ChVector3d GetNormal(const ChVector3d& loc) const override { return ChVector3d(0, 0, 1); }
        virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return 0.8f; }
    };

    ChSystem* m_sys;
    std::shared_ptr<ChBody> m_spindle;
    std::shared_ptr<ChBody> m_spindle_dummy;
    std::shared_ptr<ChWheel> m_wheel;
    std::shared_ptr<ChTire> m_tire;
    WheelState m_wheelstate;
    TerrainForce m_tireforce;
    Terrain m_terrain;
};

MechanismYUP::MechanismYUP(ChSystem* sys) : m_sys(sys) {
    auto ground = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(ground);
    ground->SetFixed(true);
    {
        auto box = chrono_types::make_shared<ChVisualShapeBox>(20, 0.4, 4);
        ground->AddVisualShape(box, ChFrame<>(ChVector3d(0, -0.1, 0)));
    }

    // Create the spindle body (mass and inertia properly set below).
    m_spindle = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(m_spindle);
    m_spindle->SetPos(ChVector3d(0, wheel_init_height, 0));
    m_spindle->SetAngVelLocal(ChVector3d(0, 0, -wheel_init_omega));  // for the wheel to move in positive X
    {
        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.1, 0.2);
        m_spindle->AddVisualShape(cyl);
    }

    // Connect spindle to ground.
    auto link = chrono_types::make_shared<ChLinkLockPlanePlane>();
    link->Initialize(ground, m_spindle, ChCoordsys<>(VNULL, QUNIT));
    m_sys->AddLink(link);

    // Create the "dummy" spindle body, used to provide wheel state information to the tire in an ISO frame.
    // We will manually overwrite the state of this body (in MechanismYUP::Advance).
    m_spindle_dummy = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(m_spindle_dummy);
    m_spindle_dummy->SetFixed(true);

    // Create a wheel object and associate it with the "dummy" spindle body.
    m_wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");
    m_wheel->Initialize(m_spindle_dummy, LEFT);
    m_wheel->SetVisualizationType(VisualizationType::NONE);

    // Create the tire object and associate it with the wheel body.
    m_tire = chrono_types::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");
    m_tire->SetStepsize(tire_step_size);
    m_tire->Initialize(m_wheel);

    // This is actually not needed here (we have to explicitly add tire forces to the spindle body anyway).
    m_wheel->SetTire(m_tire);

    // IMPORTANT: override spindle body mass and inertia!
    // This must be done manually here because the wheel and tire objects append to the spindle's properties (but
    // assuming an ISO frame)
    ChVector3d spindle_Ixx(spindle_I2, spindle_I2, spindle_I1);
    ChVector3d wheel_Ixx = ConvertInertia_ISO_to_YUP(m_wheel->GetInertia().diagonal());
    ChVector3d tire_Ixx = ConvertInertia_ISO_to_YUP(m_tire->GetInertia().diagonal());
    m_spindle->SetMass(spindle_mass + m_wheel->GetMass() + m_tire->GetMass());
    m_spindle->SetInertiaXX(spindle_Ixx + wheel_Ixx + tire_Ixx);

    // Cannot directly use tire visualization (incorrect rotation).
    m_tire->SetVisualizationType(VisualizationType::NONE);
    {
        auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile("hmmwv/hmmwv_tire_left.obj"), false, false);
        trimesh->Transform(VNULL, ChMatrix33<>(QuatFromAngleX(-CH_PI_2)));
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName("hmmwv_tire_geom");
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetColor(ChColor(1.0f, 0.0f, 0.0f));
        m_spindle->AddVisualShape(trimesh_shape);
    }
}

void MechanismYUP::Advance(double step) {
    double time = m_sys->GetChTime();

    // Wheel (spindle) body state in YUP frame
    m_wheelstate.pos = m_spindle->GetPos();
    m_wheelstate.rot = m_spindle->GetRot();
    m_wheelstate.lin_vel = m_spindle->GetPosDt();
    m_wheelstate.ang_vel = m_spindle->GetAngVelParent();
    auto ang_vel_loc = m_wheelstate.rot.RotateBack(m_wheelstate.ang_vel);
    m_wheelstate.omega = ang_vel_loc.z();

    // Wheel body state in ISO frame
    WheelState ws_ISO = ConvertState_YUP_to_ISO(m_wheelstate);

    // Overwrite state of "dummy" spindle body
    m_spindle_dummy->SetPos(ws_ISO.pos);
    m_spindle_dummy->SetRot(ws_ISO.rot);
    m_spindle_dummy->SetPosDt(ws_ISO.lin_vel);
    m_spindle_dummy->SetAngVelParent(ws_ISO.ang_vel);

    // Get tire forces in ISO frame and convert to YUP frame
    auto tf_ISO = m_tire->ReportTireForce(&m_terrain);
    m_tireforce = ConvertForce_ISO_to_YUP(tf_ISO);

    // Let tire update itself given current wheel state (these will correspond to the state of the "dummy" spindle).
    // In Synchronize, the tire requests the WheelState from the associated wheel object, which will calculate it
    // based on the state of the "dummy" spindle, thus reproducing ws_ISO.
    m_tire->Synchronize(time, m_terrain);

    // Apply tire forces to spindle body
    m_spindle->EmptyAccumulators();
    m_spindle->AccumulateForce(m_tireforce.force, m_tireforce.point, false);
    m_spindle->AccumulateTorque(m_tireforce.moment, false);

    m_tire->Advance(step);
    m_sys->DoStepDynamics(step);
}

// Debug: inject wheel state from outside (e.g. from the ISO model)
void MechanismYUP::Advance(double step, const WheelState& ws_ISO) {
    double time = m_sys->GetChTime();

    // Overwrite state of "dummy" spindle body
    m_spindle_dummy->SetPos(ws_ISO.pos);
    m_spindle_dummy->SetRot(ws_ISO.rot);
    m_spindle_dummy->SetPosDt(ws_ISO.lin_vel);
    m_spindle_dummy->SetAngVelParent(ws_ISO.ang_vel);

    // Get tire forces in ISO frame and convert to YUP frame
    auto tf_ISO = m_tire->ReportTireForce(&m_terrain);
    m_tireforce = ConvertForce_ISO_to_YUP(tf_ISO);

    // Let tire update itself given current wheel state (these will correspond to the state of the "dummy" spindle).
    // In Synchronize, the tire requests the WheelState from the associated wheel object, which will calculate it
    // based on the state of the "dummy" spindle, thus reproducing ws_ISO.
    m_tire->Synchronize(time, m_terrain);

    // Apply tire forces to spindle body
    m_spindle->EmptyAccumulators();
    m_spindle->AccumulateForce(m_tireforce.force, m_tireforce.point, false);
    m_spindle->AccumulateTorque(m_tireforce.moment, false);

    m_tire->Advance(step);
    m_sys->DoStepDynamics(step);
}

// =============================================================================

void TestConversions() {
    WheelState ws_YUP;
    ws_YUP.pos = ChVector3d(1, 2, 3);
    ws_YUP.rot = QuatFromAngleZ(CH_PI / 6);
    ws_YUP.lin_vel = ChVector3d(0.1, 0.2, 0.3);
    ws_YUP.ang_vel = ChVector3d(0, 0, 10);
    auto ang_vel_loc = ws_YUP.rot.RotateBack(ws_YUP.ang_vel);
    ws_YUP.omega = ang_vel_loc.z();
    ChMatrix33<> R_YUP(ws_YUP.rot);

    WheelState ws_ISO = ConvertState_YUP_to_ISO(ws_YUP);
    ChMatrix33<> R_ISO(ws_ISO.rot);

    TerrainForce tf_ISO;
    tf_ISO.point = ChVector3d(1, 2, 3);
    tf_ISO.force = ChVector3d(0.1, 0.2, 0.3);
    tf_ISO.moment = ChVector3d(10, 20, 30);

    TerrainForce tf_YUP = ConvertForce_ISO_to_YUP(tf_ISO);
}

// =============================================================================

int main(int argc, char* argv[]) {
    ////TestConversions();

    ChSystemNSC sysISO;
    sysISO.SetGravitationalAcceleration(ChVector3d(0, 0, -grav));
    sysISO.GetSolver()->AsIterative()->SetMaxIterations(150);
    sysISO.SetMaxPenetrationRecoverySpeed(4.0);
    sysISO.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    MechanismISO mISO(&sysISO);

    ChSystemNSC sysYUP;
    sysYUP.SetGravitationalAcceleration(ChVector3d(0, -grav, 0));
    sysYUP.GetSolver()->AsIterative()->SetMaxIterations(150);
    sysYUP.SetMaxPenetrationRecoverySpeed(4.0);
    sysYUP.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    MechanismYUP mYUP(&sysYUP);

    auto visISO = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    visISO->SetWindowSize(800, 600);
    visISO->SetWindowTitle("Tire ISO csys");
    visISO->Initialize();
    visISO->AddLogo();
    visISO->AddSkyBox();
    visISO->AddCamera(ChVector3d(1, 1, 3));
    visISO->AddTypicalLights();
    visISO->AttachSystem(&sysISO);

    auto visYUP = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    visYUP->SetWindowSize(800, 600);
    visYUP->SetWindowTitle("Tire YUP csys");
    visYUP->Initialize();
    visYUP->AddLogo();
    visYUP->AddSkyBox();
    visYUP->AddCamera(ChVector3d(1, 1, 3));
    visYUP->AddTypicalLights();
    visYUP->AttachSystem(&sysYUP);

    while (visISO->Run()) {
        visISO->BeginScene();
        visISO->Render();
        irrlicht::tools::drawAllCOGs(visISO.get(), 1);
        visISO->EndScene();

        visYUP->BeginScene();
        visYUP->Render();
        irrlicht::tools::drawAllCOGs(visYUP.get(), 1);
        visYUP->EndScene();

        mISO.Advance(step_size);
        mYUP.Advance(step_size);

        auto wsISO = mISO.GetWheelState();
        auto wsYUP = mYUP.GetWheelState();

        auto tfISO = mISO.GetTireForce();
        auto tfYUP = mYUP.GetTireForce();

        std::cout << "\n" << sysISO.GetChTime() << std::endl;
        std::cout << "Wheel states (pos, rot, lin vel, ang vel)" << std::endl;
        std::cout << "   ISO  " << wsISO.pos << std::endl;
        std::cout << "        " << wsISO.rot << std::endl;
        std::cout << "        " << wsISO.lin_vel << std::endl;
        std::cout << "        " << wsISO.ang_vel << std::endl;
        std::cout << "   YUP  " << wsYUP.pos << std::endl;
        std::cout << "        " << wsYUP.rot << std::endl;
        std::cout << "        " << wsYUP.lin_vel << std::endl;
        std::cout << "        " << wsYUP.ang_vel << std::endl;
        std::cout << "Tire forces (force, point, moment)" << std::endl;
        std::cout << "   ISO  " << tfISO.force << std::endl;
        std::cout << "        " << tfISO.point << std::endl;
        std::cout << "        " << tfISO.moment << std::endl;
        std::cout << "   YUP  " << tfYUP.force << std::endl;
        std::cout << "        " << tfYUP.point << std::endl;
        std::cout << "        " << tfYUP.moment << std::endl;
    }

    return 0;
}
