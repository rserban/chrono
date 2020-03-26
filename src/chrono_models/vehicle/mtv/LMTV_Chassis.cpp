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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// LMTV 2.5t chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mtv/LMTV_Chassis.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double LMTV_Chassis::m_mass = 3796;
// const ChVector<> LMTV_Chassis::m_inertiaXX(222.8, 944.1, 1053.5);
const ChVector<> LMTV_Chassis::m_inertiaXX(3.1721e3, 5.1645e3, 4.4865e3);
const ChVector<> LMTV_Chassis::m_inertiaXY(0, -0.4154e3, 0);
const ChVector<> LMTV_Chassis::m_COM_loc(-0.7079, 0, 0.6790);
const ChCoordsys<> LMTV_Chassis::m_driverCsys(ChVector<>(0.0, 0.7, 1.2), ChQuaternion<>(1, 0, 0, 0));

const double LMTV_Chassis::m_rear_mass = 1788.333;
const ChVector<> LMTV_Chassis::m_rear_inertiaXX(2.3677e3, 2.3766e3, 3.2246e3);
const ChVector<> LMTV_Chassis::m_rear_inertiaXY(0, -0.0915e3, 0);
const ChVector<> LMTV_Chassis::m_rear_COM_loc(-3.1765, 0, 0.8799);

const ChVector<> LMTV_Chassis::m_torsion_joint_pos(-1.748, 0, 0.744);
const double LMTV_Chassis::m_torsion_stiffness = 7085;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LMTV_Chassis::LMTV_Chassis(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type)
    : ChTorsionChassis(name, fixed) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    m_rear_inertia(0, 0) = m_rear_inertiaXX.x();
    m_rear_inertia(1, 1) = m_rear_inertiaXX.y();
    m_rear_inertia(2, 2) = m_rear_inertiaXX.z();

    m_rear_inertia(0, 1) = m_rear_inertiaXY.x();
    m_rear_inertia(0, 2) = m_rear_inertiaXY.y();
    m_rear_inertia(1, 2) = m_rear_inertiaXY.z();
    m_rear_inertia(1, 0) = m_rear_inertiaXY.x();
    m_rear_inertia(2, 0) = m_rear_inertiaXY.y();
    m_rear_inertia(2, 1) = m_rear_inertiaXY.z();

    //// TODO:
    //// A more appropriate contact shape from primitives
    /*
    BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.0, 0.5, 0.2));

    m_has_primitives = true;
    m_vis_boxes.push_back(box1);

    m_has_mesh = true;
    m_vis_mesh_file = "mtv/meshes/LMTV_Chassis.obj";

    m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
        case ChassisCollisionType::PRIMITIVES:
            m_coll_boxes.push_back(box1);
            break;
        case ChassisCollisionType::MESH:
            m_coll_mesh_names.push_back("mtv/meshes/LMTV_Chassis_col.obj");
            break;
    }
     */
    BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.0, 0.5, 0.2));

    m_has_collision = false;

    m_has_primitives = true;
    m_vis_boxes.push_back(box1);

    m_has_mesh = true;
    m_vis_mesh_file = "mtv/meshes/LMTV_Chassis.obj";
}

void LMTV_Chassis::Initialize(ChSystem* system,                ///< [in] containing system
                              const ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
                              double chassisFwdVel,            ///< [in] initial chassis forward velocity
                              int collision_family             ///< [in] chassis collision family
) {
    // Invoke the base class method to construct the frame body.
    ChTorsionChassis::Initialize(system, chassisPos, chassisFwdVel);
}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
