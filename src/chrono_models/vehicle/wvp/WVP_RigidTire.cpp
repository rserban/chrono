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
// WVP wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_models/vehicle/wvp/WVP_RigidTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_RigidTire::m_radius = 1.096 / 2.0;
const double WVP_RigidTire::m_width = 0.372;

const double WVP_RigidTire::m_mass = 71.1;
const ChVector<> WVP_RigidTire::m_inertia(9.62, 16.84, 9.62);

const std::string WVP_RigidTire::m_meshFile = "wvp/wvp_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_RigidTire::WVP_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    SetContactFrictionCoefficient(0.8f);
    SetContactRestitutionCoefficient(0.5f);
    SetContactMaterialProperties(2e7f, 0.3f);
    /*SetContactMaterialProperties(2e7f, 0.3f);*/
    /*SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);*/
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);

    if (use_mesh) {
        SetMeshFilename(GetDataFile("wvp/wvp_tire_fine.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(vehicle::GetDataFile(m_meshFile),   // left side
                                               vehicle::GetDataFile(m_meshFile));  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void WVP_RigidTire::RemoveVisualizationAssets() {
    ChRigidTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono