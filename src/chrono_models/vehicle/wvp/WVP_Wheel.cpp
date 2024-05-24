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
// Authors: Radu Serban, Justin Madsen, Asher Elmquist
// =============================================================================
//
// WVP wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/wvp/WVP_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_Wheel::m_mass = 52.0;
const ChVector3d WVP_Wheel::m_inertia(2.2170, 3.2794, 2.2170);

const double WVP_Wheel::m_radius = 0.254;
const double WVP_Wheel::m_width = 0.254;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_Wheel::WVP_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "wvp/tire/WVP_Wheel.obj";
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
