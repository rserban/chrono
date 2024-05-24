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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// WVP rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_RackPinion::m_steeringLinkMass = 1.889;
const ChVector3d WVP_RackPinion::m_steeringLinkInertia(.138, 0.00009, .138);
const double WVP_RackPinion::m_steeringLinkCOM = 0;
//const double WVP_RackPinion::m_steeringLinkLength = 0.896;
const double WVP_RackPinion::m_steeringLinkLength = 0.25;
const double WVP_RackPinion::m_steeringLinkRadius = 0.03;

const double WVP_RackPinion::m_pinionRadius = 0.03;

const double WVP_RackPinion::m_maxAngle = 0.08 / 0.03;


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_RackPinion::WVP_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
