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
// UAZBUS rotary arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/gclass/G500_RotaryArm.h"

namespace chrono {
namespace vehicle {
namespace gclass {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double G500_RotaryArm::m_pitmanArmMass = 1.605;

const double G500_RotaryArm::m_pitmanArmRadius = 0.02;

const double G500_RotaryArm::m_maxAngle = 12.5 * (CH_PI / 180);

const ChVector3d G500_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector3d G500_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
G500_RotaryArm::G500_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector3d G500_RotaryArm::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector3d(0.6, 0.7325 - 0.2, 0.2);
        case ARM_C:
            return ChVector3d(0.6, 0.7325 - 0.2, 0.4);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d G500_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector3d(0, 1, 0);
        default:
            return ChVector3d(0, 1, 0);
    }
}

}  // namespace gclass
}  // end namespace vehicle
}  // end namespace chrono
