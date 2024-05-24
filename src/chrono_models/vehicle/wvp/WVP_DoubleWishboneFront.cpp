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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist
// =============================================================================
//
// Sedan concrete double wishbone suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include <vector>
#include <algorithm>

#include "chrono_models/vehicle/wvp/WVP_DoubleWishboneFront.h"
#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double WVP_DoubleWishboneFront::m_spindleMass = 5.0;
const double WVP_DoubleWishboneFront::m_uprightMass = 150.0;
const double WVP_DoubleWishboneFront::m_UCAMass = 14.0;
const double WVP_DoubleWishboneFront::m_LCAMass = 42.0;

const double WVP_DoubleWishboneFront::m_spindleRadius = 0.1;
const double WVP_DoubleWishboneFront::m_spindleWidth = 0.02;
const double WVP_DoubleWishboneFront::m_uprightRadius = 0.025;
const double WVP_DoubleWishboneFront::m_UCARadius = 0.02;
const double WVP_DoubleWishboneFront::m_LCARadius = 0.03;

const ChVector3d WVP_DoubleWishboneFront::m_spindleInertia(0.000478, 0.000496, 0.000478);
const ChVector3d WVP_DoubleWishboneFront::m_uprightInertiaMoments(0.0138, 0.0146, 0.0028);
const ChVector3d WVP_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);
const ChVector3d WVP_DoubleWishboneFront::m_UCAInertiaMoments(0.0058, 0.0023, 0.0074);
const ChVector3d WVP_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector3d WVP_DoubleWishboneFront::m_LCAInertiaMoments(0.0151, 0.0216, 0.0346);
const ChVector3d WVP_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);

const double WVP_DoubleWishboneFront::m_axleInertia = 0.4;

const double WVP_DoubleWishboneFront::m_springCoefficient = 562219.8646;
const double WVP_DoubleWishboneFront::m_springRestLength = 0.89;
const double WVP_DoubleWishboneFront::m_springMaxLength = 1.061772808;
const double WVP_DoubleWishboneFront::m_springMinLength = 0.706772808;

// simple linear damper model
const double WVP_DoubleWishboneFront::m_dampingCoefficient = 72374.57157;

// more realistic damper model
const double WVP_DoubleWishboneFront::m_damperCoefExpansion = WVP_DoubleWishboneFront::m_dampingCoefficient;
const double WVP_DoubleWishboneFront::m_damperDegresExpansion = 4.0;
const double WVP_DoubleWishboneFront::m_damperCoefCompression = 0.6 * WVP_DoubleWishboneFront::m_damperCoefExpansion;
const double WVP_DoubleWishboneFront::m_damperDegresCompression = 4.0;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
WVP_DoubleWishboneFront::WVP_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name) {
    const std::vector<std::pair<double, double>> data_bump{
        {0.0000, 0},     {0.0075, 126},   {0.0125, 315},   {0.0180, 1305},  {0.0205, 1800},
        {0.0225, 2700},  {0.0228, 2979},  {0.0232, 3780},  {0.0240, 8280},  {0.0245, 11070},
        {0.0250, 13950}, {0.0255, 17100}, {0.0260, 21150}, {0.0265, 26100}, {0.0270, 31050}};
    const std::vector<std::pair<double, double>> data_jounce{{0.0000, 0},    {0.0010, 2000}, {0.0020, 4000},
                                                             {0.0030, 6000}, {0.0040, 8000}, {0.0050, 10000}};
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient);
    auto force = std::static_pointer_cast<SpringForce>(m_springForceCB);
    force->set_stops(data_bump, data_jounce);
    force->enable_stops(m_springMinLength, m_springMaxLength);
    // m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient); simple linear = unrealistic
    m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(
        m_damperCoefCompression, m_damperDegresCompression, m_damperCoefExpansion, m_damperDegresExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
WVP_DoubleWishboneFront::~WVP_DoubleWishboneFront() {}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector3d WVP_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector3d(0.0000, 1.0330, 0.0000);  // location of spindle center of mass
        case UPRIGHT:
            return ChVector3d(0.0000, 0.95, 0.0000);  // location of upright center of mass
        case UCA_F:
            return ChVector3d(0.3092, 0.1922, 0.3646);  // UCA front connection point to chassis
        case UCA_B:
            return ChVector3d(-0.1488, 0.1922, 0.3646);  // UCA rear (back) connection point to chassis
        case UCA_U:
            return ChVector3d(-0.0080, 0.7722, 0.2337);  // UCA connection point to upright
        case UCA_CM:
            return ChVector3d(0.0410, 0.4280, 0.3110);  // location of UCA center of mass
        case LCA_F:
            return ChVector3d(0.3600, 0.1922, 0.0118);  // LCA front connection point to chassis
        case LCA_B:
            return ChVector3d(-0.2496, 0.1922, 0.0118);  // LCA rear (back) connection point to chassis
        case LCA_U:
            return ChVector3d(0.0070, 0.8322, -0.1520);  // LCA connection point to upright
        case LCA_CM:
            return ChVector3d(0.0391, 0.4060, -0.0428);  // location of LCA center of mass
        case SHOCK_C:
            return ChVector3d(-0.1070, 0.5400, 0.7237);  // shock connection to chassis
        case SHOCK_A:
            return ChVector3d(-0.1070, 0.5474, -0.1108);  // shock connection point to LCA
        case SPRING_C:
            return ChVector3d(-0.1070, 0.5400, 0.7237);  // spring connection point to chassis
        case SPRING_A:
            return ChVector3d(-0.1070, 0.5474, -0.1108);  // spring connection point to LCA
        case TIEROD_C:
            return ChVector3d(-0.3834, 0.1250, 0.0292);  // tierod connection point to chassis
        case TIEROD_U:
            return ChVector3d(-0.3984, 0.7500, -0.1278);  // tierod connection point to upright
        default:
            return ChVector3d(0, 0, 0);
    }
}

}  // namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
