// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace wvp {

const double rpm2rads = CH_PI / 30;

WVP_AutomaticTransmissionSimpleMap::WVP_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void WVP_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.2;

    fwd.push_back(0.1708);  // 1st
    fwd.push_back(0.2791);  // 2nd
    fwd.push_back(0.4218);  // 3rd
    fwd.push_back(0.6223);  // 4th
    fwd.push_back(1.0173);  // 5th
    fwd.push_back(1.5361);  // 6th
}

void WVP_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2700 * rpm2rads));
}

}  // namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
