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

#include "chrono_models/vehicle/wvp/WVP_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace wvp {

const double rpm2rads = CH_PI / 30;
const double drive_eff = 0.5137;  // based on auxillary power consumption and gearbox efficiencies

WVP_EngineSimpleMap::WVP_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double WVP_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2700 * rpm2rads;
}

void WVP_EngineSimpleMap::SetEngineTorqueMaps(ChFunctionInterp& map0, ChFunctionInterp& mapF) {
    map0.AddPoint(-100 * rpm2rads, 0.0);
    map0.AddPoint(800 * rpm2rads, -20.0);
    map0.AddPoint(1000 * rpm2rads, -20.0);
    map0.AddPoint(1200 * rpm2rads, -30.0);
    map0.AddPoint(1400 * rpm2rads, -30.0);
    map0.AddPoint(1600 * rpm2rads, -30.0);
    map0.AddPoint(1800 * rpm2rads, -40.0);
    map0.AddPoint(2000 * rpm2rads, -50.0);
    map0.AddPoint(2200 * rpm2rads, -70.0);
    map0.AddPoint(2400 * rpm2rads, -100.0);
    map0.AddPoint(2700 * rpm2rads, -800.0);

    mapF.AddPoint(-100 * rpm2rads, 806.7 * drive_eff);
    mapF.AddPoint(800 * rpm2rads, 817.9 * drive_eff);
    mapF.AddPoint(1000 * rpm2rads, 926.0 * drive_eff);
    mapF.AddPoint(1200 * rpm2rads, 1216.2 * drive_eff);
    mapF.AddPoint(1400 * rpm2rads, 1300.2 * drive_eff);
    mapF.AddPoint(1600 * rpm2rads, 1300.2 * drive_eff);
    mapF.AddPoint(1800 * rpm2rads, 1227.0 * drive_eff);
    mapF.AddPoint(2000 * rpm2rads, 1136.2 * drive_eff);
    mapF.AddPoint(2200 * rpm2rads, 1041.3 * drive_eff);
    mapF.AddPoint(2400 * rpm2rads, 0);
    mapF.AddPoint(2700 * rpm2rads, -200.0);
}

}  // namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
