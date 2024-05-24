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
// Authors: Radu Serban, Mike Taylor, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/generic/powertrain/Generic_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace generic {

const double rpm2rads = CH_PI / 30;

Generic_EngineSimpleMap::Generic_EngineSimpleMap(const std::string& name)
    : ChEngineSimpleMap(name) {}

double Generic_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2700 * rpm2rads;
}

void Generic_EngineSimpleMap::SetEngineTorqueMaps(chrono::ChFunctionInterp& map0, chrono::ChFunctionInterp& mapF) {
    map0.AddPoint(-10.472, 0.000);
    map0.AddPoint(83.776, -20.0);
    map0.AddPoint(104.720, -20.0);
    map0.AddPoint(125.664, -30.0);
    map0.AddPoint(146.608, -30.0);
    map0.AddPoint(167.552, -30.0);
    map0.AddPoint(188.496, -40.0);
    map0.AddPoint(209.440, -50.0);
    map0.AddPoint(230.383, -70.0);
    map0.AddPoint(251.327, -100.0);
    map0.AddPoint(282.743, -800.0);

    mapF.AddPoint(-10.472, 406.7);
    mapF.AddPoint(83.776, 517.9);
    mapF.AddPoint(104.720, 926.0);
    mapF.AddPoint(125.664, 1216.2);
    mapF.AddPoint(146.608, 1300.2);
    mapF.AddPoint(167.552, 1300.2);
    mapF.AddPoint(188.496, 1227.0);
    mapF.AddPoint(209.440, 1136.2);
    mapF.AddPoint(230.383, 1041.3);
    mapF.AddPoint(251.327, -271.2);
    mapF.AddPoint(282.743, -800.0);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
