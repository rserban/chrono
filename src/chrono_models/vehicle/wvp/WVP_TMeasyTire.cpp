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
// Authors: Rainer Gericke, Asher Elmquist
// =============================================================================
//
// Sedan TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/wvp/WVP_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string WVP_TMeasyTire::m_meshFile = "wvp/tire/WVP_Tire.obj";

const double WVP_TMeasyTire::m_mass = 71.1;
const ChVector3d WVP_TMeasyTire::m_inertia(9.8713, 18.1640, 9.8713);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_TMeasyTire::WVP_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMeasyTire::SetTMeasyParams() {
    const double in2m = 0.0254;

    // Tire 365/80 R20 152 K
    unsigned int li = 152;
    double w = 0.365;
    double r = 0.80;
    double rimdia = 20.0 * in2m;
    double pres_li = 800000;
    double pres_use = 234000;

    GuessTruck80Par(li,       // tire load index []
                      w,        // tire width [m]
                      r,        // aspect ratio []
                      rimdia,   // rim diameter [m],
                      pres_li,  // infl. pressure for load index
                      pres_use  // infl. pressure for usage
    );
}

void WVP_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/365.80R20_" + GetName() + ".gpl";
    WritePlots(filename, "365.80R28");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void WVP_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
