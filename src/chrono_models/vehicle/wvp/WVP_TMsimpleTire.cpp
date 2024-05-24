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
// Authors: Rainer Gericke
// =============================================================================
//
// U401 TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/wvp/WVP_TMsimpleTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string WVP_TMsimpleTire::m_meshFile_left = "wvp/tire/WVP_Tire.obj";
const std::string WVP_TMsimpleTire::m_meshFile_right = "wvp/tire/WVP_Tire.obj";

const double WVP_TMsimpleTire::m_mass = 71.1;
const ChVector3d WVP_TMsimpleTire::m_inertia(9.8713, 18.1640, 9.8713);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_TMsimpleTire::WVP_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMsimpleTire::SetTMsimpleParams() {
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

void WVP_TMsimpleTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/365.80R20_" + GetName() + ".gpl";
    WritePlots(filename, "365/80R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void WVP_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
