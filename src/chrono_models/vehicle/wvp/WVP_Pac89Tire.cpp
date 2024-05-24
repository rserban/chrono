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
// Authors: Radu Serban, Michael Taylor, Asher Elmquist
// =============================================================================
//
// WVP PAC89 tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/wvp/WVP_Pac89Tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double WVP_Pac89Tire::m_normalDamping = 8154 * .01;

const double WVP_Pac89Tire::m_mass = 71.1;
const ChVector3d WVP_Pac89Tire::m_inertia(9.8713, 18.1640, 9.8713);

const std::string WVP_Pac89Tire::m_meshFile_left = "wvp/tire/WVP_Tire.obj";
const std::string WVP_Pac89Tire::m_meshFile_right = "wvp/tire/WVP_Tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_Pac89Tire::WVP_Pac89Tire(const std::string& name) : ChPac89Tire(name) {
    ChPac89Tire::SetGammaLimit(3);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_Pac89Tire::SetPac89Params() {
    m_unloaded_radius = 1.0960 / 2;  // Diameter to Radius
    m_width = 0.372;
    m_rolling_resistance = 0.015;       // 0.0;
    m_lateral_stiffness = 185 * 1000.;  // N/mm -> N/m
    m_measured_side = LEFT;

    m_PacCoeff.A0 = 1.0012;
    m_PacCoeff.A1 = -0.6774;
    m_PacCoeff.A2 = 1017.452;
    m_PacCoeff.A3 = 5707.292;
    m_PacCoeff.A4 = 62.6561;
    m_PacCoeff.A5 = 0.0;
    m_PacCoeff.A6 = 0.008;
    m_PacCoeff.A7 = 0.9419;
    m_PacCoeff.A8 = 0.1124;
    m_PacCoeff.A9 = -0.0026;
    m_PacCoeff.A10 = -0.1197;
    m_PacCoeff.A11 = -17.2366;
    m_PacCoeff.A12 = 9.9733;
    m_PacCoeff.A13 = -10.7445;

    m_PacCoeff.B0 = 1.0000;
    m_PacCoeff.B1 = -5.0940;
    m_PacCoeff.B2 = 883.1000;
    m_PacCoeff.B3 = 0.2720;
    m_PacCoeff.B4 = 117.3000;
    m_PacCoeff.B5 = 0.000;
    m_PacCoeff.B6 = 0.0002;
    m_PacCoeff.B7 = -0.0593;
    m_PacCoeff.B8 = 0.2285;
    m_PacCoeff.B9 = 0.0026;
    m_PacCoeff.B10 = 0.9228;

    m_PacCoeff.C0 = 1.8392;
    m_PacCoeff.C1 = -0.2289;
    m_PacCoeff.C2 = -11.239;
    m_PacCoeff.C3 = -0.0298;
    m_PacCoeff.C4 = -6.6725;
    m_PacCoeff.C5 = 0.0;
    m_PacCoeff.C6 = 0.0;
    m_PacCoeff.C7 = .0035;
    m_PacCoeff.C8 = -0.282;
    m_PacCoeff.C9 = 2.0;
    m_PacCoeff.C10 = 0.3595;
    m_PacCoeff.C11 = 0.0157;
    m_PacCoeff.C12 = -0.0011;
    m_PacCoeff.C13 = -0.2298;
    m_PacCoeff.C14 = 0.0194;
    m_PacCoeff.C15 = -0.764;
    m_PacCoeff.C16 = -2.275;
    m_PacCoeff.C17 = 40.78;

    m_stiffnessMap.AddPoint(0.0, 0.0);
    m_stiffnessMap.AddPoint(10.0e-3, 3276.0);
    m_stiffnessMap.AddPoint(20.0e-3, 6729.0);
    m_stiffnessMap.AddPoint(30.0e-3, 10361.0);
    m_stiffnessMap.AddPoint(40.0e-3, 14171.0);
    m_stiffnessMap.AddPoint(50.0e-3, 18159.0);
    m_stiffnessMap.AddPoint(60.0e-3, 22325.0);
    m_stiffnessMap.AddPoint(70.0e-3, 26670.0);
    m_stiffnessMap.AddPoint(80.0e-3, 31192.0);
    m_stiffnessMap.AddPoint(90.0e-3, 35893.0);
    m_stiffnessMap.AddPoint(100.0e-3, 40772.0);
}

double WVP_Pac89Tire::GetNormalStiffnessForce(double depth) const {
    // ensure that depth is greater that 0
    if (depth < 0)
        depth = 0;

    // linear extrapolation if outside of map range
    if (depth > 0.1) {
        return 487900 * (depth - 0.1) +
               40772.0;  // extrapolate with slope between last two data points in the map (slope=487900)
    }
    // normal case - interpolate from tire map
    else {
        return m_stiffnessMap.GetVal(depth);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_Pac89Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPac89Tire::AddVisualizationAssets(vis);
    }
}

void WVP_Pac89Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac89Tire::RemoveVisualizationAssets();
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
