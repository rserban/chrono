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
// WVP TMeasy tire subsystem
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

const std::string WVP_TMeasyTire::m_meshName = "hmmwv_tire_POV_geom";
const std::string WVP_TMeasyTire::m_meshFile = "hmmwv/hmmwv_tire.obj";

const double WVP_TMeasyTire::m_mass = 71.1;
const ChVector<> WVP_TMeasyTire::m_inertia(9.62, 16.84, 9.62);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WVP_TMeasyTire::WVP_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMeasyTire::SetTMeasyParams() {
	// Set Handling Charecteristics as close as possible to Pacejka89 model data
    
    m_TMeasyCoeff.pn = GetTireMaxLoad(152)/2.0;
    m_TMeasyCoeff.mu_0 = 0.8;
    
    // Set nonlinear vertical stiffness
    std::vector<double> disp, force;
    disp.push_back(0.01); force.push_back(3276.0);
    disp.push_back(0.02); force.push_back(6729.0);
    disp.push_back(0.03); force.push_back(10361.0);
    disp.push_back(0.04); force.push_back(14171.0);
    disp.push_back(0.05); force.push_back(18159.0);
    disp.push_back(0.06); force.push_back(22325.0);
    disp.push_back(0.07); force.push_back(26670.0);
    disp.push_back(0.08); force.push_back(31192.0);
    disp.push_back(0.09); force.push_back(35893.0);
    disp.push_back(0.10); force.push_back(40772.0);
    
    SetVerticalStiffness(disp,force);
    
    m_width 		  	 = 0.372;
    m_unloaded_radius 	 = 1.096 / 2.0;
    m_rolling_resistance = 0.015;
    m_TMeasyCoeff.mu_0 	 = 0.8;
    
 	// Simple damping model from single mass oscilator
    double xi = 0.05; // tire damping ratio
	double C1 = 1000.0*sqrt(pow(m_a1,2)+4.0*m_a2*m_TMeasyCoeff.pn/1000);
	double C2 = 1000.0*sqrt(pow(m_a1,2)+8.0*m_a2*m_TMeasyCoeff.pn/1000);
	double CZM = (C1 + C2) / 2.0;
	
	m_TMeasyCoeff.dz 		 =  2.0 * xi * sqrt(CZM * WVP_TMeasyTire::m_mass);

	m_TMeasyCoeff.dfx0_pn    =  215.013101;
	m_TMeasyCoeff.sxm_pn     =  0.139216;
	m_TMeasyCoeff.fxm_pn     =  13.832671;
	m_TMeasyCoeff.sxs_pn     =  0.500000;
	m_TMeasyCoeff.fxs_pn     =  12.387002;
	m_TMeasyCoeff.dfx0_p2n   =  615.071218;
	m_TMeasyCoeff.sxm_p2n    =  0.112538;
	m_TMeasyCoeff.fxm_p2n    =  24.576280;
	m_TMeasyCoeff.sxs_p2n    =  0.800000;
	m_TMeasyCoeff.fxs_p2n    =  22.079064;
	m_TMeasyCoeff.dfy0_pn    =  168.715739;
	m_TMeasyCoeff.sym_pn     =  0.258411;
	m_TMeasyCoeff.fym_pn     =  13.038949;
	m_TMeasyCoeff.sys_pn     =  0.800000;
	m_TMeasyCoeff.fys_pn     =  12.387002;
	m_TMeasyCoeff.dfy0_p2n   =  320.100587;
	m_TMeasyCoeff.sym_p2n    =  0.238826;
	m_TMeasyCoeff.fym_p2n    =  23.241121;
	m_TMeasyCoeff.sys_p2n    =  1.000000;
	m_TMeasyCoeff.fys_p2n    =  22.079064;
	m_TMeasyCoeff.nto0_pn    =  0.160000;
	m_TMeasyCoeff.synto0_pn  =  0.180000;
	m_TMeasyCoeff.syntoE_pn  =  0.480000;
	m_TMeasyCoeff.nto0_p2n   =  0.170000;
	m_TMeasyCoeff.synto0_p2n =  0.160000;
	m_TMeasyCoeff.syntoE_p2n =  0.500000;


	if(CheckParameters()) { 
		std::cout << "Parameter Set seems to be ok." << std::endl;
	} else {
		std::cout << "Badly formed parameter set." << std::endl;
	}
	
	if(GetName().compare("FL") == 0) {
		GenerateCharacteristicPlots("../");
	}
}

void WVP_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/365_80R20_" + GetName() + ".gpl";
    WritePlots(filename, "365/80R20 152K");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WVP_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void WVP_TMeasyTire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by WVP_FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono