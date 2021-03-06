// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// WVP powertrain model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/wvp/WVP_Powertrain.h"

namespace chrono {
namespace vehicle {
namespace wvp {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double WVP_Powertrain::m_motorblock_inertia = 10.5;
const double WVP_Powertrain::m_crankshaft_inertia = 1.1;
const double WVP_Powertrain::m_ingear_shaft_inertia = 0.3;

// -----------------------------------------------------------------------------
// Constructor of the HMMW_Powertrain.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
WVP_Powertrain::WVP_Powertrain() : ChShaftsPowertrain("Powertrain", ChVector<>(1, 0, 0)) {}

// -----------------------------------------------------------------------------
// Initialize vector of gear ratios
// -----------------------------------------------------------------------------
void WVP_Powertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.2;  // reverse gear;

    fwd.push_back(0.1708);  // 1: 1st gear;
    fwd.push_back(0.2791);  // 2: 2nd gear;
    fwd.push_back(0.4218);  // 3: 3rd gear;
    fwd.push_back(0.6223);  // 4: 4th gear;
    fwd.push_back(1.0173);  // 5: 5th gear;
    fwd.push_back(1.5361);  // 6: 6th gear;
}

// -----------------------------------------------------------------------------
// Set the engine and torque converter maps:
//
// (1) engine speed [rad/s] - torque [Nm] map
//     must be defined beyond max speed too - engine might be 'pulled'
//
// (2) TC capacity factor map
//
// (3) TC torque ratio map
//
// -----------------------------------------------------------------------------
void WVP_Powertrain::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;
    double ftlb_to_Nm = 1.355818;

    map->AddPoint(-100 * rpm_to_radsec, 300*ftlb_to_Nm);  // to start engine
    /*map->AddPoint(800 * rpm_to_radsec, 382*ftlb_to_Nm); //*/
    map->AddPoint(1000 * rpm_to_radsec, 683*ftlb_to_Nm);
    map->AddPoint(1200 * rpm_to_radsec, 897*ftlb_to_Nm);
    map->AddPoint(1400 * rpm_to_radsec, 959*ftlb_to_Nm);
    map->AddPoint(1600 * rpm_to_radsec, 959*ftlb_to_Nm);
    map->AddPoint(1800 * rpm_to_radsec, 905*ftlb_to_Nm);
    map->AddPoint(2000 * rpm_to_radsec, 838*ftlb_to_Nm);
    map->AddPoint(2200 * rpm_to_radsec, 768*ftlb_to_Nm);
    
    map->AddPoint(2400 * rpm_to_radsec, -200*ftlb_to_Nm); //
    /*map->AddPoint(2700 * rpm_to_radsec, -200*ftlb_to_Nm);  // fading out of engine torque*/
}

void WVP_Powertrain::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
}

void WVP_Powertrain::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 15);
    map->AddPoint(0.25, 15);
    map->AddPoint(0.50, 15);
    map->AddPoint(0.75, 16);
    map->AddPoint(0.90, 18);
    map->AddPoint(1.00, 35);
    /*
        map->AddPoint(0     ,   81.0000);
        map->AddPoint(0.1000,   81.1589);
        map->AddPoint(0.2000,   81.3667);
        map->AddPoint(0.3000,   81.6476);
        map->AddPoint(0.4000,   82.0445);
        map->AddPoint(0.5000,   82.6390);
        map->AddPoint(0.6000,   83.6067);
        map->AddPoint(0.7000,   85.3955);
        map->AddPoint(0.8000,   89.5183);
        map->AddPoint(0.9000,  105.1189);
        map->AddPoint(0.9700,  215.5284);
        map->AddPoint(1.0000,  235.5284);
    */
}

void WVP_Powertrain::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(1.0, 1.0);
    /*map->AddPoint(0.25, 1.80);
    map->AddPoint(0.50, 1.50);
    map->AddPoint(0.75, 1.15);
    map->AddPoint(1.00, 1.00);*/
    /*
        map->AddPoint(0,        1.7500);
        map->AddPoint(0.1000,    1.6667);
        map->AddPoint(0.2000,    1.5833);
        map->AddPoint(0.3000,    1.5000);
        map->AddPoint(0.4000,    1.4167);
        map->AddPoint(0.5000,    1.3334);
        map->AddPoint(0.6000,    1.2500);
        map->AddPoint(0.7000,    1.1667);
        map->AddPoint(0.8000,    1.0834);
        map->AddPoint(0.9000,    1.0000);
        map->AddPoint(0.9700,    1.0000);
        map->AddPoint(1.0000,    1.0000);
    */
}

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono
