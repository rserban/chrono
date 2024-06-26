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
// Authors: Radu Serban
// =============================================================================
//
// Double-A arm suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a double wishbone suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
DoubleWishbone::DoubleWishbone(const std::string& filename)
    : ChDoubleWishbone(""),
      m_springForceCB(nullptr),
      m_shockForceCB(nullptr),
      m_UCABushingData(nullptr),
      m_LCABushingData(nullptr),
      m_tierodBushingData(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

DoubleWishbone::DoubleWishbone(const rapidjson::Document& d)
    : ChDoubleWishbone(""),
      m_springForceCB(nullptr),
      m_shockForceCB(nullptr),
      m_UCABushingData(nullptr),
      m_LCABushingData(nullptr),
      m_tierodBushingData(nullptr) {
    Create(d);
}

DoubleWishbone::~DoubleWishbone() {}

// -----------------------------------------------------------------------------
// Worker function for creating a DoubleWishbone suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void DoubleWishbone::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read flag indicating that inertia matrices are expressed in
    // vehicle-aligned centroidal frame.
    if (d.HasMember("Vehicle-Frame Inertia")) {
        bool flag = d["Vehicle-Frame Inertia"].GetBool();
        SetVehicleFrameInertiaFlag(flag);
    }

    if (d.HasMember("Camber Angle (deg)"))
        m_camber_angle = d["Camber Angle (deg)"].GetDouble() * CH_DEG_TO_RAD;
    else
        m_camber_angle = 0;

    if (d.HasMember("Toe Angle (deg)"))
        m_toe_angle = d["Toe Angle (deg)"].GetDouble() * CH_DEG_TO_RAD;
    else
        m_toe_angle = 0;

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_points[SPINDLE] = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    // Read Upright data
    assert(d.HasMember("Upright"));
    assert(d["Upright"].IsObject());

    m_uprightMass = d["Upright"]["Mass"].GetDouble();
    m_points[UPRIGHT] = ReadVectorJSON(d["Upright"]["COM"]);
    m_uprightInertiaMoments = ReadVectorJSON(d["Upright"]["Moments of Inertia"]);
    m_uprightInertiaProducts = ReadVectorJSON(d["Upright"]["Products of Inertia"]);
    m_uprightRadius = d["Upright"]["Radius"].GetDouble();

    // Read UCA data
    assert(d.HasMember("Upper Control Arm"));
    assert(d["Upper Control Arm"].IsObject());

    m_UCAMass = d["Upper Control Arm"]["Mass"].GetDouble();
    m_points[UCA_CM] = ReadVectorJSON(d["Upper Control Arm"]["COM"]);
    m_UCAInertiaMoments = ReadVectorJSON(d["Upper Control Arm"]["Moments of Inertia"]);
    m_UCAInertiaProducts = ReadVectorJSON(d["Upper Control Arm"]["Products of Inertia"]);
    m_UCARadius = d["Upper Control Arm"]["Radius"].GetDouble();
    m_points[UCA_F] = ReadVectorJSON(d["Upper Control Arm"]["Location Chassis Front"]);
    m_points[UCA_B] = ReadVectorJSON(d["Upper Control Arm"]["Location Chassis Back"]);
    m_points[UCA_U] = ReadVectorJSON(d["Upper Control Arm"]["Location Upright"]);
    if (d["Upper Control Arm"].HasMember("Bushing Data")) {
        m_UCABushingData = ReadBushingDataJSON(d["Upper Control Arm"]["Bushing Data"]);
    }

    // Read LCA data
    assert(d.HasMember("Lower Control Arm"));
    assert(d["Lower Control Arm"].IsObject());

    m_LCAMass = d["Lower Control Arm"]["Mass"].GetDouble();
    m_points[LCA_CM] = ReadVectorJSON(d["Lower Control Arm"]["COM"]);
    m_LCAInertiaMoments = ReadVectorJSON(d["Lower Control Arm"]["Moments of Inertia"]);
    m_LCAInertiaProducts = ReadVectorJSON(d["Lower Control Arm"]["Products of Inertia"]);
    m_LCARadius = d["Lower Control Arm"]["Radius"].GetDouble();
    m_points[LCA_F] = ReadVectorJSON(d["Lower Control Arm"]["Location Chassis Front"]);
    m_points[LCA_B] = ReadVectorJSON(d["Lower Control Arm"]["Location Chassis Back"]);
    m_points[LCA_U] = ReadVectorJSON(d["Lower Control Arm"]["Location Upright"]);
    if (d["Lower Control Arm"].HasMember("Bushing Data")) {
        m_LCABushingData = ReadBushingDataJSON(d["Lower Control Arm"]["Bushing Data"]);
    }

    // Read Tierod data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    if (d["Tierod"].HasMember("Mass")) {
        assert(d["Tierod"].HasMember("Inertia"));
        assert(d["Tierod"].HasMember("Radius"));
        m_tierodMass = d["Tierod"]["Mass"].GetDouble();
        m_tierodRadius = d["Tierod"]["Radius"].GetDouble();
        m_tierodInertia = ReadVectorJSON(d["Tierod"]["Inertia"]);
        m_use_tierod_bodies = true;
        if (d["Tierod"].HasMember("Bushing Data")) {
            m_tierodBushingData = ReadBushingDataJSON(d["Tierod"]["Bushing Data"]);
        }
    } else {
        m_tierodMass = 0;
        m_tierodRadius = 0;
        m_tierodInertia = ChVector3d(0);
        m_use_tierod_bodies = false;
    }

    m_points[TIEROD_C] = ReadVectorJSON(d["Tierod"]["Location Chassis"]);
    m_points[TIEROD_U] = ReadVectorJSON(d["Tierod"]["Location Upright"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());
    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Arm"]);
    m_springForceCB = ReadTSDAFunctorJSON(d["Spring"], m_springRestLength);

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());
    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Arm"]);
    m_shockForceCB = ReadTSDAFunctorJSON(d["Shock"], m_shockRestLength);

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
