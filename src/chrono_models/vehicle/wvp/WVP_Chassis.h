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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// WVP chassis subsystem.
//
// =============================================================================

#ifndef WVP_CHASSIS_H
#define WVP_CHASSIS_H

#include <string>

#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace wvp {

/// WVP chassis subsystem.
class CH_MODELS_API WVP_Chassis : public ChRigidChassis {
  public:
    WVP_Chassis(const std::string& name,
                bool fixed = false,
                CollisionType chassis_collision_type = CollisionType::NONE);
    ~WVP_Chassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }
  
    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }
  
    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_inertiaXY;
    static const ChVector<> m_COM_loc;
    static const ChCoordsys<> m_driverCsys;
};

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono

#endif
