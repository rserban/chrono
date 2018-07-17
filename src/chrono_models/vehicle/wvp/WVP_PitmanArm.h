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
// WVP Pitman arm steering model.
//
// =============================================================================

#ifndef WVP_PITMAN_ARM_H
#define WVP_PITMAN_ARM_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArm.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace wvp {

class CH_MODELS_API WVP_PitmanArm : public ChPitmanArm {
  public:
    WVP_PitmanArm(const std::string& name);
    ~WVP_PitmanArm() {}

    virtual double getSteeringLinkMass() const override { return m_steeringLinkMass; }
    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getSteeringLinkRadius() const override { return m_steeringLinkRadius; }
    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector<>& getSteeringLinkInertiaMoments() const override { return m_steeringLinkInertiaMoments; }
    virtual const ChVector<>& getSteeringLinkInertiaProducts() const override { return m_steeringLinkInertiaProducts; }
    virtual const ChVector<>& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector<>& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const ChVector<> getLocation(PointId which) override;
    virtual const ChVector<> getDirection(DirectionId which) override;

  private:
    static const double m_steeringLinkMass;
    static const double m_pitmanArmMass;

    static const double m_steeringLinkRadius;
    static const double m_pitmanArmRadius;

    static const double m_maxAngle;

    static const ChVector<> m_steeringLinkInertiaMoments;
    static const ChVector<> m_steeringLinkInertiaProducts;
    static const ChVector<> m_pitmanArmInertiaMoments;
    static const ChVector<> m_pitmanArmInertiaProducts;
};

}  // end namespace wvp
}  // end namespace vehicle
}  // end namespace chrono

#endif