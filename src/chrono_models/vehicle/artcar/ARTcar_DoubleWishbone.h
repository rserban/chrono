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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Jayne Henry
// =============================================================================
//
// Front and Rear ARTcar suspension subsystems (double A-arm)
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef ARTCAR_DOUBLEWISHBONE_H
#define ARTCAR_DOUBLEWISHBONE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

#include "chrono_models/ChApiModels.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace artcar {

/// @addtogroup vehicle_models_artcar
/// @{

/// Full double wishbone front suspension for the ARTcar vehicle.
/// The control arms are modeled using rigid bodies.
class CH_MODELS_API ARTcar_DoubleWishboneFront : public chrono::vehicle::ChDoubleWishbone {
  public:
    ARTcar_DoubleWishboneFront(const std::string& name);
    ~ARTcar_DoubleWishboneFront();

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUCAMass() const override { return m_UCAMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUCARadius() const override { return m_UCARadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getUCAInertiaMoments() const override { return m_UCAInertiaMoments; }
    virtual const ChVector3d& getUCAInertiaProducts() const override { return m_UCAInertiaProducts; }
    virtual const ChVector3d& getLCAInertiaMoments() const override { return m_LCAInertiaMoments; }
    virtual const ChVector3d& getLCAInertiaProducts() const override { return m_LCAInertiaProducts; }
    virtual const ChVector3d& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector3d& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector3d getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_UCAMass;
    static const double m_LCAMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_UCARadius;
    static const double m_LCARadius;
    static const double m_uprightRadius;

    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_UCAInertiaMoments;
    static const ChVector3d m_UCAInertiaProducts;
    static const ChVector3d m_LCAInertiaMoments;
    static const ChVector3d m_LCAInertiaProducts;
    static const ChVector3d m_uprightInertiaMoments;
    static const ChVector3d m_uprightInertiaProducts;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

// -----------------------------------------------------------------------------

/// Full double wishbone rear suspension for the ARTcar vehicle.
/// The control arms are modeled using rigid bodies.
class CH_MODELS_API ARTcar_DoubleWishboneRear : public chrono::vehicle::ChDoubleWishbone {
  public:
    ARTcar_DoubleWishboneRear(const std::string& name);
    ~ARTcar_DoubleWishboneRear();

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUCAMass() const override { return m_UCAMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUCARadius() const override { return m_UCARadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getUCAInertiaMoments() const override { return m_UCAInertiaMoments; }
    virtual const ChVector3d& getUCAInertiaProducts() const override { return m_UCAInertiaProducts; }
    virtual const ChVector3d& getLCAInertiaMoments() const override { return m_LCAInertiaMoments; }
    virtual const ChVector3d& getLCAInertiaProducts() const override { return m_LCAInertiaProducts; }
    virtual const ChVector3d& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector3d& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector3d getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_UCAMass;
    static const double m_LCAMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_UCARadius;
    static const double m_LCARadius;
    static const double m_uprightRadius;

    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_UCAInertiaMoments;
    static const ChVector3d m_UCAInertiaProducts;
    static const ChVector3d m_LCAInertiaMoments;
    static const ChVector3d m_LCAInertiaProducts;
    static const ChVector3d m_uprightInertiaMoments;
    static const ChVector3d m_uprightInertiaProducts;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

/// @} vehicle_models_artcar

}  // namespace artcar
}  // namespace vehicle
}  // namespace chrono

#endif
