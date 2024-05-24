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
// Base class for a terrain subsystem.
//
// =============================================================================

#ifndef CH_TERRAIN_H
#define CH_TERRAIN_H

#include "chrono/core/ChVector3.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Base class for a terrain system.
class CH_VEHICLE_API ChTerrain {
  public:
    ChTerrain();

    virtual ~ChTerrain() {}

    /// Update the state of the terrain system at the specified time.
    virtual void Synchronize(double time) {}

    /// Advance the state of the terrain system by the specified duration.
    virtual void Advance(double step) {}

    /// Get the terrain height below the specified location.
    virtual double GetHeight(const ChVector3d& loc) const;

    /// Get the terrain normal at the point below the specified location.
    virtual ChVector3d GetNormal(const ChVector3d& loc) const;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    virtual float GetCoefficientFriction(const ChVector3d& loc) const;

    /// Get all terrain characteristics at the point below the specified location.
    virtual void GetProperties(const ChVector3d& loc, double& height, ChVector3d& normal, float& friction) const;

    /// Class to be used as a functor interface for location-dependent terrain height.
    class CH_VEHICLE_API HeightFunctor {
      public:
        virtual ~HeightFunctor() {}

        /// Return the terrain height below the given location.
        virtual double operator()(const ChVector3d& loc) = 0;
    };

    /// Class to be used as a functor interface for location-dependent terrain normal.
    class CH_VEHICLE_API NormalFunctor {
      public:
        virtual ~NormalFunctor() {}

        /// Return the terrain normal below the given location.
        virtual ChVector3d operator()(const ChVector3d& loc) = 0;
    };

    /// Class to be used as a functor interface for location-dependent coefficient of friction.
    class CH_VEHICLE_API FrictionFunctor {
      public:
        virtual ~FrictionFunctor() {}

        /// Return the coefficient of friction below the given location.
        virtual float operator()(const ChVector3d& loc) = 0;
    };

    /// Specify the functor object to provide the terrain height at given locations.
    /// This is provided as a potential optimization mechanism (certain derived classes may choose to ignore it).
    void RegisterHeightFunctor(std::shared_ptr<HeightFunctor> functor) { m_height_fun = functor; }

    /// Specify the functor object to provide the terrain normal at given locations.
    /// This is provided as a potential optimization mechanism (certain derived classes may choose to ignore it).
    void RegisterNormalFunctor(std::shared_ptr<NormalFunctor> functor) { m_normal_fun = functor; }

    /// Specify the functor object to provide the coefficient of friction at given locations.
    void RegisterFrictionFunctor(std::shared_ptr<FrictionFunctor> functor) { m_friction_fun = functor; }

  protected:
    std::shared_ptr<HeightFunctor> m_height_fun;      ///< functor for location-dependent terrain height
    std::shared_ptr<NormalFunctor> m_normal_fun;      ///< functor for location-dependent terrain normal
    std::shared_ptr<FrictionFunctor> m_friction_fun;  ///< functor for location-dependent coefficient of friction
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
