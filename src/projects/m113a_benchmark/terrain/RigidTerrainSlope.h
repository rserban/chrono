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
// Rigid terrain with a slope
//
//                       Z
//                       ^
//                       |
//                       |        ---------- -----+
//                       |      / |               |
//                       |    /   |               |
//                       |  /     |               |
//                       |/  (%)  |               |
//       +---------------+--------|               |
//       |               |        |               | h2
//    h1 |               |        |               |
//       |<------------->|        |<------------->|
//       |     sizeX     |        |     sizeX     |
//  ---------------------------------------------------------------> X
//
//
// =============================================================================

#ifndef RIGID_TERRAIN_SLOPE_H
#define RIGID_TERRAIN_SLOPE_H

#include <string>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// Rigid terrain slope model.
class RigidTerrainSlope : public ChTerrain {
  public:
    RigidTerrainSlope(ChSystem* system);
    ~RigidTerrainSlope() {}

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float tex_scale_x = 1,       ///< [in] texture scale in X
                    float tex_scale_y = 1        ///< [in] texture scale in Y
                    );

    /// Return a handle to the ground body.
    std::shared_ptr<ChBody> GetGroundBody() { return m_ground; }

    /// Initialize the terrain system (flat).
    /// This version uses a rigid box of specified dimensions and with specified
    /// material properties. If tiled = true, multiple side-by-side boxes are used.
    void Initialize(std::shared_ptr<ChContactMaterial> mat,  ///< [in] contact material
                    double height1,                          ///< [in] terrain
                    double height2,                          ///< [in] terrain height (after slope)
                    double grade,                            ///< [in] slope grade (percentage)
                    double sizeX,                            ///< [in] terrain dimension in the X direction (flat areas)
                    double sizeY                             ///< [in] terrain dimension in the Y direction

    );

    /// Get the terrain height at the specified (x,y) location.
    virtual double GetHeight(const ChVector3d& loc) const override;

    /// Get the terrain normal at the specified (x,y) location.
    virtual chrono::ChVector3d GetNormal(const ChVector3d& loc) const override;

    /// Return the coefficient of friction at the specified (x,y) location.
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return m_friction; }

  private:
    std::shared_ptr<ChBody> m_ground;
    double m_height1;
    double m_height2;
    double m_grade;
    double m_rise;
    double m_run;
    double m_angle;
    float m_friction;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
