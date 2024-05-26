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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Rigid terrain with a trapezoid shape
//
//                   Z (X = 0 + offset)
//                   ^
//                   |
//                   |        --------------+
//                   |      / |             |  \
//                   |    /   |             |     \
//                   |  /     |             |       \
//                   |/ (Ang) |             |h2       \
//     +-------------+--------|             |           \
//     |             |        |             |       (Ang) \
//  h1 |             |        |             |------------- +-------------+
//     |<----------->|        |<----------->|            h3|<----------->|
//     |    sizeX1   |        |    sizeX2   |              |    sizeX3   |
//  -------------------------------------------------------------------------> X
//
//
// =============================================================================

#ifndef RIGID_TERRAIN_TRAPEZOID_H
#define RIGID_TERRAIN_TRAPEZOID_H

#include <string>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChVector2.h"

#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// Rigid terrain slope model.
class RigidTerrainTrapezoid : public ChTerrain {
  public:
    RigidTerrainTrapezoid(ChSystem* system);
    ~RigidTerrainTrapezoid() {}

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float tex_scale_x = 1,       ///< [in] texture scale in X
                    float tex_scale_y = 1        ///< [in] texture scale in Y
                    );

    /// Return a handle to the ground body.
    std::shared_ptr<ChBody> GetGroundBody() { return m_ground; }

    /// Initialize the terrain system.
    /// This version creates a berm across the entire width (Y direction) and
    /// allows different terrain heights before and after the berm.
    void Initialize(std::shared_ptr<ChContactMaterial> mat,  ///< [in] contact material
                    double height1,                          ///< [in] terrain height (before slope)
                    double height2,                          ///< [in] terrain height (after 1st slope)
                    double height3,                          ///< [in] terrain height (after 2nd slope)
                    double angle1,                           ///< [in] 1st slope angle with respect to ground
                    double angle2,                           ///< [in] 2nd slope angle with respect to ground
                    double sizeX1,      ///< [in] terrain dimension in the X direction (1st flat area)
                    double sizeX2,      ///< [in] terrain dimension in the X direction (2nd flat area)
                    double sizeX3,      ///< [in] terrain dimension in the X direction (3rd flat area)
                    double sizeY,       ///< [in] terrain dimension in the Y direction
                    double offsetX = 0  ///< [in] offset in the +X direction for the start of 1st slope
    );

    /// Initialize the terrain system.
    /// This version creates a berm with a width possibly smaller than that of the
    /// underlying flat terrain (in this case, the terrain height before the berm
    /// is equal to that after the berm). The terrain height must be lower than the
    /// berm height.
    void Initialize(std::shared_ptr<ChContactMaterial> mat,  ///< [in] contact material
                    double heightT,                          ///< [in] terrain height
                    ChVector2<> sizeT,                       ///< [in] total terrain length x width
                    double heightB,                          ///< [in] berm height
                    ChVector2<> sizeB,                       ///< [in] berm length x width
                    double angle,                            ///< [in] slope angle
                    double offsetX = 0  ///< [in] offset in the +X direction for the start of up-slope
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
    double m_height3;
    double m_sizeX2;
    double m_offsetX;
    double m_angle1;
    double m_angle2;
    double m_rise1;
    double m_rise2;
    double m_run1;
    double m_run2;
    double m_sizeY;
    float m_friction;

    bool m_widebase;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
