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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_LINEARC_H
#define CHC_LINEARC_H

#include <cmath>

#include "chrono/geometry/ChLine.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object representing an arc or a circle in 3D space.
/// By default it is evaluated clockwise from angle1 to angle2.
class ChApi ChLineArc : public ChLine {
  public:
    ChCoordsys<> origin;    ///< center position and plane of the arc: xy used for plane, z for axis.
    double radius;          ///< arc radius
    double angle1;          ///< start angle in radians
    double angle2;          ///< end angle in radians
    bool counterclockwise;  ///< flag indicating arc direction

  public:
    ChLineArc(const ChCoordsys<>& morigin = CSYSNULL,
              double mradius = 1,
              double mangle1 = CH_2PI,
              double mangle2 = 0,
              bool mcounterclockwise = false);
    ChLineArc(const ChLineArc& source);
    ~ChLineArc() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineArc* Clone() const override { return new ChLineArc(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::LINE_ARC; }

    virtual int GetComplexity() const override { return 2; }

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector3d Evaluate(double U) const override;

    /// Returns curve length. sampling does not matter
    double Length(int sampling) const override { return fabs(radius * (angle1 - angle2)); }

    // Shortcut for setting evaluation direction
    void SetCounterclockwise(bool mcc) { counterclockwise = mcc; }

    // Shortcut for setting angle1 in degrees instead than radians
    void SetAngle1deg(double a1) { angle1 = a1 * CH_DEG_TO_RAD; }

    // shortcut for setting angle2 in degrees instead than radians
    void SetAngle2deg(double a2) { angle2 = a2 * CH_DEG_TO_RAD; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLineArc, 0)

}  // end namespace chrono

#endif
