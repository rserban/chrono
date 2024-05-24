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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHC_LINEBSPLINE_H
#define CHC_LINEBSPLINE_H

#include <cmath>
#include <vector>

#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object representing a Bspline spline.
class ChApi ChLineBSpline : public ChLine {
  public:
    std::vector<ChVector3d> points;
    ChVectorDynamic<> knots;
    int p;

  public:
    /// Constructor. By default, a segment (order = 1, two points on X axis, at -1, +1)
    ChLineBSpline();

    /// Constructor from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    ChLineBSpline(
        int morder,                              ///< order p: 1= linear, 2=quadratic, etc.
        const std::vector<ChVector3d>& mpoints,  ///< control points, size n. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0  ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    );

    ChLineBSpline(const ChLineBSpline& source);
    ~ChLineBSpline() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineBSpline* Clone() const override { return new ChLineBSpline(*this); }

    virtual int GetComplexity() const override { return (int)points.size(); }

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector3d Evaluate(double U) const override;

    /// Return the tangent unit vector at the parametric coordinate U (in [0,1]).
    virtual ChVector3d GetTangent(double parU) const override;

    // Bspline specific functions

    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert u->U,
    /// where u is in knot range, calling this:
    double ComputeUfromKnotU(double u) const { return (u - knots(p)) / (knots(knots.size() - 1 - p) - knots(p)); }

    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert U->u,
    /// where u is in knot range, calling this:
    double ComputeKnotUfromU(double U) const { return U * (knots(knots.size() - 1 - p) - knots(p)) + knots(p); }

    /// Access the points
    std::vector<ChVector3d>& Points() { return points; }

    /// Access the knots
    ChVectorDynamic<>& Knots() { return knots; }

    /// Get the order of spline
    int GetOrder() { return p; }

    /// Initial easy setup from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    virtual void Setup(
        int morder,                              ///< order p: 1= linear, 2=quadratic, etc.
        const std::vector<ChVector3d>& mpoints,  ///< control points, size n. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0  ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    );

    /// Set as closed spline: start and end will overlap at 0 and 1 abscyssa as p(0)=p(1),
    /// and the Evaluate() and GetTangent() functions will operate in periodic way (abscyssa
    /// greater than 1 or smaller than 0 will wrap to 0..1 range).
    /// The closure will change the knot vector (multiple start end knots will be lost) and
    /// will create auxiliary p control points at the end that will be wrapped to the beginning control point.
    virtual void SetClosed(bool mc) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLineBSpline, 0)

}  // end namespace chrono

#endif
