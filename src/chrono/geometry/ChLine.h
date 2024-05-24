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

#ifndef CHC_LINE_H
#define CHC_LINE_H

#include <cmath>

#include "chrono/geometry/ChGeometry.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Base class for all geometric objects representing lines in 3D space.
/// This is the base for all U-parametric object, implementing Evaluate()
/// that returns a point as a function of the U parameter.
class ChApi ChLine : public ChGeometry {
  public:
    ChLine() : closed(false), complexityU(2) {}
    ChLine(const ChLine& source);
    virtual ~ChLine() {}

    /// "Virtual" copy constructor (covariant return type).
    // virtual ChLine* Clone() const override { };

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::LINE; }

    /// Return a point on the line, given parametric coordinate U.
    /// Parameter U always work in 0..1 range.
    /// The default implementation always returns the origin of the surface frame.
    virtual ChVector3d Evaluate(double U) const = 0;

    /// Return the tangent unit vector at the parametric coordinate U (in [0,1]).
    /// This default implementation uses finite differences.
    virtual ChVector3d GetTangent(double parU) const;

    /// Tell if the curve is closed
    virtual bool IsClosed() const { return closed; }
    virtual void SetClosed(bool mc) { closed = mc; }

    /// Tell the complexity
    virtual int GetComplexity() const { return complexityU; }
    virtual void SetComplexity(int mc) { complexityU = mc; }

    /// This is a line
    virtual int GetManifoldDimension() const override { return 1; }

    /// Find the parameter resU for the nearest point on curve to "point".
    bool FindNearestLinePoint(ChVector3d& point, double& resU, double approxU, double tol) const;

    /// Returns curve length. Typical sampling 1..5 (1 already gives correct result with degree1 curves)
    virtual double Length(int sampling) const;

    /// Return the start point of the line.
    /// By default, evaluates line at U=0.
    virtual ChVector3d GetEndA() const { return Evaluate(0); }

    /// Return the end point of the line.
    /// By default, evaluates line at U=1.
    virtual ChVector3d GetEndB() const { return Evaluate(1); }

    /// Returns adimensional information on "how much" this curve is similar to another
    /// in its overall shape (does not matter parametrization or start point). Try with 20 samples.
    /// The return value is somewhat the "average distance between the two curves".
    /// Note that the result is affected by "weight" of curves. If it chnges from default 1.0, the
    /// distance estimation is higher/lower (ex: if a curve defines low 'weight' in its central segment,
    /// its CurveCurveDistance from another segment is not much affected by errors near the central segment).
    double CurveCurveDist(ChLine* compline, int samples) const;

    /// Same as before, but returns "how near" is \a complinesegm to
    /// whatever segment of this line (does not matter the percentual of line).
    /// Again, this is affected by "weight" of curves. If weight changes along curves ->'weighted' distance
    double CurveSegmentDist(ChLine* complinesegm, int samples) const;

    /// Same as above, but instead of making average of the distances,
    /// these functions return the maximum of the distances...
    double CurveCurveDistMax(ChLine* compline, int samples) const;
    double CurveSegmentDistMax(ChLine* complinesegm, int samples) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    bool closed;
    int complexityU;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLine, 0)

}  // end namespace chrono

#endif
