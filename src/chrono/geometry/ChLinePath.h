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

#ifndef CHC_LINEPATH_H
#define CHC_LINEPATH_H

#include <cmath>

#include "chrono/geometry/ChLine.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object representing an sequence of other ChLine objects,
/// The ChLine objects are assumed to be properly concatenated and to have C0 continuity.
class ChApi ChLinePath : public ChLine {
  public:
    ChLinePath() {}
    ChLinePath(const ChLinePath& source);
    ~ChLinePath() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinePath* Clone() const override { return new ChLinePath(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::LINE_PATH; }

    virtual int GetComplexity() const override { return 2; }

    /// Return curve length.
    /// Sampling does not matter.
    virtual double Length(int sampling) const override;

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector3d Evaluate(double U) const override;

    /// Return the start point of the line.
    virtual ChVector3d GetEndA() const override { return (lines.front())->GetEndA(); }

    /// Return the end point of the line.
    virtual ChVector3d GetEndB() const override { return (lines.back())->GetEndB(); }

    /// Get count of sub-lines that have been added.
    size_t GetSubLinesCount() const { return lines.size(); }

    /// Access the nth sub-line.
    std::shared_ptr<ChLine> GetSubLineN(size_t n) const { return lines[n]; }

    /// Get the nth sub-line duration.
    double GetSubLineDurationN(size_t n) const { return durations[n]; }

    /// Set the nth sub-line duration
    void SetSubLineDurationN(size_t n, double mduration);

    /// Queue a line (push it back to the array of lines).
    void AddSubLine(std::shared_ptr<ChLine> mline,  ///< line to add
                    double duration = 1             ///< duration of the abscyssa when calling the Evaluate() function
    );

    /// Queue a line (push it back to the array of lines)
    void AddSubLine(ChLine& mline,       ///< line to add
                    double duration = 1  ///< duration of the abscyssa when calling the Evaluate() function
    );

    /// Insert a line at the specified index in line array.
    void InsertSubLine(size_t n,                       ///< index in line array
                       std::shared_ptr<ChLine> mline,  ///< line to add
                       double duration = 1  ///< duration of the abscyssa when calling the Evaluate() function
    );

    /// Insert a line at a specified index  n  in line array.
    /// Note that  n  cannot be higher than GetLineCount().
    void InsertSubLine(size_t n,            ///< index of line, 0 is first, etc.
                       ChLine& mline,       ///< line to add
                       double duration = 1  ///< duration of the abscyssa when calling the Evaluate() function
    );

    /// Erase the line at the specified index n in line array.
    /// Note that  n  cannot be higher than GetLineCount().
    void EraseSubLine(size_t n);

    /// Tells the duration of the path, sum of the durations of all sub-lines.
    /// This is useful because ifyou use the Evaluate() function on the path, the U
    /// parameter should range between 0 and the max duration.
    double GetPathDuration() const;

    /// Shrink or stretch all the durations of the sub-lines so that the
    /// total duration of the path is equal to a specified value.
    /// For example, you can normalize to 1 so you can use Evaluate() with U in
    /// the 0..1 range like with other lines.
    void SetPathDuration(double mUduration);

    /// Check if the path is topologically connected,
    /// i.e. if all the sub lines are queued to have C0 continuity
    double GetContinuityMaxError() const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    std::vector<std::shared_ptr<ChLine> > lines;
    std::vector<double> end_times;
    std::vector<double> durations;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLinePath, 0)

}  // end namespace chrono

#endif