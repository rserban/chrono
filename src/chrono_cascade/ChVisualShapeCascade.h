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

#ifndef CH_CASCADE_VISUAL_SHAPE_H
#define CH_CASCADE_VISUAL_SHAPE_H

#include "chrono_cascade/ChApiCASCADE.h"
#include "chrono/assets/ChVisualShape.h"

#include <TopoDS_Shape.hxx>

namespace chrono {
namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Class for an asset that contains an OpenCASCADE shape which can be included in a visual model.
class ChApiCASCADE ChVisualShapeCascade : public ChVisualShape {
  public:
    ChVisualShapeCascade();
    ChVisualShapeCascade(const TopoDS_Shape& ms);
    virtual ~ChVisualShapeCascade();

    /// Access the OpenCASCADE shape.
    TopoDS_Shape& Shape() { return mshape; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  protected:
    TopoDS_Shape mshape;  ///< OpenCASCADE shape
};

/// @} cascade_module

}  // namespace cascade
}  // end namespace chrono

#endif
