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
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#ifndef AV_PATH_H
#define AV_PATH_H

#include <unordered_map>

#include "chrono/core/ChBezierCurve.h"
#include "chrono/assets/ChColor.h"

#include "scene.h"

namespace av {

class Framework;
class Path;

typedef std::unordered_map<unsigned int, std::shared_ptr<Path>> PathList;

class Path {
  public:
    ~Path();

    static std::shared_ptr<Path> Find(unsigned int id);
    static PathList GetList() { return m_paths; }

  private:
    Path(Framework* framework, std::shared_ptr<chrono::ChBezierCurve> curve);
    Path(Framework* framework, const std::vector<GPScoord>& gps_points, double v_offset, bool closed);
    Path(Framework* framework, const std::vector<chrono::ChVector3d>& points, double v_offset, bool closed);

    unsigned int m_id;
    std::shared_ptr<chrono::ChBezierCurve> m_curve;
    chrono::ChColor m_color;

    static PathList m_paths;

    friend class Framework;
};

}  // end namespace av

#endif
