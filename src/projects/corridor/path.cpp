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
// =============================================================================

#include "chrono/core/ChVector.h"

#include "framework.h"
#include "path.h"

using namespace chrono;

namespace av {

PathList Path::m_paths;

// -----------------------------------------------------------------------------

Path::Path(Framework* framework, std::shared_ptr<chrono::ChBezierCurve> curve)
    : m_curve(curve), m_color(0.0f, 0.8f, 0.0f) {}

Path::Path(Framework* framework, const std::vector<GPScoord>& gps_points, double v_offset, bool closed)
    : m_color(0.0f, 0.8f, 0.0f) {

    std::vector<ChVector3d> pp;
    for (const auto& p : gps_points) {
        auto point = framework->GetLocation(p);
        point.z() += v_offset;
        pp.push_back(point);
    }
    if (closed && (gps_points.back() - gps_points.front()).Length() > 1e-6) {
        pp.push_back(pp.front());
    }

    m_curve = chrono_types::make_shared<ChBezierCurve>(pp, closed);
}

Path::Path(Framework* framework, const std::vector<ChVector3d>& points, double v_offset, bool closed)
    : m_color(0.0f, 0.8f, 0.0f) {
    std::vector<ChVector3d> pp;
    for (const auto& p : points) {
        pp.push_back(p + ChVector3d(0, 0, v_offset));
    }
    if (closed && (points.back() - points.front()).Length() > 1e-6) {
        pp.push_back(pp.front());
    }
    m_curve = chrono_types::make_shared<ChBezierCurve>(pp, closed);
}

Path::~Path() {}

std::shared_ptr<Path> Path::Find(unsigned int id) {
    auto it = m_paths.find(id);
    if (it != m_paths.end())
        return it->second;
    return nullptr;
}

}  // end namespace av
