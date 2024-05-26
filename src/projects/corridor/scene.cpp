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

#include "scene.h"

using namespace chrono;

namespace av {

const double Scene::m_radius = 6371e3;

Scene::Scene(const GPScoord& origin, const std::string& vis_model_file, const std::string& coll_model_file)
    : m_origin(origin), m_vis_file(vis_model_file), m_coll_file(coll_model_file) {
    m_lat0 = origin.x() * CH_DEG_TO_RAD;
    m_long0 = origin.y() * CH_DEG_TO_RAD;
    m_cos0 = std::cos(m_lat0);
}

ChVector2<> Scene::FromGPS(const GPScoord& gps) const {
    auto latitude = gps.x() * CH_DEG_TO_RAD;
    auto longitude = gps.y() * CH_DEG_TO_RAD;

    auto x = m_radius * (latitude - m_lat0);
    auto y = m_radius * (longitude - m_long0) * m_cos0;

    return ChVector2<>(x, -y);
}

////Area::Area(const ChFrame<>& frame, const ChVector3d& dims) : m_frame(frame), m_dims(dims) {}

}  // end namespace av
