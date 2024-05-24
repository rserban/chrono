// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Rainer Gericke
// =============================================================================

#include "chrono_vsg/shapes/GetDieShapeData.h"

namespace chrono {
namespace vsg3d {

void GetDieShapeData(vsg::ref_ptr<vsg::vec3Array>& vertices,
                     vsg::ref_ptr<vsg::vec3Array>& normals,
                     vsg::ref_ptr<vsg::vec2Array>& texcoords,
                     vsg::ref_ptr<vsg::ushortArray>& indices) {
    const float a = 1.0;
    vertices = vsg::vec3Array::create({{-a, -a, -a}, {a, -a, -a}, {a, -a, a},  {-a, -a, a},  {a, a, -a},  {-a, a, -a},
                                       {-a, a, a},   {a, a, a},   {-a, a, -a}, {-a, -a, -a}, {-a, -a, a}, {-a, a, a},
                                       {a, -a, -a},  {a, a, -a},  {a, a, a},   {a, -a, a},   {a, -a, -a}, {-a, -a, -a},
                                       {-a, a, -a},  {a, a, -a},  {-a, -a, a}, {a, -a, a},   {a, a, a},   {-a, a, a}});

    normals = vsg::vec3Array::create({{0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, 1, 0},  {0, 1, 0},
                                      {0, 1, 0},  {0, 1, 0},  {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0},
                                      {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {0, 0, -1}, {0, 0, -1},
                                      {0, 0, -1}, {0, 0, -1}, {0, 0, 1},  {0, 0, 1},  {0, 0, 1},  {0, 0, 1}});

    texcoords = vsg::vec2Array::create({{0.25, 0},      {0.5, 0},       {0.5, 0.3333}, {0.25, 0.3333}, {0.25, 0.6666},
                                        {0.5, 0.6666},  {0.5, 1.0},     {0.25, 1.0},   {0.0, 0.3333},  {0.25, 0.3333},
                                        {0.25, 0.6666}, {0.0, 0.6666},  {0.5, 0.3333}, {0.75, 0.3333}, {0.75, 0.6666},
                                        {0.5, 0.6666},  {0.25, 0.3333}, {0.5, 0.3333}, {0.5, 0.6666},  {0.25, 0.6666},
                                        {0.75, 0.3333}, {1.0, 0.3333},  {1.0, 0.6666}, {0.75, 0.6666}});

    indices = vsg::ushortArray::create({0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
                                        12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23});
}

}  // namespace vsg3d
}  // namespace chrono
