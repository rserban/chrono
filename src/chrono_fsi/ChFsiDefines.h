// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#pragma once

namespace chrono {
namespace fsi {

// Output mode
enum class CHFSI_OUTPUT_MODE { CSV, CHPF, NONE };

// Time integration methods
enum class CHFSI_TIME_INTEGRATOR { ExplicitSPH, IISPH, I2SPH };

// Linear solver type
enum class CHFSI_SOLVER_TYPE { JACOBI, BICGSSTAB, GMRES, CR, CG, SAP };

} // namespace fsi
} // namespace chrono