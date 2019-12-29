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
// Authors: Radu Serban
// =============================================================================
//
// MAN 10t simple driveline model.
//
// =============================================================================

#ifndef MAN10T_SIMPLEDRIVELINE_H
#define MAN10T_SIMPLEDRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline8WD.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

/// Simple MAN 10t driveline subsystem (purely kinematic).
class CH_MODELS_API MAN_10t_SimpleDriveline : public ChSimpleDriveline8WD {
  public:
    MAN_10t_SimpleDriveline(const std::string& name);

    ~MAN_10t_SimpleDriveline() {}

    virtual double GetFront1DifferentialMaxBias() const override { return m_front1_diff_bias; }
    virtual double GetFront2DifferentialMaxBias() const override { return m_front2_diff_bias; }
    virtual double GetRear1DifferentialMaxBias() const override { return m_rear1_diff_bias; }
    virtual double GetRear2DifferentialMaxBias() const override { return m_rear2_diff_bias; }

  private:
    static const double m_front1_diff_bias;
    static const double m_front2_diff_bias;
    static const double m_rear1_diff_bias;
    static const double m_rear2_diff_bias;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
