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

#ifndef GONOGO_DRIVER_MODEL_H
#define GONOGO_DRIVER_MODEL_H

#include <string>

#include "chrono/core/ChMathematics.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

class GONOGO_Driver : public chrono::vehicle::ChDriver {
  public:
    GONOGO_Driver(chrono::vehicle::ChVehicle& vehicle,          // associated vehicle
                  std::shared_ptr<chrono::ChBezierCurve> path,  // target path
                  double time_start,                            // time throttle start
                  double time_max                               // time throttle max
                  );

    void SetGains(double Kp, double Ki, double Kd) { m_steeringPID.SetGains(Kp, Ki, Kd); }
    void SetLookAheadDistance(double dist) { m_steeringPID.SetLookAheadDistance(dist); }

    void Reset() { m_steeringPID.Reset(m_vehicle.GetRefFrame()); }

    virtual void Synchronize(double time) override;

    virtual void Advance(double step) override;

    void ExportPathPovray(const std::string& out_dir);

  private:
    chrono::vehicle::ChPathSteeringController m_steeringPID;
    double m_start;
    double m_end;
};

inline GONOGO_Driver::GONOGO_Driver(chrono::vehicle::ChVehicle& vehicle,
                                    std::shared_ptr<chrono::ChBezierCurve> path,
                                    double time_start,
                                    double time_max)
    : chrono::vehicle::ChDriver(vehicle), m_steeringPID(path), m_start(time_start), m_end(time_max) {
    m_steeringPID.Reset(m_vehicle.GetRefFrame());

    auto road = chrono_types::make_shared<ChBody>();
    road->SetFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto path_asset = chrono_types::make_shared<chrono::ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<chrono::ChLineBezier>(m_steeringPID.GetPath()));
    path_asset->SetColor(chrono::ChColor(0.0f, 0.8f, 0.0f));
    path_asset->SetName("straight_path");
    road->AddVisualShape(path_asset);
}

inline void GONOGO_Driver::Synchronize(double time) {
    m_braking = 0;
    if (time < m_start) {
        m_throttle = 0;
    } else if (time < m_end) {
        m_throttle = (time - m_start) / (m_end - m_start);
    } else {
        m_throttle = 1;
    }
}

inline void GONOGO_Driver::Advance(double step) {
    double out_steering = m_steeringPID.Advance(m_vehicle.GetRefFrame(), m_vehicle.GetChTime(), step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

inline void GONOGO_Driver::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steeringPID.GetPath(), "straight_path", out_dir, 0.04,
                            chrono::ChColor(0.8f, 0.5f, 0.0f));
}

#endif
