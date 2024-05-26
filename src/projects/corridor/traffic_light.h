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

#ifndef AV_TRAFFIC_LIGHT_H
#define AV_TRAFFIC_LIGHT_H

#include "agent.h"
#include "fsm/fsm.hpp"
#include "message.h"
#include "scene.h"

namespace av {

class TrafficLight;

typedef std::vector<std::shared_ptr<TrafficLight>> TrafficLightList;

class TrafficLight : public Agent {
  public:
    ~TrafficLight();

    chrono::ChVector3d GetCenter() const { return m_center; }
    double GetRadius() const { return m_radius; }

    virtual chrono::ChCoordsys<> GetPosition() const override { return m_pos; }

    static TrafficLightList GetList() { return m_traffic_lights; }

    virtual void Broadcast(double time) override;
    virtual void Unicast(double time) override;
    virtual void ProcessMessages() override;

    virtual void Synchronize(double time) override;
    virtual void Advance(double step) override;

  private:
    TrafficLight(Framework* framework,
                 unsigned int id,
                 const chrono::ChVector3d& center,
                 double radius,
                 const chrono::ChCoordsys<>& pos);

    void ProcessMessageVEH(std::shared_ptr<MessageVEH> msg);

    chrono::ChVector3d m_center;
    double m_radius;

    chrono::ChCoordsys<> m_pos;
    std::shared_ptr<chrono::ChBody> m_body;

    fsm::stack m_fsm;

    std::shared_ptr<MessageMAP> m_map_msg;
    std::shared_ptr<MessageSPAT> m_spat_msg;

    static TrafficLightList m_traffic_lights;

    friend class Framework;
};

}  // end namespace av

#endif
