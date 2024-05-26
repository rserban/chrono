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

#include "traffic_light.h"
#include "framework.h"

using namespace chrono;

namespace av {

TrafficLightList TrafficLight::m_traffic_lights;

// -----------------------------------------------------------------------------

TrafficLight::TrafficLight(Framework* framework,
                           unsigned int id,
                           const chrono::ChVector3d& center,
                           double radius,
                           const chrono::ChCoordsys<>& pos)
    : Agent(framework, id), m_center(center), m_radius(radius), m_pos(pos) {
    m_body = chrono_types::make_shared<ChBody>();
    m_body->SetPos(pos.pos);
    m_body->SetRot(pos.rot);
    m_body->SetFixed(true);
    m_body->EnableCollision(false);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 3);
    m_body->AddVisualShape(cyl, ChFrame<>(ChVector3d(0,0,1.5)));

    auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(0.4);
    sphere->SetColor(ChColor(0.6f, 0, 0));
    m_body->AddVisualShape(sphere, ChFrame<>(ChVector3d(0, 0, 3)));

    framework->m_system->AddBody(m_body);

    // Prepare messages sent by this agent
    m_map_msg = chrono_types::make_shared<MessageMAP>();
    m_map_msg->type = Message::MAP;
    m_map_msg->senderID = id;
    m_map_msg->time = 0;
    m_map_msg->intersectionID = 9999;
    m_map_msg->num_lanes = 3;

    m_spat_msg = chrono_types::make_shared<MessageSPAT>();
    m_spat_msg->type = Message::SPAT;
    m_spat_msg->senderID = id;
    m_spat_msg->time = 0;
    m_spat_msg->phase = 1;
    m_spat_msg->time_phase = 0;
}

TrafficLight::~TrafficLight() {}

void TrafficLight::Broadcast(double time) {
    m_map_msg->time = static_cast<float>(time);

    m_spat_msg->time = static_cast<float>(time);

    for (auto v : Vehicle::GetList()) {
        if ((GetPosition().pos - v->GetPosition().pos).Length() <= m_bcast_radius) {
            Send(v, m_map_msg);
            Send(v, m_spat_msg);
        }
    }
}

void TrafficLight::Unicast(double time) {
    //// TODO
}

void TrafficLight::ProcessMessages() {
    while (!m_messages.empty()) {
        auto msg = m_messages.front();
        if (m_framework->Verbose()) {
            std::cout << "Traffic Light " << m_id << " received message from " << msg->senderID
                      << " msg type: " << msg->type << " time stamp: " << msg->time << std::endl;
        }
        switch (msg->type) {
            case Message::VEH:
                ProcessMessageVEH(std::static_pointer_cast<MessageVEH>(msg));
                break;
            default:
                break;
        }
        m_messages.pop();
    }
}

void TrafficLight::ProcessMessageVEH(std::shared_ptr<MessageVEH> msg) {
    //// TODO
    if (m_framework->Verbose()) {
        auto& loc = msg->location;
        std::cout << "      Vehicle location: " << loc.x() << " " << loc.y() << " " << loc.z() << std::endl;
    }
}

void TrafficLight::Synchronize(double time) {
    //// TODO:
    //// - implement FSM for this agent
    //// - update SPAT message for next broadcast
    m_spat_msg->time_phase = static_cast<float>(10 - std::fmod(time, 10));
}

void TrafficLight::Advance(double step) {
    //// TODO
}

}  // end namespace av
