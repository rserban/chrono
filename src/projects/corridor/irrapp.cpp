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

#include "irrapp.h"
#include "framework.h"

using namespace chrono;

namespace av {

// -----------------------------------------------------------------------------

class EventReceiver : public irr::IEventReceiver {
  public:
    EventReceiver(IrrApp* app, int index) : m_app(app), m_index(index) {}
    virtual bool OnEvent(const irr::SEvent& event) override;

  private:
    IrrApp* m_app;
    int m_index;
};

bool EventReceiver::OnEvent(const irr::SEvent& event) {
    // Only interpret keyboard inputs.
    if (event.EventType != irr::EET_KEY_INPUT_EVENT)
        return false;

    if (!event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case irr::KEY_PLUS:
                m_index = (++m_index) % Vehicle::GetList().size();
                m_app->ChangeVehicle(m_index);
                return true;
            case irr::KEY_MINUS:
                m_index = (--m_index) % Vehicle::GetList().size();
                m_app->ChangeVehicle(m_index);
                return true;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------

IrrApp::IrrApp(Framework* framework, unsigned int width, unsigned int height)
    : m_framework(framework) {
    SetWindowTitle("Corridor Demo");
    SetWindowSize(width, height);
    // Find index of current ego vehicle (ugly!)
    auto index = -1;
    for (auto v : Vehicle::GetList()) {
        index++;
        if (v->GetId() == framework->m_ego_vehicle->GetId())
            break;
    }

    // Create the event receiver for changing ego vehicle
    m_evrec = new EventReceiver(this, index);
    AddUserEventReceiver(m_evrec);
}

void IrrApp::ChangeVehicle(int index) {
    // Set new ego vehicle
    m_framework->m_ego_vehicle = Vehicle::GetList()[index];

    // Update the embedded chase cam
    m_camera->SetChassis(m_framework->m_ego_vehicle->GetVehicle().GetChassisBody());
    m_camera->Initialize(
        ChVector3d(0.0, 0.0, .75),                                                        // point on chassis
        m_framework->m_ego_vehicle->GetVehicle().GetChassis()->GetLocalDriverCoordsys(),  // driver position
        6.0,                                                                              // chase distance
        0.5                                                                               // chase height
    );
}

void IrrApp::renderOtherStats(int left, int top) {
    char msg[100];
    
    if (m_framework->m_ego_vehicle->m_SPAT_phase >= 0) {
        sprintf(msg, "MAP id: %d", m_framework->m_ego_vehicle->m_MAP_id);
        renderTextBox(std::string(msg), left, top - 130, 120, 15, irr::video::SColor(255, 255, 0, 0));
        sprintf(msg, "SPaT phase: %d", m_framework->m_ego_vehicle->m_SPAT_phase);
        renderTextBox(std::string(msg), left, top - 110, 120, 15, irr::video::SColor(255, 255, 0, 0));
        sprintf(msg, "SPaT time: %.1f", m_framework->m_ego_vehicle->m_SPAT_time);
        renderTextBox(std::string(msg), left, top - 90, 120, 15, irr::video::SColor(255, 255, 0, 0));
    }
}

}  // end namespace av
