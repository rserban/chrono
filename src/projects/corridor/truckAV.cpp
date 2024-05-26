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

#include "truckAV.h"
#include "framework.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

namespace av {

TruckAV::TruckAV(Framework* framework, unsigned int id, const chrono::ChCoordsys<>& init_pos)
    : Vehicle(framework, id, Vehicle::TRUCK) {
    m_hmmwv = chrono_types::make_shared<HMMWV_Reduced>(framework->m_system);

    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetChassisCollisionType(CollisionType::MESH);
    m_hmmwv->SetEngineType(EngineModelType::SIMPLE_MAP);
    m_hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    m_hmmwv->SetDriveType(DrivelineTypeWV::RWD);
    m_hmmwv->SetTireType(TireModelType::RIGID);
    m_hmmwv->SetTireStepSize(framework->m_step);
    m_hmmwv->SetInitPosition(init_pos);
    m_hmmwv->Initialize();

    m_hmmwv->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    m_hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

TruckAV::~TruckAV() {
    //
}

ChWheeledVehicle& TruckAV::GetVehicle() const {
    return m_hmmwv->GetVehicle();
}

void TruckAV::Synchronize(double time) {
    m_hmmwv->Synchronize(time, m_driver_inputs, *m_framework->m_terrain);
}

void TruckAV::Advance(double step) {
    m_hmmwv->Advance(step);
    AdvanceDriver(step);
}

}  // end namespace av
