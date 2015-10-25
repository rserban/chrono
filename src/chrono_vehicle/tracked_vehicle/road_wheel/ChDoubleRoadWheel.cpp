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
// Base class for a double road wheel (template definition).
// A double road wheel is of type CENTRAL_PIN.
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleRoadWheel::ChDoubleRoadWheel(const std::string& name) : ChRoadWheel(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleRoadWheel::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                                   ChSharedPtr<ChBody> carrier,
                                   const ChVector<>& location) {
    // Invoke the base class method
    ChRoadWheel::Initialize(chassis, carrier, location);

    // Add contact geometry.
    double radius = GetWheelRadius();
    double width = 0.5 * (GetWheelWidth() - GetWheelGap());
    double offset = 0.25 * (GetWheelWidth() + GetWheelGap());

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();
    m_wheel->GetCollisionModel()->AddCylinder(radius, radius, width / 2, ChVector<>(0, offset, 0));
    m_wheel->GetCollisionModel()->AddCylinder(radius, radius, width / 2, ChVector<>(0, -offset, 0));
    m_wheel->GetCollisionModel()->BuildModel();

    switch (m_wheel->GetContactMethod()) {
        case ChMaterialSurfaceBase::DVI:
            m_wheel->GetMaterialSurface()->SetFriction(m_friction);
            m_wheel->GetMaterialSurface()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurfaceBase::DEM:
            m_wheel->GetMaterialSurfaceDEM()->SetFriction(m_friction);
            m_wheel->GetMaterialSurfaceDEM()->SetRestitution(m_restitution);
            m_wheel->GetMaterialSurfaceDEM()->SetYoungModulus(m_young_modulus);
            m_wheel->GetMaterialSurfaceDEM()->SetPoissonRatio(m_poisson_ratio);
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleRoadWheel::AddWheelVisualization() {
    double radius = GetWheelRadius();
    double width = GetWheelWidth();
    double gap = GetWheelGap();

    ChSharedPtr<ChCylinderShape> cyl_1(new ChCylinderShape);
    cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, gap / 2, 0);
    cyl_1->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl_1);

    ChSharedPtr<ChCylinderShape> cyl_2(new ChCylinderShape);
    cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, -width / 2, 0);
    cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, -gap / 2, 0);
    cyl_2->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl_2);

    chrono::ChSharedPtr<chrono::ChTexture> tex(new chrono::ChTexture);
    tex->SetTextureFilename(chrono::GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(tex);
}

}  // end namespace vehicle
}  // end namespace chrono
