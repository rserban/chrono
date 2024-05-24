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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// ARTcar rigid tire subsystem
//
// =============================================================================

#ifndef ARTCAR_RIGID_TIRE_H
#define ARTCAR_RIGID_TIRE_H

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_models/ChApiModels.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace artcar {

/// @addtogroup vehicle_models_artcar
/// @{

/// Rigid tire model for the ARTcar vehicle.
class CH_MODELS_API ARTcar_RigidTire : public chrono::vehicle::ChRigidTire {
  public:
    ARTcar_RigidTire(const std::string& name, bool use_mesh = false);
    ~ARTcar_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;
    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector3d m_inertia;

    static const std::string m_meshName;
    // static const std::string m_meshFile;
    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_models_artcar

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono

#endif
