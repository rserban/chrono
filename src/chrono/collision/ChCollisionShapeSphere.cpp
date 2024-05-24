// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionShapeSphere.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeSphere)
CH_UPCASTING(ChCollisionShapeSphere, ChCollisionShape)

ChCollisionShapeSphere::ChCollisionShapeSphere() : ChCollisionShape(Type::SPHERE) {}

ChCollisionShapeSphere::ChCollisionShapeSphere(std::shared_ptr<ChContactMaterial> material, double radius)
    : ChCollisionShape(Type::SPHERE, material) {
    gsphere.rad = radius;
}

ChCollisionShapeSphere::ChCollisionShapeSphere(std::shared_ptr<ChContactMaterial> material, const ChSphere& sphere)
    : ChCollisionShape(Type::SPHERE, material), gsphere(sphere) {}

void ChCollisionShapeSphere::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeSphere>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gsphere);
}

void ChCollisionShapeSphere::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeSphere>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gsphere);
}

}  // end namespace chrono
