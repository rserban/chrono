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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/fea/ChLoadContactSurfaceMesh.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

ChLoadContactSurfaceMesh::ChLoadContactSurfaceMesh(std::shared_ptr<ChContactSurfaceMesh> contact_mesh)
    : m_contact_mesh(contact_mesh) {}

void ChLoadContactSurfaceMesh::OutputSimpleMesh(std::vector<ChVector3d>& vert_pos,
                                                std::vector<ChVector3d>& vert_vel,
                                                std::vector<ChVector3i>& triangles) {
    std::vector<ChVector3b> owns_node;
    std::vector<ChVector3b> owns_edge;
    m_contact_mesh->OutputSimpleMesh(vert_pos, vert_vel, triangles, owns_node, owns_edge);
}

void ChLoadContactSurfaceMesh::InputSimpleForces(const std::vector<ChVector3d>& vert_forces,
                                                 const std::vector<int>& vert_ind) {
    // check the vert_forces and vert_ind arrays must have same size:
    assert(vert_forces.size() == vert_ind.size());

    // reset the previously applied forces if any:
    m_forces.clear();
    m_forces_rot.clear();

    int vertex_index = 0;

    // FEA nodes with position DOFs
    {
        std::map<ChNodeFEAxyz*, int> ptr_ind_map;                  // map from pointer-based mesh to index-based mesh
        std::map<int, std::shared_ptr<ChNodeFEAxyz>> ind_ptr_map;  // map from index-based mesh to pointer-based mesh

        for (const auto& tri : m_contact_mesh->GetTrianglesXYZ()) {
            if (ptr_ind_map.insert({tri->GetNode(0).get(), vertex_index}).second) {
                ind_ptr_map.insert({vertex_index, tri->GetNode(0)});
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(1).get(), vertex_index}).second) {
                ind_ptr_map.insert({vertex_index, tri->GetNode(1)});
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(2).get(), vertex_index}).second) {
                ind_ptr_map.insert({vertex_index, tri->GetNode(2)});
                ++vertex_index;
            }
        }

        for (size_t i = 0; i < vert_forces.size(); ++i) {
            auto node = ind_ptr_map.find(vert_ind[i]);
            if (node != ind_ptr_map.end()) {
                auto frc = chrono_types::make_shared<ChLoadNodeXYZ>(node->second, vert_forces[i]);
                m_forces.push_back(frc);
            }
        }
    }

    // FEA nodes with position and rotation DOFs
    {
        std::map<ChNodeFEAxyzrot*, int> ptr_ind_map;                  // map from pointer-based mesh to index-based mesh
        std::map<int, std::shared_ptr<ChNodeFEAxyzrot>> ind_ptr_map;  // map from index-based mesh to pointer-based mesh

        for (const auto& tri : m_contact_mesh->GetTrianglesXYZRot()) {
            if (ptr_ind_map.insert({tri->GetNode(0).get(), vertex_index}).second) {
                ind_ptr_map.insert({vertex_index, tri->GetNode(0)});
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(1).get(), vertex_index}).second) {
                ind_ptr_map.insert({vertex_index, tri->GetNode(1)});
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(2).get(), vertex_index}).second) {
                ind_ptr_map.insert({vertex_index, tri->GetNode(2)});
                ++vertex_index;
            }
        }

        for (size_t i = 0; i < vert_forces.size(); ++i) {
            auto node = ind_ptr_map.find(vert_ind[i]);
            if (node != ind_ptr_map.end()) {
                auto frc = chrono_types::make_shared<ChLoadNodeXYZRotForceAbs>(node->second, vert_forces[i]);
                m_forces_rot.push_back(frc);
            }
        }
    }

    // Force an update of the system containing the associated mesh
    m_contact_mesh->GetPhysicsItem()->GetSystem()->ForceUpdate();
}

void ChLoadContactSurfaceMesh::SetContactMesh(std::shared_ptr<ChContactSurfaceMesh> contact_mesh) {
    m_contact_mesh = contact_mesh;
    m_forces.clear();
    m_forces_rot.clear();
}

// -----------------------------------------------------------------------------

int ChLoadContactSurfaceMesh::LoadGetNumCoordsPosLevel() {
    int ndoftot = 0;
    for (const auto& f : m_forces)
        ndoftot += f->LoadGetNumCoordsPosLevel();
    for (const auto& f : m_forces_rot)
        ndoftot += f->LoadGetNumCoordsPosLevel();

    return ndoftot;
}

int ChLoadContactSurfaceMesh::LoadGetNumCoordsVelLevel() {
    int ndoftot = 0;
    for (const auto& f : m_forces)
        ndoftot += f->LoadGetNumCoordsVelLevel();
    for (const auto& f : m_forces_rot)
        ndoftot += f->LoadGetNumCoordsVelLevel();

    return ndoftot;
}

void ChLoadContactSurfaceMesh::LoadGetStateBlock_x(ChState& mD) {
    int ndoftot = 0;
    for (const auto& f : m_forces) {
        f->loader->GetLoadable()->LoadableGetStateBlockPosLevel(ndoftot, mD);
        ndoftot += f->loader->GetLoadable()->GetLoadableNumCoordsPosLevel();
    }
    for (const auto& f : m_forces_rot) {
        f->loadable->LoadableGetStateBlockPosLevel(ndoftot, mD);
        ndoftot += f->loadable->GetLoadableNumCoordsPosLevel();
    }
}

void ChLoadContactSurfaceMesh::LoadGetStateBlock_w(ChStateDelta& mD) {
    int ndoftot = 0;
    for (const auto& f : m_forces) {
        f->loader->GetLoadable()->LoadableGetStateBlockVelLevel(ndoftot, mD);
        ndoftot += f->loader->GetLoadable()->GetLoadableNumCoordsVelLevel();
    }
    for (const auto& f : m_forces_rot) {
        f->loadable->LoadableGetStateBlockVelLevel(ndoftot, mD);
        ndoftot += f->loadable->GetLoadableNumCoordsVelLevel();
    }
}

void ChLoadContactSurfaceMesh::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    int ndoftotx = 0;
    int ndoftotw = 0;
    for (const auto& f : m_forces) {
        f->loader->GetLoadable()->LoadableStateIncrement(ndoftotx, x_new, x, ndoftotw, dw);
        ndoftotx += f->loader->GetLoadable()->GetLoadableNumCoordsPosLevel();
        ndoftotw += f->loader->GetLoadable()->GetLoadableNumCoordsVelLevel();
    }
    for (const auto& f : m_forces_rot) {
        f->loadable->LoadableStateIncrement(ndoftotx, x_new, x, ndoftotw, dw);
        ndoftotx += f->loadable->GetLoadableNumCoordsPosLevel();
        ndoftotw += f->loadable->GetLoadableNumCoordsVelLevel();
    }
}

// -----------------------------------------------------------------------------

void ChLoadContactSurfaceMesh::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    for (const auto& f : m_forces)
        f->ComputeQ(state_x, state_w);
    for (const auto& f : m_forces_rot)
        f->ComputeQ(state_x, state_w);
}

void ChLoadContactSurfaceMesh::ComputeJacobian(ChState* state_x, ChStateDelta* state_w) {
    for (const auto& f : m_forces)
        f->ComputeJacobian(state_x, state_w);
    for (const auto& f : m_forces_rot)
        f->ComputeJacobian(state_x, state_w);
}

// -----------------------------------------------------------------------------

void ChLoadContactSurfaceMesh::CreateJacobianMatrices() {
    for (const auto& f : m_forces)
        f->CreateJacobianMatrices();
    for (const auto& f : m_forces_rot)
        f->CreateJacobianMatrices();
}

void ChLoadContactSurfaceMesh::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    for (const auto& f : m_forces)
        f->LoadIntLoadResidual_F(R, c);
    for (const auto& f : m_forces_rot)
        f->LoadIntLoadResidual_F(R, c);
}

void ChLoadContactSurfaceMesh::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    for (const auto& f : m_forces)
        f->InjectKRMMatrices(descriptor);
    for (const auto& f : m_forces_rot)
        f->InjectKRMMatrices(descriptor);
}

void ChLoadContactSurfaceMesh::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    for (const auto& f : m_forces)
        f->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
    for (const auto& f : m_forces_rot)
        f->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
}

}  // end namespace fea
}  // end namespace chrono
