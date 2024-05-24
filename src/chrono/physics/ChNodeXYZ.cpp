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

#include "chrono/physics/ChNodeXYZ.h"

namespace chrono {

ChNodeXYZ::ChNodeXYZ() : pos(VNULL), pos_dt(VNULL), pos_dtdt(VNULL) {}

ChNodeXYZ::ChNodeXYZ(const ChVector3d& initial_pos) : pos(initial_pos), pos_dt(VNULL), pos_dtdt(VNULL) {}

ChNodeXYZ::ChNodeXYZ(const ChNodeXYZ& other) {
    offset_x = other.offset_x;
    offset_w = other.offset_w;

    pos = other.pos;
    pos_dt = other.pos_dt;
    pos_dtdt = other.pos_dtdt;
}

ChNodeXYZ& ChNodeXYZ::operator=(const ChNodeXYZ& other) {
    if (&other == this)
        return *this;

    ChNodeBase::operator=(other);

    pos = other.pos;
    pos_dt = other.pos_dt;
    pos_dtdt = other.pos_dtdt;

    return *this;
}

void ChNodeXYZ::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset, 3) = pos.eigen();
}

void ChNodeXYZ::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset, 3) = pos_dt.eigen();
}

void ChNodeXYZ::LoadableStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) {
    NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
}

void ChNodeXYZ::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&Variables());
};

void ChNodeXYZ::ComputeNF(
    const double U,              // x coordinate of application point in absolute space
    const double V,              // y coordinate of application point in absolute space
    const double W,              // z coordinate of application point in absolute space
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is 3, it is Force x,y,z in absolute coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    Qi.segment(0, 3) = F.segment(0, 3);
    detJ = 1;  // not needed because not used in quadrature.
}

void ChNodeXYZ::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChNodeXYZ>();

    // serialize parent class
    ChNodeBase::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(pos);
    archive_out << CHNVP(pos_dt);
    archive_out << CHNVP(pos_dtdt);
}

/// Method to allow de serialization of transient data from archives.
void ChNodeXYZ::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChNodeXYZ>();

    // deserialize parent class:
    ChNodeBase::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(pos);
    archive_in >> CHNVP(pos_dt);
    archive_in >> CHNVP(pos_dtdt);
}

}  // end namespace chrono
