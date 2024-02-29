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

#include "chrono/geometry/ChLineSegment.h"
#include "chrono/physics/ChLinkPointSpline.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

static const double FD_STEP = 1e-4;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPointSpline)

ChLinkPointSpline::ChLinkPointSpline() : tolerance(1e-6) {
    // default trajectory is a segment
    trajectory_line = chrono_types::make_shared<ChLineSegment>();

    // Mask: initialize our LinkMaskLF (lock formulation mask) to X  only
    mask.SetLockMask(false, true, true, false, false, false, false);

    BuildLink();
}

ChLinkPointSpline::ChLinkPointSpline(const ChLinkPointSpline& other) : ChLinkLockLock(other) {
    trajectory_line = std::shared_ptr<ChLine>((ChLine*)other.trajectory_line->Clone());  // deep copy
    tolerance = other.tolerance;
}

void ChLinkPointSpline::Set_trajectory_line(std::shared_ptr<ChLine> mline) {
    trajectory_line = mline;
}

// UPDATE TIME

void ChLinkPointSpline::UpdateTime(double time) {
    ChTime = time;

    if (trajectory_line) {
        ChVector3d param, vdir, vdir2, vnorm, vrad, vpoint;
        double mu, ds, dh, mrad;

        // find nearest point
        vpoint = marker1->GetAbsCsys().pos;
        vpoint = Body2->TransformPointParentToLocal(vpoint);
        trajectory_line->FindNearestLinePoint(vpoint, mu, 0, tolerance);

        param.y() = 0;
        param.z() = 0;
        param.x() = mu;
        auto ptang = trajectory_line->Evaluate(param.x());

        if (param.x() < 0)
            param.x() = 0;
        vdir = trajectory_line->GetTangent(param.x());

        param.x() = mu + FD_STEP;
        if (param.x() > 1)
            param.x() = 1;
        auto ptang2 = trajectory_line->Evaluate(param.x());

        vdir2 = trajectory_line->GetTangent(param.x());

        ChMatrix33<> ma;

        vdir = Vnorm(vdir);
        vdir2 = Vnorm(vdir2);
        vnorm = Vcross(vdir2, vdir);
        if (vnorm.Length() < 1e-7) {
            // on a straight segment, no curvature, so define normal and radial by these:
            ma.SetFromAxisX(vdir, -VECT_Z);
        } else {
            vnorm.Normalize();
            vrad = Vnorm(Vcross(vdir, vnorm));
            ma.SetFromDirectionAxes(vdir, vnorm, vrad);
        }
        ChQuaterniond qabsdir = ma.GetQuaternion();

        ptang = Body2->TransformPointLocalToParent(ptang);
        qabsdir = Body2->GetRot() * qabsdir;

        marker2->ImposeAbsoluteTransform(ChFrame<>(ptang, qabsdir));  // move "main" marker2 into tangent position
        marker2->SetMotionType(ChMarker::MotionType::EXTERNAL);

        ds = Vlength(Vsub(ptang, ptang2));
        dh = Vdot(Vsub(ptang2, ptang), vrad);
        mrad = ((ds * ds) / (2 * dh));  // radius of curvature on spline

        //ChMatrix33<> mw(marker2->GetAbsCsys().rot);

        deltaC.pos = VNULL;
        deltaC_dt.pos = VNULL;
        deltaC_dtdt.pos.x() = 0;  // csys X axis aligned to vdir: just
        deltaC_dtdt.pos.y() = 0;  // impose centripetal acceleration
        // deltaC_dtdt.pos.z() =   pow(Vdot(this->GetRelM_dt().pos, vdir), 2) / mrad;
        deltaC_dtdt.pos.z() = pow(GetRelM_dt().pos.x(), 2) / mrad;

        deltaC.rot = QUNIT;
        deltaC_dt.rot = QNULL;
        deltaC_dtdt.rot = QNULL;
    }
}

void ChLinkPointSpline::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkPointSpline>();

    // serialize parent class
    ChLinkLockLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(trajectory_line);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkPointSpline::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChLinkPointSpline>();

    // deserialize parent class
    ChLinkLockLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(trajectory_line);
}

}  // end namespace chrono
