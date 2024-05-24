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
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of the revolute-spherical composite joint
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(1, -1, 1));

    // Create the ground body
    // ----------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.04, 0.4);
    ground->AddVisualShape(cyl, ChFrame<>(ChVector3d(0, 0, 1), QUNIT));

    // Create a pendulum body
    // ----------------------

    auto pend = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend);
    pend->SetFixed(false);
    pend->EnableCollision(false);
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector3d(0.2, 1, 1));

    // Initial position of the pendulum (horizontal, pointing towards positive X).
    pend->SetPos(ChVector3d(1.5, 0, 1));

    // Attach visualization assets.
    auto cyl_p = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 1.92);
    cyl_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_p, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    auto sph_p = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
    sph_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(sph_p, ChFrame<>(ChVector3d(-1, 0, 0), QUNIT));

    // Create a revolute-spherical joint to connect pendulum to ground.
    auto rev_sph = chrono_types::make_shared<ChLinkRevoluteSpherical>();
    sys.AddLink(rev_sph);

    // Initialize the pendulum specifying a coordinate sys (expressed in the
    // absolute frame) and a distance.
    rev_sph->Initialize(ground, pend, ChCoordsys<>(ChVector3d(0, 0, 1), ChQuaternion<>(1, 0, 0, 0)), 0.5);

    // Alternatively, the joint can be initialized by specifying a point and a
    // direction on the first body and a point on the second body.
    // rev_sph->Initialize(ground, pend, false, ChVector3d(0, 0, 1), ChVector3d(0, 0, 1), ChVector3d(0.5, 0, 1));

    // Create the Irrlicht application
    // -------------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkRevoluteSpherical demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 3, 6));
    vis->AddTypicalLights();

    vis->EnableLinkFrameDrawing(true);

    // Cache for point trajectory plot
    std::vector<ChVector3d> trajectory;

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        // Render the rev-sph massless link.
        tools::drawSegment(vis.get(), rev_sph->GetPoint1Abs(), rev_sph->GetPoint2Abs(), ChColor(0, 0.2f, 0), true);

        // Render the point trajectory
        tools::drawPolyline(vis.get(), trajectory, ChColor(0, 0.6f, 0), false);

        vis->EndScene();

        sys.DoStepDynamics(0.005);

        // Add latest point to trajectory. Only keep a buffer of latest 1500 points.
        trajectory.push_back(rev_sph->GetPoint2Abs());
        if (trajectory.size() > 1500)
            trajectory.erase(trajectory.begin());
    }

    return 0;
}
