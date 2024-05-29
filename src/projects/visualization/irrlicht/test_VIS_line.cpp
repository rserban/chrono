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
// Test program for rendering polygonal line
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono/core/ChBezierCurve.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineSegment.h"

#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePath.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

unsigned int num_line_points = 500;
unsigned int num_render_points = 1000;

int main(int argc, char* argv[]) {
    // Create the system (Y up)
    ChSystemNSC sys;

    // Create a ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sys.AddBody(ground);

    // Create a set of points
    double angle = CH_PI / 12;
    std::vector<ChVector3d> points;
    for (unsigned int i = 0; i < num_line_points; i++) {
        double r = i * 0.02;
        double x = 10 + r * std::cos(i * angle);
        double y = r * std::sin(i * angle);
        points.push_back(ChVector3d(x, y, 0));
    }

    // Create a path asset and add segments to it
    auto path_asset = chrono_types::make_shared<ChVisualShapePath>();
    for (unsigned int i = 1; i < num_line_points; i++) {
        double len = (points[i] - points[i - 1]).Length();
        ChLineSegment segment(points[i - 1], points[i]);
        path_asset->GetPathGeometry()->AddSubLine(segment, len);
    }
    path_asset->SetNumRenderPoints(num_render_points);
    ground->AddVisualShape(path_asset);

    // Shift all points in X direction
    for (auto& p : points)
        p.x() -= 20;

    // Create a Bezier curve asset, reusing the points
    auto bezier_curve = chrono_types::make_shared<ChBezierCurve>(points);
    auto bezier_line = chrono_types::make_shared<ChLineBezier>(bezier_curve);
    auto bezier_asset = chrono_types::make_shared<ChVisualShapeLine>();
    bezier_asset->SetLineGeometry(bezier_line);
    bezier_asset->SetNumRenderPoints(num_render_points);
    ground->AddVisualShape(bezier_asset);

    // Create the Irrlicht visualization
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Line render test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 0, -20));
    vis->AttachSystem(&sys);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.1);
    }

    return 0;
}
