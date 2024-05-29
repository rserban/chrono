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
// Test program for coloring a mesh visualization asset
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono/assets/ChVisualShapeModelFile.h"

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

    // Visualization
    auto vis_mesh = chrono_types::make_shared<ChVisualShapeModelFile>();
    vis_mesh->SetFilename(GetChronoDataFile("models/tractor_wheel/tractor_wheel.obj"));
    vis_mesh->SetColor(ChColor(0.5f, 0.0f, 0.0f));
    ground->AddVisualShape(vis_mesh);

    // Create the Irrlicht visualization

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Mesh color");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(2, 0, 0));
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
