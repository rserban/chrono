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
// Collision test of ANCF shell mesh.
// Test two-sided collision and effect of the swept sphere radius.
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLoadContactSurfaceMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

double sphere_swept_thickness = 0.01;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // max inside penetration (Bullet specific setting)
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);

    // Create contact surface material.
    auto contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    contact_mat->SetYoungModulus(6e4f);
    contact_mat->SetFriction(0.3f);
    contact_mat->SetRestitution(0.5f);
    contact_mat->SetAdhesion(0);

    // Adding fixed bodies and collision shapes
    auto cyl1 =
        chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.1, 0.4, 1000, true, true, contact_mat);
    cyl1->SetFixed(true);
    cyl1->SetPos(ChVector3d(0, 0, -0.25));
    cyl1->SetRot(QuatFromAngleZ(CH_PI_2));
    sys.Add(cyl1);

    auto cyl2 =
        chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.1, 0.4, 1000, true, true, contact_mat);
    cyl2->SetFixed(true);
    cyl2->SetPos(ChVector3d(0, 0, 0.25));
    cyl2->SetRot(QuatFromAngleZ(CH_PI_2));
    sys.Add(cyl2);

    // Create a mesh and import from file
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    auto material = chrono_types::make_shared<ChMaterialShellANCF>(1000, 1e8, 0.3);

    ChVector3d center(0, 0.5, 0);
    ChMatrix33<> rot(-CH_PI_2, ChVector3d(0, 0, 1));

    try {
        std::vector<int> BC_NODES;
        std::vector<double> NODE_AVE_AREA;
        ChMeshFileLoader::ANCFShellFromGMFFile(my_mesh, GetChronoDataFile("fea/Plate.mesh").c_str(), material,
                                               NODE_AVE_AREA, BC_NODES, center, rot, 1, false, false);
    } catch (std::exception myerr) {
        std::cout << myerr.what();
        return 0;
    }

    // Add a single layer of thickness 0.01 and with orientation 0 to each element.
    // Set structural damping.
    for (auto el : my_mesh->GetElements()) {
        auto element = std::dynamic_pointer_cast<ChElementShellANCF_3423>(el);
        element->AddLayer(0.01, 0 * CH_DEG_TO_RAD, material);
        element->SetAlphaDamp(0.08);
    }

    my_mesh->SetAutomaticGravity(true);

    // Mesh contact surface
    auto contact_surface = chrono_types::make_shared<ChContactSurfaceMesh>(contact_mat);
    my_mesh->AddContactSurface(contact_surface);
    contact_surface->AddFacesFromBoundary(sphere_swept_thickness);

    // Mesh visualization
    auto vis_mesh1 = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    vis_mesh1->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh1->SetColorscaleMinMax(0.0, 0.50);
    vis_mesh1->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(vis_mesh1);

    auto vis_mesh2 = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    vis_mesh2->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    vis_mesh2->SetWireframe(true);
    vis_mesh2->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(vis_mesh2);

    // Add the mesh to the system
    sys.Add(my_mesh);

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ANCF Collision Test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(1.0, 0.0, 0.0));
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
    vis->AttachSystem(&sys);

    // ---------------
    // Simulation loop
    // ---------------

    // Setup solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetMaxIterations(400);
    solver->SetTolerance(1e-6);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Setup timestepper
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto stepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    stepper->SetAlpha(-0.2);
    stepper->SetMaxIters(200);
    stepper->SetAbsTolerances(1e-06);
    stepper->SetVerbose(false);

    // Simulate, flipping the direction of gravity every 0.5 seconds
    double time_step = 5e-4;

    int flip_frames = 1000;
    int frame = 0;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        if (frame % flip_frames == 0)
            sys.SetGravitationalAcceleration(-sys.GetGravitationalAcceleration());
        vis->EndScene();
        sys.DoStepDynamics(time_step);
        frame++;
    }

    return 0;
}
