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
//
// Demosntration of the Chrono::Irrlicht run-time visualization system
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeCone.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono system
    ChSystemNSC sys;

    //
    // EXAMPLE 1:
    //

    // Create a ChBody, and attach assets that define 3D shapes for visualization purposes.
    // Note: these assets are independent from collision shapes!

    // Create a rigid body and add it to the physical system:
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);

    // Contact material
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Define a collision shape
    auto floor_shape = chrono_types::make_shared<collision::ChCollisionShapeBox>(floor_mat, 20, 1, 20);
    floor->GetCollisionModel()->AddShape(floor_shape, ChFrame<>(ChVector<>(0, -1, 0), QUNIT));
    floor->GetCollisionModel()->Build();
    floor->SetCollide(true);

    // Add body to system
    sys.Add(floor);

    // Attach a 'box' shape.
    // Note that assets are managed via shared pointer, so they can also be shared).
    auto boxfloor = chrono_types::make_shared<ChVisualShapeBox>(20, 1, 20);
    boxfloor->SetColor(ChColor(0.2f, 0.3f, 1.0f));
    floor->AddVisualShape(boxfloor, ChFrame<>(ChVector<>(0, -1, 0), QUNIT));

    // Attach a 'path' shape populated with segments and arcs.
    auto pathfloor = chrono_types::make_shared<ChVisualShapePath>();
    ChLineSegment mseg1(ChVector<>(1, 2, 0), ChVector<>(1, 3, 0));
    pathfloor->GetPathGeometry()->AddSubLine(mseg1);
    ChLineSegment mseg2(ChVector<>(1, 3, 0), ChVector<>(2, 3, 0));
    pathfloor->GetPathGeometry()->AddSubLine(mseg2);
    ChLineArc marc1(ChCoordsys<>(ChVector<>(2, 3.5, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2);
    pathfloor->GetPathGeometry()->AddSubLine(marc1);
    pathfloor->SetColor(ChColor(0.0f, 0.5f, 0.8f));
    floor->AddVisualShape(pathfloor);

    // Attach a 'nurbs line' shape:
    // First create the ChLineNurbs geometry, then put it inside a ChVisualShapeLine
    auto nurbs = chrono_types::make_shared<ChLineNurbs>();
    std::vector<ChVector<>> controlpoints = {ChVector<>(1, 2, -1), ChVector<>(1, 3, -1), ChVector<>(1, 3, -2),
                                             ChVector<>(1, 4, -2)};
    nurbs->SetupData(3, controlpoints);

    auto nurbsasset = chrono_types::make_shared<ChVisualShapeLine>();
    nurbsasset->SetLineGeometry(nurbs);
    nurbsasset->SetColor(ChColor(0.0f, 0.3f, 1.0f));
    floor->AddVisualShape(nurbsasset);

    // Attach a 'nurbs surface' shape:
    // First create the ChSurfaceNurbs geometry, then put it inside a ChVisualShapeSurface
    auto surf = chrono_types::make_shared<ChSurfaceNurbs>();
    ChMatrixDynamic<ChVector<>> surfpoints(4, 2);  // u points, v points
    surfpoints(0, 0) = ChVector<>(1, 2, 3);
    surfpoints(1, 0) = ChVector<>(1, 3, 3);
    surfpoints(2, 0) = ChVector<>(2, 3, 3);
    surfpoints(3, 0) = ChVector<>(2, 4, 3);
    surfpoints(0, 1) = ChVector<>(1, 2, 1);
    surfpoints(1, 1) = ChVector<>(1, 3, 1);
    surfpoints(2, 1) = ChVector<>(3, 3, 1);
    surfpoints(3, 1) = ChVector<>(2, 4, 1);
    surf->SetupData(3, 1, surfpoints);

    auto surfasset = chrono_types::make_shared<ChVisualShapeSurface>();
    surfasset->SetSurfaceGeometry(surf);
    surfasset->SetWireframe(true);
    surfasset->SetColor(ChColor(0.2f, 0.8f, 0.3f));
    floor->AddVisualShape(surfasset, ChFrame<>(ChVector<>(3, -1, 3), QUNIT));

    //
    // EXAMPLE 2:
    //

    // Create the rigid body (this won't move, it is only for visualization tests)
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(true);
    sys.Add(body);

    // Create shared visualization materials
    auto orange_mat = chrono_types::make_shared<ChVisualMaterial>();
    orange_mat->SetDiffuseColor(ChColor(0.9f, 0.4f, 0.2f));

    auto pink_mat = chrono_types::make_shared<ChVisualMaterial>();
    pink_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));

    // Attach a sphere shape
    auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(0.5);
    sphere->AddMaterial(orange_mat);
    body->AddVisualShape(sphere, ChFrame<>(ChVector<>(-1, 0, 0), QUNIT));

    // Attach a box shape
    auto box = chrono_types::make_shared<ChVisualShapeBox>(0.6, 1.0, 0.2);
    box->AddMaterial(orange_mat);
    body->AddVisualShape(box, ChFrame<>(ChVector<>(1, 1, 0), QUNIT));

    // Attach a cylinder shape
    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.3, 0.7);
    cyl->AddMaterial(orange_mat);
    body->AddVisualShape(cyl, ChFrame<>(ChVector<>(2, 0.15, 0), Q_from_AngX(CH_C_PI_2)));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector<>(2, -0.2, 0), QUNIT));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector<>(2, +0.5, 0), QUNIT));

    // Attach a capsule shape
    auto capsule = chrono_types::make_shared<ChVisualShapeCapsule>(0.5, 2);
    capsule->AddMaterial(orange_mat);
    body->AddVisualShape(capsule, ChFrame<>(ChVector<>(-3, 1, -1), QUNIT));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector<>(-3, 1, -2.5), QUNIT));
    body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(0.03),
                         ChFrame<>(ChVector<>(-3, 1, +0.5), QUNIT));

    // Attach a cone shape (NOT YET SUPPORTED)
    auto cone = chrono_types::make_shared<ChVisualShapeCone>(0.3, 1.0);
    cone->SetMaterial(0, pink_mat);
    body->AddVisualShape(cone, ChFrame<>(ChVector<>(-4, 1.5, -1), QUNIT));

    // Attach three instances of the same 'triangle mesh' shape
    auto mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh->GetMesh()->getCoordsVertices().push_back(ChVector<>(0, 0, 0));
    mesh->GetMesh()->getCoordsVertices().push_back(ChVector<>(0, 1, 0));
    mesh->GetMesh()->getCoordsVertices().push_back(ChVector<>(1, 0, 0));
    mesh->GetMesh()->getIndicesVertexes().push_back(ChVector<int>(0, 1, 2));
    mesh->AddMaterial(orange_mat);

    body->AddVisualShape(mesh, ChFrame<>(ChVector<>(2, 0, 2), QUNIT));
    body->AddVisualShape(mesh, ChFrame<>(ChVector<>(3, 0, 2), QUNIT));
    body->AddVisualShape(mesh, ChFrame<>(ChVector<>(2, 1, 2), QUNIT));

    // Attach a 'Wavefront mesh' asset, referencing a .obj file and offset it.
    auto objmesh = chrono_types::make_shared<ChVisualShapeModelFile>();
    objmesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));
    body->AddVisualShape(objmesh, ChFrame<>(ChVector<>(0, 0, 2), QUNIT));

    // Attach an array of boxes, each rotated to make a spiral
    for (int j = 0; j < 20; j++) {
        auto smallbox = chrono_types::make_shared<ChVisualShapeBox>(0.2, 0.2, 0.02);
        smallbox->SetColor(ChColor(j * 0.05f, 1 - j * 0.05f, 0.0f));
        ChMatrix33<> rot(Q_from_AngY(j * 21 * CH_C_DEG_TO_RAD));
        ChVector<> pos = rot * ChVector<>(0.4, 0, 0) + ChVector<>(0, j * 0.02, 0);
        body->AddVisualShape(smallbox, ChFrame<>(pos, rot));
    }

    //
    // EXAMPLE 3:
    //

    // Create a ChParticleClones cluster, and attach 'assets' that define a single "sample" 3D shape.
    // This will be shown N times in Irrlicht.

    // Create the ChParticleClones, populate it with some random particles,
    // and add it to physical system:
    auto particles = chrono_types::make_shared<ChParticleCloud>();

    double particle_radius = 0.05;

    // Add visualization (shared by all particles in the cloud)
    auto particle_vis = chrono_types::make_shared<ChVisualShapeSphere>(particle_radius);
    particles->AddVisualShape(particle_vis);

    // Note: the collision shape, if needed, must be specified before creating particles.
    // This will be shared among all particles in the ChParticleCloud.
    auto particle_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto particle_shape = chrono_types::make_shared<collision::ChCollisionShapeSphere>(particle_mat, 0.05);
    particles->GetCollisionModel()->AddShape(particle_shape);
    particles->GetCollisionModel()->Build();
    particles->SetCollide(true);

    // Create the random particles
    for (int np = 0; np < 100; ++np)
        particles->AddParticle(ChCoordsys<>(ChVector<>(ChRandom() - 2, 1.5, ChRandom() + 2)));

    // Mass and inertia properties.
    // This will be shared among all particles in the ChParticleCloud.
    particles->SetMass(0.1);
    particles->SetInertiaXX(ChVector<>(0.001, 0.001, 0.001));

    // Do not forget to add the particle cluster to the system:
    sys.Add(particles);

    //
    // EXAMPLE 4:
    //

    // Create a convex hull shape

    ChVector<> displ(1, 0.0, 0);
    std::vector<ChVector<>> points;
    points.push_back(ChVector<>(0.8, 0.0, 0.0) + displ);
    points.push_back(ChVector<>(0.8, 0.3, 0.0) + displ);
    points.push_back(ChVector<>(0.8, 0.3, 0.3) + displ);
    points.push_back(ChVector<>(0.0, 0.3, 0.3) + displ);
    points.push_back(ChVector<>(0.0, 0.0, 0.3) + displ);
    points.push_back(ChVector<>(0.8, 0.0, 0.3) + displ);
    auto hull = chrono_types::make_shared<ChBodyEasyConvexHullAuxRef>(
        points, 1000, true, true, chrono_types::make_shared<ChMaterialSurfaceNSC>());
    ////hull->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(2,0.3,0)));
    ////hull->SetPos(ChVector<>(2,0.3,0));
    hull->Move(ChVector<>(2, 0.3, 0));

    // Create a visualization material
    auto cadet_blue = chrono_types::make_shared<ChVisualMaterial>();
    cadet_blue->SetDiffuseColor(ChColor(0.37f, 0.62f, 0.62f));
    hull->GetVisualShape(0)->SetMaterial(0, cadet_blue);

    sys.Add(hull);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Chrono::Irrlicht visualization");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(-2, 3, -4));
    vis->AddTypicalLights();
    vis->AddGrid(0.5, 0.5, 12, 12, ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(CH_C_PI_2)),
                 ChColor(0.31f, 0.43f, 0.43f));

    // Rendering loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.01);
    }

    return 0;
}
