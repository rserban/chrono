#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono_opengl/ChVisualSystemOpenGL.h"

using namespace chrono;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    ////auto body0 = chrono_types::make_shared<ChBodyEasyBox>(1.0, 2.0, 3.0, 100);
    ////system.AddBody(body0);

    auto body1 = chrono_types::make_shared< ChBodyEasyMesh>(GetChronoDataFile("models/ob_chess_table.obj"), 1000, true, true);
    body1->SetFrame_REF_to_abs(ChFrame<>(ChVector3d(-3.0, 0.0, 0.0), QUNIT));

    auto body2 = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/pallet.obj"), 1000, true, true);
    body2->SetFrame_REF_to_abs(ChFrame<>(ChVector3d(+3.0, 0.0, 0.0), QUNIT));

    auto body3 = chrono_types::make_shared<ChBody>();
    body3->SetFixed(true);
    std::vector<ChVector3d> points = {ChVector3d(0,0,0), ChVector3d(10, 0, 0), ChVector3d(20, 5, 0), ChVector3d(30, 5, 0)};
    auto path = chrono_types::make_shared<ChBezierCurve>(points);
    auto num_points = static_cast<unsigned int>(path->GetNumPoints());
    auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<ChLineBezier>(path));
    path_asset->SetColor(ChColor(0.8f, 0.8f, 0.0f));
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    body3->AddVisualShape(path_asset);

    system.AddBody(body1);
    system.AddBody(body2);
    system.AddBody(body3);
    ////auto body1 = chrono_types::make_shared<ChBody>();
    ////body1->SetPos(ChVector3d(-3.0, 0.0, 0.0));
    ////system.AddBody(body1);
    ////auto trimesh1 = chrono_types::make_shared< ChTriangleMeshConnected>();
    ////trimesh1->LoadWavefrontMesh(GetChronoDataFile("models/ob_chess_table.obj"), true, false);
    ////auto vshape1 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    ////vshape1->SetMesh(trimesh1);
    ////vshape1->SetStatic(true);
    ////vshape1->SetName("mesh1");
    ////body1->AddAsset(vshape1);

    ////auto body2 = chrono_types::make_shared<ChBody>();
    ////body2->SetPos(ChVector3d(3.0, 0.0, 0.0));
    ////system.AddBody(body2);
    ////auto trimesh2 = chrono_types::make_shared< ChTriangleMeshConnected>();
    ////trimesh2->LoadWavefrontMesh(GetChronoDataFile("models/pallet.obj"), true, false);
    ////auto vshape2 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    ////vshape2->SetMesh(trimesh2);
    ////vshape2->SetStatic(true);
    ////vshape2->SetName("mesh2");
    ////body2->AddAsset(vshape2);

    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&system);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0.0, -10.0, 5.0), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    double step = 0.01;
    while (vis.Run()) {
        system.DoStepDynamics(step);
        vis.Render();
    }
    return 0;
}