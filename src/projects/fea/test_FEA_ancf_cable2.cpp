#include <assert.h>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace chrono;
using namespace chrono::irrlicht;

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    // Create system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(-9.8, 0, 0));

    // FEA mesh
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionCable>();
    section->SetDensity(1);
    section->SetYoungModulus(1e4);
    section->SetArea(1);
    section->SetI(1e-20);

    auto new_node_0 = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzD>(chrono::ChVector3d(0, 0, 0));
    auto new_node_1 = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzD>(chrono::ChVector3d(-1, 0, 0));
    
    auto new_element = chrono_types::make_shared<chrono::fea::ChElementCableANCF>();
    new_element->SetNodes(new_node_0, new_node_1);
    new_element->SetSection(section);

    auto mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    mesh->AddElement(new_element);
    mesh->AddNode(new_node_0);
    mesh->AddNode(new_node_1);

    new_node_0->SetFixed(true);

    sys.Add(mesh);

    // Mesh visualization
    auto vis_cable = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_cable->SetFEMdataType(ChVisualShapeFEA::DataType::ANCF_BEAM_BD);
    vis_cable->SetColorscaleMinMax(-20, 20);
    vis_cable->SetSmoothFaces(true);
    vis_cable->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_cable);

    // ----------------------
    // Run-time visualization
    // ----------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Test ANCF Cables");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0.05, -0.3, 0.05), ChVector3d(0.05, 0.0, 0.0));
    vis->AttachSystem(&sys);

    // Solver settings
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sys.SetSolver(mkl_solver);

    // Integrator settings
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    stepper->SetAlpha(-0.1);
    stepper->SetMaxIters(100);
    stepper->SetAbsTolerances(1e-10, 1e-10);
    stepper->SetStepControl(false);
    stepper->SetModifiedNewton(true);
    stepper->SetVerbose(false);

    // Static analysis

    ChStaticNonLinearAnalysis statics;
    statics.SetMaxIterations(20);
    statics.SetCorrectionTolerance(1e-6, 1e-8);  //// option 1: stopping based on correction WRMS norm
    //statics.SetResidualTolerance(1e-4);        //// option 2: stopping based on residual norm
    statics.SetVerbose(true);
    sys.DoStaticAnalysis(statics);

    std::cout << "mass: " << new_element->GetMass() << std::endl;
    std::cout << "init1: " << new_node_0->GetPos() << std::endl;
    std::cout << "init1: " << new_node_1->GetPos() << std::endl;

    chrono::ChVectorDynamic<> F(12);
    new_element->ComputeInternalForces(F);
    std::cout << "Node1 Force: " << F(0) << ", " << F(1) << ", " << F(2) << std::endl;
    std::cout << "Node2 Force: " << -F(6) << ", " << -F(7) << ", " << -F(8) << std::endl;

    // Simulation loop
    double dt = 0.01;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        irrlicht::tools::drawAllCOGs(vis.get(), 0.05);
        vis->EndScene();

        sys.DoStepDynamics(dt);

        ////new_element->ComputeInternalForces(F);
        ////std::cout << "Node1 Force: " << F(0) << ", " << F(1) << ", " << F(2) << std::endl;
        ////std::cout << "Node2 Force: " << -F(6) << ", " << -F(7) << ", " << -F(8) << std::endl;
    }

    return 0;
}
