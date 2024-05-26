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
    // Settings
    bool use_MKL = true;
    bool use_HHT = true;

    // -------------
    // Create system
    // -------------
    ChSystemSMC mphysicalSystem;
    mphysicalSystem.SetGravitationalAcceleration(ChVector3d(0.01, 0.0, 0.0));

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetTag(-1);
    ground->SetFixed(true);
    mphysicalSystem.Add(ground);

    // FEA mesh
    auto msection_cable = chrono_types::make_shared<fea::ChBeamSectionCable>();
    msection_cable->SetDiameter(0.002);
    msection_cable->SetYoungModulus(5e3);
    msection_cable->SetDensity(2000);
    msection_cable->SetBeamRaleyghDamping(0.2);

    int N_x = 2;
    int N_y = 2;
    int num_elements = 4;
    double delta = 0.1;

    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    fea::ChBuilderCableANCF builder;
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> base_nodes;
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> tip_nodes;

    for (int j = 0; j < N_y; j++) {
        for (int i = 0; i < N_x; i++) {
            double loc_x = i * delta;
            double loc_y = j * delta;
            builder.BuildBeam(
                my_mesh,                        // the mesh where to put the created nodes and elements
                msection_cable,                 // the ChBeamSectionCable to use for the ChElementBeamANCF elements
                num_elements,                   // the number of ChElementBeamANCF to create
                ChVector3d(loc_x, loc_y, 0.0),  // the 'A' point in space (beginning of beam)
                ChVector3d(loc_x, loc_y, 0.1)   // the 'B' point in space (end of beam) _1D_elementsNodes_mesh,
            );

            base_nodes.push_back(builder.GetLastBeamNodes().front());
            tip_nodes.push_back(builder.GetLastBeamNodes().back());
        }
    }

    for (auto node : base_nodes) {
        node->SetFixed(true);

        ////auto pos_const = chrono_types::make_shared<fea::ChLinkPointFrame>();
        ////pos_const->Initialize(node, ground);
        ////mphysicalSystem.Add(pos_const);

        ////auto dir_const = chrono_types::make_shared<fea::ChLinkDirFrame>();
        ////dir_const->Initialize(node, ground);
        ////dir_const->SetDirectionInAbsoluteCoords(node->D);
        ////mphysicalSystem.Add(dir_const);
    }

    mphysicalSystem.Add(my_mesh);

    // Mesh visualization
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ANCF_BEAM_BD);
    mvisualizebeamA->SetColorscaleMinMax(-20, 20);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

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
    vis->AttachSystem(&mphysicalSystem);

    // ---------------
    // Solver settings
    // ---------------
    if (use_MKL) {
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        mphysicalSystem.SetSolver(mkl_solver);
    } else {
        auto minres_solver = chrono_types::make_shared<ChSolverMINRES>();
        minres_solver->SetMaxIterations(400);
        minres_solver->SetTolerance(1e-12);
        minres_solver->EnableWarmStart(true);
        mphysicalSystem.SetSolver(minres_solver);
    }

    // -------------------
    // Integrator settings
    // -------------------
    if (use_HHT) {
        mphysicalSystem.SetTimestepperType(ChTimestepper::Type::HHT);
        auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(mphysicalSystem.GetTimestepper());
        mystepper->SetAlpha(-0.1);
        mystepper->SetMaxIters(100);
        mystepper->SetAbsTolerances(1e-10, 1e-10);
        mystepper->SetVerbose(false);
    } else {
        mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        ////mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }

    // ------------
    // Setup output
    // ------------
    std::string out_dir = "../TEST_ancf_cable";
    if (filesystem::path(out_dir).exists()) {
        cout << "Output directory already exists" << endl;
    } else if (filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Create directory = " << filesystem::path(out_dir).make_absolute() << endl;
    } else {
        cout << "Error creating output directory" << endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------
    utils::ChWriterCSV csv(" ");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(18);

    int step_number = 0;
    double dt = 0.01;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        irrlicht::tools::drawAllCOGs(vis.get(), 0.05);
        vis->EndScene();

        mphysicalSystem.DoStepDynamics(dt);
        step_number++;

        if (step_number == 1) {
            for (auto node : my_mesh->GetNodes()) {
                cout << node->GetIndex() << " " << node->NodeGetOffsetX() << " " << node->NodeGetOffsetW() << endl;
            }
        }

        csv << mphysicalSystem.GetChTime();
        for (auto node : tip_nodes) {
            csv << node->GetPos();
        }
        csv << endl;
        csv.WriteToFile(out_dir + "/results.dat");
    }

    return 0;
}
