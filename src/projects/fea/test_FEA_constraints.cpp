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
// Test for disabling mesh constraints
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

using namespace chrono;
using namespace chrono::irrlicht;

// Define FIX_NODES to fix mesh nodes.
// Otherwise, use constraints.
////#define FIX_NODES

int main(int argc, char* argv[]) {
    // Settings
    bool use_MKL = false;
    bool use_HHT = false;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(-1.0, 0.0, 0.0));
    ////sys.SetGravitationalAcceleration(ChVector3d(0.0, -1.0, 0.0));
    ////sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -1.0));
    ////sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetTag(-1);
    ground->SetFixed(true);
    sys.Add(ground);

    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    auto section = chrono_types::make_shared<fea::ChBeamSectionCable>();
    section->SetDiameter(0.002);
    section->SetYoungModulus(1e6);
    section->SetDensity(2000);
    section->SetRayleighDamping(0.8);

    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> base_nodes;
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> tip_nodes;

    for (int j = 0; j < 1; ++j) {
        fea::ChBuilderCableANCF builder;
        builder.BuildBeam(mesh,                            //
                          section,                         //
                          4,                               //
                          ChVector3d(0.0, 0.0, -0.1 * j),  // base location
                          ChVector3d(0.0, 0.1, -0.1 * j)   // tip location
        );

        ////builder.GetLastBeamNodes().back()->SetForce(ChVector3d(-0.1, 0, 0));

        base_nodes.push_back(builder.GetLastBeamNodes().front());
        tip_nodes.push_back(builder.GetLastBeamNodes().back());
    }

#ifdef FIX_NODES

    for (auto node : base_nodes)
        node->SetFixed(true);

    for (auto node : tip_nodes)
        node->SetFixed(true);

#else

    std::vector<std::shared_ptr<ChLinkBase>> base_cnstr;
    std::vector<std::shared_ptr<ChLinkBase>> tip_cnstr;

    for (auto node : base_nodes) {
        auto pos_cnstr = chrono_types::make_shared<fea::ChLinkNodeFrame>();
        pos_cnstr->Initialize(node, ground);
        sys.Add(pos_cnstr);

        auto dir_cnstr = chrono_types::make_shared<fea::ChLinkNodeSlopeFrame>();
        dir_cnstr->Initialize(node, ground);
        dir_cnstr->SetDirectionInAbsoluteCoords(node->GetSlope1());
        sys.Add(dir_cnstr);

        base_cnstr.push_back(pos_cnstr);
        base_cnstr.push_back(dir_cnstr);
    }

    for (auto node : tip_nodes) {
        auto pos_cnstr = chrono_types::make_shared<fea::ChLinkNodeFrame>();
        pos_cnstr->Initialize(node, ground);
        sys.Add(pos_cnstr);

        auto dir_cnstr = chrono_types::make_shared<fea::ChLinkNodeSlopeFrame>();
        dir_cnstr->Initialize(node, ground);
        dir_cnstr->SetDirectionInAbsoluteCoords(node->GetSlope1());
        sys.Add(dir_cnstr);

        tip_cnstr.push_back(pos_cnstr);
        tip_cnstr.push_back(dir_cnstr);
    }

#endif

    sys.Add(mesh);

    auto mvisA = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    mvisA->SetFEMdataType(ChVisualShapeFEA::DataType::ANCF_BEAM_BD);
    mvisA->SetColorscaleMinMax(-02, 20);
    mvisA->SetSmoothFaces(true);
    mvisA->SetWireframe(false);
    mesh->AddVisualShapeFEA(mvisA);

    auto mvisB = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    mvisB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisB->SetSymbolsThickness(0.004);
    mvisB->SetSymbolsScale(0.002);
    mvisB->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(mvisB);

    // Run-time visualization
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Cables FEM");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0.0, 0.05, 0.2), ChVector3d(0.0, 0.0, 0.0));
    vis->AttachSystem(&sys);

    // Solver settings
    if (use_MKL) {
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);
    } else {
        auto minres_solver = chrono_types::make_shared<ChSolverMINRES>();
        minres_solver->SetMaxIterations(400);
        minres_solver->SetTolerance(1e-12);
        minres_solver->EnableWarmStart(true);
        sys.SetSolver(minres_solver);
    }

    // Integrator settings
    // -------------------
    if (use_HHT) {
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        stepper->SetAlpha(-0.1);
        stepper->SetMaxIters(100);
        stepper->SetAbsTolerances(1e-10, 1e-10);
        stepper->SetVerbose(false);
    } else {
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        ////sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }

    // Simulation loop
    bool disabled = false;
    while (vis->Run()) {
        if (!disabled && sys.GetChTime() > 2) {
#ifdef FIX_NODES
            for (auto node : tip_nodes)
                node->SetFixed(false);
#else
            for (auto cnstr : tip_cnstr)
                cnstr->SetDisabled(true);
#endif
            disabled = true;
        }

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(0.01);
    }

    return 0;
}
