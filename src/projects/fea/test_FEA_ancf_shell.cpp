//=============================================================================
// ANCF_Shell_Spherical.cpp
// Authors: Qiyuan Zhou
// WaveLab
// Illinois Institute of Technology
// Nov. 14 2019
//=============================================================================
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
// Code for generating a toroidal ANCF Shell Element
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using std::cout;
using std::endl;

// If true, let each element compute gravitational forces.
// Otherwise, use generic mesh-level gravity load.
// For ANCF elements, prefer 'true'.
bool element_gravity = true;

// Integration step size.
double time_step = 1e-4;

// Solver (PARDISO_MKL or GMRES).
// Prefer PARDISO_MKL (alternatively MUMPS).
ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;

// Timestepper (HHT or EULER_IMPLICIT or EULER_IMPLICIT_LINEARIZED). 
// Prefer HHT (or maybe EULER_IMPLICIT).
ChTimestepper::Type timestepper_type = ChTimestepper::Type::HHT;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemSMC my_system;

    //--------------------------------------//
    //      CREATE THE PHYSICAL SYSTEM      //
    //--------------------------------------//
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);  // Effective radius of curvature for all SCM contacts.
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);    // max outside detection envelope
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);      // max inside penetration
    double sphere_swept_thickness = 0.001;                   // outward additional layer around meshes

    //--------------------------------------//
    //       Create materials               //
    //--------------------------------------//
    
    // Contact material
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // Membrane
    // Create an orthotropic material. All layers for all elements share the same material.
    double rho_mem = 1000;
    ChVector3d E_mem(2.1e7, 2.1e7, 2.1e7);
    ChVector3d nu_mem(0.3, 0.3, 0.3);
    ChVector3d G_mem(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto mem_mat = chrono_types::make_shared<ChMaterialShellANCF>(rho_mem, E_mem, nu_mem, G_mem);

    //--------------------------------------//
    //        Settings for membrane         //
    //--------------------------------------//

    double dz = 0.02;  // [m] thickness of shell

    // Size of torus
    double c = 0.3;  // [m]: radius from center of hole to center of tube
    double a = 0.1;  // [m]: radius of tube

    // Discretization of torus **Must be divisible by 4**
    int n_c = 8;              // Half the discretizations in large structure (X-Z plane)
    int n_a = 4;              // Half the discretizations in torus tube (along cirumference of small tube)
    int tot = 4 * n_c * n_a;  // Total discretizations in torus

    // Initial height of objects
    double init_height = 2 * a;

    // Logic check
    if (a >= c) {
        cout << "a cannot be larger than c"
             << "\n";
        return 0;
    }

    //--------------------------------------//
    //        Create FEA nodes              //
    //--------------------------------------//

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto mem_mesh = chrono_types::make_shared<ChMesh>();

    for (double u = 0; u < CH_2PI; u = u + CH_PI / n_c) {
        for (double v = 0; v < CH_2PI; v = v + CH_PI / n_a) {
            // Point on torus
            double x = cos(u) * (c + a * cos(v));
            double y = a * sin(v) + init_height;
            double z = sin(u) * (c + a * cos(v));
            ChVector3d location(x, y, z);

            /*
            // Tangent with respect to parameter u
            double t1_x = sin(u) * (-(a * cos(v) + c));
            double t1_y = 0;
            double t1_z = cos(u) * (a * cos(v) + c);

            // Tangent with respect to parameter v
            double t2_x = -a * cos(u) * sin(v);
            double t2_y = a * cos(v);
            double t2_z = -a * sin(u) * sin(v);

            // Normal vector components (cross product of tangents)
            double uvi = t1_y * t2_z - t2_y * t1_z;
            double uvj = t2_x * t1_z - t1_x * t2_z;
            double uvk = t1_x * t2_y - t2_x * t1_y;

            ChVector3d D(uvi, uvj, uvk);
            D.Normalize();
            */

            double dx = -cos(v) * cos(u);
            double dy = -sin(v);
            double dz = -cos(v) * sin(u);
            ChVector3d D(dx, dy, dz);

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(x, y, z), D);
            node->SetMass(0);
            mem_mesh->AddNode(node);
        }
    }

    // Get a handle to the tip node.
    auto first_node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(0));

    //--------------------------------------//
    //         Adding ANCF elements         //
    //--------------------------------------//

    // Regular elements:
    for (int i = 0; i < (2 * n_c) - 1; i++) {
        for (int j = 0; j < (2 * n_a) - 1; j++) {
            // Adjacent nodes
            int node0 = (i * n_c) + j;
            int node1 = (i * n_c) + j + (2 * n_a);
            int node2 = (i * n_c) + j + (2 * n_a) + 1;
            int node3 = (i * n_c) + j + 1;
            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node3)));

            element->SetDimensions(a / n_a, a / n_a);             // Set element dimensions
            element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
            element->SetAlphaDamp(0.0);                           // Structural damping for this element
            mem_mesh->AddElement(element);
        }
    }

    // End (large R) elements:
    for (int l = 0; l < 2 * n_a - 1; l++) {
        // Adjacent nodes
        int node0 = (4 * n_c * n_a) - (2 * n_a) + l;
        int node1 = l;
        int node2 = l + 1;
        int node3 = (4 * n_c * n_a) - (2 * n_a) + 1 + l;
        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node3)));

        element->SetDimensions(a / n_a, a / n_a);             // Set element dimensions
        element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
        element->SetAlphaDamp(0.0);                           // Structural damping for this element
        mem_mesh->AddElement(element);
    }

    // Ends (small r) elements:
    for (int m = 1; m < (2 * n_c); m++) {
        // Adjacent nodes
        int node0 = (m * n_c) - 1;
        int node1 = (m * n_c) + (2 * n_a) - 1;
        int node2 = ((m - 1) * n_c) + (2 * n_a);
        int node3 = ((m - 1) * n_c);
        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node3)));

        element->SetDimensions(a / n_a, a / n_a);             // Set element dimensions
        element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
        element->SetAlphaDamp(0.0);                           // Structural damping for this element
        mem_mesh->AddElement(element);
    }

    // End element(R and r)
    auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
    element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(tot - 1)),
                      std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode((n_a * 2) - 1)),
                      std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(0)),
                      std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode((2 * n_c) * n_c - 2 * n_a)));

    element->SetDimensions(a / n_a, a / n_a);             // Set element dimensions
    element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
    element->SetAlphaDamp(0.0);                           // Structural damping for this element
    mem_mesh->AddElement(element);

    // Enable/disable mesh-level automatic gravity load
    mem_mesh->SetAutomaticGravity(!element_gravity);

    // FEA mesh contact surface
    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>(mysurfmaterial);
    mem_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddFacesFromBoundary(sphere_swept_thickness);  // do this after my_mesh->AddContactSurface

    //--------------------------------------//
    //           ANCF Visualization         //
    //--------------------------------------//

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(mem_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    mem_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(mem_mesh);
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    mem_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(mem_mesh);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    mem_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(mem_mesh);
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    mem_mesh->AddVisualShapeFEA(mvisualizemeshD);

    my_system.Add(mem_mesh);  // Add the mesh to the system

    //--------------------------------------//
    //        Generating environment        //
    //--------------------------------------//

    // Create a floor as a simple collision primitive:
    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(5, 0.1, 5, 700, true, true, mysurfmaterial);
    mfloor->SetPos(ChVector3d(0, -0.1, 0));
    mfloor->SetFixed(true);
    my_system.Add(mfloor);

    // two falling objects:
    auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 5000, true, true, mysurfmaterial);
    mcube->SetPos(ChVector3d(0.6, init_height, 0.6));
    my_system.Add(mcube);

    auto msphere = chrono_types::make_shared<ChBodyEasySphere>(0.1, 5000, true, true, mysurfmaterial);
    msphere->SetPos(ChVector3d(0.8, init_height, 0.6));
    my_system.Add(msphere);

    //--------------------------------------//
    //   Create the Irrlicht visualization  //
    //--------------------------------------//

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ANCF Shells");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0.0, 0.6, -1.0), ChVector3d(0.0, 0.0, 0.0));
    vis->AttachSystem(&my_system);


    //--------------------------------------//
    //         Running the simulation       //
    //--------------------------------------//

    switch (solver_type) {
        case ChSolver::Type::PARDISO_MKL: {
            cout << "Using PARDISO solver" << endl;
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            my_system.SetSolver(solver);
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            break;
        }
        case ChSolver::Type::GMRES: {
            cout << "Using GMRES solver" << endl;
            auto solver = chrono_types::make_shared<ChSolverGMRES>();
            my_system.SetSolver(solver);
            solver->SetMaxIterations(100);
            solver->SetTolerance(1e-10);
            solver->EnableDiagonalPreconditioner(true);
            if (timestepper_type == ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED)
                solver->EnableWarmStart(true);
            solver->SetVerbose(false);
            break;
        }
        default:
            cout << "\nUnknown solver (use PARDISO or GMRES)\n" << endl;
            return 1;
    }

    switch (timestepper_type) {
        case ChTimestepper::Type::HHT: {
            cout << "Using HHT timestepper" << endl;
            auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&my_system);
            my_system.SetTimestepper(stepper);
            stepper->SetAlpha(-0.2);
            stepper->SetMaxIters(20);
            stepper->SetAbsTolerances(1e-5, 1e-3);
            stepper->SetStepControl(true);
            stepper->SetMinStepSize(1e-8);
            stepper->SetVerbose(false);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT: {
            cout << "Using EULER_IMPLICIT timestepper" << endl;
            my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
            break;
        }
        case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED: {
            cout << "Using EULER_IMPLICIT_LINEARIZED timestepper" << endl;
            my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
            break;
        }
        default:
            cout << "\nUnknown timestepper (use HHT or EULER_IMPLICIT or EULER_IMPLICIT_LINEARIZED)\n" << endl;
            return 1;
    }

    while (vis->Run()) {
        cout << "Simulation Time= " << my_system.GetChTime() << "\n";
        cout << "  node: " << first_node->GetPos() << "\n";
        cout << "  ball: " << msphere->GetPos() << "\n";
        cout << "  cube: " << mcube->GetPos() << "\n";
        cout << endl;

        vis->BeginScene();
        vis->Render();
        my_system.DoStepDynamics(time_step);
        vis->EndScene();
    }

    return 0;
}