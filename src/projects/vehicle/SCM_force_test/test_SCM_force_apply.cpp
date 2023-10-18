// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Shu Wang, Radu Serban
// =============================================================================
//
// Viper SCM test - apply forces from disk
//
// =============================================================================

#include "test_SCM_ML_force.h"

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

bool enable_moving_patch = true;
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Output directory MUST exist
    if (!filesystem::path(out_dir).exists() || !filesystem::path(out_dir).is_directory()) {
        cout << "Data directory does NOT exist!" << endl;
        return 1;
    }
    utils::CSV_writer csv(" ");

    // Create the Chrono system (Z up)
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the rover
    auto viper = CreateViper(sys);

    // Cache wheel bodies
    std::vector<std::shared_ptr<ChBodyAuxRef>> wheels{
        viper->GetWheel(ViperWheelID::V_LF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_LB)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RB)->GetBody()   //
    };

    // Create the ML deformable terrain
    auto terrain = CreateMLTerrain(0.02, false, sys, wheels);

     // Add moving patches for each wheel
    if (enable_moving_patch) {
        double wheel_range = 0.5;
        ChVector<> size(0.5, 2 * wheel_range, 2 * wheel_range);
       
        terrain->AddMovingPatch(wheels[0], VNULL, size);
        terrain->AddMovingPatch(wheels[1], VNULL, size);
        terrain->AddMovingPatch(wheels[2], VNULL, size);
        terrain->AddMovingPatch(wheels[3], VNULL, size);
    }

    // Create the run-time visualization interface
    // auto vis = CreateVisualization(vis_type, true, sys); // vis ground grid
    auto vis = CreateVisualization(vis_type, false, sys);

    // Open I/O files
    std::ofstream ROVER_states(out_dir + "/ROVER_states_MLTerrain_applySCMMacroF.txt", std::ios::trunc);
    // std::ofstream SCM_forces_o(out_dir + "/SCM_forces_MLTerrain_applySCMMacroF.txt", std::ios::trunc);

    // Simulation loop
    std::string line;
    double force_time;
    ChVector<> abs_force;
    ChVector<> abs_torque;

    ChVector<> wheelContFList_F;
    ChVector<> wheelContFList_M;

    for (int istep = 0; istep < num_steps; istep++) {
        if (istep % 100 == 0)
            cout << "Time: " << sys.GetChTime() << endl;

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif

        // Advance system dynamics
        sys.DoStepDynamics(time_step);
        viper->Update();

        double time = sys.GetChTime();

        // SCM_forces_o << time << "    ";
        // // std::cout << time << std::endl;
        // for (int i = 0; i < 4; i++) {
        //     terrain->GetContactForceBody(wheels[i], wheelContFList_F, wheelContFList_M);
        //     SCM_forces_o << wheelContFList_F << "  " << wheelContFList_M << "    ";
        // }
        // SCM_forces_o << endl;

        // Save vehicle states
        ROVER_states << time << "   ";
        for (int i = 0; i < 4; i++) {
            ROVER_states << wheels[i]->GetPos() << "   " << wheels[i]->GetPos_dt() << "   " << wheels[i]->GetWvel_par()
                         << "   ";
        }
        ROVER_states << endl;
    }

    // SCM_forces_o.close();
    ROVER_states.close();

    return 0;
}
