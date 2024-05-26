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
// Viper SCM test - save terrain forces to disk
//
// =============================================================================

#include <limits>
#include <iomanip>

#include "test_SCM_force.h"

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// SCM grid spacing
double SCM_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = false;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }
    utils::ChWriterCSV csv(" ");

    // Create the Chrono system (Z up)
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Create the rover
    auto viper = CreateViper(sys);

    // Cache wheel bodies
    std::vector<std::shared_ptr<ChBodyAuxRef>> wheels{
        viper->GetWheel(ViperWheelID::V_LF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RF)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_LB)->GetBody(),  //
        viper->GetWheel(ViperWheelID::V_RB)->GetBody()   //
    };

    // Create the SCM deformable terrain
    auto terrain = CreateTerrain(SCM_resolution, enable_bulldozing, sys);

    // Add moving patches for each wheel
    if (enable_moving_patch) {
        double wheel_range = 0.5;
        ChVector3d size(0.5, 2 * wheel_range, 2 * wheel_range);
       
        terrain->AddMovingPatch(wheels[0], VNULL, size);
        terrain->AddMovingPatch(wheels[1], VNULL, size);
        terrain->AddMovingPatch(wheels[2], VNULL, size);
        terrain->AddMovingPatch(wheels[3], VNULL, size);
    }

    // Create the run-time visualization interface
    auto vis = CreateVisualization(vis_type, false, sys);

    // Open I/O files
    std::ofstream SCM_forces(SCM_filename, std::ios::trunc);
    std::ofstream ROVER_states(out_dir + "/ROVER_states_saved.txt", std::ios::trunc);

    // Simulation loop
    ChVector3d wheelContFList_F;
    ChVector3d wheelContFList_M;

    for (int istep = 0; istep < num_steps; istep++) {
        if (istep % 100 == 0)
            cout << "Time: " << sys.GetChTime() << endl;

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif

        // Time at *beginning* of step
        double time = sys.GetChTime();

        // Advance system dynamics
        sys.DoStepDynamics(time_step);
        viper->Update();

        // Save SCM terrain forces that were applied during the *previous* step
        SCM_forces << time << "    ";
        for (int i = 0; i < 4; i++) {
            terrain->GetContactForceBody(wheels[i], wheelContFList_F, wheelContFList_M);
            SCM_forces << std::setprecision(20) << wheelContFList_F << "  "
                       << wheelContFList_M << "    ";
        }
        SCM_forces << endl;

        // Save vehicle states
        ROVER_states << time << "   ";
        for (int i = 0; i < 4; i++) {
            ROVER_states << wheels[i]->GetPos() << "   " << wheels[i]->GetPosDt() << "   " << wheels[i]->GetAngVelParent()
                         << "   ";
        }
        ROVER_states << endl;
    }

    SCM_forces.close();
    ROVER_states.close();

    return 0;
}
