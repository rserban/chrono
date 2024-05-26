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

#include "test_SCM_force.h"

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

// Load container to apply wheel forces and torques from data file
class TerrainForceLoader : public ChLoadContainer {
  public:
    TerrainForceLoader(const std::string& filename, std::vector<std::shared_ptr<ChBodyAuxRef>> wheels)
        : m_wheels(wheels), m_num_frames(0), m_crt_frame(0) {
        m_fstream = std::ifstream(filename);
        if (!m_fstream.is_open()) {
            cout << "Unable to open the " << filename << " file!" << endl;
        }

        double time;
        std::string line;
        while (std::getline(m_fstream, line)) {
            std::istringstream sstream(line);
            std::array<ChVector3d, 4> force;
            std::array<ChVector3d, 4> torque;
            sstream >> time;
            for (int i = 0; i < 4; i++) {
                sstream >> force[i].x() >> force[i].y() >> force[i].z();
                sstream >> torque[i].x() >> torque[i].y() >> torque[i].z();
            }
            m_forces.push_back(force);
            m_torques.push_back(torque);
            m_num_frames++;
        }

        std::cout << "Read " << m_num_frames << " frames from data file" << filename << std::endl;
    }

    ~TerrainForceLoader() { m_fstream.close(); }

    // Apply forces to wheel bodies, at the beginning of each timestep.
    virtual void Setup() override {
        // Reset load list
        GetLoadList().clear();

        // Generate loads for this time step
        for (int i = 0; i < 4; i++) {
            auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_wheels[i], m_forces[m_crt_frame][i], false,
                                                                         m_wheels[i]->GetPos(), false);
            auto torque_load =
                chrono_types::make_shared<ChLoadBodyTorque>(m_wheels[i], m_torques[m_crt_frame][i], false);

            Add(force_load);
            Add(torque_load);
        }
        if (m_crt_frame < m_num_frames - 1)
            m_crt_frame++;

        // Invoke base class method
        ChLoadContainer::Update(ChTime, true);
    }

  private:
    std::ifstream m_fstream;
    int m_num_frames;
    int m_crt_frame;
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_wheels;
    std::vector<std::array<ChVector3d, 4>> m_forces;
    std::vector<std::array<ChVector3d, 4>> m_torques;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Output directory MUST exist
    if (!filesystem::path(out_dir).exists() || !filesystem::path(out_dir).is_directory()) {
        cout << "Data directory does NOT exist!" << endl;
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

    // Create the terrain loader
    auto terrain = chrono_types::make_shared<TerrainForceLoader>(SCM_filename, wheels);
    sys.Add(terrain);

    // Create the run-time visualization interface
    auto vis = CreateVisualization(vis_type, true, sys);

    // Open I/O files
    std::ofstream ROVER_states(out_dir + "/ROVER_states_applied.txt", std::ios::trunc);

    /*
    std::ifstream SCM_forces(SCM_filename);
    if (!SCM_forces.is_open()) {
        cout << "Unable to open the " << SCM_filename << " file!" << endl;
        return 1;
    }
    std::string line;
    double force_time;
    ChVector3d abs_force;
    ChVector3d abs_torque;
    */

    // Simulation loop
    for (int istep = 0; istep < num_steps + 10; istep++) {
        if (istep % 100 == 0)
            cout << "Time: " << sys.GetChTime() << endl;

#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(viper->GetChassis()->GetPos());
        vis->Render();
        vis->EndScene();
#endif

        /*
        // Read SCM force from file and apply to vehicle wheels using FORCE ACCUMULATORS
        std::getline(SCM_forces, line);
        std::istringstream iss(line);
        iss >> force_time;
        for (int i = 0; i < 4; i++) {
            iss >> abs_force.x() >> abs_force.y() >> abs_force.z();
            iss >> abs_torque.x() >> abs_torque.y() >> abs_torque.z();

            wheels[i]->EmptyAccumulators();
            wheels[i]->AccumulateForce(abs_force, wheels[i]->GetPos(), false);
            wheels[i]->AccumulateTorque(abs_torque, false);
        }
        */

        // Advance system dynamics
        sys.DoStepDynamics(time_step);
        viper->Update();

        double time = sys.GetChTime();

        // Save vehicle states
        ROVER_states << time << "   ";
        for (int i = 0; i < 4; i++) {
            ROVER_states << wheels[i]->GetPos() << "   " << wheels[i]->GetPosDt() << "   " << wheels[i]->GetAngVelParent()
                         << "   ";
        }
        ROVER_states << endl;
    }

    ////SCM_forces.close();
    ROVER_states.close();

    return 0;
}
