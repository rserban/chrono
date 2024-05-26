// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Polaris wheeled vehicle on external terrain (SCM data)
//
// =============================================================================

#include <cstdio>
#include <string>
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

class SCMDataTerrain : public ChTerrain {
  public:
    SCMDataTerrain(WheeledVehicle& vehicle, const std::string& data_file);
    virtual void Synchronize(double time) override;
    bool Done() const { return m_done; }

  private:
    const WheeledVehicle& m_vehicle;
    std::array<std::shared_ptr<ChWheel>, 4> m_wheels;
    std::array<TerrainForce, 4> m_tire_forces;

    std::vector<double> m_time;
    std::array<std::vector<ChVector3d>, 4> m_frc_data;
    std::array<std::vector<ChVector3d>, 4> m_trq_data;
    int m_last;

    bool m_done;
};

SCMDataTerrain::SCMDataTerrain(WheeledVehicle& vehicle, const std::string& data_file) : m_vehicle(vehicle), m_last(0), m_done(false) {
    m_wheels[0] = vehicle.GetWheel(0, VehicleSide::LEFT);
    m_wheels[1] = vehicle.GetWheel(0, VehicleSide::RIGHT);
    m_wheels[2] = vehicle.GetWheel(1, VehicleSide::LEFT);
    m_wheels[3] = vehicle.GetWheel(1, VehicleSide::RIGHT);

    // Create a ground body (only to carry some visualization assets)
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto box = chrono_types::make_shared<ChVisualShapeBox>(20, 20, 1);
    box->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 10, 10);
    ground->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -0.5), QUNIT));

    vehicle.GetSystem()->AddBody(ground);

    // Read data
    std::ifstream ifile;
    std::string line;
    try {
        ifile.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        ifile.open(data_file.c_str());
    } catch (const std::exception&) {
        cerr << "Cannot open input file " << data_file << endl;
        return;
    }

    // Read number of data points
    int num_times = 0;
    {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> num_times;
        m_time.reserve(num_times);
        for (int i = 0; i < 4; i++) {
            m_frc_data[i].reserve(num_times);
            m_trq_data[i].reserve(num_times);
        }
    }

    double time;
    double x, y, z;
    for (int it = 0; it < num_times; it++) {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> time;
        m_time.push_back(time);
        for (int i = 0; i < 4; i++) {
            iss >> x >> y >> z;
            m_frc_data[i].push_back(ChVector3d(x, y, z));
            iss >> x >> y >> z;
            m_trq_data[i].push_back(ChVector3d(x, y, z));
        }
    }

    cout << "Read SCM data file: " << data_file << endl;
}

void SCMDataTerrain::Synchronize(double time) {
    m_done = m_last > m_time.size() - 1;
    if (m_done)
        return;

    // 1. Interpolate forces from data
    assert(time >= m_time[m_last]);
    while (time > m_time[m_last + 1])
      m_last++;

    double tau = (time - m_time[m_last]) / (m_time[m_last + 1] - m_time[m_last]);

    for (int i = 0; i < 4; i++) {
        auto f0 = m_frc_data[i][m_last];
        auto f1 = m_frc_data[i][m_last + 1];
        m_tire_forces[i].force = f0 + (f1 - f0) * tau;

        auto t0 = m_trq_data[i][m_last];
        auto t1 = m_trq_data[i][m_last + 1];
        m_tire_forces[i].moment = t0 + (t1 - t0) * tau;

        m_tire_forces[i].point = m_wheels[i]->GetSpindle()->GetPos();
    }

    m_last++;

    // 2. Add tire forces as external forces to spindle bodies
    for (int i = 0; i < 4; i++) {
        m_wheels[i]->GetSpindle()->AccumulateForce(m_tire_forces[i].force, m_tire_forces[i].point, false);
        m_wheels[i]->GetSpindle()->AccumulateTorque(m_tire_forces[i].moment, false);
    }
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create the Chrono system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Create vehicle
    ChCoordsys<> init_pos(ChVector3d(0, 0, 0.5), QUNIT);

    std::string vehicle_json = "Polaris/Polaris.json";
    std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_json = "Polaris/Polaris_AutomaticTransmisionSimpleMap.json";
    std::string tire_json = "Polaris/Polaris_RigidTire.json";

    // Create and initialize the vehicle
    WheeledVehicle vehicle(&sys, vehicle::GetDataFile(vehicle_json));
    vehicle.Initialize(init_pos);
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Create terrain
    SCMDataTerrain terrain(vehicle, GetChronoOutputPath() + "POLARIS_SCM/tire_forces.dat");

    // Create the vehicle run-time visualization interface
    ChWheeledVehicleVisualSystemIrrlicht vis;
    vis.SetWindowTitle("Polaris - SCM data terrain");
    vis.SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis.Initialize();
    vis.AddTypicalLights();
    vis.AddSkyBox();
    vis.AddLogo();
    vis.AttachVehicle(&vehicle);

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 1};

    double step_size = 1e-3;
    while (vis.Run()) {
        double time = sys.GetChTime();

        vis.BeginScene();
        vis.Render();
        vis.EndScene();

        // Synchronize subsystems
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        if (terrain.Done())
            break;

        // Advance system state
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        vis.Advance(step_size);
        sys.DoStepDynamics(step_size);
    }

    return 0;
}
