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

#include <string>
#include <fstream>
#include <sstream>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/ChDriver.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_models/vehicle/feda/FEDA.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

double step_size = 1e-3;
const std::string out_dir = "../TEST_REINIT";

// =============================================================================

DriverInputs GetDriverInputs(double time) {
    DriverInputs d = {0, 0, 0};
    if (time < 0.5)
        return d;
    time = time - 0.5;
    if (time < 1) {
        d.m_throttle = time;
        return d;
    }
    time = time - 1;
    if (time < 1) {
        d.m_throttle = 1;
        d.m_steering = 1 * time;
        return d;
    }
    d.m_throttle = 1;
    d.m_steering = 1;
    return d;
}

// =============================================================================

void SaveCheckpoint(const std::string& filename, ChVehicle& vehicle, const DriverInputs& d) {
    std::cout << "SAVE checkpoint at t = " << vehicle.GetSystem()->GetChTime() << std::endl;
    std::cout << "  Driver inputs = " << d.m_throttle << " " << d.m_braking << " " << d.m_steering << std::endl;
    std::cout << "  Vehicle speed = " << vehicle.GetSpeed() << std::endl;

    double time;
    ChState X;
    ChStateDelta V;
    ChStateDelta A;
    vehicle.GetSystem()->StateSetup(X, V, A);
    vehicle.GetSystem()->StateGather(X, V, time);

    std::ofstream stream;
    stream.open(filename, std::ios_base::trunc);
    stream << time << " " << d.m_throttle << " " << d.m_braking << " " << d.m_steering << std::endl;
    for (int i = 0; i < X.size(); i++)
        stream << X[i] << std::endl;
    for (int i = 0; i < V.size(); i++)
        stream << V[i] << std::endl;
    stream.close();
}

void LoadCheckpoint(const std::string& filename, ChVehicle& vehicle, DriverInputs& d) {
    std::cout << "LOAD checkpoint at time = " << vehicle.GetSystem()->GetChTime() << std::endl;
    std::cout << "  Current driver inputs = " << d.m_throttle << " " << d.m_braking << " " << d.m_steering << std::endl;
    std::cout << "  Current vehicle speed = " << vehicle.GetSpeed() << std::endl;

    double time = 0;
    ChState X;
    ChStateDelta V;
    ChStateDelta A;
    vehicle.GetSystem()->Setup();
    vehicle.GetSystem()->StateSetup(X, V, A);

    std::string line;
    std::ifstream stream;
    stream.open(filename, std::ios_base::in);
    std::getline(stream, line);
    std::istringstream iss(line);
    iss >> time >> d.m_throttle >> d.m_braking >> d.m_steering;
    for (int i = 0; i < X.size(); i++) {
        std::getline(stream, line);
        std::istringstream iss(line);
        iss >> X[i];
    }
    for (int i = 0; i < V.size(); i++) {
        std::getline(stream, line);
        std::istringstream iss(line);
        iss >> V[i];
    }
    stream.close();

    vehicle.GetSystem()->StateScatter(X, V, vehicle.GetSystem()->GetChTime(), false);
    vehicle.GetSystem()->Update(false);

    std::cout << "  Checkpoint time = " << time << std::endl;
    std::cout << "  New driver inputs = " << d.m_throttle << " " << d.m_braking << " " << d.m_steering << std::endl;
    std::cout << "  New vehicle speed = " << vehicle.GetSpeed() << std::endl;

    vehicle.LogConstraintViolations();
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the vehicle, set parameters, and initialize
    /*
    ////hmmwv::HMMWV_Full veh;
    hmmwv::HMMWV_Reduced veh;
    veh.SetContactMethod(ChContactMethod::NSC);
    veh.SetChassisFixed(false);
    veh.SetChassisCollisionType(CollisionType::NONE);
    veh.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 1.0), QUNIT));
    veh.SetPowertrainType(PowertrainModelType::SHAFTS);
    veh.SetDriveType(DrivelineTypeWV::AWD);
    veh.SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    veh.SetTireType(TireModelType::TMEASY);
    veh.SetTireStepSize(step_size);
    */
    
    feda::FEDA veh;
    veh.SetContactMethod(ChContactMethod::NSC);
    veh.SetChassisFixed(false);
    veh.SetChassisCollisionType(CollisionType::NONE);
    veh.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 1.0), QUNIT));
    veh.SetEngineType(EngineModelType::SHAFTS);
    veh.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    veh.SetTireType(TireModelType::PAC02);
    veh.SetTireStepSize(step_size);
    
    veh.Initialize();

    veh.SetChassisVisualizationType(VisualizationType::NONE);
    veh.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    veh.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    veh.SetWheelVisualizationType(VisualizationType::NONE);
    veh.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    RigidTerrain terrain(veh.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200.0, 100.0);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Vehicle reinitialization test");
    vis->SetWindowSize(1000, 800);
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&veh.GetVehicle());

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    std::string filename = out_dir + "/checkpoint.txt";

    // --------------------------------------
    // Simulatin loop
    // --------------------------------------
    DriverInputs driver_inputs;

    /*
    // Start simulation from checkpoint
    bool check_saved = true;
    bool check_loaded = true;
    LoadCheckpoint(filename, veh.GetVehicle(), driver_inputs);
    */

    // Start simulation from scratch
    bool check_saved = false;
    bool check_loaded = false;

    while (vis->Run()) {
        double time = veh.GetSystem()->GetChTime();

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Driver inputs (do not change driver inputs after reset)
        if (!check_saved && !check_loaded)
            driver_inputs = GetDriverInputs(time);
        else if (!check_loaded)
            driver_inputs = {0, 0, 0.5};

        // Update modules (process inputs from other modules)
        terrain.Synchronize(time);
        veh.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        terrain.Advance(step_size);
        veh.Advance(step_size);
        vis->Advance(step_size);

        if (!check_saved && time > 3.0) {
            SaveCheckpoint(filename, veh.GetVehicle(), driver_inputs);
            vis->SetChaseCameraState(utils::ChChaseCamera::Track);
            check_saved = true;
        }

        if (!check_loaded && time > 5.0) {
            LoadCheckpoint(filename, veh.GetVehicle(), driver_inputs);
            check_loaded = true;
        }
    }

    return 0;
}
