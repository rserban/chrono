// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test tire models on terrain with varying coefficient of friction.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/m113/powertrain/M113_EngineSimpleMap.h"
#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================

// Simulation step size, end time
double step_size = 1e-3;

// Output
bool output = false;
double out_fps = 50;
const std::string out_dir = "../M113_FRICTION_VARIABLE";

// =============================================================================

class MyFrictionFunctor : public ChTerrain::FrictionFunctor {
  public:
    MyFrictionFunctor() : m_friction_left(0.1f), m_friction_right(0.9f) {}
    virtual float operator()(const ChVector3d& loc) override { return loc.y() > 0 ? m_friction_left : m_friction_right; }
    float m_friction_left;
    float m_friction_right;
};

// =============================================================================

int main(int argc, char* argv[]) {
    // Chrono system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Create and initialize the vehicle
    M113_Vehicle_SinglePin vehicle(false, DrivelineTypeTV::SIMPLE, BrakeType::SIMPLE, false, false, false, &sys,
                                   CollisionType::NONE);
    vehicle.Initialize(ChCoordsys<>(ChVector3d(-90.0, 0.0, 1.0), QUNIT));
    vehicle.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Control steering type (enable crossdrive capability)
    vehicle.GetDriveline()->SetGyrationMode(true);

    // Create and initialize the powertrain system
    auto engine = chrono_types::make_shared<M113_EngineSimpleMap>("engine");
    auto transmission = chrono_types::make_shared<M113_AutomaticTransmissionSimpleMap>("transmission");
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create the terrain
    RigidTerrain terrain(&sys);

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.01f);
    mat->SetYoungModulus(2e7f);
    auto patch = terrain.AddPatch(mat, CSYSNORM, 200, 10);
    patch->SetColor(ChColor(0.8f, 0.8f, 1.0f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 10);

    auto ffun = chrono_types::make_shared<MyFrictionFunctor>();
    terrain.RegisterFrictionFunctor(ffun);
    terrain.UseLocationDependentFriction(true);

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("M113 friction test");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 0.0), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Output interval
    double out_step_size = 1 / out_fps;
    int out_steps = (int)std::ceil(out_step_size / step_size);
    int sim_frame = 0;

    while (vis->Run()) {
        double time = sys.GetChTime();

        auto veh_pos = vehicle.GetPos();

        // Extract output
        if (output && sim_frame % out_steps == 0) {
            double veh_speed = vehicle.GetSpeed();

            csv << time << veh_pos.x() << veh_speed << std::endl;
        }

        // Stop before end of terrain patch
        if (veh_pos.x() > 80 || std::abs(veh_pos.y()) > 4) {
            std::cout << "Longitudinal travel distance = " << veh_pos.x() + 90 << std::endl;
            break;
        }

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Driver inputs
        DriverInputs driver_inputs;
        driver_inputs.m_steering = 0;
        driver_inputs.m_braking = 0;
        driver_inputs.m_throttle = 0;
        if (time > 1 && time < 2)
            driver_inputs.m_throttle = 0.75 * (time - 1);
        else if (time > 2)
            driver_inputs.m_throttle = 0.75;

        // Collect output data from sub-systems
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules.
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        // Advance state of entire system
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    if (output) {
        csv.WriteToFile(out_dir + "/m113.out");
    }

    return 0;
}
