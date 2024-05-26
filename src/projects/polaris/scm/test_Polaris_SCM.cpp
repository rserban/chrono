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
// Author: Radu Serban
// =============================================================================
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <array>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// -----------------------------------------------------------------------------
// Terrain parameters
// -----------------------------------------------------------------------------

double terrainLength = 8.0;  // size in X direction
double terrainWidth = 4.0;   // size in Y direction
double delta = 0.05;         // SCM grid spacing

bool heightmapterrain = false;
std::string heightmap_file = "";

ChCoordsys<> init_pos(ChVector3d(1.3, 0, 0.1), QUNIT);

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-3;

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 1.75);

// Output directories
const std::string out_dir = GetChronoOutputPath() + "POLARIS_SCM";

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------
    // Create the Chrono system
    // ------------------------

    ChSystemNSC sys;
    sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()));
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // --------------------------
    // Create the Polaris vehicle
    // --------------------------

    cout << "Create vehicle..." << endl;
    std::string vehicle_json = "Polaris/Polaris.json";
    std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
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

    std::array<std::shared_ptr<ChWheel>, 4> wheels = {
        vehicle.GetWheel(0, VehicleSide::LEFT), vehicle.GetWheel(0, VehicleSide::RIGHT),
        vehicle.GetWheel(1, VehicleSide::LEFT), vehicle.GetWheel(1, VehicleSide::RIGHT)};

    // ------------------
    // Create the terrain
    // ------------------

    SCMTerrain terrain(&sys);
    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Optionally, enable moving patch feature (multiple patches around each wheel)
    ////for (auto& axle : vehicle.GetAxles()) {
    ////    terrain.AddMovingPatch(axle->m_wheels[0]->GetSpindle(), ChVector3d(0, 0, 0), ChVector3d(1, 0.5, 1));
    ////    terrain.AddMovingPatch(axle->m_wheels[1]->GetSpindle(), ChVector3d(0, 0, 0), ChVector3d(1, 0.5, 1));
    ////}

    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);

    if (heightmapterrain) {
        terrain.Initialize(heightmap_file,  ///< [in] filename for the height map (image file)
                           terrainLength,   ///< [in] terrain dimension in the X direction
                           terrainWidth,    ///< [in] terrain dimension in the Y direction
                           0.0,             ///< [in] minimum height (black level)
                           1.5,             ///< [in] maximum height (white level)
                           delta            ///< [in] grid spacing (may be slightly decreased)
        );
    } else {
        terrain.Initialize(terrainLength, terrainWidth, delta);
    }

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChWheeledVehicleVisualSystemIrrlicht vis;
    vis.SetWindowTitle("HMMWV Deformable Soil Demo");
    vis.SetChaseCamera(trackPoint, 6.0, 0.5);
    vis.Initialize();
    vis.AddLightDirectional();
    vis.AddSkyBox();
    vis.AddLogo();
    vis.AttachVehicle(&vehicle);

    // -----------------
    // Initialize output
    // -----------------
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    utils::ChWriterCSV csv(" ");

    // ---------------
    // Simulation loop
    // ---------------
    std::cout << "Total vehicle mass: " << vehicle.GetMass() << std::endl;

    DriverInputs driver_inputs = {0.0, 0.0, 1.0};

    int num_times = 0;
    while (vis.Run()) {
        double time = sys.GetChTime();

        // Render scene
        vis.BeginScene();
        vis.Render();
        vis.EndScene();

        // Output tire forces
        csv << time;
        for (int i = 0; i < 4; i++) {
            ChVector3d force, moment;
            const auto& tfrc = terrain.GetContactForceBody(wheels[i]->GetSpindle(), force, moment);
            csv << force << moment;
        }
        csv << endl;
        num_times++;

        // Update modules
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        vis.Synchronize(time, driver_inputs);

        // Advance dynamics
        sys.DoStepDynamics(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        vis.Advance(step_size);
    }

    csv.WriteToFile(out_dir + "/tire_forces.dat", std::to_string(num_times));

    return 0;
}
