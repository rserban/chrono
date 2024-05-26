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
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

// Uncomment the following line to unconditionally disable Irrlicht support
//#undef CHRONO_IRRLICHT
#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
#endif

#include "../terrain/RigidTerrainSlope.h"      // slope rigid terrain
#include "../terrain/RigidTerrainStep.h"       // step rigid terrain
#include "../terrain/RigidTerrainTrapezoid.h"  // trapezoid rigid terrain

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector3d initLoc(-100, 0, 0.75);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Simulation step size
double step_size = 1e-2;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 0.0);

// Output directories
const std::string out_dir = "../M113";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Visualization type for all vehicle subsystems (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::PRIMITIVES;

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = true;

// =============================================================================
int main(int argc, char* argv[]) {
    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    // Create the vehicle system
    M113 m113;
    m113.SetContactMethod(ChContactMethod::SMC);
    m113.SetChassisFixed(false);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetDrivelineType(DrivelineTypeTV::SIMPLE);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);

    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();
    auto& vehicle = m113.GetVehicle();
    auto engine = vehicle.GetEngine();

    // Set visualization type for subsystems
    m113.SetChassisVisualizationType(vis_type);
    m113.SetSprocketVisualizationType(vis_type);
    m113.SetIdlerVisualizationType(vis_type);
    m113.SetSuspensionVisualizationType(vis_type);
    m113.SetRoadWheelVisualizationType(vis_type);
    m113.SetTrackShoeVisualizationType(vis_type);

    // Control steering type (enable crossdrive capability).
    m113.GetDriveline()->SetGyrationMode(true);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->EnableWarmStart(true);
    solver->SetTolerance(1e-10);
    m113.GetSystem()->SetSolver(solver);

    // ------------------
    // Create the terrain
    // ------------------

    auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    patch_mat->SetFriction(0.8f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);

    // Height-map terrain
    //     texture in Chrono::Vehicle data directory
    //     bitmap in VehicleTests data directory
    ////RigidTerrain terrain(m113.GetSystem());
    ////auto patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("M113a_benchmark/height_maps/slope4.bmp"), "my_mesh", 270, 20, -3, 3);
    ////patch->SetColor(ChColor(0.4f, 0.2f, 0.0f));
    ////patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 12, 12);
    ////terrain.Initialize();

    // Step terrain
    //    height change:  0 -> 0.2
    //    X-Y dimensions of flat areas: 120x40
    ////RigidTerrainStep terrain(m113.GetSystem());
    ////terrain.SetColor(ChColor(0.4f, 0.2f, 0.0f));
    ////terrain.Initialize(patch_mat, 0, 0.2, 120, 40);
    ////terrain.SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 12, 12);

    // Slope terrain
    //    height change: 0 -> 15
    //    grade: 20%
    //    X-Y dimensions of flat areas: 120x40
    ////RigidTerrainSlope terrain(m113.GetSystem());
    ////terrain.SetColor(ChColor(0.4f, 0.2f, 0.0f));
    ////terrain.Initialize(patch_mat, 0, 15, 20, 120, 40);
    ////terrain.SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 12, 12);

    // Trapezoid terrain
    //    10cm tall, 50cm top width, 45 deg trapezoid bump at x = -95m
    //    X-Y dimensions of the first flat area: 100x4
    //    X-Y dimensions of the last flat area:  100x4
    RigidTerrainTrapezoid terrain(m113.GetSystem());
    terrain.Initialize(patch_mat, 0, 0.1, 0, CH_PI_4, CH_PI_4, 100, 0.5, 100, 4, -95);
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 12, 12);

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        ////terrain.ExportMeshPovray(out_dir);
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

// ---------------------------------------
// Create the vehicle Irrlicht application
// Create the driver system
// ---------------------------------------

#ifdef CHRONO_IRRLICHT

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("M113 Vehicle Demo");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->SetChaseCameraMultipliers(1e-4, 10);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // Create the interactive driver system
    ChInteractiveDriverIRR driver(*vis);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();

#else

    ChDataDriver driver(vehicle, vehicle::GetDataFile("M113a_benchmark/AccelerationBraking.txt"));
    driver.Initialize();

#endif

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));
    DriverInputs driver_inputs = {0, 0, 0};

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

#ifdef CHRONO_IRRLICHT

    while (vis->Run()) {
        // Debugging output
        if (dbg_output) {
            cout << "Time: " << m113.GetSystem()->GetChTime() << endl;
            const ChFrameMoving<>& c_ref = m113.GetChassisBody()->GetFrameRefToAbs();
            const ChVector3d& c_pos = m113.GetChassis()->GetPos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector3d& i_pos_abs = vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector3d& i_pos_abs = vehicle.GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
        }

        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(m113.GetSystem(), filename);
            }

            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
            }

            render_frame++;
        }

        // Collect output data from modules
        driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

#else

//// TODO

#endif

    vehicle.WriteContacts("M113_contacts.out");

    return 0;
}
