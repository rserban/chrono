//
// Test program to identify issues with enabling SIMD for Chrono.
// M113 accelerating on an RMS lane (provided as a trimesh).
// With the original mesh (rms1_normalized) the vehicle detracks with code 
// with SIMD enabled (Linux GCC) but completes if SIMD is disabled.
// This issue is resolved with the better rms1_normalized_mod mesh (more balanced triangles).
// Note that this issue cannot be reproduced on Windows witrh Visual Studio!

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/collision/ChCollisionSystemChrono.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Mesh terrain file
////std::string terrainFile = "terrain/rms1_normalized.obj";
std::string terrainFile = "terrain/rms1_normalized_mod.obj";

// Whether to use Irrlicht
bool useIrrlicht = true;

// Collision system type (BULLET or CHRONO)
collision::ChCollisionSystemType collsys_type = collision::ChCollisionSystemType::BULLET;

// Initial vehicle position
ChVector<> initLoc(0, 0, 0.8);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Simulation step size
double step_size = 5e-4;

// Time interval between two render frames
double render_step_size = 1.0 / 60;
double output_step_size = 1.0 / 100;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.0);

// =============================================================================
int main(int argc, char* argv[]) {
    // --------------------------
    // Construct the M113 vehicle
    // --------------------------
    TrackedVehicle m113(vehicle::GetDataFile("M113/vehicle/M113_Vehicle_SinglePin.json"), ChContactMethod::SMC);

    if (collsys_type == collision::ChCollisionSystemType::CHRONO) {
        auto cd_chrono = chrono_types::make_shared<collision::ChCollisionSystemChrono>();
        cd_chrono->SetBroadphaseGridResolution(ChVector<int>(1000, 20, 1));
        m113.GetSystem()->SetCollisionSystem(cd_chrono);
    }
    m113.GetSystem()->SetNumThreads(1, 8, 1);

    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile("M113/powertrain/M113_SimpleCVTPowertrain.json"));
    m113.InitializePowertrain(powertrain);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    m113.Initialize(ChCoordsys<>(initLoc, initRot));

    // Set visualization type for vehicle components.
    m113.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // ------------------
    // Create the terrain
    // ------------------

    double terrainLength = 50;
    RigidTerrain terrain(m113.GetSystem());
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);

    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength, 20);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 20, 20);

    // Add the mesh
    patch = terrain.AddPatch(patch_mat,
                             chrono::ChCoordsys<>(chrono::ChVector<>(terrainLength / 2.0 - 0.05, 0, 0), chrono::QUNIT),
                             vehicle::GetDataFile(terrainFile), false, 0.005);
    patch->SetColor(chrono::ChColor(0.2f, 0.2f, 0.5f));

    terrain.Initialize();

    // Driver
    auto path = StraightLinePath(ChVector<>(0, 0, 0), ChVector<>(300, 0, 0));
    ChPathFollowerDriver driver(m113, path, "my_path", 10.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp *app;
    if (useIrrlicht) {
        app = new ChTrackedVehicleIrrApp(&m113, L"M113 Vehicle Demo");
        app->SetSkyBox();
        app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
        app->SetChaseCamera(trackPoint, 6.0, 1.5);
        app->SetChaseCameraMultipliers(1e-4, 10);
        app->SetTimestep(step_size);
        app->AssetBindAll();
        app->AssetUpdateAll();
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(m113.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(m113.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(m113.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(m113.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;

    utils::CSV_writer csv(" ");

    while (m113.GetVehiclePos().x() < 300) {
        if (useIrrlicht) {
            if (!app->GetDevice()->run())
                break;
            if (step_number % render_steps == 0) {
                app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
                app->DrawAll();
                app->EndScene();
            }
        }

        // Collect output data from modules
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        m113.GetTrackShoeStates(LEFT, shoe_states_left);
        m113.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = m113.GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        if (useIrrlicht) {
            app->Synchronize("", driver_inputs);
        }
        
        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
        if (useIrrlicht) {
            app->Advance(step_size);
        }

        if (step_number % output_steps == 0) {
            csv << time << step_number;
            csv << m113.GetVehiclePos().x() << m113.GetVehicleSpeed();
            csv << "    ";
            csv << m113.GetSystem()->GetNcontacts();
            csv << "    ";
            csv << m113.GetSystem()->GetTimerStep()
                << m113.GetSystem()->GetTimerCollisionBroad()
                << m113.GetSystem()->GetTimerCollisionNarrow() << m113.GetSystem()->GetTimerCollision() << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    csv.write_to_file("timing.out");

    return 0;
}
