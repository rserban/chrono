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
#include "chrono/collision/multicore/ChCollisionSystemMulticore.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

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

// Collision system type (BULLET or MULTICORE)
auto collsys_type = ChCollisionSystem::Type::BULLET;



// Initial vehicle position
ChVector3d initLoc(0, 0, 0.8);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Simulation step size
double step_size = 5e-4;

// Time interval between two render frames
double render_step_size = 1.0 / 60;
double output_step_size = 1.0 / 100;

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 0.0);

// =============================================================================
int main(int argc, char* argv[]) {
    // --------------------------
    // Construct the M113 vehicle
    // --------------------------
    TrackedVehicle m113(vehicle::GetDataFile("M113/vehicle/M113_Vehicle_SinglePin.json"), ChContactMethod::SMC);

    if (collsys_type == ChCollisionSystem::Type::MULTICORE) {
        auto cd_chrono = chrono_types::make_shared<ChCollisionSystemMulticore>();
        cd_chrono->SetBroadphaseGridResolution(ChVector3<int>(1000, 20, 1));
        m113.GetSystem()->SetCollisionSystem(cd_chrono);
    }
    m113.GetSystem()->SetNumThreads(1, 8, 1);

    auto engine = ReadEngineJSON(vehicle::GetDataFile("M113/powertrain/M113_EngineSimpleMap.json"));
    auto transmission =
        ReadTransmissionJSON(vehicle::GetDataFile("M113/powertrain/M113_AutomaticTransmissionSimpleMap.json"));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    m113.InitializePowertrain(powertrain);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    m113.Initialize(ChCoordsys<>(initLoc, initRot));

    // Set visualization type for vehicle components.
    m113.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // ------------------
    // Create the terrain
    // ------------------

    double terrainLength = 50;
    RigidTerrain terrain(m113.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);

    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, 20);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 20, 20);

    // Add the mesh
    patch = terrain.AddPatch(patch_mat,
                             chrono::ChCoordsys<>(chrono::ChVector3d(terrainLength / 2.0 - 0.05, 0, 0), chrono::QUNIT),
                             vehicle::GetDataFile(terrainFile), false, 0.005);
    patch->SetColor(chrono::ChColor(0.2f, 0.2f, 0.5f));

    terrain.Initialize();

    // Driver
    auto path = StraightLinePath(ChVector3d(0, 0, 0), ChVector3d(300, 0, 0));
    ChPathFollowerDriver driver(m113, path, "my_path", 10.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    std::shared_ptr<ChTrackedVehicleVisualSystemIrrlicht> vis;
    if (useIrrlicht) {
        vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
        vis->SetWindowTitle("M113 Vehicle Demo");
        vis->SetChaseCamera(trackPoint, 6.0, 0.5);
        vis->SetChaseCameraMultipliers(1e-4, 10);
        vis->Initialize();
        vis->AddTypicalLights();
        vis->AddSkyBox();
        vis->AddLogo();
        vis->AttachVehicle(&m113);
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

    utils::ChWriterCSV csv(" ");

    while (m113.GetPos().x() < 300) {
        if (useIrrlicht) {
            if (!vis->Run())
                break;
            if (step_number % render_steps == 0) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
            }
        }

        // Collect output data from modules
        DriverInputs driver_inputs = driver.GetInputs();
        m113.GetTrackShoeStates(LEFT, shoe_states_left);
        m113.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = m113.GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        if (useIrrlicht) {
            vis->Synchronize(time, driver_inputs);
        }
        
        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
        if (useIrrlicht) {
            vis->Advance(step_size);
        }

        if (step_number % output_steps == 0) {
            csv << time << step_number;
            csv << m113.GetPos().x() << m113.GetSpeed();
            csv << "    ";
            csv << m113.GetSystem()->GetNumContacts();
            csv << "    ";
            csv << m113.GetSystem()->GetTimerStep()
                << m113.GetSystem()->GetTimerCollisionBroad()
                << m113.GetSystem()->GetTimerCollisionNarrow() << m113.GetSystem()->GetTimerCollision() << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    csv.WriteToFile("timing.out");

    return 0;
}
