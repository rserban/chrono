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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Test program for M113 Gap, Trapezoidal fixed barrier (up) or ditch (down)
// crossing.  Commandline inputs allow for all three types of tests to be
// created
//
// For a gap test, set the input angle to 90 deg and the height to -200 inches
//
// For a Trapezoidal barrier, set the height to +30 inches and set the top
// width and the angle with respect to ground as needed for the test
//
// For a Trapezoidal gap, set the height to -30 inches and set the bottom
// width and the angle with respect to ground as needed for the test
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChStream.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

#include "chrono_models/vehicle/m113a/M113a_Vehicle.h"
#include "chrono_models/vehicle/m113a/M113a_SimplePowertrain.h"

#include "chrono_thirdparty/filesystem/path.h"

// Uncomment the following line to unconditionally disable Irrlicht support
//#undef CHRONO_IRRLICHT
#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"
#endif

#include "../terrain/RigidTerrainTrapezoid.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================

// Initial vehicle position
ChVector<> initLoc(-0.1, 0.0, 0.75);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
// N/A - Full throttle is applied

// Chassis Corner Point Locations
ChVector<> FrontLeftCornerLoc(13.5 * 0.0254, 53.0 * 0.0254, 0.0);
ChVector<> FrontRightCornerLoc(13.5 * 0.0254, -53.0 * 0.0254, 0.0);
ChVector<> RearLeftCornerLoc(-178.2 * 0.0254, 53.0 * 0.0254, -5.9 * 0.0254);
ChVector<> RearRightCornerLoc(-178.2 * 0.0254, -53.0 * 0.0254, -5.9 * 0.0254);

// Input file names for the driver model
std::string driver_file("M113a_benchmark/Maneuver_Forward.txt");

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 100;  // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Simulation length (set to a negative value to disable for Irrlicht)
double tend = 10.0;

// Output directories (Povray only)
const std::string out_dir = "../M113_TRAPEZOIDGAPBARRIERDITCH";
const std::string pov_dir = out_dir + "/POVRAY";

// Visualization type for all vehicle subsystems (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::PRIMITIVES;

// POV-Ray output
bool povray_output = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = true;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    double height = 30.0 * 0.0254;
    double width = 6.0 * 0.0254;
    double angle = 16.0 * CH_C_PI / 180.0;

    // Check for input arguments for running this test in batch
    // First argument is the width of the top/bottom of the barrier/ditch in inches
    // Second argument is the angle of the barrier with respect to ground in degrees
    // Third argument is the height of the barrier with respect to ground in inches
    if (argc > 1)
        width = std::atof(argv[1]) * 0.0254;
    if (argc > 2)
        angle = std::atof(argv[2]) * CH_C_PI / 180.0;
    if (argc > 3)
        height = std::atof(argv[3]) * 0.0254;

    // ---------------
    // Create the M113
    // ---------------

    // Create the vehicle system
    M113a_Vehicle vehicle(false, ChContactMethod::SMC);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Set visualization type for subsystems
    vehicle.SetChassisVisualizationType(vis_type);
    vehicle.SetSprocketVisualizationType(vis_type);
    vehicle.SetIdlerVisualizationType(vis_type);
    vehicle.SetRoadWheelAssemblyVisualizationType(vis_type);
    vehicle.SetRoadWheelVisualizationType(vis_type);
    vehicle.SetTrackShoeVisualizationType(vis_type);

    // Control steering type (enable crossdrive capability).
    vehicle.GetDriveline()->SetGyrationMode(true);

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<M113a_SimplePowertrain>("Powertrain");
    vehicle.InitializePowertrain(powertrain);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->EnableWarmStart(true);
    solver->SetTolerance(1e-10);
    vehicle.GetSystem()->SetSolver(solver);

    // ------------------
    // Create the terrain
    // ------------------

    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(0.8f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);

    // Create a trapezoidal rigid terrain.
    //    X-Y dimensions of the first flat area: 10x40
    //    X-Y dimensions of the middle flat area:  widthx40
    //    X-Y dimensions of the last flat area:  100x40
    //    Upslopes and downslopes are set to the same angle
    //    so that the middle flat area is at height "height"
    RigidTerrainTrapezoid terrain(vehicle.GetSystem());
    terrain.SetColor(ChColor(0.4f, 0.2f, 0.0f));
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 12, 12);
    terrain.Initialize(patch_mat, 0, height, 0, angle, angle, 10, width, 100, 400);

    // -------------------------------------
    // Create the driver system
    // -------------------------------------

    ChDataDriver driver(vehicle, vehicle::GetDataFile(driver_file));
    driver.Initialize();

#ifdef CHRONO_IRRLICHT

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp app(&vehicle, L"M113 gap crossing, trapezoidal barrier climb, trapezoidal ditch crossing");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.AssetBindAll();
    app.AssetUpdateAll();

#endif

    // ------------------------------------
    // Prepare output directories and files
    // ------------------------------------

    state_output = state_output || povray_output;

    // Create output directories
    if (state_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage vert_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage vert_acc_driver_filter(filter_window_size);

    // Driver location in vehicle local frame
    ChVector<> driver_pos = vehicle.GetChassis()->GetLocalDriverCoordsys().pos;

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));
    ChDriver::Inputs driver_inputs = {0, 0, 0};

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    double time = 0;
    int step_number = 0;
    int render_frame = 0;
    double theta = 0;

#ifdef CHRONO_IRRLICHT

    while (app.GetDevice()->run()) {
        time = vehicle.GetChTime();

        // End simulation
        if ((time > tend) && (tend > 0))
            break;
#else

    while (time <= tend) {
        time = vehicle.GetChTime();

#endif

        // Extract accelerations to add to the filter
        ChVector<> acc_CG = vehicle.GetChassisBody()->GetPos_dtdt();
        acc_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(acc_CG);
        ChVector<> acc_driver = vehicle.GetVehiclePointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double vert_acc_CG = vert_acc_GC_filter.Add(acc_CG.z());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());
        double vert_acc_driver = vert_acc_driver_filter.Add(acc_driver.z());

        // Render scene
        if (step_number % render_steps == 0) {
#ifdef CHRONO_IRRLICHT
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
#endif

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);
            }

            if (state_output) {
                ChVector<> vel_CG = vehicle.GetChassisBody()->GetPos_dt();
                vel_CG = vehicle.GetChassisBody()->GetCoord().TransformDirectionParentToLocal(vel_CG);

                ChVector<> vel_driver_abs =
                    vehicle.GetChassisBody()->GetFrame_REF_to_abs().PointSpeedLocalToParent(driver_pos);
                ChVector<> vel_driver_local =
                    vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformDirectionParentToLocal(vel_driver_abs);

                ChVector<> FrontLeftCornerPos =
                    vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontLeftCornerLoc);
                ChVector<> FrontRightCornerPos =
                    vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(FrontRightCornerLoc);
                ChVector<> RearLeftCornerPos =
                    vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearLeftCornerLoc);
                ChVector<> RearRightCornerPos =
                    vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(RearRightCornerLoc);

                // Vehicle and Control Values
                csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
                csv << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetAxleSpeed()
                    << vehicle.GetTrackAssembly(RIGHT)->GetSprocket()->GetAxleSpeed();
                csv << powertrain->GetMotorSpeed() << powertrain->GetMotorTorque();
                // Chassis Position, Velocity, & Acceleration (Unfiltered and Filtered)
                csv << vehicle.GetChassis()->GetPos().x() << vehicle.GetChassis()->GetPos().y()
                    << vehicle.GetChassis()->GetPos().z();
                csv << vel_CG.x() << vel_CG.y() << vel_CG.z();
                csv << acc_CG.x() << acc_CG.y() << acc_CG.z();
                csv << fwd_acc_CG << lat_acc_CG << vert_acc_CG;
                // Driver Position, Velocity, & Acceleration (Unfiltered and Filtered)
                csv << vehicle.GetDriverPos().x() << vehicle.GetDriverPos().y() << vehicle.GetDriverPos().z();
                csv << vel_driver_local.x() << vel_driver_local.y() << vel_driver_local.z();
                csv << acc_driver.x() << acc_driver.y() << acc_driver.z();   // Chassis CSYS
                csv << fwd_acc_driver << lat_acc_driver << vert_acc_driver;  // filtered Chassis CSYS
                // Chassis Corner Point Positions
                csv << FrontLeftCornerPos.x() << FrontLeftCornerPos.y() << FrontLeftCornerPos.z();
                csv << FrontRightCornerPos.x() << FrontRightCornerPos.y() << FrontRightCornerPos.z();
                csv << RearLeftCornerPos.x() << RearLeftCornerPos.y() << RearLeftCornerPos.z();
                csv << RearRightCornerPos.x() << RearRightCornerPos.y() << RearRightCornerPos.z();
                csv << std::endl;
            }

            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        terrain.Synchronize(time);
#ifdef CHRONO_IRRLICHT
        app.Synchronize("Follower driver", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        app.Advance(step_size);
#endif

        // Increment frame number
        step_number++;
    }

    if (state_output) {
        char filename[100];
        sprintf(filename, "%s/output_%d_%d_%d.dat", out_dir.c_str(), int(std::round(width / 0.0254)),
                int(std::round(angle * 180 / CH_C_PI)), int(std::round(height / 0.0254)));
        csv.write_to_file(filename);
    }

    return 0;
}
