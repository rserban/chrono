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
// Authors: Radu Serban
// =============================================================================
//
// Demonstration program for M113 vehicle on rigid terrain.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector3d initLoc(-40, 0, 0.8);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double sizeX = 100.0;  // size in X direction
double sizeY = 40.0;   // size in Y direction

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector3d trackPoint(-3.5, 0.0, 0.0);

// Output
const std::string out_dir = GetChronoOutputPath() + "M113";
bool dbg_output = false;

// =============================================================================
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------------------
    // Construct the Chrono::Multicore system
    // --------------------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;

    double tolerance = 0.01;

    int max_iteration_bilateral = 1000;  // 1000;
    int max_iteration_normal = 0;
    int max_iteration_sliding = 200;  // 2000;
    int max_iteration_spinning = 0;

    float contact_recovery_speed = -1;

    int threads = 8;

    std::shared_ptr<ChSystemMulticore> system;
    switch (contact_method) {
        case ChContactMethod::SMC: {
            auto sysSMC = chrono_types::make_shared<ChSystemMulticoreSMC>();

            sysSMC->GetSettings()->solver.contact_force_model = ChSystemSMC::PlainCoulomb;

            system = sysSMC;
            break;
        }
        case ChContactMethod::NSC: {
            auto sysNSC = chrono_types::make_shared<ChSystemMulticoreNSC>();

            sysNSC->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            sysNSC->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
            sysNSC->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
            sysNSC->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
            sysNSC->GetSettings()->solver.alpha = 0;
            sysNSC->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
            sysNSC->ChangeSolverType(SolverType::APGD);
            sysNSC->GetSettings()->collision.collision_envelope = 0.01;

            system = sysNSC;
            break;
        }
    }

    system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Set number of threads
    system->SetNumThreads(threads);

    // Set solver parameters
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = tolerance;

    system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    bool fix_chassis = false;
    bool create_track = true;

    CollisionType chassis_collision_type = CollisionType::NONE;
    TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
    BrakeType brake_type = BrakeType::SIMPLE;
    DrivelineTypeTV driveline_type = DrivelineTypeTV::BDS;
    EngineModelType engine_type = EngineModelType::SHAFTS;
    TransmissionModelType transmission_type = TransmissionModelType::AUTOMATIC_SHAFTS;

    //// TODO
    if (shoe_type == TrackShoeType::DOUBLE_PIN)
        contact_method = ChContactMethod::NSC;

    M113 m113(system.get());
    m113.SetTrackShoeType(shoe_type);
    m113.SetDrivelineType(driveline_type);
    m113.SetBrakeType(brake_type);
    m113.SetEngineType(engine_type);
    m113.SetTransmissionType(transmission_type);
    m113.SetChassisCollisionType(chassis_collision_type);

    m113.SetChassisFixed(fix_chassis);
    m113.CreateTrack(create_track);

    // Replace collision shapes from cylindrical to cylshell for road-wheels and/or idlers.
    ////m113.SetWheelCollisionType(LEFT, false, false);
    ////m113.SetWheelCollisionType(RIGHT, false, false);

    // Disable gravity in this simulation
    ////m113.GetSystem()->SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Control steering type (enable crossdrive capability)
    ////m113.GetDriveline()->SetGyrationMode(true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();

    // Set visualization type for vehicle components.
    VisualizationType track_vis =
        (shoe_type == TrackShoeType::SINGLE_PIN) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    m113.SetChassisVisualizationType(VisualizationType::NONE);
    m113.SetSprocketVisualizationType(track_vis);
    m113.SetIdlerVisualizationType(track_vis);
    m113.SetSuspensionVisualizationType(track_vis);
    m113.SetRoadWheelVisualizationType(track_vis);
    m113.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////m113.GetVehicle().EnableCollision(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////m113.GetVehicle().EnableCollision(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    m113.GetVehicle().SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////m113.GetVehicle().SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Monitor contacts involving one of the sprockets.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Monitor only contacts involving the left idler.
    m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::IDLER_LEFT);
    
    // Monitor only contacts involving one of the left track shoes.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::SHOES_LEFT);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////m113.GetVehicle().SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(m113.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, sizeX, sizeY);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), sizeX, sizeY);
    terrain.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("M113 Vehicle Demo");
    vis->SetChaseCamera(trackPoint, 3.0, 0.0);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&m113.GetVehicle());

    // ------------------------
    // Create the driver system
    // ------------------------

    ////ChInteractiveDriverIRR driver(app);
    ////double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
    ////double throttle_time = 1.0;  // time to go from 0 to +1
    ////double braking_time = 0.3;   // time to go from 0 to +1
    ////driver.SetSteeringDelta(render_step_size / steering_time);
    ////driver.SetThrottleDelta(render_step_size / throttle_time);
    ////driver.SetBrakingDelta(render_step_size / braking_time);
    ////driver.SetGains(2, 5, 5);

    std::vector<ChDataDriver::Entry> driver_data;
    driver_data.push_back({0.0, 0, 0.0, 0, 0});
    driver_data.push_back({0.5, 0, 0.0, 0, 0});
    driver_data.push_back({2.0, 0, 0.5, 0, 0});
    driver_data.push_back({3.0, 0, 0.8, 0, 0});
    driver_data.push_back({4.0, 0, 1.0, 0, 0});
    ChDataDriver driver(m113.GetVehicle(), driver_data);

    driver.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Set up vehicle output
    ////m113.GetVehicle().SetChassisOutput(true);
    ////m113.GetVehicle().SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    ////m113.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    ////m113.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(m113.GetVehicle().GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(m113.GetVehicle().GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(m113.GetVehicle().GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(m113.GetVehicle().GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;

    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = m113.GetVehicle().GetTrackAssembly(LEFT);
            auto track_R = m113.GetVehicle().GetTrackAssembly(RIGHT);
            cout << "Time: " << m113.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << m113.GetSystem()->GetNumContacts() << endl;
            const ChFrameMoving<>& c_ref = m113.GetChassisBody()->GetFrameRefToAbs();
            const ChVector3d& c_pos = m113.GetVehicle().GetPos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector3d& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                const ChVector3d& s_omg_rel = track_L->GetSprocket()->GetGearBody()->GetAngVelLocal();
                auto s_appl_trq = track_L->GetSprocket()->GetAxle()->GetAppliedLoad();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
                cout << "      L sprocket omg: " << s_omg_rel << endl;
                cout << "      L sprocket trq: " << s_appl_trq << endl;
            }
            {
                const ChVector3d& i_pos_abs = track_R->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = track_R->GetSprocket()->GetGearBody()->GetPos();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):";
            for (size_t i = 0; i < track_L->GetNumTrackSuspensions(); i++) {
                cout << " " << track_L->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):";
            for (size_t i = 0; i < track_R->GetNumTrackSuspensions(); i++) {
                cout << " " << track_R->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
        }

        if (step_number % render_steps == 0) {
            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Collect output data from modules
        DriverInputs driver_inputs = driver.GetInputs();
        m113.GetVehicle().GetTrackShoeStates(LEFT, shoe_states_left);
        m113.GetVehicle().GetTrackShoeStates(RIGHT, shoe_states_right);

        double t = m113.GetSystem()->GetChTime();
        if (fix_chassis) {
            if (t > 4) {
                std::cout << "Sprocket RPM at t=  " << t << "  ";
                std::cout << m113.GetDriveline()->GetSprocketSpeed(LEFT) * (30 / CH_PI) << "  ";
                std::cout << m113.GetDriveline()->GetSprocketSpeed(RIGHT) * (30 / CH_PI) << std::endl;
                break;
            }
        } else {
            ////std::cout << t << "  " << m113.GetVehicle().GetSpeed() << std::endl;

            ////auto xdir = m113.GetChassisBody()->GetRot().GetXaxis();
            ////std::cout << xdir << std::endl;

            if (t > 12) {
                std::cout << "SPEED at t= " << t << "  " << m113.GetVehicle().GetSpeed() << std::endl;     //// <=== ??????
                break;
            }
        }


        // Update modules (process inputs from other modules)
        double time = m113.GetVehicle().GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize(time, driver_inputs);

        // Apply resistive torque at sprocket
        if (fix_chassis && !create_track) {
            double M = m113.GetVehicle().GetMass();                            // kg
            double G = M * 9.81 / 2.0;                                                // N
            double R = 0.23;                                                          // m
            auto track_L = m113.GetVehicle().GetTrackAssembly(LEFT);                  //
            auto track_R = m113.GetVehicle().GetTrackAssembly(RIGHT);                 //
            double omg_L = track_L->GetSprocket()->GetGearBody()->GetAngVelLocal().y();  // rad/s
            double omg_R = track_R->GetSprocket()->GetGearBody()->GetAngVelLocal().y();  // rad/s
            double v_L = R * omg_L * 3.6;                                             // km/h
            double v_R = R * omg_R * 3.6;                                             // km/h
            double fr_L = 0.03 + 0.0009 * v_L;                                        //
            double fr_R = 0.03 + 0.0009 * v_R;                                        //
            double T_L = R * G * fr_L;                                                // N.m
            double T_R = R * G * fr_R;                                                // N.m
            ////std::cout << "     resistive torque L: " << T_L << std::endl;
            ////std::cout << "     resistive torque R: " << T_R << std::endl;
            if (omg_L > 0)
                track_L->GetSprocket()->ApplyAxleTorque(-T_L);
            if (omg_R > 0)
                track_R->GetSprocket()->ApplyAxleTorque(-T_R);
        }

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
        system->DoStepDynamics(step_size);
        vis->Advance(step_size);

        ////std::cout << m113.GetVehicle().GetSprocketResistiveTorque(LEFT) << std::endl;

        // Report if the chassis experienced a collision
        if (m113.GetVehicle().IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    m113.GetVehicle().WriteContacts(out_dir + "/M113_contacts.out");

    return 0;
}
