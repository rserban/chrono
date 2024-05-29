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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Sample test program for FEDA steady state cornering simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include <chrono>
#include <thread>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/feda/FEDA.h"
#include "chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, PAC02)
TireModelType tire_model = TireModelType::PAC02;

// Type of steering model (PITMAN_ARM or PITMAN_ARM_SHAFTS)
SteeringTypeWV steering_model = SteeringTypeWV::PITMAN_ARM;

// Point on chassis tracked by the camera
ChVector3d trackPoint(1.0, 0.0, 1.75);

// output directory
const std::string out_dir = "../FEDA_CONST_TURN";

// Simulation step sizes
double step_size = 2e-4;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// radius of the turn circle
const double turn_radius = 30.0;

// =============================================================================

int main(int argc, char* argv[]) {
    // --------------
    // Create systems
    // --------------
    const double steering_gear_ratio = 540.0;
    bool turn_left = true;
    const double ay_max = 0.5 * 9.81;  // cannot be reached by this model due to flying inner front wheel
    if (argc == 2 && argv[1][0] == 'r')
        turn_left = false;
    double target_speed = 5;
    // Parameter for aerodynamic force
    double Cd = 0.6;
    double area = 3.8;
    double air_density = 1.2041;
    // Create the vehicle, set parameters, and initialize
    FEDA feda;
    feda.SetChassisFixed(false);
    feda.SetContactMethod(ChContactMethod::NSC);
    feda.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    feda.SetTireType(tire_model);
    feda.SetTireStepSize(tire_step_size);
    feda.SetAerodynamicDrag(Cd, area, air_density);
    feda.SetRideHeight_OnRoad();
    // feda.SetRideHeight_ObstacleCrossing();
    // feda.SetRideHeight_Low();

    // feda.setSteeringType(steering_model);
    feda.SetInitFwdVel(0.0);
    feda.Initialize();

    feda.SetChassisVisualizationType(chassis_vis_type);
    feda.SetSuspensionVisualizationType(suspension_vis_type);
    feda.SetSteeringVisualizationType(steering_vis_type);
    feda.SetWheelVisualizationType(wheel_vis_type);
    feda.SetTireVisualizationType(tire_vis_type);

    {
        auto suspF = std::static_pointer_cast<FEDA_DoubleWishboneFront>(feda.GetVehicle().GetSuspension(0));
        auto springFL = suspF->GetSpring(VehicleSide::LEFT);
        auto shockFL = suspF->GetShock(VehicleSide::RIGHT);

        // std::cout << "Spring rest length front: " << springFL->GetSpringRestLength() << std::endl;
        // std::cout << "Shock rest length front:  " << shockFL->GetSpringRestLength() << std::endl;
    }
    {
        auto suspR = std::static_pointer_cast<FEDA_DoubleWishboneRear>(feda.GetVehicle().GetSuspension(1));
        auto springRL = suspR->GetSpring(VehicleSide::LEFT);
        auto shockRL = suspR->GetShock(VehicleSide::RIGHT);

        // std::cout << "Spring rest length rear: " << springRL->GetSpringRestLength() << std::endl;
        // std::cout << "Shock rest length rear:  " << shockRL->GetSpringRestLength() << std::endl;
    }

    std::cout << "Vehicle mass:               " << feda.GetVehicle().GetMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.8f);
    patch_mat->SetRestitution(0.01f);
    RigidTerrain terrain(feda.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

    // auto path = StraightLinePath(ChVector3d(-42 / 2, 0, 0.5), ChVector3d(42 / 2, 0, 0.5), 1);
    auto path = CirclePath(ChVector3d(0.0, 0, 0.5), turn_radius, 30.0, turn_left, 15);

    // Create the driver system
    ChPathFollowerDriver driver(feda.GetVehicle(), path, "circle", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0.05, 0);
    driver.GetSpeedController().SetGains(0.4, 0.04, 0);
    driver.Initialize();

    std::string winTitle("FED Alpha steady state cornering test");
    if (turn_left) {
        winTitle.append(" (left turn)");
    } else {
        winTitle.append(" (right turn)");
    }

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle(winTitle);
    vis->SetChaseCamera(trackPoint, 8.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&feda.GetVehicle());

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

    driver.Initialize();

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

#ifdef CHRONO_POSTPROCESS
    utils::ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);
    csv << "time"
        << "steering"
        << "speed"
        << "ay"
        << "roll"
        << "yawrate" << std::endl;
#endif

    // ---------------
    // Simulation loop
    // ---------------

    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;

    double T1 = 25;
    double T2 = T1 + 120;
    double target_speed_max = 12.0;  // sqrt(turn_radius * ay_max);

    while (vis->Run()) {
        double time = feda.GetSystem()->GetChTime();

        target_speed = ChFunctionSineStep::Eval(time, T1, 5, T2, target_speed_max);
        driver.SetDesiredSpeed(target_speed);

        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        feda.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

#ifdef CHRONO_POSTPROCESS
        if ((time >= T1) && (step_number % 5000 == 0)) {
            double steering_wheel_input = steering_gear_ratio * driver_inputs.m_steering;
            double speed = feda.GetVehicle().GetSpeed();
            double ay = pow(feda.GetVehicle().GetSpeed(), 2.0) / turn_radius / 9.81;
            ChQuaternion<> q = feda.GetVehicle().GetRot();
            double roll = q.GetCardanAnglesXYZ().x() * CH_RAD_TO_DEG;
            double yawrate = feda.GetChassisBody()->GetAngVelLocal().z() * CH_RAD_TO_DEG;
            if (!turn_left)
                ay *= -1.0;
            csv << time << steering_wheel_input << speed << ay << roll << yawrate << std::endl;
        }
#endif

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector3d& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector3d& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        feda.Advance(step_size);
        vis->Advance(step_size);

        auto susp0 = std::static_pointer_cast<ChDoubleWishbone>(feda.GetVehicle().GetSuspension(0));
        auto susp1 = std::static_pointer_cast<ChDoubleWishbone>(feda.GetVehicle().GetSuspension(1));

        ////auto susp = feda.GetVehicle().GetAxle(0)->GetSuspension();
        ////auto states = (std::static_pointer_cast<FEDA_DoubleWishboneFront>(susp))->GetShock(LEFT)->GetStates();
        ////std::cout << time << "   " << states(0) << "  " << states(1) << std::endl;

        ChCoordsys<> vehCoord = ChCoordsys<>(feda.GetVehicle().GetPos(), feda.GetVehicle().GetRot());
        ChVector3d vehCOM = vehCoord.TransformPointParentToLocal(feda.GetVehicle().GetCOMFrame().GetPos());
        // std::cout << "Vehicle COM: " << vehCOM.x() << "|" << vehCOM.y() << "|" << vehCOM.z() << std::endl;
        /* std::cout<<"Vehicle
        COM|"<<feda.GetVehicle().GetCOMFrame().GetPos().x()-feda.GetVehicle().GetPos().x()<<"|"
            <<feda.GetVehicle().GetCOMFrame().GetPos().y()-feda.GetVehicle().GetPos().y()<<"|"
            <<(feda.GetVehicle().GetCOMFrame().GetPos().z()-feda.GetVehicle().GetPos().z());
        std::cout<<std::endl; */

        if (time > T2)
            break;

        // Increment frame number
        step_number++;
    }

    std::cout << "CoG of the FED alpha above ground = " << feda.GetVehicle().GetCOMFrame().GetPos().z() << " m\n";
    // Ride height test, originally the tests where made with body reference points, we don't have it
    // Ride height OnRoad is reached when the drive shafts are straight lines
    double refHeightFront =
        (feda.GetVehicle().GetWheel(0, LEFT)->GetPos().z() + feda.GetVehicle().GetWheel(0, RIGHT)->GetPos().z()) / 2.0;
    double refHeightRear =
        (feda.GetVehicle().GetWheel(1, LEFT)->GetPos().z() + feda.GetVehicle().GetWheel(1, RIGHT)->GetPos().z()) / 2.0;
    double bodyHeightFront = feda.GetVehicle().GetPointLocation(ChVector3d(0, 0, 0)).z();
    double bodyHeightRear = feda.GetVehicle().GetPointLocation(ChVector3d(-3.302, 0, 0)).z();
    double rideHeightFront = bodyHeightFront - refHeightFront;
    double rideHeightRear = bodyHeightRear - refHeightRear;
    std::cout << "Ride Height Front = " << rideHeightFront << " m, Rear = " << rideHeightRear << " m\n";

#ifdef CHRONO_POSTPROCESS
    if (turn_left)
        csv.WriteToFile(out_dir + "/feda_steady_state_cornering_left.txt");
    else
        csv.WriteToFile(out_dir + "/feda_steady_state_cornering_right.txt");
#endif

    return 0;
}
