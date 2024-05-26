// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// FED alpha NATO double lane change test
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChTimer.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/feda/FEDA.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

typedef enum { ISO3888_1, ISO3888_2, NATO } DLC_Variant;

// =============================================================================
// ISO 3888 helper class
class ISO3888_Helper {
  public:
    ISO3888_Helper(double xmin, double acc_length, double vehicle_width, DLC_Variant variant, bool left_turn = true)
        : xmin(xmin), acc_length(acc_length), left_turn(left_turn), vehicle_width(vehicle_width) {
        switch (variant) {
            default:
            case ISO3888_1:
                // gate A
                lengthA = 15.0;
                widthA = 1.1 * vehicle_width + 0.25;
                // gate B
                lengthAB = 30.0;
                lengthB = 25.0;
                widthB = 1.2 * vehicle_width + 0.25;
                if (left_turn) {
                    ofsB = widthB / 2.0 + 3.5;
                } else {
                    ofsB = -(widthB / 2.0 + 3.5);
                }
                // gate C
                lengthBC = 25.0;
                lengthC = 15.0;
                widthC = 1.3 * vehicle_width + 0.25;
                if (left_turn) {
                    ofsC = (widthC - widthA) / 2;
                } else {
                    ofsC = (widthA - widthC) / 2;
                }
                std::cout << "Section 1 Length  = " << lengthA << " m   Width = " << widthA << " m\n";
                std::cout << "Section 2 Length  = " << lengthAB << " m\n";
                std::cout << "Section 3 Length  = " << lengthB << " m   Width = " << widthB << " m\n";
                std::cout << "Section 4 Length  = " << lengthBC << " m\n";
                std::cout << "Section 5 Length  = " << lengthC << " m   Width = " << widthC << " m\n";
                break;
            case ISO3888_2:
                // gate A
                lengthA = 12.0;
                widthA = 1.1 * vehicle_width + 0.25;
                // gate B
                lengthAB = 13.5;
                lengthB = 11.0;
                widthB = vehicle_width + 1.0;
                if (left_turn) {
                    ofsB = widthB / 2.0 + widthA / 2.0 + 1.0;
                } else {
                    ofsB = -(widthB / 2.0 + widthA / 2.0 + 1.0);
                }
                // gate C
                lengthBC = 12.5;
                lengthC = 12.0;
                widthC = 3.0;
                if (left_turn) {
                    ofsC = (widthC - widthA) / 2;
                } else {
                    ofsC = (widthA - widthC) / 2;
                }
                break;
            case NATO:
                // we want to use exactly the values from the FEDA tests, for general use a NATO formula
                // based calulation should be used
                // gate A
                lengthA = 15.0;
                widthA = 2.8;
                // gate B
                lengthAB = 28.6;
                lengthB = 25.0;
                widthB = 3.0;
                if (left_turn) {
                    ofsB = widthB / 2.0 + widthA / 2.0 + 1.0;
                } else {
                    ofsB = -(widthB / 2.0 + widthA / 2.0 + 1.0);
                }
                // gate C
                lengthBC = 28.6;
                lengthC = 15.0;
                widthC = 2.8;
                if (left_turn) {
                    ofsC = (widthC - widthA) / 2;
                } else {
                    ofsC = (widthA - widthC) / 2;
                }
                std::cout << "Section 1 Length  = " << lengthA << " m   Width = " << widthA << " m\n";
                std::cout << "Section 2 Length  = " << lengthAB << " m\n";
                std::cout << "Section 3 Length  = " << lengthB << " m   Width = " << widthB << " m\n";
                std::cout << "Section 4 Length  = " << lengthBC << " m\n";
                std::cout << "Section 5 Length  = " << lengthC << " m   Width = " << widthC << " m\n";
                break;
        }
        //================== setup definition points ========================================
        double zl = 0.1;
        // P1
        leftLine.push_back(ChVector3d(xmin + acc_length, widthA / 2.0, 0));
        centerLine.push_back(ChVector3d(xmin, 0, 0));
        rightLine.push_back(ChVector3d(xmin + acc_length, -widthA / 2.0, 0));
        // P2
        leftLine.push_back(ChVector3d(xmin + acc_length + lengthA, widthA / 2.0, 0));
        centerLine.push_back(ChVector3d(xmin + acc_length + lengthA, 0, zl));
        rightLine.push_back(ChVector3d(xmin + acc_length + lengthA, -widthA / 2.0, 0));
        // P3
        leftLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB, widthB / 2.0, 0));
        centerLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB, 0, zl));
        rightLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB, -widthB / 2.0, 0));
        leftLine.back().y() += ofsB;
        centerLine.back().y() += ofsB;
        rightLine.back().y() += ofsB;
        // P4
        leftLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB, widthB / 2.0, 0));
        centerLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB, 0, zl));
        rightLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB, -widthB / 2.0, 0));
        leftLine.back().y() += ofsB;
        centerLine.back().y() += ofsB;
        rightLine.back().y() += ofsB;
        // P5
        leftLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB + lengthBC, widthC / 2.0, 0));
        centerLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB + lengthBC, 0, zl));
        rightLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB + lengthBC, -widthC / 2.0, 0));
        leftLine.back().y() += ofsC;
        centerLine.back().y() += ofsC;
        rightLine.back().y() += ofsC;
        // P6
        leftLine.push_back(
            ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB + lengthBC + lengthC, widthC / 2.0, 0));
        centerLine.push_back(ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB + lengthBC + lengthC, 0, zl));
        rightLine.push_back(
            ChVector3d(xmin + acc_length + lengthA + lengthAB + lengthB + lengthBC + lengthC, -widthC / 2.0, 0));
        leftLine.back().y() += ofsC;
        centerLine.back().x() += 100.0;
        centerLine.back().y() += ofsC;
        rightLine.back().y() += ofsC;
        // luxury: add some road cone positions like in the standard
        leftCones.push_back(leftLine[0]);
        leftCones.push_back((leftLine[0] + leftLine[1]) / 2);
        leftCones.push_back(leftLine[1]);
        leftCones.push_back(leftLine[2]);
        leftCones.push_back((leftLine[2] + leftLine[3]) / 2);
        leftCones.push_back(leftLine[3]);
        leftCones.push_back(leftLine[4]);
        leftCones.push_back((leftLine[4] + leftLine[5]) / 2);
        leftCones.push_back(leftLine[5]);

        rightCones.push_back(rightLine[0]);
        rightCones.push_back((rightLine[0] + rightLine[1]) / 2);
        rightCones.push_back(rightLine[1]);
        rightCones.push_back(rightLine[2]);
        rightCones.push_back((rightLine[2] + rightLine[3]) / 2);
        rightCones.push_back(rightLine[3]);
        rightCones.push_back(rightLine[4]);
        rightCones.push_back((rightLine[4] + rightLine[5]) / 2);
        rightCones.push_back(rightLine[5]);

        std::ofstream tst("bla.txt");
        for (int i = 0; i < leftLine.size(); i++) {
            tst << leftLine[i].x() << "\t" << centerLine[i].x() << "\t" << leftLine[i].y() << "\t" << centerLine[i].y()
                << "\t" << rightLine[i].y() << std::endl;
        }
        tst.close();
        // prepare path spline definition
        ChVector3d offset(lengthB / 3, 0, 0);
        for (size_t i = 0; i < centerLine.size(); i++) {
            inCV.push_back(centerLine[i] - offset);
            outCV.push_back(centerLine[i] + offset);
        }
        path = chrono_types::make_shared<ChBezierCurve>(centerLine, inCV, outCV);
    }

    bool GateTestLeft(ChVector3d& p) {
        if (p.x() >= leftLine[0].x() && p.x() <= leftLine[1].x()) {
            if (p.y() > leftLine[0].y())
                return false;
        }
        if (p.x() >= leftLine[2].x() && p.x() <= leftLine[3].x()) {
            if (p.y() > leftLine[2].y())
                return false;
        }
        if (p.x() >= leftLine[4].x() && p.x() <= leftLine[5].x()) {
            if (p.y() > leftLine[4].y())
                return false;
        }
        return true;
    }

    bool GateTestRight(ChVector3d& p) {
        if (p.x() >= leftLine[0].x() && p.x() <= leftLine[1].x()) {
            if (p.y() < rightLine[0].y())
                return false;
        }
        if (p.x() >= leftLine[2].x() && p.x() <= leftLine[3].x()) {
            if (p.y() < rightLine[2].y())
                return false;
        }
        if (p.x() >= leftLine[4].x() && p.x() <= leftLine[5].x()) {
            if (p.y() < rightLine[4].y())
                return false;
        }
        return true;
    }

    size_t GetConePositions() { return leftCones.size(); }
    ChVector3d& GetConePosition(size_t idx, bool left) {
        if (left)
            return leftCones[idx];
        else
            return rightCones[idx];
    }

    ~ISO3888_Helper() {}

    double GetManeuverLength() { return leftLine[5].x() - leftLine[0].x(); }
    double GetXmax() { return leftLine[5].x(); }
    std::shared_ptr<ChBezierCurve> GetPath() { return path; }

  private:
    std::vector<ChVector3d> leftLine;
    std::vector<ChVector3d> centerLine;
    std::vector<ChVector3d> rightLine;
    std::vector<ChVector3d> leftCones;
    std::vector<ChVector3d> rightCones;
    double widthA;
    double lengthA;
    double widthB;
    double lengthB;
    double widthC;
    double lengthC;
    double lengthAB;
    double lengthBC;
    double ofsB;
    double ofsC;
    bool left_turn;
    double vehicle_width;
    double xmin;
    double xmax;
    double acc_length;
    std::vector<ChVector3d> inCV;
    std::vector<ChVector3d> outCV;
    std::shared_ptr<ChBezierCurve> path;
};
// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Type of powertrain model (SHAFTS, SIMPLE_MAP)
EngineModelType engine_model = EngineModelType::SHAFTS;
TransmissionModelType transmision_model = TransmissionModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89, PAC02, TMEASY)
TireModelType tire_model = TireModelType::PAC02;

// Terrain length (X direction)
double terrainLength = 350.0;
double accelerationLength = 250.0;
double terrainWidth = 16.0;

// Simulation step sizes
double step_size = 2e-4;
double tire_step_size = 2e-4;

const double mph2kmh = 1.609344;
// =============================================================================
// call variants:
//                          1) demo_VEH_HMMWV_DoubleLaneChange
//                              -> standard demo velkmh= 30km/h, NATO, left turn
//
//  on the Mac:
//  instead of 'demo_VEH_HMMWV_DoubleLaneChange' use 'run_app.sh demo_VEH_HMMWV_DoubleLaneChange.app'

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    std::string wtitle;
    DLC_Variant dlc_mode = NATO;
    int velkmh = 32;
    int velmph = 20;
    bool left_turn = true;
    switch (argc) {
        case 1:
        default:
            // no parameters given
            if (left_turn) {
                wtitle = "NATO Double Lane Change Test v = " + std::to_string(velmph) + " mph - Left Turn";

            } else {
                wtitle = "NATO Double Lane Change Test v = " + std::to_string(velmph) + " mph - Right Turn ";
            }
            break;
        case 2:
            // one parameter given (velmph!)
            velkmh = ChClamp(atoi(argv[1]), 5, 40) * mph2kmh;
            velmph = ChClamp(atoi(argv[1]), 5, 40);
            if (left_turn) {
                wtitle = "NATO Double Lane Change Test v = " + std::to_string(velmph) + " mph - Left Turn";
            } else {
                wtitle = "NATO Double Lane Change Test v = " + std::to_string(velmph) + " mph - Right Turn";
            }
            break;
    }
    double target_speed = velkmh / 3.6;
    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for HMMWV: Cd = 0.5 and area ~5 m2
    FEDA feda;
    feda.SetContactMethod(ChContactMethod::SMC);
    feda.SetChassisFixed(false);
    feda.SetInitPosition(ChCoordsys<>(ChVector3d(-terrainLength / 2 + 5, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    feda.SetTireType(tire_model);
    feda.SetTireStepSize(tire_step_size);
    feda.SetAerodynamicDrag(0.5, 5.0, 1.2);
    feda.Initialize();

    // important vehicle data
    double wheel_base = feda.GetVehicle().GetWheelbase();
    double vehicle_width = 2.28;
    double steering_gear_ratio = 540.0;  // Ricardo data

    // Set subsystem visualization mode
    VisualizationType tire_vis_type =
        (tire_model == TireModelType::RIGID_MESH) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    feda.SetChassisVisualizationType(chassis_vis_type);
    feda.SetSuspensionVisualizationType(suspension_vis_type);
    feda.SetSteeringVisualizationType(steering_vis_type);
    feda.SetWheelVisualizationType(wheel_vis_type);
    feda.SetTireVisualizationType(tire_vis_type);

    ISO3888_Helper helper(-accelerationLength + 5.0, accelerationLength, vehicle_width, dlc_mode, left_turn);
    ////std::cout << "Maneuver Length = " << helper.GetManeuverLength() << " m\n";

    // Create the terrain
    auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);
    RigidTerrain terrain(feda.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), (float)terrainLength, (float)terrainWidth);
    terrain.Initialize();

    // ---------------------------------------------------
    // Create the lane change path and the driver system
    // ---------------------------------------------------

    auto path = helper.GetPath();
    ChPathFollowerDriver driver(feda.GetVehicle(), path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle(wtitle);
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->SetHUDLocation(500, 20);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&feda.GetVehicle());

    std::string pltName("../FEDA_LaneChange_" + std::to_string(velmph) + "mph.plt");
    std::ofstream plt(pltName);
    plt << "$limits << EOD" << std::endl;

    // now we can add the road cones
    double coneBaseWidth = 0.415;
    for (size_t i = 0; i < helper.GetConePositions(); i++) {
        ChVector3d pL = helper.GetConePosition(i, true) + ChVector3d(0, coneBaseWidth / 2, 0);
        irr::scene::IAnimatedMesh* mesh_coneL =
           vis->GetSceneManager()->getMesh(GetChronoDataFile("models/traffic_cone/trafficCone750mm.obj").c_str());
        irr::scene::IAnimatedMeshSceneNode* node_coneL = vis->GetSceneManager()->addAnimatedMeshSceneNode(mesh_coneL);
        node_coneL->getMaterial(0).EmissiveColor = irr::video::SColor(0, 100, 0, 0);
        node_coneL->setPosition(irr::core::vector3dfCH(pL));
        plt << pL.x() << "\t" << pL.y();

        ChVector3d pR = helper.GetConePosition(i, false) + ChVector3d(0, -coneBaseWidth / 2, 0);
        irr::scene::IAnimatedMesh* mesh_coneR =
            vis->GetSceneManager()->getMesh(GetChronoDataFile("models/traffic_cone/trafficCone750mm.obj").c_str());
        irr::scene::IAnimatedMeshSceneNode* node_coneR = vis->GetSceneManager()->addAnimatedMeshSceneNode(mesh_coneR);
        node_coneR->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 100, 0);
        node_coneR->setPosition(irr::core::vector3dfCH(pR));
        plt << "\t" << pR.y() << std::endl;
    }

    plt << "EOD" << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);

    // Running average of vehicle lateral acceleration
    utils::ChButterworthLowpass accel_filter(4, step_size, 2.0);

    // Running average of vehicle steering wheel angle
    utils::ChButterworthLowpass steer_filter(4, step_size, 2.0);

    // Differentiate steering signal
    utils::ChFilterD ang_diff(step_size);

    // Record vehicle speed
    ChFunctionInterp speed_recorder;

    // Record lateral vehicle acceleration
    ChFunctionInterp accel_recorder;

    // Record lateral vehicle steering wheel angle
    ChFunctionInterp steer_recorder;

    // Record lateral vehicle steering wheel angular speed
    ChFunctionInterp angspeed_recorder;

    plt << "$data << EOD" << std::endl;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    bool done = false;
    double xpos = 0;
    double xend = 30.0 + helper.GetXmax();
    while (vis->Run()) {
        time = feda.GetSystem()->GetChTime();
        double speed = speed_filter.Add(feda.GetVehicle().GetSpeed());
        double accel =
            accel_filter.Filter(feda.GetVehicle().GetPointAcceleration(ChVector3d(-wheel_base / 2, 0, 0)).y());

        speed_recorder.AddPoint(time, speed);
        accel_recorder.AddPoint(time, accel);
        xpos = feda.GetVehicle().GetPos().x();
        double ypos = feda.GetVehicle().GetPos().y();
        ChVector3d pFrontLeft = feda.GetVehicle().GetPointLocation(ChVector3d(0, vehicle_width / 2, 1));
        ChVector3d pRearLeft = feda.GetVehicle().GetPointLocation(ChVector3d(-wheel_base, vehicle_width / 2, 1));
        ChVector3d pFrontRight = feda.GetVehicle().GetPointLocation(ChVector3d(0, -vehicle_width / 2, 1));
        ChVector3d pRearRight =
            feda.GetVehicle().GetPointLocation(ChVector3d(-wheel_base, -vehicle_width / 2, 1));
        if (!helper.GateTestLeft(pFrontLeft)) {
            std::cout << "Test Failure: vehicle left the course with the front left wheel.\n";
            break;
        }
        if (!helper.GateTestLeft(pRearLeft)) {
            std::cout << "Test Failure: vehicle left the course with the rear left wheel.\n";
            break;
        }
        if (!helper.GateTestRight(pFrontRight)) {
            std::cout << "Test Failure: vehicle left the course with the front right wheel.\n";
            break;
        }
        if (!helper.GateTestRight(pRearRight)) {
            std::cout << "Test Failure: vehicle left the course with the rear right wheel.\n";
            break;
        }
        // End simulation
        if (time >= 100 || xpos > xend) {
            std::cout << "Test Run: terminated normally.\n";
            break;
        }

        vis->BeginScene();
        vis->Render();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        double steer = steering_gear_ratio * steer_filter.Filter(driver_inputs.m_steering);
        steer_recorder.AddPoint(time, steer);
        double angspeed = ang_diff.Filter(steer);
        angspeed_recorder.AddPoint(time, angspeed);
        ChQuaternion<> q = feda.GetVehicle().GetRot();
        double rollangle = q.Q_to_Euler123().x() * CH_RAD_TO_DEG;
        double yawrate = feda.GetVehicle().GetChassisBody()->GetAngVelLocal().z() * CH_RAD_TO_DEG;
        double latacc = accel_recorder.GetVal(time) / 9.81;
        if (xpos >= -50.0 && step_number % 500 == 0) {
            plt << time << "\t" << xpos << "\t" << ypos << "\t" << steer << "\t" << angspeed << "\t" << rollangle
                << "\t" << yawrate << "\t" << latacc << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        feda.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        feda.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        vis->EndScene();
    }
    plt << "EOD" << std::endl;
    plt << "set title 'FEDA Double Lane Change V = " << velmph << " mph'" << std::endl;
    plt << "set xlabel 'X (m)'" << std::endl;
    plt << "set ylabel 'Y (m)'" << std::endl;
    plt << "plot $limits u 1:2 t 'Left Limit' with lines, $limits u 1:3 t 'Right Limit' with lines, $data u 2:3 t "
           "'Vehicle Trace' "
           "with lines"
        << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set xlabel 'Time (s)'" << std::endl;
    plt << "set ylabel 'Steering Wheel Angle (deg)'" << std::endl;
    plt << "plot $data u 1:4 t 'Chrono Simulation' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set xlabel 'Time (s)'" << std::endl;
    plt << "set ylabel 'Steering Wheel Anglular Speed (deg/s)'" << std::endl;
    plt << "plot $data u 1:5 t 'Chrono Simulation' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set xlabel 'Time (s)'" << std::endl;
    plt << "set ylabel 'Vehicle Roll Angle (deg)'" << std::endl;
    plt << "plot $data u 1:6 t 'Chrono Simulation' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set xlabel 'Time (s)'" << std::endl;
    plt << "set ylabel 'Vehicle Yaw Rate (deg/s)'" << std::endl;
    plt << "plot $data u 1:7 t 'Chrono Simulation' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set xlabel 'Time (s)'" << std::endl;
    plt << "set ylabel 'Vehicle Latral Acceleration (g)'" << std::endl;
    plt << "plot $data u 1:8 t 'Chrono Simulation' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt.close();
    return 0;
}
