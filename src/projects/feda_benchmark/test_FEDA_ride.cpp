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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Shock performance calculation
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/feda/FEDA.h"
#include "chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

// =============================================================================
// Problem parameters

// Type of tire model (PAC02)
TireModelType tire_model = TireModelType::PAC02;

// OpenCRG input file
std::string crg_road_file = "terrain/crg_roads/krc_rms_";
// std::string crg_road_file = "terrain/crg_roads/Barber.crg";
////std::string crg_road_file = "terrain/crg_roads/Horstwalde.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_arc.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_banked.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_circle.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_sloped.crg";

// Road visualization (mesh or boundary lines)
bool useMesh = false;

// Desired vehicle speed (m/s)
double target_speed = 3;

// Simulation step size
double step_size = 2e-4;
double tire_step_size = 2e-4;

// Output frame images
bool output_images = false;
double fps = 60;
const std::string out_dir = GetChronoOutputPath() + "OPENCRG_DEMO";
const double mph2ms = 0.44704;
// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    unsigned int rmsLevel = 1;
    // This curve let us know, which accelration distance has to be provided
    // units (m/s) rsp. (m)
    // Curve calulated with test_FEDA_accel
    ChFunctionInterp feda_acc_curve;
    // AddPoint (speed,acceleration_distance)
    feda_acc_curve.AddPoint(0, 0);
    feda_acc_curve.AddPoint(5, 10);
    feda_acc_curve.AddPoint(10, 27);
    feda_acc_curve.AddPoint(15, 72);
    feda_acc_curve.AddPoint(20, 153);
    feda_acc_curve.AddPoint(25, 296);
    feda_acc_curve.AddPoint(30, 494);
    feda_acc_curve.AddPoint(33, 694);

    FEDA::DamperMode theDamperMode = FEDA::DamperMode::FSD;

    // ---------------------------------------
    // Create the vehicle, terrain, and driver
    // ---------------------------------------
    double velmph = 5;
    switch (argc) {
        case 2:
            crg_road_file.append("1in.crg");
            velmph = atof(argv[1]);
            break;
        case 3:
            switch (atoi(argv[1])) {
                default:
                case 1:
                    rmsLevel = 1;
                    crg_road_file.append("1in.crg");
                    break;
                case 2:
                    rmsLevel = 2;
                    crg_road_file.append("1p5in.crg");
                    break;
                case 3:
                    rmsLevel = 3;
                    crg_road_file.append("2in.crg");
                    break;
                case 4:
                    rmsLevel = 4;
                    crg_road_file.append("3in.crg");
                    break;
                case 5:
                    rmsLevel = 5;
                    crg_road_file.append("4in.crg");
                    break;
                case 6:
                    rmsLevel = 6;
                    crg_road_file.append("1_1p5in.crg");
                    break;
                case 7:
                    rmsLevel = 7;
                    crg_road_file.append("1p5_2in.crg");
                    break;
            }
            velmph = atof(argv[2]);
            break;
        case 4:
            switch (argv[1][0]) {
                case 'f':
                case 'F':
                default:
                    theDamperMode = FEDA::DamperMode::FSD;
                    break;
                case 'l':
                case 'L':
                    theDamperMode = FEDA::DamperMode::PASSIVE_LOW;
                    // step_size = tire_step_size = 1.0e-3;
                    break;
                case 'h':
                case 'H':
                    theDamperMode = FEDA::DamperMode::PASSIVE_HIGH;
                    // step_size = tire_step_size = 1.0e-3;
                    break;
            }
            switch (atoi(argv[2])) {
                default:
                case 1:
                    rmsLevel = 1;
                    crg_road_file.append("1in.crg");
                    break;
                case 2:
                    rmsLevel = 2;
                    crg_road_file.append("1p5in.crg");
                    break;
                case 3:
                    rmsLevel = 3;
                    crg_road_file.append("2in.crg");
                    break;
                case 4:
                    rmsLevel = 4;
                    crg_road_file.append("3in.crg");
                    break;
                case 5:
                    rmsLevel = 5;
                    crg_road_file.append("4in.crg");
                    break;
                case 6:
                    rmsLevel = 6;
                    crg_road_file.append("1_1p5in.crg");
                    break;
                case 7:
                    rmsLevel = 7;
                    crg_road_file.append("1p5_2in.crg");
                    break;
            }
            velmph = atof(argv[3]);
            break;
        default:
            std::cout << "usage form 1: test_FEDA_ride Speed_in_mph\n";
            std::cout << "usage form 2: test_FEDA_ride RMScase Speed_in_mph\n";
            std::cout << "usage form 3: test_FEDA_ride DamperMode RMScase Speed_in_mph\n\tDamperMode = "
                        "one of:  fsd, low or high\n";
            std::cout << "\tRMScase = "
                        "one of:  1 (1in), 2 (1.5in), 3 (2in), 4 (3in), 5 (4in), 6 (1+1.5in), 7 (1.5+2in)\n";
            return 1;
    }
    target_speed = mph2ms * velmph;

    double start_pos = -feda_acc_curve.GetVal(target_speed);

    // Create the FEDA vehicle, set parameters, and initialize
    FEDA my_feda;
    my_feda.SetContactMethod(ChContactMethod::SMC);
    my_feda.SetChassisFixed(false);
    my_feda.SetInitPosition(ChCoordsys<>(ChVector3d(start_pos - 1, 0, 0.5), QUNIT));
    my_feda.SetTireType(tire_model);
    my_feda.SetTireStepSize(tire_step_size);
    my_feda.SetRideHeight_ObstacleCrossing();
    my_feda.SetDamperMode(theDamperMode);
    my_feda.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    my_feda.Initialize();

    my_feda.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_feda.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_feda.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_feda.SetWheelVisualizationType(VisualizationType::NONE);
    my_feda.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    CRGTerrain terrain(my_feda.GetSystem());
    terrain.UseMeshVisualization(useMesh);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.Initialize(vehicle::GetDataFile(crg_road_file));

    // Get the vehicle path (middle of the road)
    // auto path = terrain.GetRoadCenterLine();
    bool path_is_closed = terrain.IsPathClosed();
    double road_length = terrain.GetLength();
    double road_width = terrain.GetWidth();
    auto path = StraightLinePath(ChVector3d(start_pos, 0, 0.5), ChVector3d(road_length + 20.0, 0, 0.5), 1);

    std::string wTitle = "FED Alpha Ride Performance: KRC Course ";
    switch (rmsLevel) {
        case 1:
            wTitle.append("1 in RMS, V = " + std::to_string(int(velmph)) + " mph");
            break;
        case 2:
            wTitle.append("1.5 in RMS, V = " + std::to_string(int(velmph)) + " mph");
            break;
        case 3:
            wTitle.append("2 in RMS, V = " + std::to_string(int(velmph)) + " mph");
            break;
        case 4:
            wTitle.append("3 in RMS, V = " + std::to_string(int(velmph)) + " mph");
            break;
        case 5:
            wTitle.append("4 in RMS, V = " + std::to_string(int(velmph)) + " mph");
            break;
        case 6:
            wTitle.append("1 in / 1.5 in RMS, V = " + std::to_string(int(velmph)) + " mph");
            break;
        case 7:
            wTitle.append("1.5 in / 2 in RMS, V = " + std::to_string(int(velmph)) + " mph");
            break;
    }
    switch (theDamperMode) {
        case FEDA::DamperMode::FSD:
            wTitle.append(", FSD Dampers");
            break;
        case FEDA::DamperMode::PASSIVE_LOW:
            wTitle.append(", Passive Dampers with low damping");
            break;
        case FEDA::DamperMode::PASSIVE_HIGH:
            wTitle.append(", Passive Dampers with high damping");
            break;
    }

    // Create the driver system based on PID steering controller
    ChPathFollowerDriver driver(my_feda.GetVehicle(), path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle(wTitle);
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->SetHUDLocation(500, 20);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&my_feda.GetVehicle());

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // ----------------
    // Output directory
    // ----------------

    if (output_images) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    ChISO2631_Vibration_SeatCushionLogger driverSeatLogger(step_size), passengerSeatLogger(step_size);

    // ---------------
    // Simulation loop
    // ---------------

    // Final posistion
    double xend = road_length - 10.0;
    std::cout << "Road length:     " << road_length << std::endl;
    std::cout << "Road width:      " << road_width << std::endl;
    std::cout << "Closed loop?     " << path_is_closed << std::endl;

    // Number of simulation steps between image outputs
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize frame counters
    int sim_frame = 0;
    int render_frame = 0;

    ChVector3d local_driver_pos = my_feda.GetChassis()->GetLocalDriverCoordsys().pos;
    ChVector3d local_passenger_pos = local_driver_pos;
    local_passenger_pos.y() *= -1.0;

    ChButterworthLowpass wesFilter(4, step_size, 30.0);
    ChFunctionInterp seatFkt;
    ChFunctionInterp shockVelFkt;
    std::vector<double> t, azd, azdf, shvel_exp, shvel_com;
    while (vis->Run()) {
        double time = my_feda.GetSystem()->GetChTime();
        double xpos = my_feda.GetVehicle().GetPos().x();
        if (xpos > road_length - 10)
            break;

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector3d& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector3d& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Render scene and output images
        vis->BeginScene();
        vis->Render();

        if (output_images && sim_frame % render_steps == 0) {
            char filename[200];
            sprintf(filename, "%s/image_%05d.bmp", out_dir.c_str(), render_frame++);
            vis->WriteImageToFile(filename);
            render_frame++;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_feda.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_feda.Advance(step_size);
        vis->Advance(step_size);

        double sv1 =
            std::static_pointer_cast<ChDoubleWishbone>(my_feda.GetVehicle().GetSuspension(0))->GetShockVelocity(LEFT);
        double sv2 =
            std::static_pointer_cast<ChDoubleWishbone>(my_feda.GetVehicle().GetSuspension(0))->GetShockVelocity(RIGHT);
        double sv3 =
            std::static_pointer_cast<ChDoubleWishbone>(my_feda.GetVehicle().GetSuspension(1))->GetShockVelocity(LEFT);
        double sv4 =
            std::static_pointer_cast<ChDoubleWishbone>(my_feda.GetVehicle().GetSuspension(1))->GetShockVelocity(RIGHT);
        xpos = my_feda.GetVehicle().GetPos().x();
        if (xpos >= 0.0) {
            double speed = my_feda.GetVehicle().GetSpeed();
            ChVector3d seat_acc_driver = my_feda.GetVehicle().GetPointAcceleration(local_driver_pos);
            ChVector3d seat_acc_passenger = my_feda.GetVehicle().GetPointAcceleration(local_passenger_pos);
            t.push_back(time);
            azd.push_back(seat_acc_driver.z() / 9.80665);
            azdf.push_back(wesFilter.Filter(seat_acc_driver.z() / 9.80665));
            seatFkt.AddPoint(time, azdf.back());
            shvel_exp.push_back(std::max(std::max(sv1, sv2), std::max(sv3, sv4)));
            shvel_com.push_back(std::min(std::min(sv1, sv2), std::min(sv3, sv4)));
            driverSeatLogger.AddData(target_speed, seat_acc_driver);
            passengerSeatLogger.AddData(target_speed, seat_acc_passenger);
        }

        // Increment simulation frame number
        sim_frame++;

        vis->EndScene();
    }

    // filter test
    if (azd.size() > 0) {
        double azdmax = azd[0];
        double azdfmax = azdf[0];
        for (size_t i = 0; i < azd.size(); i++) {
            if (azd[i] > azdmax)
                azdmax = azd[i];
            if (azdf[i] > azdfmax)
                azdfmax = azdf[i];
        }
        std::cout << "Shock Performance Result Obstcle: " << rmsLevel << " in, Speed = " << velmph
                 << " mph,  Az maximum = " << azdfmax << " g\n";
        double shvmax = 0.0;
        double shvmin = 0.0;
        for (int i = 0; i < shvel_exp.size(); i++) {
            if (shvel_exp[i] > shvmax) {
                shvmax = shvel_exp[i];
            }
            if (shvel_com[i] < shvmin) {
                shvmin = shvel_com[i];
            }
        }
        std::cout << "Max. Damper Expansion Velocity = " << shvmax << " m/s\n";
        std::cout << "Min. Damper Compression Velocity = " << shvmin << " m/s\n";
        double mps2mph = 2.2369362921;
        double effSpeed = driverSeatLogger.GetAVGSpeed();
        double absPowDriver = driverSeatLogger.GetAbsorbedPowerVertical();
        double absPowPassenger = passengerSeatLogger.GetAbsorbedPowerVertical();
        std::cout << "\nDriver's Seat:    Absorbed Power = " << absPowDriver << " W at speed = " << effSpeed
                 << " m/s ( = " << effSpeed * mps2mph << " mph)\n";
        std::cout << "Passenger's Seat: Absorbed Power = " << absPowPassenger << " W\n";
        std::cout << "Exposition Time = " << driverSeatLogger.GetExposureTime() << " s\n";
#ifdef CHRONO_POSTPROCESS
        std::string title =
            "Fed alpha on halfround " + std::to_string(rmsLevel) + " in, V = " + std::to_string(velmph) + " mph";
        postprocess::ChGnuPlot gplot_seat;
        gplot_seat.SetGrid();
        gplot_seat.SetTitle(title.c_str());
        gplot_seat.SetLabelX("time (s)");
        gplot_seat.SetLabelY("driver seat acceleration (g)");
        gplot_seat.Plot(seatFkt, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif
    }
    return 0;
}
