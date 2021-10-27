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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Sample test program for sequential WVP simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

//#define USE_IRRLICHT


#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#ifdef USE_IRRLICHT
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#endif

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_models/vehicle/wvp/WVP_FollowerDataDriver.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_models/vehicle/wvp/WVP.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>
#include <thread>
#include <math.h>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(-180, 0, 0.8);
ChQuaternion<> initRot(1, 0, 0, 0);

ChVector<>gravity(0,0,-9.81);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of tire model (RIGID, FIALA, PAC89)
TireModelType tire_model = TireModelType::PAC89;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

double terrainLength = 400;
double terrainWidth = 5;
double terrainHeight = 0;

// Simulation step sizes
double step_size =1e-4;
double tire_step_size = step_size;

// Simulation end time
double tend = 15;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50
double output_step_size = 1e-3;

//output directory
const std::string out_dir = "../WVP_SIDE_SLOPE";
const std::string pov_dir = out_dir + "/POVRAY";
bool povray_output = false;
bool data_output = true;

//vehicle driver inputs
// Desired vehicle speed (m/s)
double mph_to_ms = 0.44704;
double target_speed = 35*mph_to_ms;

// std::string path_file("paths/NATO_double_lane_change.txt");
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("wvp/SteeringController.json");
std::string speed_controller_file("wvp/SpeedController.json");

std::string output_file_name("DLC_Paved_RtL_35");

std::string steering_input_fileLeft("wvp/30Per Side Slope - Left Side Down Serpentine.csv");
std::string steering_input_fileRight("wvp/30Per Side Slope - Left Side Down Serpentine.csv");
std::string steering_input_file("wvp/30Per Side Slope - Left Side Down Serpentine.csv");

bool LtR = false;


// =============================================================================

int main(int argc, char* argv[]) {
    //send in args: filename time speed LtR(or RtL)
    //read from args

    //arg1=simulation length, arg2=target speed, arg3=0 for left side down, arg3=1 for right side down
    if(argc > 2){
        tend = atof(argv[1]);
        target_speed = atof(argv[2]);
        if(atof(argv[3])==0){
            steering_input_file = steering_input_fileLeft;
            output_file_name = "LeftSideDown" + std::to_string((int)target_speed);
            gravity = ChVector<>({0,2.80604,-9.353468});

        }
        else if(atof(argv[3])==1){ //0 = LTR, 1=RtL
            steering_input_file = steering_input_fileLeft;
            output_file_name = "RightSideDown" + std::to_string((int)target_speed);
            gravity = ChVector<>({0,-2.80604,-9.353468});
            std::cout<<output_file_name<<std::endl;
        }

        target_speed = target_speed*mph_to_ms;
    }
    else{
        std::cout<<"Standard Setup"<<std::endl;
    }



    // --------------
    // Create systems
    // --------------

    // Create the vehicle, set parameters, and initialize
    WVP wvp;
    wvp.SetContactMethod(ChContactMethod::NSC);
    wvp.SetChassisFixed(false);
    wvp.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    wvp.SetTireType(tire_model);
    wvp.SetTireStepSize(tire_step_size);
    wvp.Initialize();

    wvp.GetSystem()->Set_G_acc(gravity);

    wvp.SetChassisVisualizationType(chassis_vis_type);
    wvp.SetSuspensionVisualizationType(suspension_vis_type);
    wvp.SetSteeringVisualizationType(steering_vis_type);
    wvp.SetWheelVisualizationType(wheel_vis_type);
    wvp.SetTireVisualizationType(tire_vis_type);

    std::cout << "Vehicle mass:               " << wvp.GetVehicle().GetVehicleMass() << std::endl;
    std::cout << "Vehicle mass (with tires):  " << wvp.GetTotalMass() << std::endl;

    // ------------------
    // Create the terrain
    // ------------------

    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.4f);
    patch_mat->SetRestitution(0.01f);
    RigidTerrain terrain(wvp.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 2000, 1000);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 20, 20);
    terrain.Initialize();



    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    // //driver for open loop controller
    WVP_FollowerDataDriver driver(wvp.GetVehicle(),
            vehicle::GetDataFile(steering_controller_file),
            vehicle::GetDataFile(speed_controller_file), path, "side slope slalom",
            target_speed, vehicle::GetDataFile(steering_input_file), 1, 2, 20.0 //1,2,20 corresponds to read column 1 for time, 2 for steering, and wait 20 seconds before switching from closed loop to open loop
            );


    //create the driver
    // auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    // ChPathFollowerDriver driver(wvp.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
    //                             vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);
    driver.Initialize();



#ifdef USE_IRRLICHT
    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // Create the driver system
    // -------------------------------------

    ChWheeledVehicleIrrApp app(&wvp.GetVehicle(), L"WVP sequential test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    /*app.SetTimestep(step_size);*/
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);
#endif

    // -------------
    // Prepare output
    // -------------

    if (data_output || povray_output) {
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
        driver.ExportPathPovray(out_dir);
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // ---------------
    // Simulation loop
    // ---------------

    int step_number = 0;
    int render_frame = 0;

    double time = 0;
	
	csv << "time (s)";
    csv << "steering wheel angle (deg)";
    csv << "vehicle speed (m/s)";
	csv << "lateral acceleration (g)";
    csv << "roll angle (deg)";
    csv << "roll rate (deg/s)";
    csv << "yaw angle (deg)";
    csv << "yaw rate (deg/s)";
	csv << "CG X Pos (m)";
	csv << "CG Y Pos (m)";
	csv << "longitudinal tire force FL (N)";
    csv << "longitudinal tire force FR (N)";
    csv << "longitudinal tire force RL (N)";
    csv << "longitudinal tire force RR (N)";
    csv << "lateral tire force FL (N)";
    csv << "lateral tire force FR (N)";
    csv << "lateral tire force RL (N)";
    csv << "lateral tire force RR (N)";
    csv << "vertical tire force FL (N)";
    csv << "vertical tire force FR (N)";
    csv << "vertical tire force RL (N)";
    csv << "vertical tire force RR (N)";
    csv << "strut length FL (m)";
    csv << "strut length FR (m)";
    csv << "strut length RL (m)";
    csv << "strut length RR (m)";
    csv << std::endl;
	

    //output headings for the saved data file
        //output headings for the saved data file
    /* csv <<"time"<<"Steering Angle"<<"vehicle Speed";

    csv <<"Chassis Attitude"<<"Chassis Bank"<<"Chassis Heading";
    csv <<"Chassis Omega X"<<"Chassis Omega Y"<<"Chassis Omega Z";

    csv <<"Lateral Acceleration"<<"W0 long"<<"W0 lat"<<"W0 vert"<<"W1 long"<<"W1 lat"<<"W1 vert";
    csv <<"W2 long"<<"W2 lat"<<"W2 vert"<<"W3 long"<<"W3 lat"<<"W3 vert";
    csv <<"W0 Pos x"<<"W0 Pos Y"<<"W0 Pos Z"<<"W1 Pos x"<<"W1 Pos Y"<<"W1 Pos Z";
    csv <<"W2 Pos x"<<"W2 Pos Y"<<"W2 Pos Z"<<"W3 Pos x"<<"W3 Pos Y"<<"W3 Pos Z";
    csv << "GravityX"<<"GravityY"<<"GravityZ"<< std::endl; */


#ifdef USE_IRRLICHT
    while (app.GetDevice()->run()) {


        //path visualization
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        /*std::cout<<"Target:\t"<<(irr::f32)pT.x()<<",\t "<<(irr::f32)pT.y()<<",\t "<<(irr::f32)pT.z()<<std::endl;
        std::cout<<"Vehicle:\t"<<wvp.GetVehicle().GetChassisBody()->GetPos().x()
          <<",\t "<<wvp.GetVehicle().GetChassisBody()->GetPos().y()<<",\t "
          <<wvp.GetVehicle().GetChassisBody()->GetPos().z()<<std::endl;*/

        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }
#else
    while(wvp.GetSystem()->GetChTime() < tend){
#endif

        time = wvp.GetSystem()->GetChTime();

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        wvp.Synchronize(time, driver_inputs, terrain);
#ifdef USE_IRRLICHT
        app.Synchronize("Follower driver", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules

        driver.Advance(step_size);
        terrain.Advance(step_size);
        wvp.Advance(step_size);
#ifdef USE_IRRLICHT
        app.Advance(step_size);
#endif

        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteVisualizationAssets(wvp.GetSystem(), filename);
            render_frame++;

        }

        if(data_output && step_number % output_steps == 0){
            std::cout<<time<<std::endl;


            auto susp0 = std::static_pointer_cast<ChDoubleWishbone>(wvp.GetVehicle().GetSuspension(0));
            auto susp1 = std::static_pointer_cast<ChDoubleWishbone>(wvp.GetVehicle().GetSuspension(1));
            double def0 = susp0->GetSpringLength(LEFT)-.779;
            double def1 = susp0->GetSpringLength(RIGHT)-.779;
            double def2 = susp1->GetSpringLength(LEFT)-.831;
            double def3 = susp1->GetSpringLength(RIGHT)-.831;

            if(def0 > .124 || def0 < -.169){
                std::cout<<"Strut 0 BUMP STOP ENGAGED!\n";
            }
            if(def1 > .124 || def1 < -.169){
                std::cout<<"Strut 1 BUMP STOP ENGAGED!\n";
            }
            if(def2 > .124 || def2 < -.169){
                std::cout<<"Strut 2 BUMP STOP ENGAGED!\n";
            }
            if(def3 > .124 || def3 < -.169){
                std::cout<<"Strut 3 BUMP STOP ENGAGED!\n";
            }
            csv << time;    //time
            csv << driver_inputs.m_steering / -.001282;  // steering wheel angle in degrees
            csv << wvp.GetVehicle().GetVehicleSpeed();   // vehicle speed m/s
            csv << wvp.GetVehicle().GetVehiclePointAcceleration({-2.070, .01, .495}).y() / 9.81;  // lateral acc. in g's
            ChQuaternion<> q = wvp.GetVehicle().GetVehicleRot();
            csv << q.Q_to_Euler123().x() * 180.0 / CH_C_PI;                    // roll angle in deg
            csv << wvp.GetChassisBody()->GetWvel_loc().x() * 180.0 / CH_C_PI;  // roll rate deg
            csv << q.Q_to_Euler123().z() * 180.0 / CH_C_PI;                    // yaw angle deg
            csv << wvp.GetChassisBody()->GetWvel_loc().z() * 180.0 / CH_C_PI;  // yaw rate deg
            csv << wvp.GetVehicle().GetVehicleCOMPos().x();                    // COM path x
            csv << wvp.GetVehicle().GetVehicleCOMPos().y();                    // COM path y

            std::vector<ChVector<>> tire_forces;
            for (auto& axle : wvp.GetVehicle().GetAxles()) {
                for (auto& wheel : axle->GetWheels()) {
                    tire_forces.push_back(wheel->GetTire()->ReportTireForce(&terrain).force);
                }
            }

            for (int i = 0; i < 4; i++) {
                csv << tire_forces[i].x();  // longitudinal tire forces FL, FR, RL, RR
            }
            for (int i = 0; i < 4; i++) {
                csv << tire_forces[i].y();  // lateral tire forces FL, FR, RL, RR
            }
            for (int i = 0; i < 4; i++) {
                csv << tire_forces[i].z();  // vertical tire forces FL, FR, RL, RR
            }

            csv << susp0->GetSpringLength(LEFT);   // individual strut lengths m
            csv << susp0->GetSpringLength(RIGHT);//individual strut lengths m
            csv << susp1->GetSpringLength(LEFT);//individual strut lengths m
            csv << susp1->GetSpringLength(RIGHT);//individual strut lengths m
            csv << std::endl;
        }


        // std::cout<<time<<std::endl;
        // Increment frame number
        step_number++;
    }

    if (data_output) {
        csv.write_to_file(out_dir + "/" + output_file_name + ".dat");
    }

    return 0;
}
