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
//
// =============================================================================

#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "framework.h"
#include "ChMAPMessage.h"
#include "chrono_vehicle/ChVehicleModelData.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    std::string vis_file("corridor/park_visual.obj");
    std::string coll_file("corridor/park_collision.obj");

    // Load MAP message
    ChMAPMessage message;
    auto res = message.ParseFromFile(vehicle::GetDataFile("univ_park_map_info.json"));
    if (res != "") {
        std::cout << "Unable to parse MAP data. Error in field " << res << '\n';
        return 1;
    }

    av::Scene scene(av::GPScoord(43.072172, -89.400391), vis_file, coll_file);
    av::Framework framework(scene, false);
    framework.SetIntegrationStep(5e-3);
    framework.SetVerbose(true);
    ////framework.SetRender(false);

    // Create paths

    auto pos1 = framework.GetLocation(av::GPScoord(43.0726234, -89.40045));
    pos1.z() += 0.25;
    auto circle_path = framework.AddPath(vehicle::CirclePath(pos1, 15, 80, false, 3));

    std::vector<ChVector3d> local_points = {ChVector3d(64.8927, 13.249, -0.481722),    //
                                            ChVector3d(80.3284, 12.7266, -0.603526),   //
                                            ChVector3d(94.6545, 12.2176, -0.694871),   //
                                            ChVector3d(105.61, 12.3841, -0.739324),    //
                                            ChVector3d(114.282, 16.0502, -0.693778),   //
                                            ChVector3d(120.643, 26.2061, -0.528403),   //
                                            ChVector3d(121.556, 41.204, -0.317819),    //
                                            ChVector3d(122.405, 63.1628, 0.00486306),  //
                                            ChVector3d(121.647, 76.5746, 0.20102),     //
                                            ChVector3d(120.709, 88.7467, 0.389847),    //
                                            ChVector3d(120.974, 107.356, 0.759666),    //
                                            ChVector3d(121.9, 124.316, 1.09734),       //
                                            ChVector3d(123.194, 147.227, 1.59602),     //
                                            ChVector3d(123.864, 155.117, 1.87896),     //
                                            ChVector3d(123.159, 160.661, 1.82409),     //
                                            ChVector3d(118.760, 163.799, 1.61755),     //
                                            ChVector3d(106.761, 164.621, 1.2629),      //
                                            ChVector3d(84.4186, 165.658, 0.667642),    //
                                            ChVector3d(60.3295, 166.995, 0.0448156),   //
                                            ChVector3d(41.2747, 168.255, -0.414149),   //
                                            ChVector3d(18.7352, 169.553, -0.935203),   //
                                            ChVector3d(7.37289, 169.942, -1.14195),    //
                                            ChVector3d(0.416377, 167.835, -1.23743),   //
                                            ChVector3d(-1.55959, 159.595, -0.857577),  //
                                            ChVector3d(-2.12039, 140.422, -0.056825),  //
                                            ChVector3d(-3.4468, 114.978, 0.612327),    //
                                            ChVector3d(-2.90567, 98.4509, 0.934667),   //
                                            ChVector3d(-2.13378, 87.3479, 1.06922),    //
                                            ChVector3d(-1.94289, 76.7192, 1.0948),     //
                                            ChVector3d(-2.36216, 63.751, 0.885862),    //
                                            ChVector3d(-3.5639, 46.2598, 0.512486),    //
                                            ChVector3d(-3.72494, 35.4111, 0.233689),   //
                                            ChVector3d(-3.96505, 28.2692, 0.0131532),  //
                                            ChVector3d(-2.41712, 24.2123, -0.130718),  //
                                            ChVector3d(3.0726, 22.2459, -0.157238),    //
                                            ChVector3d(17.785, 19.1502, -0.209135),    //
                                            ChVector3d(34.5575, 15.9428, -0.298507),   //
                                            ChVector3d(52.1876, 14.0408, -0.381472)};
    auto loop_path = framework.AddPath(local_points, true);

    std::vector<av::GPScoord> gps_points = {av::GPScoord(43.0723306, -89.4006454),  //
                                            av::GPScoord(43.0724198, -89.4006307),  //
                                            av::GPScoord(43.0725187, -89.4006173),  //
                                            av::GPScoord(43.0726324, -89.4006025),  //
                                            av::GPScoord(43.0727739, -89.4005972),  //
                                            av::GPScoord(43.0728785, -89.4005972),  //
                                            av::GPScoord(43.0730984, -89.4005915),  //
                                            av::GPScoord(43.0731849, -89.4006079),  //
                                            av::GPScoord(43.0732143, -89.4006562),  //
                                            av::GPScoord(43.0732319, -89.4007192),  //
                                            av::GPScoord(43.0732329, -89.4008466),  //
                                            av::GPScoord(43.0732358, -89.4010759),  //
                                            av::GPScoord(43.0732437, -89.4015011),  //
                                            av::GPScoord(43.0732515, -89.4019302),  //
                                            av::GPScoord(43.0732545, -89.4022507),  //
                                            av::GPScoord(43.0732525, -89.4023594),  //
                                            av::GPScoord(43.0732388, -89.4024224),  //
                                            av::GPScoord(43.0732045, -89.4024506),  //
                                            av::GPScoord(43.0731281, -89.4024546),  //
                                            av::GPScoord(43.0728763, -89.4024573),  //
                                            av::GPScoord(43.0726921, -89.4024559),  //
                                            av::GPScoord(43.0724551, -89.4024640),  //
                                            av::GPScoord(43.0723169, -89.4024680),  //
                                            av::GPScoord(43.0722376, -89.4024774),  //
                                            av::GPScoord(43.0721915, -89.4024613),  //
                                            av::GPScoord(43.0721719, -89.4023916),  //
                                            av::GPScoord(43.0721670, -89.4022454),  //
                                            av::GPScoord(43.0721621, -89.4018913),  //
                                            av::GPScoord(43.0721582, -89.4015172),  //
                                            av::GPScoord(43.0721514, -89.4011537),  //
                                            av::GPScoord(43.0721484, -89.4009914),  //
                                            av::GPScoord(43.0721474, -89.4008318),  //
                                            av::GPScoord(43.0721553, -89.4007836),  //
                                            av::GPScoord(43.0721651, -89.4007460),  //
                                            av::GPScoord(43.0721812, -89.4007098),  //
                                            av::GPScoord(43.0722033, -89.4006790),  //
                                            av::GPScoord(43.0722807, -89.4006588),  //
                                            av::GPScoord(43.0723306, -89.4006454)};
    auto gps_loop_path = framework.AddPath(gps_points, true);

    std::vector<std::vector<av::GPScoord>> MAPPoints;
    std::vector<unsigned int> MAPPaths;
    message.MakeLanePoints(MAPPoints);

    for (auto p : MAPPoints)
        MAPPaths.push_back(framework.AddPath(p, false));

    framework.SetPathColor(circle_path, ChColor(0.6f, 0.6f, 0.0f));
    framework.SetPathColor(loop_path, ChColor(0.0f, 0.6f, 0.0f));
    framework.SetPathColor(gps_loop_path, ChColor(0.0f, 0.0f, 0.6f));
    for (auto p : MAPPaths)
        framework.SetPathColor(p, ChColor(0.6f, 0.0f, 0.6f));

    // Create vehicles

    auto van1 = framework.AddVehicle(av::Vehicle::Type::VAN, circle_path, pos1, 4.0);
    auto truck1 =
        framework.AddVehicle(av::Vehicle::Type::TRUCK, loop_path, ChVector3d(41.2747, 168.255, -0.414149), 8.25);
    auto truck2 =
        framework.AddVehicle(av::Vehicle::Type::TRUCK, loop_path, ChVector3d(0.416377, 167.835, -1.23743), 8.0);
    auto truck3 =
        framework.AddVehicle(av::Vehicle::Type::TRUCK, gps_loop_path, av::GPScoord(43.0732437, -89.4015011), 4.0);

    framework.SetAgentBroadcast(van1, 1.0, 100);
    ////framework.SetAgentBroadcast(truck1, 1.0, 100);
    ////framework.SetAgentBroadcast(truck2, 1.0, 100);
    ////framework.SetAgentBroadcast(truck3, 1.0, 100);

    framework.SetEgoVehicle(truck1);

    // Create traffic lights

    auto l1 = framework.AddTrafficLight(av::GPScoord(43.073269, -89.400572), 40, av::GPScoord(43.073430, -89.400839));
    ////auto l2 = framework.AddTrafficLight(av::GPScoord(43.072172, -89.400391), 2, av::GPScoord(43.072172, -89.400391));

    framework.SetAgentBroadcast(l1, 2.0, 150.0);

    // Start simulation loop

    framework.ListAgents();
    framework.Run(200, 20, true);

    return 0;
}
