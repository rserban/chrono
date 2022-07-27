// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Generator functions for Polaris on SPH terrain system
//
// =============================================================================

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "CreateObjects.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fsi;

chrono::ChCoordsys<> CreateTerrain(ChSystem& sys,
                                   ChSystemFsi& sysFSI,
                                   const std::string& terrain_dir,
                                   double ramp_length,
                                   bool terrain_mesh_vis,
                                   bool terrain_mesh_contact) {
    // Include acceleration ramp?
    bool create_ramp = (ramp_length > 0);

    // Create SPH markers with initial locations from file
    int num_particles = 0;
    ChVector<> aabb_min(std::numeric_limits<double>::max());
    ChVector<> aabb_max(-std::numeric_limits<double>::max());

    ChVector<> marker;
    std::string line;
    std::string cell;

    std::ifstream ifile(vehicle::GetDataFile(terrain_dir + "/particles_20mm.txt"));
    getline(ifile, line);  // Comment line
    while (getline(ifile, line)) {
        std::stringstream ls(line);
        for (int i = 0; i < 3; i++) {
            getline(ls, cell, ',');
            marker[i] = stod(cell);
            aabb_min[i] = std::min(aabb_min[i], marker[i]);
            aabb_max[i] = std::max(aabb_max[i], marker[i]);
        }
        ////ChVector<> tau(-sysFSI.GetSensity() * std::abs(gravity.z) * (-marker.z() + fzDim));
        ChVector<> tau(0);
        sysFSI.AddSPHParticle(marker, sysFSI.GetDensity(), 0, sysFSI.GetViscosity(), sysFSI.GetKernelLength(), VNULL,
                              tau, VNULL);
        num_particles++;
    }
    ifile.close();

    // Set computational domain
    ChVector<> aabb_dim = aabb_max - aabb_min;
    aabb_dim.z() *= 50;

    //// RADU TODO:  FIX THIS SOMEHOW ELSE!!!
    if (create_ramp)
        aabb_min.x() -= 5;

    sysFSI.SetBoundaries(aabb_min - 0.1 * aabb_dim, aabb_max + 0.1 * aabb_dim);

    // Create ground body
    auto body = std::shared_ptr<ChBody>(sys.NewBody());
    body->SetBodyFixed(true);
    sys.AddBody(body);

    // Attach BCE markers (Note: BCE markers must be created after SPH markers!)
    sysFSI.AddFileBCE(body, vehicle::GetDataFile(terrain_dir + "/bce_20mm.txt"), VNULL, QUNIT, 1.0, false);

    // Extract slope and banking of the terrain patch
    double slope = 0;
    double banking = 0;
    if (filesystem::path(vehicle::GetDataFile(terrain_dir + "/slope.txt")).exists()) {
        std::ifstream is(vehicle::GetDataFile(terrain_dir + "/slope.txt"));
        is >> slope >> banking;
        is.close();
    }

    // Ramp dimensions and orientation
    double hlen = ramp_length / 2;
    double hwidth = 2;
    double hheight = 0.5;
    auto ramp_rot = Q_from_AngX(banking) * Q_from_AngY(-slope);
    auto ramp_loc = ramp_rot.Rotate(ChVector<>(-hlen, 0, -hheight));

    // Create visual and collision shapes
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile(terrain_dir + "/mesh.obj"), true, false);

    if (create_ramp) {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(hlen, hwidth, hheight);
        body->AddVisualShape(box, ChFrame<>(ramp_loc, ramp_rot));
    }

    if (terrain_mesh_vis) {
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetMutable(false);
        body->AddVisualShape(trimesh_shape);
    }

    MaterialInfo mat_info;
    mat_info.mu = 0.9f;
    auto mat = mat_info.CreateMaterial(sys.GetContactMethod());

    body->GetCollisionModel()->ClearModel();
    if (create_ramp) {
        body->GetCollisionModel()->AddBox(mat, hlen, hwidth, hheight, ramp_loc, ramp_rot);
    }
    if (terrain_mesh_contact) {
        body->GetCollisionModel()->AddTriangleMesh(mat, trimesh, true, false, VNULL, ChMatrix33<>(1), 0.01);
    }
    body->GetCollisionModel()->BuildModel();
    body->SetCollide(true);

    // Set and return vehicle initial location
    double init_x = create_ramp ? -2 * hlen + 4 : 4;
    ChVector<> init_loc = ramp_rot.Rotate(ChVector<>(init_x, 0, 0.25));

    return ChCoordsys<>(init_loc, ramp_rot);
}

std::shared_ptr<ChBezierCurve> CreatePath(const std::string& terrain_dir, double ramp_length) {
    // Include acceleration ramp?
    bool create_ramp = (ramp_length > 0);

    // Open input file
    std::ifstream ifile(vehicle::GetDataFile(terrain_dir + "/path.txt"));
    std::string line;

    // Read number of knots and type of curve
    size_t numPoints;
    size_t numCols;

    std::getline(ifile, line);
    std::istringstream iss(line);
    iss >> numPoints >> numCols;

    assert(numCols == 3);

    // Read path points
    std::vector<ChVector<>> points;

    if (create_ramp) {
        // Include additional point if creating a ramp
        points.push_back(ChVector<>(-ramp_length, 0, 0));
    }

    for (size_t i = 0; i < numPoints; i++) {
        double x, y, z;
        std::getline(ifile, line);
        std::istringstream jss(line);
        jss >> x >> y >> z;
        points.push_back(ChVector<>(x, y, z));
    }

    // Include point beyond SPH patch
    { 
        auto np = points.size();
        points.push_back(2.0 * points[np - 2] + points[np - 1]);
    }

    ifile.close();

    return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points));
}

std::shared_ptr<WheeledVehicle> CreateVehicle(PolarisModel model,
                                              ChSystem& sys,
                                              const ChCoordsys<>& init_pos,
                                              ChSystemFsi& sysFSI) {
    std::string model_dir = (model == PolarisModel::ORIGINAL) ? "mrzr/JSON_orig/" : "mrzr/JSON_new/";

    std::string vehicle_json = model_dir + "vehicle/MRZR.json";
    ////std::string powertrain_json = model_dir + "powertrain/MRZR_SimplePowertrain.json";
    std::string powertrain_json = model_dir + "powertrain/MRZR_SimpleMapPowertrain.json";
    std::string tire_json = model_dir + "tire/MRZR_RigidTire.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(powertrain_json));
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    return vehicle;
}

void CreateWheelBCEMarkers(std::shared_ptr<WheeledVehicle> vehicle, ChSystemFsi& sysFSI) {
    // Create BCE markers for a tire
    std::string tire_coll_obj = "mrzr/meshes_new/Polaris_tire_collision.obj";

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(vehicle::GetDataFile(tire_coll_obj));
    std::vector<ChVector<>> point_cloud;
    CreateMeshBCEMarkers(trimesh, sysFSI.GetInitialSpacing(), point_cloud);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            sysFSI.AddFsiBody(wheel->GetSpindle());
            sysFSI.AddPointsBCE(wheel->GetSpindle(), point_cloud, VNULL, QUNIT);
        }
    }
}

void CreateMeshBCEMarkers(const geometry::ChTriangleMeshConnected& mesh,
                          double delta,
                          std::vector<ChVector<>>& point_cloud) {
    ChVector<> minV = mesh.m_vertices[0];
    ChVector<> maxV = mesh.m_vertices[0];
    ChVector<> currV = mesh.m_vertices[0];
    for (unsigned int i = 1; i < mesh.m_vertices.size(); ++i) {
        currV = mesh.m_vertices[i];
        if (minV.x() > currV.x())
            minV.x() = currV.x();
        if (minV.y() > currV.y())
            minV.y() = currV.y();
        if (minV.z() > currV.z())
            minV.z() = currV.z();
        if (maxV.x() < currV.x())
            maxV.x() = currV.x();
        if (maxV.y() < currV.y())
            maxV.y() = currV.y();
        if (maxV.z() < currV.z())
            maxV.z() = currV.z();
    }
    ////printf("start coords: %f, %f, %f\n", minV.x(), minV.y(), minV.z());
    ////printf("end coords: %f, %f, %f\n", maxV.x(), maxV.y(), maxV.z());

    const double EPSI = 1e-6;

    ChVector<> ray_origin;
    for (double x = minV.x(); x < maxV.x(); x += delta) {
        ray_origin.x() = x + 1e-9;
        for (double y = minV.y(); y < maxV.y(); y += delta) {
            ray_origin.y() = y + 1e-9;
            for (double z = minV.z(); z < maxV.z(); z += delta) {
                ray_origin.z() = z + 1e-9;

                ChVector<> ray_dir[2] = {ChVector<>(5, 0.5, 0.25), ChVector<>(-3, 0.7, 10)};
                int intersectCounter[2] = {0, 0};

                for (unsigned int i = 0; i < mesh.m_face_v_indices.size(); ++i) {
                    auto& t_face = mesh.m_face_v_indices[i];
                    auto& v1 = mesh.m_vertices[t_face.x()];
                    auto& v2 = mesh.m_vertices[t_face.y()];
                    auto& v3 = mesh.m_vertices[t_face.z()];

                    // Find vectors for two edges sharing V1
                    auto edge1 = v2 - v1;
                    auto edge2 = v3 - v1;

                    bool t_inter[2] = {false, false};

                    for (unsigned int j = 0; j < 2; j++) {
                        // Begin calculating determinant - also used to calculate uu parameter
                        auto pvec = Vcross(ray_dir[j], edge2);
                        // if determinant is near zero, ray is parallel to plane of triangle
                        double det = Vdot(edge1, pvec);
                        // NOT CULLING
                        if (det > -EPSI && det < EPSI) {
                            t_inter[j] = false;
                            continue;
                        }
                        double inv_det = 1.0 / det;

                        // calculate distance from V1 to ray origin
                        auto tvec = ray_origin - v1;

                        // Calculate uu parameter and test bound
                        double uu = Vdot(tvec, pvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (uu < 0.0 || uu > 1.0) {
                            t_inter[j] = false;
                            continue;
                        }

                        // Prepare to test vv parameter
                        auto qvec = Vcross(tvec, edge1);

                        // Calculate vv parameter and test bound
                        double vv = Vdot(ray_dir[j], qvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (vv < 0.0 || ((uu + vv) > 1.0)) {
                            t_inter[j] = false;
                            continue;
                        }

                        double tt = Vdot(edge2, qvec) * inv_det;
                        if (tt > EPSI) {  // ray intersection
                            t_inter[j] = true;
                            continue;
                        }

                        // No hit, no win
                        t_inter[j] = false;
                    }

                    intersectCounter[0] += t_inter[0] ? 1 : 0;
                    intersectCounter[1] += t_inter[1] ? 1 : 0;
                }

                if (((intersectCounter[0] % 2) == 1) && ((intersectCounter[1] % 2) == 1))  // inside mesh
                    point_cloud.push_back(ChVector<>(x, y, z));
            }
        }
    }
}
