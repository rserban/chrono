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

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

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

    std::ifstream ifile;
    std::string line;
    std::string cell;

    // Create SPH markers with initial locations from file
    int num_particles = 0;
    ChVector3d aabb_min(std::numeric_limits<double>::max());
    ChVector3d aabb_max(-std::numeric_limits<double>::max());

    ifile.open(vehicle::GetDataFile(terrain_dir + "/particles_20mm.txt"));
    getline(ifile, line);  // Comment line

    ChVector3d marker;
    while (getline(ifile, line)) {
        std::stringstream ls(line);
        for (int i = 0; i < 3; i++) {
            getline(ls, cell, ',');
            marker[i] = stod(cell);
            aabb_min[i] = std::min(aabb_min[i], marker[i]);
            aabb_max[i] = std::max(aabb_max[i], marker[i]);
        }
        ////ChVector3d tau(-sysFSI.GetSensity() * std::abs(gravity.z) * (-marker.z() + fzDim));
        ChVector3d tau(0);
        sysFSI.AddSPHParticle(marker, sysFSI.GetDensity(), 0.0, sysFSI.GetViscosity(), VNULL, tau, VNULL);
        num_particles++;
    }
    ifile.close();

    // Set computational domain
    ChVector3d aabb_dim = aabb_max - aabb_min;
    aabb_dim.z() *= 50;

    //// RADU TODO:  FIX THIS SOMEHOW ELSE!!!
    if (create_ramp)
        aabb_min.x() -= 5;

    sysFSI.SetBoundaries(aabb_min - 0.1 * aabb_dim, aabb_max + 0.1 * aabb_dim);

    // Create ground body
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    sys.AddBody(body);

    // Attach BCE markers (Note: BCE markers must be created after SPH markers!)
    std::vector<ChVector3d> bces;

    ifile.open(vehicle::GetDataFile(terrain_dir + "/bce_20mm.txt"));
    getline(ifile, line);  // Comment line

    ChVector3d bce;
    while (getline(ifile, line)) {
        std::stringstream ls(line);
        for (int i = 0; i < 3; i++) {
            getline(ls, cell, ',');
            bce[i] = stod(cell);
        }
        bces.push_back(bce);
    }
    sysFSI.AddPointsBCE(body, bces, ChFrame<>(), false);

    // Extract slope and banking of the terrain patch
    double slope = 0;
    double banking = 0;
    if (filesystem::path(vehicle::GetDataFile(terrain_dir + "/slope.txt")).exists()) {
        std::ifstream is(vehicle::GetDataFile(terrain_dir + "/slope.txt"));
        is >> slope >> banking;
        is.close();
    }

    // Ramp dimensions and orientation
    double width = 4;
    double height = 1;
    auto ramp_rot = QuatFromAngleX(banking) * QuatFromAngleY(-slope);
    auto ramp_loc = ramp_rot.Rotate(ChVector3d(-ramp_length / 2, 0, -height / 2));

    // Create visual and collision shapes
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile(terrain_dir + "/mesh.obj"), true, false);

    if (create_ramp) {
        auto box = chrono_types::make_shared<ChVisualShapeBox>(ramp_length, width, height);
        body->AddVisualShape(box, ChFrame<>(ramp_loc, ramp_rot));
    }

    if (terrain_mesh_vis) {
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetMutable(false);
        body->AddVisualShape(trimesh_shape);
    }

    ChContactMaterialData mat_info;
    mat_info.mu = 0.9f;
    auto mat = mat_info.CreateMaterial(sys.GetContactMethod());

    if (create_ramp) {
        body->GetCollisionModel()->AddBox(mat, ramp_length, width, height, ramp_loc, ramp_rot);
    }
    if (terrain_mesh_contact) {
        body->GetCollisionModel()->AddTriangleMesh(mat, trimesh, true, false, VNULL, ChMatrix33<>(1), 0.01);
    }
    body->EnableCollision(true);

    // Set and return vehicle initial location
    double init_x = create_ramp ? -ramp_length + 4 : 4;
    ChVector3d init_loc = ramp_rot.Rotate(ChVector3d(init_x, 0, 0.25));

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
    std::vector<ChVector3d> points;

    if (create_ramp) {
        // Include additional point if creating a ramp
        auto np = std::ceil(ramp_length / 10);
        auto dx = (ramp_length - 4) / np;
        for (int i = 0; i < (int)np; i++)
            points.push_back(ChVector3d(i * dx - (ramp_length - 4), 0, 0));
    }

    for (size_t i = 0; i < numPoints; i++) {
        double x, y, z;
        std::getline(ifile, line);
        std::istringstream jss(line);
        jss >> x >> y >> z;
        points.push_back(ChVector3d(x, y, z));
    }

    // Include point beyond SPH patch
    {
        auto np = points.size();
        points.push_back(2.0 * points[np - 1] - points[np - 2]);
    }

    // Raise all path points
    for (auto& p : points)
        p.z() += 0.1;

    ifile.close();

    return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points));
}

std::shared_ptr<ChBody> CreateSentinel(ChSystem& sys, const ChCoordsys<>& init_pos) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->SetPos(init_pos.pos);
    sys.AddBody(body);

    auto box = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    body->AddVisualShape(box, ChFrame<>());

    return body;
}

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos) {
    std::string vehicle_json = "Polaris/Polaris.json";
    std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_json = "Polaris/Polaris_AutomaticTransmisionSimpleMap.json";
    std::string tire_json = "Polaris/Polaris_RigidTire.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
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
    std::string tire_coll_obj = "Polaris/meshes/Polaris_tire_collision.obj";

    ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(vehicle::GetDataFile(tire_coll_obj));
    std::vector<ChVector3d> point_cloud;
    sysFSI.CreateMeshPoints(trimesh, sysFSI.GetInitialSpacing(), point_cloud);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            sysFSI.AddFsiBody(wheel->GetSpindle());
            sysFSI.AddPointsBCE(wheel->GetSpindle(), point_cloud, ChFrame<>(), true);
        }
    }
}
