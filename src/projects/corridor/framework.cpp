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
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLineBezier.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "framework.h"
#include "sedanAV.h"
#include "truckAV.h"
#include "vanAV.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace av {

const double Framework::m_vertical_offset = 0.3;

// -----------------------------------------------------------------------------

Framework::Framework(const Scene& scene, bool render_coll)
    : m_scene(scene),
      m_render_coll(render_coll),
      m_step(1e-3),
      m_render(true),
      m_verbose(false),
      m_initialized(false) {
    m_system = new ChSystemNSC();
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    m_system->GetSolver()->AsIterative()->SetMaxIterations(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);
    m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    CreateTerrain();
}

Framework::~Framework() {
    delete m_terrain;
    delete m_system;
}

ChVector3d Framework::GetLocation(const GPScoord& gps) const {
    auto loc = m_scene.FromGPS(gps);
    auto h = m_terrain->GetHeight(ChVector3d(loc.x(), loc.y(), 0));
    return ChVector3d(loc.x(), loc.y(), h);
}

// Note: cannot use chrono_types::make_shared because the various constructors are not public.

unsigned int Framework::AddPath(std::shared_ptr<chrono::ChBezierCurve> curve) {
    auto id = Agent::GenerateID();
    auto path = std::shared_ptr<Path>(new Path(this, curve));
    path->m_id = id;
    Path::m_paths.insert(std::make_pair(id, path));

    return id;
}

unsigned int Framework::AddPath(const std::vector<GPScoord>& gps_points, bool closed) {
    auto id = Agent::GenerateID();
    auto path = std::shared_ptr<Path>(new Path(this, gps_points, m_vertical_offset, closed));
    path->m_id = id;
    Path::m_paths.insert(std::make_pair(id, path));

    return id;
}

unsigned int Framework::AddPath(const std::vector<chrono::ChVector3d>& points, bool closed) {
    auto id = Agent::GenerateID();
    auto path = std::shared_ptr<Path>(new Path(this, points, m_vertical_offset, closed));
    path->m_id = id;
    Path::m_paths.insert(std::make_pair(id, path));

    return id;
}

// Create a vehicle associated with given path
unsigned int Framework::AddVehicle(Vehicle::Type type,
                                   unsigned int path_id,
                                   const chrono::ChVector3d& loc,
                                   double target_speed) {
    auto path = Path::Find(path_id);

    // Find closest path node to provided location
    auto np = path->m_curve->GetNumPoints();
    size_t i1 = 0;
    auto min_d2 = (path->m_curve->GetPoint(i1) - loc).Length2();
    for (size_t i = 1; i < np; i++) {
        auto crt_d2 = (path->m_curve->GetPoint(i) - loc).Length2();
        if (crt_d2 < min_d2) {
            i1 = i;
            min_d2 = crt_d2;
        }
    }
    auto point = path->m_curve->GetPoint(i1);

    // Find orientation from path chord
    auto u = (i1 < np - 1) ? (path->m_curve->GetPoint(i1 + 1) - point).GetNormalized()
                           : (point - path->m_curve->GetPoint(i1 - 1)).GetNormalized();
    auto v = Vcross(ChVector3d(0, 0, 1), u);
    auto w = Vcross(u, v);
    ChMatrix33<> A(u,v,w);

    // Create vehicle at path node
    if (m_verbose) {
        std::cout << "Add vehicle" << std::endl;
        std::cout << "  " << point.x() << "  " << point.y() << "  " << point.z() << std::endl;
    }
    auto vehicle = AddVehicle(type, ChCoordsys<>(point + ChVector3d(0, 0, 0.5), A.GetQuaternion()));
    if (!vehicle)
        return -1;

    // Disable contact between vehicle chassis and other vehicle components
    vehicle->GetVehicle().SetChassisVehicleCollide(false);

    // Add lidar to vehicle
    vehicle->SetupLidar();

    // Set AV driver
    vehicle->SetupDriver(path->m_curve, target_speed);

    return vehicle->GetId();
}

unsigned int Framework::AddVehicle(Vehicle::Type type,
                                   unsigned int path_id,
                                   const GPScoord& gps_loc,
                                   double target_speed) {
    return AddVehicle(type, path_id, GetLocation(gps_loc), target_speed);
}

std::shared_ptr<Vehicle> Framework::AddVehicle(Vehicle::Type type, const ChCoordsys<>& pos) {
    auto id = Agent::GenerateID();
    std::shared_ptr<Vehicle> vehicle;

    switch (type) {
        case Vehicle::Type::TRUCK:
            vehicle = std::shared_ptr<TruckAV>(new TruckAV(this, id, pos));
            break;
        case Vehicle::Type::VAN:
            vehicle = std::shared_ptr<VanAV>(new VanAV(this, id, pos));
            break;
        case Vehicle::Type::SEDAN:
            vehicle = std::shared_ptr<SedanAV>(new SedanAV(this, id, pos));
            break;
        default:
            std::cout << "Unknown vehicle type" << std::endl;
            return vehicle;
    }

    Vehicle::m_vehicles.push_back(vehicle);
    Agent::m_agents.insert(std::make_pair(id, vehicle));

    return vehicle;
}

unsigned int Framework::AddTrafficLight(const chrono::ChVector3d& center, double radius, const ChCoordsys<>& pos) {
    auto id = Agent::GenerateID();
    auto light = std::shared_ptr<TrafficLight>(new TrafficLight(this, id, center, radius, pos));
    TrafficLight::m_traffic_lights.push_back(light);
    Agent::m_agents.insert(std::make_pair(id, light));

    return id;
}

//// TODO: add angle
unsigned int Framework::AddTrafficLight(const GPScoord& gps_center, double radius, const GPScoord& gps_pos) {
    return AddTrafficLight(GetLocation(gps_center), radius, ChCoordsys<>(GetLocation(gps_pos), QUNIT));
}

void Framework::SetPathColor(unsigned int id, const ChColor& color) {
    auto path = Path::Find(id);
    if (path)
        path->m_color = color;
}

void Framework::SetAgentBroadcast(unsigned int id, double freq, double radius) {
    auto agent = Agent::Find(id);
    if (agent) {
        agent->m_bcast_freq = freq;
        agent->m_bcast_radius = radius;
    }
}

void Framework::SetEgoVehicle(unsigned int id) {
    m_ego_vehicle = std::dynamic_pointer_cast<Vehicle>(Agent::Find(id));
}

void Framework::Run(double time_end, int fps, bool real_time) {
    Initialize();

    double time = 0;
    double next_draw = 0;
    double delta = 1.0 / fps;
    ChTimer timer;

    timer.start();
    while (time <= time_end) {
        if (m_render) {
            if (!m_vis->Run())
                break;
            if (time >= next_draw) {
                m_vis->BeginScene();
                m_vis->Render();
                m_vis->EndScene();
                next_draw += delta;
            }
        }

        Advance();
        time = m_system->GetChTime();

        if (real_time)
            while (timer() < time) {
            }
    }
    timer.stop();

    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Run time:        " << timer() << std::endl;
}

void Framework::CreateTerrain() {
    m_terrain = new RigidTerrain(m_system);

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(m_system->GetContactMethod());

    auto patch = m_terrain->AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), GetChronoDataFile(m_scene.m_coll_file),
                                     0.01, m_render_coll);

    auto aabb = patch->GetGroundBody()->GetCollisionModel()->GetBoundingBox();
    std::cout << "Scene Bounding Box" << std::endl;
    std::cout << "  " << aabb.min.x() << "  " << aabb.min.y() << "  " << aabb.min.z() << std::endl;
    std::cout << "  " << aabb.max.x() << "  " << aabb.max.y() << "  " << aabb.max.z() << std::endl;

    if (m_render_coll) {
        patch->GetGroundBody()->GetVisualShape(0)->SetTexture(GetChronoDataFile("concrete.jpg"), 1, 1);
    }

    m_terrain->Initialize();
}

void Framework::Initialize() {
    if (m_initialized)
        return;

    // Create visualization assets for all paths
    auto road = chrono_types::make_shared<ChBody>();
    road->SetFixed(true);
    m_system->AddBody(road);
    for (auto p : Path::GetList()) {
        auto n = static_cast<unsigned int>(p.second->m_curve->GetNumPoints());
        auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
        path_asset->SetLineGeometry(chrono_types::make_shared<ChLineBezier>(p.second->m_curve));
        path_asset->SetColor(p.second->m_color);
        path_asset->SetName("path_" + std::to_string(p.first));
        path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * n, 400));
        road->AddVisualShape(path_asset);
    }

    // Create visualization assets for all traffic lights
    for (auto t : TrafficLight::GetList()) {
        auto origin = ChCoordsys<>(t->GetCenter() + ChVector3d(0, 0, 2 * m_vertical_offset), QUNIT);
        auto circle_line = chrono_types::make_shared<ChLineArc>(origin, t->GetRadius());
        auto circle_asset = chrono_types::make_shared<ChVisualShapeLine>();
        circle_asset->SetColor(ChColor(1.0f, 0.0f, 0.0f));
        circle_asset->SetName("circle_" + std::to_string(t->GetId()));
        circle_asset->SetLineGeometry(circle_line);
        road->AddVisualShape(circle_asset);
    }

    // Create Irrlicht visualization app
    if (!m_ego_vehicle)
        m_ego_vehicle = Vehicle::GetList()[0];

    if (m_render) {
        auto pos = m_ego_vehicle->GetPosition().pos;
        auto pos1 = pos + ChVector3d(30, -30, 100);
        auto pos2 = pos + ChVector3d(30, +30, 100);

        m_vis = chrono_types::make_shared<IrrApp>(this);
        m_vis->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
        m_vis->Initialize();
        m_vis->AddLogo();
        m_vis->AddTypicalLights();

        // Create scene visualization mesh
        if (!m_render_coll) {
            auto smanager = m_vis->GetSceneManager();
            auto imesh = smanager->getMesh(GetChronoDataFile(m_scene.m_vis_file).c_str());

            {
                // Take care of left-handed frames in Irrlicht

                const irr::u32 bcount = imesh->getMeshBufferCount();
                for (irr::u32 b = 0; b < bcount; ++b) {
                    irr::scene::IMeshBuffer* buffer = imesh->getMeshBuffer(b);
                    const irr::u32 idxcnt = buffer->getIndexCount();
                    irr::u16* idx = buffer->getIndices();
                    irr::s32 tmp;

                    for (irr::u32 i = 0; i < idxcnt; i += 3) {
                        tmp = idx[i + 1];
                        idx[i + 1] = idx[i + 2];
                        idx[i + 2] = tmp;
                    }
                    const irr::u32 vertcnt = buffer->getVertexCount();
                    for (irr::u32 i = 0; i < vertcnt; i++) {
                        buffer->getPosition(i).X = -buffer->getPosition(i).X;  // mirror vertex
                        irr::core::vector3df oldnorm = buffer->getNormal(i);
                        buffer->getNormal(i).X = -oldnorm.X;  // mirrors normal on X
                    }
                }
            }

            auto node = smanager->addMeshSceneNode(imesh, 0, -1, irr::core::vector3df(0, 0, 0),
                                                   irr::core::vector3df(0, 0, 0), irr::core::vector3df(1, 1, 1));
        }

        // Complete Irrlicht asset construction
        m_vis->AttachVehicle(&m_ego_vehicle->GetVehicle());
    }

    m_initialized = true;
}

void Framework::Advance() {
    double time = m_system->GetChTime();

    // send messages
    for (auto a : Agent::GetList()) {
        a.second->SendMessages(time);
    }

    // process messages
    for (auto a : Agent::GetList()) {
        a.second->ProcessMessages();
    }

    // synchronize agents and other objects
    m_terrain->Synchronize(time);
    for (auto a : Agent::GetList()) {
        a.second->Synchronize(time);
    }
    if (m_render) {
        std::string msg = m_ego_vehicle->GetTypeName() + std::to_string(m_ego_vehicle->GetId());
        m_vis->Synchronize(time, m_ego_vehicle->m_driver_inputs);
    }

    // advance state of agents and other objects
    m_terrain->Advance(m_step);
    for (auto a : Agent::GetList()) {
        a.second->Advance(m_step);
    }
    if (m_render) {
        m_vis->Advance(m_step);
    }

    // advance state of Chrono system
    m_system->DoStepDynamics(m_step);
}

void Framework::ListAgents() {
    std::cout << "\nList of traffic agents" << std::endl;
    std::cout << "  Vehicles" << std::endl;
    for (auto v : Vehicle::GetList()) {
        auto pos = v->GetPosition().pos;
        std::cout << "    " << v->GetId() << "  " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
    }

    std::cout << "  Lights" << std::endl;
    for (auto l : TrafficLight::GetList()) {
        auto pos = l->GetPosition().pos;
        std::cout << "    " << l->GetId() << "  " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
    }
}

}  // end namespace av
