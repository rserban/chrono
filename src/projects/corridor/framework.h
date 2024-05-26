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
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#ifndef AV_FRAMEWORK_H
#define AV_FRAMEWORK_H

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "irrapp.h"
#include "path.h"
#include "scene.h"
#include "traffic_light.h"
#include "vehicle.h"

namespace av {

class Framework {
  public:
    Framework(const Scene& scene, bool render_coll = false);
    ~Framework();

    unsigned int AddPath(std::shared_ptr<chrono::ChBezierCurve> curve);
    unsigned int AddPath(const std::vector<GPScoord>& gps_points, bool closed);
    unsigned int AddPath(const std::vector<chrono::ChVector3d>& points, bool closed);

    void SetPathColor(unsigned int id, const chrono::ChColor& color);

    unsigned int AddVehicle(Vehicle::Type type,
                            unsigned int path_id,
                            const chrono::ChVector3d& loc,
                            double target_speed);
    unsigned int AddVehicle(Vehicle::Type type, unsigned int path_id, const GPScoord& gps_loc, double target_speed);

    unsigned int AddTrafficLight(const chrono::ChVector3d& center, double radius, const chrono::ChCoordsys<>& pos);
    unsigned int AddTrafficLight(const GPScoord& gps_center, double radius, const GPScoord& gps_pos);

    void SetAgentBroadcast(unsigned int id, double freq, double radius);

    void SetEgoVehicle(unsigned int vehicle_id);
    void SetIntegrationStep(double step) { m_step = step; }

    void SetVerbose(bool verbose) { m_verbose = verbose; }
    bool Verbose() const { return m_verbose; }

    void SetRender(bool render) { m_render = render; }
    bool Render() const { return m_render; }

    void Run(double time_end, int fps = 30, bool real_time = true);

    chrono::ChSystem* GetSystem() { return m_system; }
    chrono::ChVector3d GetLocation(const GPScoord& gps) const;

    void ListAgents();

  private:
    void CreateTerrain();
    void Initialize();
    void Advance();

    std::shared_ptr<Vehicle> AddVehicle(Vehicle::Type type, const chrono::ChCoordsys<>& pos);

    Scene m_scene;
    chrono::ChSystem* m_system;
    std::shared_ptr<IrrApp> m_vis;
    chrono::vehicle::RigidTerrain* m_terrain;

    double m_step;
    bool m_initialized;
    bool m_render_coll;
    bool m_verbose;
    bool m_render;

    std::shared_ptr<Vehicle> m_ego_vehicle;

    static const double m_vertical_offset;

    friend class TruckAV;
    friend class VanAV;
    friend class SedanAV;
    friend class TrafficLight;
    friend class IrrApp;
};

}  // end namespace av

#endif
