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

#ifndef AV_VEHICLE_H
#define AV_VEHICLE_H

#include "chrono/core/ChBezierCurve.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

#include "ChCollisionLidar.h"
#include "agent.h"
#include "message.h"

namespace av {

class Vehicle;

typedef std::vector<std::shared_ptr<Vehicle>> VehicleList;

class Vehicle : public Agent {
  public:
    enum Type { TRUCK, SEDAN, VAN, BUS };

    virtual ~Vehicle();

    Type GetType() const { return m_vehicle_type; }
    const std::string& GetTypeName() const { return m_types[m_vehicle_type]; }

    virtual chrono::ChCoordsys<> GetPosition() const override;
    virtual void Broadcast(double time) override;
    virtual void Unicast(double time) override;
    virtual void ProcessMessages() override;

    virtual chrono::vehicle::ChWheeledVehicle& GetVehicle() const = 0;

    virtual double GetLookAheadDistance() const = 0;
    virtual chrono::ChVector3d GetSteeringGainsPID() const = 0;
    virtual chrono::ChVector3d GetSpeedGainsPID() const = 0;

    virtual chrono::ChCoordsys<> GetLidarPosition() const = 0;

    static VehicleList GetList() { return m_vehicles; }

  protected:
    Vehicle(Framework* framework, unsigned int id, Type vehicle_type);

    void ProcessMessageMAP(std::shared_ptr<MessageMAP> msg);
    void ProcessMessageSPAT(std::shared_ptr<MessageSPAT> msg);
    void ProcessMessageVEH(std::shared_ptr<MessageVEH> msg);

    void SetupDriver(std::shared_ptr<chrono::ChBezierCurve> curve, double target_speed);
    void AdvanceDriver(double step);

    void SetupLidar();

    chrono::vehicle::DriverInputs m_driver_inputs;

  private:
    Type m_vehicle_type;

    std::shared_ptr<MessageVEH> m_vehicle_msg;  ///< outgoing VEH message

    std::shared_ptr<ChCollisionLidar> m_lidar;

    unsigned int m_MAP_id;   ///< current MAP intersection ID
    short int m_SPAT_phase;  ///< current SPaT signal phase
    float m_SPAT_time;       ///< current SPaT signal timing

    chrono::vehicle::ChPathSteeringController* m_steeringPID;
    chrono::vehicle::ChSpeedController* m_speedPID;
    double m_target_speed;

    static const double m_throttle_threshold;
    static const std::string m_types[4];
    static VehicleList m_vehicles;

    friend class Framework;
    friend class IrrApp;
};

}  // end namespace av

#endif
