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
// Authors: Asher Elmquist
// =============================================================================
//
//
// =============================================================================


#ifndef CHCOLLISIONLIDAR_H
#define CHCOLLISIONLIDAR_H

#include "ChSensor.h"
#include "ChCollisionLidarRay.h"

class ChCollisionLidar : public ChSensor{
public:
	// Constructor
	ChCollisionLidar(std::shared_ptr<chrono::ChBody> parent, double updateRate, bool visualize);

	// Destructor
	~ChCollisionLidar();

	//Initialize the ChRaySensor
	void Initialize(chrono::ChCoordsys<double> offsetPose,
			int horzSamples, int vertSamples,
			double horzMinAngle, double horzMaxAngle,
			double vertMinAngle, double vertMaxAngle,
			double minRange, double maxRange);

	// Update the rays
	void UpdateRays();

	double GetRange(unsigned int index);
	std::vector<double> Ranges();

	void Update();

private:
	void AddRay(const chrono::ChVector3d &start, const chrono::ChVector3d &end);

	std::vector<std::shared_ptr<ChCollisionLidarRay>> m_rays;

	// chrono::ChCoordsys<double> m_offsetPose;

	double m_minRange = -1;
	double m_maxRange = -1;
};

#endif
