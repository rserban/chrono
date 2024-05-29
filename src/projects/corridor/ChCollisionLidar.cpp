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


#include "ChCollisionLidar.h"

using namespace chrono;

// Constructor for a ChRaySensor
ChCollisionLidar::ChCollisionLidar (std::shared_ptr<ChBody> parent, double updateRate, bool visualize)
	:ChSensor(parent,updateRate,visualize){

}
ChCollisionLidar::~ChCollisionLidar(){
	m_rays.clear();
}

// Initialize the ChRaySensor
void ChCollisionLidar::Initialize(chrono::ChCoordsys<double> offsetPose,
		int aboutYSamples, int aboutZSamples,
		double aboutYMinAngle, double aboutYMaxAngle,
		double aboutZMinAngle, double aboutZMaxAngle,
		double minRange, double maxRange){

	m_minRange = minRange;
	m_maxRange = maxRange;

	if(aboutZSamples < 1) aboutZSamples = 1;
	if(aboutYSamples < 1) aboutYSamples = 1;

	chrono::ChVector3d start, end, axis;
	double yawAngle, pitchAngle;
	chrono::ChQuaternion<double> ray;
	double yDiff = aboutYMaxAngle - aboutYMinAngle;
	double pDiff = aboutZMaxAngle - aboutZMinAngle;

	for (int j = 0; j < aboutZSamples; ++j) {
        for (int i = 0; i < aboutYSamples; ++i) {
            yawAngle = (aboutYSamples == 1) ? 0 : i * yDiff / (aboutYSamples - 1) + aboutYMinAngle;

            pitchAngle = (aboutZSamples == 1) ? 0 : j * pDiff / (aboutZSamples - 1) + aboutZMinAngle;

            // Yaw, Roll, Pitch according to ChQuaternion.h
            ray.SetFromCardanAnglesZYX(chrono::ChVector3d(yawAngle, 0.0, -pitchAngle));

            axis = (offsetPose.rot * ray).Rotate(chrono::ChVector3d(1.0, 0, 0));

            start = (axis * minRange) + offsetPose.pos;
            end = (axis * maxRange) + offsetPose.pos;

            AddRay(start, end);
        }
    }
}

void ChCollisionLidar::UpdateRays(){

}

void ChCollisionLidar::AddRay(const chrono::ChVector3d &start,
		const chrono::ChVector3d &end){
	std::shared_ptr<ChCollisionLidarRay> ray = chrono_types::make_shared<ChCollisionLidarRay>(m_parent, m_visualize);

	ray->SetPoints(start, end);
	m_rays.push_back(ray);

}

double ChCollisionLidar::GetRange(unsigned int index){
	if(index >= m_rays.size()){
		return -1;
	}
	//add min range because we measured from min range
	return m_minRange + m_rays[index]->GetLength();

}

std::vector<double> ChCollisionLidar::Ranges(){
	std::vector<double> ranges = std::vector<double>();
	for(int i=0; i<m_rays.size(); i++){
		ranges.push_back(m_minRange + m_rays[i]->GetLength());
	}
	return ranges;
}

void ChCollisionLidar::Update(){

	double fullRange = m_maxRange - m_minRange;

	bool updateCollisions = false;
	if(m_parent->GetChTime()>=m_timeLastUpdated + 1.0/m_updateRate){
		updateCollisions = true;
		m_timeLastUpdated = m_parent->GetChTime();
		for(int i = 0; i<m_rays.size(); i++){
			m_rays[i]->SetLength(fullRange);
			if(!m_visualize) m_rays[i]->Update(updateCollisions);
		}
	}
	// need to update the rays regardless so that they move with the entity
	// they are attached to
	if(m_visualize){
		for(int i=0; i<m_rays.size(); i++){
			m_rays[i]->Update(updateCollisions);
		}
	}

}
