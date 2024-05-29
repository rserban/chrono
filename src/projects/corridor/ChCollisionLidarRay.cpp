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
#include "chrono/collision/ChCollisionSystem.h"

ChCollisionLidarRay::ChCollisionLidarRay(std::shared_ptr<chrono::ChBody> parent, bool visualize){
	this->parent = parent;	//save reference to the ChSystem
	this->visualize = visualize;
	//this is set to -1 for now since they have not been used
	this->contactLen = -1;
}

ChCollisionLidarRay::~ChCollisionLidarRay(){

}

// update the ray collision
// this checks the ray and sets the appropriate ray length and hit object name
void ChCollisionLidarRay::Update(bool updateCollision) {
    this->globalStartPos = this->parent->GetPos() + this->parent->GetRot().Rotate(relativeStartPos);
    this->globalEndPos = this->parent->GetPos() + this->parent->GetRot().Rotate(relativeEndPos);

    if (updateCollision) {
        // see if the ray collides with anything
        chrono::ChCollisionSystem::ChRayhitResult rayCollision;
        this->parent->GetSystem()->GetCollisionSystem()->RayHit(this->globalStartPos, this->globalEndPos, rayCollision);
        if (rayCollision.hit) {
            // if the ray collides, set the length accordingly
            this->SetLength((rayCollision.abs_hitPoint - this->globalStartPos).Length());
        }
    }
    if (visualize) {
        rayEnd->SetPos(globalEndPos);
        rayStart->SetPos(globalStartPos);
    }
}

// set the start and end point of the ray
void ChCollisionLidarRay::SetPoints(const chrono::ChVector3d &posStart,
		const chrono::ChVector3d &posEnd){
	//set the global start and end points
	this->relativeStartPos = posStart;
	this->relativeEndPos = posEnd;
	this->globalStartPos = this->parent->GetPos() + this->parent->GetRot().Rotate(posStart);
	this->globalEndPos = this->parent->GetPos() + this->parent->GetRot().Rotate(posEnd);

	if(visualize){
		//insert a small sphere at the end of each ray for debugging purposes
		rayEnd = chrono_types::make_shared<chrono::ChBodyEasySphere>(.02, 3000, false, true);
		rayEnd->SetPos(chrono::ChVector3d(this->globalEndPos));
		rayEnd->SetFixed(true);
		this->parent->GetSystem()->Add(rayEnd);
		rayEnd->GetVisualShape(0)->SetColor(chrono::ChColor(0.1f, 1.0f, .1f));

		rayStart = chrono_types::make_shared<chrono::ChBodyEasySphere>(.02, 3000, false, true);
		rayStart->SetPos(chrono::ChVector3d(this->globalStartPos));
		rayStart->SetFixed(true);
		this->parent->GetSystem()->Add(rayStart);
        rayStart->GetVisualShape(0)->SetColor(chrono::ChColor(0.1f, 0.1f, 1.0f));
	}
}

// set the length of the ray
void ChCollisionLidarRay::SetLength(double len){
	//set the length to the contact
	this->contactLen = len;

	chrono::ChVector3d dir = this->relativeEndPos - this->relativeStartPos;
	dir.Normalize();

	// this was in gazebo RayShape or BulletRayShape but causes the end position to be incorrect the next time around
	this->relativeEndPos = dir*len + this->relativeStartPos;
}

// returns the length of the ray
double ChCollisionLidarRay::GetLength() const{
	//return the length to contact
	return this->contactLen;

}
