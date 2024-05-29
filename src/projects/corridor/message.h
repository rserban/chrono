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

#ifndef AV_MESSAGE_H
#define AV_MESSAGE_H

#include <string>

#include "chrono/core/ChVector3.h"

namespace av {

class Message {
  public:
    enum Type { MAP, SPAT, VEH };

    Type type;
    unsigned int senderID;
    float time;
};

class MessageMAP : public Message {
  public:
    unsigned int intersectionID;
    short int num_lanes;
};

class MessageSPAT : public Message {
  public:
    short int phase;
    float time_phase;
};

class MessageVEH : public Message {
  public:
    chrono::ChVector3d location;
};

}  // end namespace av

#endif
