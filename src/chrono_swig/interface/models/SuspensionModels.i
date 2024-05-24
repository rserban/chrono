%{

/* Includes additional C++ in the wrapper code */

#include <string>
#include <vector>
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChFrame.h"
#include "chrono/assets/ChColor.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/physics/ChShaft.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

#include "chrono_models/vehicle/hmmwv/suspension/HMMWV_DoubleWishbone.h"
#include "chrono_models/vehicle/hmmwv/suspension/HMMWV_DoubleWishboneReduced.h"

#include "chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
#include "chrono_models/vehicle/sedan/Sedan_MultiLink.h"

#include "chrono_models/vehicle/citybus/CityBus_ToeBarLeafspringAxle.h"
#include "chrono_models/vehicle/citybus/CityBus_SolidAxle.h"
#include "chrono_models/vehicle/citybus/CityBus_LeafspringAxle.h"

#include "chrono_models/vehicle/man/suspension/MAN_5t_BellcrankSolid3LinkAxle.h"
#include "chrono_models/vehicle/man/suspension/MAN_5t_Solid3LinkAxle.h"
#include "chrono_models/vehicle/man/suspension/MAN_10t_Front1Axle.h"
#include "chrono_models/vehicle/man/suspension/MAN_10t_Front2Axle.h"

#include "chrono_models/vehicle/uaz/UAZBUS_ToeBarLeafspringAxle.h"
#include "chrono_models/vehicle/uaz/UAZBUS_LeafspringAxle.h"

#include "chrono_models/vehicle/gator/Gator_SingleWishbone.h"
#include "chrono_models/vehicle/gator/Gator_RigidSuspension.h"

#include "chrono_models/vehicle/artcar/ARTcar_DoubleWishbone.h"

#include "chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"

#include "chrono_models/vehicle/bmw/BMW_E90_MacPhersonStrut.h"
#include "chrono_models/vehicle/bmw/BMW_E90_DoubleWishbone.h"
%}

%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishbone)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneReduced)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneRear)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneReducedRear)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneFront)
%shared_ptr(chrono::vehicle::hmmwv::HMMWV_DoubleWishboneReducedFront)

%shared_ptr(chrono::vehicle::sedan::Sedan_DoubleWishbone)
%shared_ptr(chrono::vehicle::sedan::Sedan_MultiLink)

%shared_ptr(chrono::vehicle::citybus::CityBus_ToeBarLeafspringAxle)
%shared_ptr(chrono::vehicle::citybus::CityBus_SolidAxleFront)
%shared_ptr(chrono::vehicle::citybus::CityBus_SolidAxleRear)
%shared_ptr(chrono::vehicle::citybus::CityBus_LeafspringAxle)

%shared_ptr(chrono::vehicle::man::MAN_5t_BellcrankSolid3LinkAxle)
%shared_ptr(chrono::vehicle::man::MAN_5t_Solid3LinkAxle)
%shared_ptr(chrono::vehicle::man::MAN_10t_Front1Axle)
%shared_ptr(chrono::vehicle::man::MAN_10t_Front2Axle)

%shared_ptr(chrono::vehicle::uaz::UAZBUS_ToeBarLeafspringAxle)
%shared_ptr(chrono::vehicle::uaz::UAZBUS_LeafspringAxle)

%shared_ptr(chrono::vehicle::gator::Gator_SingleWishbone)
%shared_ptr(chrono::vehicle::gator::Gator_RigidSuspension)

%shared_ptr(chrono::vehicle::artcar::ARTcar_DoubleWishbone)
%shared_ptr(chrono::vehicle::artcar::ARTcar_DoubleWishboneRear)
%shared_ptr(chrono::vehicle::artcar::ARTcar_DoubleWishboneFront)

%shared_ptr(chrono::vehicle::feda::FEDA_DoubleWishbone)
%shared_ptr(chrono::vehicle::feda::FEDA_DoubleWishboneRear)
%shared_ptr(chrono::vehicle::feda::FEDA_DoubleWishboneFront)

%shared_ptr(chrono::vehicle::bmw::BMW_E90_MacPhersonStrut)
%shared_ptr(chrono::vehicle::bmw::BMW_E90_DoubleWishbone)

/* Parse the header file to generate wrappers */
%import "chrono_swig/interface/vehicle/ChSuspension.i"

// Model:
%include "../../../chrono_models/vehicle/hmmwv/suspension/HMMWV_DoubleWishbone.h"
%include "../../../chrono_models/vehicle/hmmwv/suspension/HMMWV_DoubleWishboneReduced.h"

%include "../../../chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
%include "../../../chrono_models/vehicle/sedan/Sedan_MultiLink.h"

%include "../../../chrono_models/vehicle/citybus/CityBus_ToeBarLeafspringAxle.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_SolidAxle.h"
%include "../../../chrono_models/vehicle/citybus/CityBus_LeafspringAxle.h"

%include "../../../chrono_models/vehicle/man/suspension/MAN_5t_BellcrankSolid3LinkAxle.h"
%include "../../../chrono_models/vehicle/man/suspension/MAN_5t_Solid3LinkAxle.h"
%include "../../../chrono_models/vehicle/man/suspension/MAN_10t_Front1Axle.h"
%include "../../../chrono_models/vehicle/man/suspension/MAN_10t_Front2Axle.h"

%include "../../../chrono_models/vehicle/uaz/UAZBUS_ToeBarLeafspringAxle.h"
%include "../../../chrono_models/vehicle/uaz/UAZBUS_LeafspringAxle.h"

%include "../../../chrono_models/vehicle/gator/Gator_SingleWishbone.h"
%include "../../../chrono_models/vehicle/gator/Gator_RigidSuspension.h"

%include "../../../chrono_models/vehicle/artcar/ARTcar_DoubleWishbone.h"

%include "../../../chrono_models/vehicle/feda/FEDA_DoubleWishbone.h"

%include "../../../chrono_models/vehicle/bmw/BMW_E90_MacPhersonStrut.h"
%include "../../../chrono_models/vehicle/bmw/BMW_E90_DoubleWishbone.h"
