//
// Created by Rainer Gericke on 16.04.24.
//

#include "Cherokee_TMeasyTire.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string Cherokee_TMeasyTire::m_meshFile_left = "jeep/Cherokee_Tire.obj";
const std::string Cherokee_TMeasyTire::m_meshFile_right = "jeep/Cherokee_Tire.obj";

const double Cherokee_TMeasyTire::m_mass = 13.78;
const ChVector3d Cherokee_TMeasyTire::m_inertia(0.8335, 1.5513, 0.8335);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Cherokee_TMeasyTire::Cherokee_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Cherokee_TMeasyTire::SetTMeasyParams() {
    const double in2m = 0.0254;
    double w = 0.225;
    double r = 0.75;
    double rimdia = 15.0 * in2m;

    int li = 102;
    double load = GetTireMaxLoad(li);

    GuessPassCar70Par(load,   // tire load [N]
                    w,      // tire width [m]
                    r,      // aspect ratio []
                    rimdia  // rim diameter [m]
    );
}

void Cherokee_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/275_80_R20" + GetName() + ".gpl";
    WritePlots(filename, "275/80 R20 128K");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Cherokee_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void Cherokee_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono