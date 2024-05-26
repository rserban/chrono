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
// Test for SCM terrain patch generated from height-map
// =============================================================================

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for tire:
    double tire_rad = 0.8;
    ChVector3d tire_center(-4.0, 0, 0.02 + tire_rad);

    ChSystemSMC my_system;
    my_system.SetGravitationalAcceleration(ChVector3d(0., 0, -9.8));

    std::shared_ptr<ChBody> mtruss(new ChBody);
    mtruss->SetFixed(true);
    my_system.Add(mtruss);

    // Create a rigid body with a cylinder collision shape
    std::shared_ptr<ChBody> mrigidbody(new ChBody);
    my_system.Add(mrigidbody);
    mrigidbody->SetMass(500);
    mrigidbody->SetInertiaXX(ChVector3d(20, 20, 20));
    mrigidbody->SetPos(tire_center + ChVector3d(0, 0, 0.3));

    double radius = 0.5;
    double width = 0.4;

    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    mrigidbody->GetCollisionModel()->AddCylinder(material, radius, radius, width / 2);
    mrigidbody->EnableCollision(true);

    auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, width);
    cyl_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    mrigidbody->AddVisualShape(cyl_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, CH_PI / 4.0));
    motor->Initialize(mrigidbody, mtruss, ChFrame<>(tire_center, QuatFromAngleX(-CH_PI_2)));
    my_system.Add(motor);

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&my_system);

    // Use either a regular grid:
    ////mterrain.Initialize(6, 2, 0.04);

    // Or use a height map:
    mterrain.Initialize(vehicle::GetDataFile("terrain/height_maps/slope.bmp"), 12, 2, 0.0, 0.5, 0.04);

    // Set the soil terramechanical parameters
    mterrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                               0,      // Bekker Kc
                               1.1,    // Bekker n exponent
                               0,      // Mohr cohesive limit (Pa)
                               30,     // Mohr friction limit (degrees)
                               0.01,   // Janosi shear coefficient (m)
                               4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                               3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 30000.2);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);
    mterrain.GetMesh()->SetWireframe(true);

    // Create the Irrlicht visualization
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("SCM test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(2, 2, 2.5));
    vis->AddTypicalLights();
    vis->AttachSystem(&my_system);

    while (vis->Run()) {
        vis->BeginScene();
        vis->GetActiveCamera()->setTarget(irr::core::vector3dfCH(mrigidbody->GetPos()));
        vis->GetActiveCamera()->setPosition(irr::core::vector3dfCH(mrigidbody->GetPos() + ChVector3d(0, 2, 0)));
        vis->Render();
        vis->EndScene();
        my_system.DoStepDynamics(2e-3);
    }

    return 0;
}
