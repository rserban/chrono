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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - using the ChParticleEmitter to create a cluster of random shapes
// - applying custom force field to particles
// - using Irrlicht to display objects
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::particlefactory;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

//     A callback executed at each particle creation can be attached to the emitter.
//     For example, we need that new particles will be bound to Irrlicht visualization:
class MyCreatorForAll : public ChRandomShapeCreator::AddBodyCallback {
  public:
    virtual void OnAddBody(std::shared_ptr<ChBody> mbody,
                           ChCoordsys<> mcoords,
                           ChRandomShapeCreator& mcreator) override {
        // Set material texture of first visual shape
        mbody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        vis->BindItem(mbody);

        // Dsable gyroscopic forces for increased integrator stability
        mbody->SetNoGyroTorque(true);
    }
    ChVisualSystemIrrlicht* vis;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(ChVector2<int>(800, 600));
    vis->SetWindowTitle("Particle emitter");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 14, -20));

    // Create a rigid body
    auto sphere_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    sphere_mat->SetFriction(0.2f);

    auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(2.1,          // radius size
                                                                   1800,         // density
                                                                   true,         // visualization?
                                                                   true,         // collision?
                                                                   sphere_mat);  // contact material
    msphereBody->SetPos(ChVector<>(1, 1, 0));
    msphereBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
    sys.Add(msphereBody);

    // Creating particles using ChBody or the ChBodyEasyXXYYZZ shortcuts
    // can be enough, e.g. if you put in a for() loop you can create a cluster.
    // However there is an easier way to the creation of cluster of particles,
    // namely the ChEmitter helper class. Here we show hof to use it.

    // Create an emitter:
    ChParticleEmitter emitter;

    // This object will take care of generating particle flows for you.
    // It accepts a lot of settings, for creating many different types of particle
    // flows, like fountains, outlets of various shapes etc.
    // For instance, set the flow rate, etc:

    emitter.ParticlesPerSecond() = 2000;

    emitter.SetUseParticleReservoir(true);
    emitter.ParticleReservoirAmount() = 200;

    // Our ChParticleEmitter object, among the main settings, it requires
    // that you give him four 'randomizer' objects: one is in charge of
    // generating random shapes, one is in charge of generating
    // random positions, one for random alignements, and one for random velocities.
    // In the following we need to instance such objects. (There are many ready-to-use
    // randomizer objects already available in chrono, but note that you could also
    // inherit your own class from these randomizers if the choice is not enough).

    // ---Initialize the randomizer for POSITIONS: random points in a large cube
    auto emitter_positions = chrono_types::make_shared<ChRandomParticlePositionOnGeometry>();

    std::shared_ptr<ChGeometry> sampled_cube(new ChBox(VNULL, ChMatrix33<>(QUNIT), ChVector<>(50, 50, 50)));
    emitter_positions->SetGeometry(sampled_cube);

    emitter.SetParticlePositioner(emitter_positions);

    // ---Initialize the randomizer for ALIGNMENTS
    auto emitter_rotations = chrono_types::make_shared<ChRandomParticleAlignmentUniform>();
    emitter.SetParticleAligner(emitter_rotations);

    // ---Initialize the randomizer for VELOCITIES, with statistical distribution
    auto mvelo = chrono_types::make_shared<ChRandomParticleVelocityAnyDirection>();
    mvelo->SetModulusDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.0, 0.5));
    emitter.SetParticleVelocity(mvelo);

    // ---Initialize the randomizer for ANGULAR VELOCITIES, with statistical distribution
    auto mangvelo = chrono_types::make_shared<ChRandomParticleVelocityAnyDirection>();
    mangvelo->SetModulusDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.0, 0.2));
    emitter.SetParticleAngularVelocity(mangvelo);

    // ---Initialize the randomizer for CREATED SHAPES, with statistical distribution

    /*
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)
    auto mcreator_spheres = chrono_types::make_shared<ChRandomShapeCreatorSpheres>();
    mcreator_spheres->SetDiameterDistribution(chrono_types::make_shared<ChZhangDistribution>(0.6, 0.23));
    mcreator_spheres->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1600));

    // Finally, tell to the emitter that it must use the creator above:
    emitter.SetParticleCreator(mcreator_spheres);
    */

    // ..as an alternative: create odd shapes with convex hulls, like faceted fragments:
    auto mcreator_hulls = chrono_types::make_shared<ChRandomShapeCreatorConvexHulls>();
    mcreator_hulls->SetNpoints(15);
    mcreator_hulls->SetChordDistribution(chrono_types::make_shared<ChZhangDistribution>(1.3, 0.4));
    mcreator_hulls->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1600));
    emitter.SetParticleCreator(mcreator_hulls);

    // --- Optional: what to do by default on ALL newly created particles?
    //     A callback executed at each particle creation can be attached to the emitter.
    //     For example, we need that new particles will be bound to Irrlicht visualization:

    // a- define a class that implement your custom OnAddBody method (see top of source file)
    // b- create the callback object...
    auto mcreation_callback = chrono_types::make_shared<MyCreatorForAll>();
    // c- set callback own data that he might need...
    mcreation_callback->vis = vis.get();
    // d- attach the callback to the emitter!
    emitter.RegisterAddBodyCallback(mcreation_callback);

    // Bind all existing visual shapes to the visualization system
    sys.SetVisualSystem(vis);

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(40);

    // Turn off default -9.8 downward gravity
    sys.Set_G_acc(ChVector<>(0, 0, 0));

    // Simulation loop
    double timestep = 0.01;
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        // Create particle flow
        emitter.EmitParticles(sys, timestep);

        // Apply custom forcefield (brute force approach..)
        // A) reset 'user forces accumulators':
        for (auto body : sys.Get_bodylist()) {
            body->Empty_forces_accumulators();
        }

        // B) store user computed force:
        // double G_constant = 6.674e-11; // gravitational constant
        double G_constant = 6.674e-3;  // gravitational constant - HACK to speed up simulation
        for (unsigned int i = 0; i < sys.Get_bodylist().size(); i++) {
            auto abodyA = sys.Get_bodylist()[i];
            for (unsigned int j = i + 1; j < sys.Get_bodylist().size(); j++) {
                auto abodyB = sys.Get_bodylist()[j];
                ChVector<> D_attract = abodyB->GetPos() - abodyA->GetPos();
                double r_attract = D_attract.Length();
                double f_attract = G_constant * (abodyA->GetMass() * abodyB->GetMass()) / (pow(r_attract, 2));
                ChVector<> F_attract = (D_attract / r_attract) * f_attract;

                abodyA->Accumulate_force(F_attract, abodyA->GetPos(), false);
                abodyB->Accumulate_force(-F_attract, abodyB->GetPos(), false);
            }
        }

        // Perform the integration timestep
        sys.DoStepDynamics(timestep);
    }

    return 0;
}
