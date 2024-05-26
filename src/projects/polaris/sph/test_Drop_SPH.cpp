// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Deniz Tanyildiz, Radu Serban
// =============================================================================
//
// Object drop on SPH terrain
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/visualization/ChFsiVisualizationGL.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "DataWriter.h"
#include "CreateObjects.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

const ChVector3d gravity(0, 0, -9.81);

double sphere_radius = 0.3;

// ===================================================================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& terrain_dir,
                     double& tend,
                     double& step_size,
                     double& active_box_dim,
                     double& output_major_fps,
                     double& output_minor_fps,
                     int& output_frames,
                     int& particle_output,
                     double& filter_window_vel,
                     double& filter_window_acc,
                     double& vis_output_fps,
                     bool& run_time_vis,
                     bool& run_time_vis_particles,
                     bool& run_time_vis_bce,
                     double& run_time_vis_fps,
                     bool& verbose);

// ===================================================================================================================

std::shared_ptr<chrono::ChBody> CreateSolidPhase(chrono::ChSystemNSC& sys,
                                                 chrono::fsi::ChSystemFsi& sysFSI,
                                                 const chrono::ChCoordsys<>& init_pos) {
    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(1e8);
    mysurfmaterial->SetFriction(0.2f);
    mysurfmaterial->SetRestitution(0.05f);
    mysurfmaterial->SetAdhesion(0);

    // Get particle spacing in the simulation
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Create a falling sphere
    auto sphere = chrono_types::make_shared<ChBody>();

    // Set the general properties of the sphere
    double volume = ChSphere::GetVolume(sphere_radius);
    // Carbon steel's density is about 7.84 g/cm3=7840 kg/m3
    // double density = sysFSI.GetDensity() * 2.0;
    double density = 7840;
    double mass = density * volume;
    ChVector3d sphere_pos = ChVector3d(2.0, 0, 0.2 + sphere_radius + 2 * initSpace0);
    ChVector3d sphere_vel = ChVector3d(0.0, 0.0, 0.0);
    ChQuaternion<> sphere_rot = QUNIT;
    ChVector3d gyration = ChSphere::GetGyration(sphere_radius).diagonal();
    sphere->SetPos(sphere_pos);
    sphere->SetPosDt(sphere_vel);
    sphere->SetMass(mass);
    sphere->SetInertiaXX(mass * gyration);

    // Set the collision type of the sphere
    sphere->EnableCollision(true);
    sphere->SetFixed(false);
    sphere->GetCollisionModel()->SetSafeMargin(initSpace0);
    chrono::utils::AddSphereGeometry(sphere.get(), mysurfmaterial, sphere_radius, VNULL, sphere_rot);

    // Add this body to chrono system
    sys.AddBody(sphere);

    // Add this body to the FSI system (only those have interaction with fluid)
    sysFSI.AddFsiBody(sphere);

    // Add BCE particles attached on the sphere into FSI system
    sysFSI.AddSphereBCE(sphere, ChFrame<>(), sphere_radius, true);

    return sphere;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string terrain_dir;
    double tend = 30;
    double step_size = 5e-4;
    double active_box_dim = 0.5;
    double output_major_fps = 20;
    double output_minor_fps = 1000;
    int output_frames = 5;
    int particle_output = 2;              // output all particle info
    double filter_window_vel = 0;         // do not filter velocity data
    double filter_window_acc = 0;         // do not filter acceleration data
    double vis_output_fps = 0;            // no post-processing visualization output
    bool run_time_vis = false;            // no run-time visualization
    double run_time_vis_fps = 0;          // render every simulation frame
    bool run_time_vis_particles = false;  // render only terrain surface mesh
    bool run_time_vis_bce = false;        // render object
    bool verbose = true;

    if (!GetProblemSpecs(argc, argv, terrain_dir, tend, step_size, active_box_dim, output_major_fps, output_minor_fps,
                         output_frames, particle_output, filter_window_vel, filter_window_acc, vis_output_fps,
                         run_time_vis, run_time_vis_particles, run_time_vis_bce, run_time_vis_fps, verbose)) {
        return 1;
    }

    bool sim_output = (output_major_fps > 0);
    bool use_filter_vel = (filter_window_vel > 0);
    bool use_filter_acc = (filter_window_acc > 0);
    bool vis_output = (vis_output_fps > 0);

    DataWriter::ParticleOutput output_level = DataWriter::ParticleOutput::ALL;
    if (particle_output == 0)
        output_level = DataWriter::ParticleOutput::NONE;
    else if (particle_output == 1)
        output_level = DataWriter::ParticleOutput::POSITIONS;

    // Check input files exist
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/sph_params.json")).exists()) {
        std::cout << "Input file sph_params.json not found in directory " << terrain_dir << std::endl;
        return 1;
    }
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/particles_20mm.txt")).exists()) {
        std::cout << "Input file particles_20mm.txt not found in directory " << terrain_dir << std::endl;
        return 1;
    }
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/bce_20mm.txt")).exists()) {
        std::cout << "Input file bce_20mm.txt not found in directory " << terrain_dir << std::endl;
        return 1;
    }

    // Create the Chrono systems
    ChSystemNSC sys;
    // ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sys);

    // Load SPH parameter file
    cout << "Load SPH parameter file..." << endl;
    sysFSI.ReadParametersFromFile(vehicle::GetDataFile(terrain_dir + "/sph_params.json"));

    sysFSI.SetActiveDomain(ChVector3d(active_box_dim, active_box_dim, 1));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);
    sysFSI.SetVerbose(false);

    // Set simulation data output and FSI information output
    std::string out_dir = GetChronoOutputPath() + "SPHEREDROP_SPH/";
    std::string vis_dir = out_dir + "Visualization/";
    sysFSI.SetOutputLength(0);
    ////sysFSI.SetOutputDirectory(out_dir);

    sys.SetGravitationalAcceleration(gravity);

    // Create terrain
    cout << "Create terrain..." << endl;
    bool terrain_mesh_contact = false;
    auto init_pos = CreateTerrain(sys, sysFSI, terrain_dir, 0.0, !run_time_vis_particles, terrain_mesh_contact);

    // Create sphere and its BCE markers
    cout << "Create sphere..." << endl;
    auto sphere = CreateSolidPhase(sys, sysFSI, init_pos);

#ifdef CHRONO_OPENGL
    // Create run-time visualization
    opengl::ChVisualSystemOpenGL vis;
    ChFsiVisualizationGL visFSI(&sysFSI, &vis);
    if (run_time_vis) {
        visFSI.SetTitle("Object drop test");
        visFSI.SetSize(1280, 720);
        visFSI.UpdateCamera(init_pos.pos + ChVector3d(-7, 0, 6), init_pos.pos + ChVector3d(1, 0, 0.5));
        visFSI.SetCameraMoveScale(1.0f);
        visFSI.EnableFluidMarkers(run_time_vis_particles);
        visFSI.EnableRigidBodyMarkers(run_time_vis_bce);
        visFSI.EnableBoundaryMarkers(false);
        visFSI.SetRenderMode(ChFsiVisualizationGL::RenderMode::SOLID);

        vis.AttachSystem(&sys);
        vis.Initialize();
    }
#endif

    // Enable data output
    cout << "===============================================================================" << endl;
    auto sim_dir = out_dir + filesystem::path(terrain_dir).filename() + "/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(sim_dir))) {
        cout << "Error creating directory " << sim_dir << endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(vis_dir))) {
        cout << "Error creating directory " << vis_dir << endl;
        return 1;
    }

    DataWriterObject data_writer(sysFSI, sphere, ChVector3d(2 * sphere_radius));
    data_writer.SetVerbose(verbose);
    data_writer.SetParticleOutput(output_level);
    data_writer.UseFilteredVelData(use_filter_vel, filter_window_vel);
    data_writer.UseFilteredAccData(use_filter_acc, filter_window_acc);
    data_writer.Initialize(sim_dir, output_major_fps, output_minor_fps, output_frames, step_size);
    cout << "Simulation output data saved in: " << sim_dir << endl;
    cout << "===============================================================================" << endl;

    // Simulation loop
    int vis_output_steps = (int)std::round((1.0 / vis_output_fps) / step_size);
    int render_steps = (run_time_vis_fps > 0) ? (int)std::round((1.0 / run_time_vis_fps) / step_size) : 1;
    int vis_output_frame = 0;

    bool on_ramp = true;

    double t = 0;
    int frame = 0;
    sysFSI.Initialize();
#ifdef CHRONO_OPENGL
    visFSI.Initialize();
#endif

    while (t < tend) {
        // Simulation data output
        if (sim_output)
            data_writer.Process(frame, t);

        // Visualization data output
        if (vis_output && frame % vis_output_steps == 0) {
            if (verbose)
                cout << "Visualization output frame = " << vis_output_frame << endl;
            sysFSI.PrintParticleToFile(vis_dir);

            vis_output_frame++;
        }

#ifdef CHRONO_OPENGL
        // Run-time visualization
        if (run_time_vis && frame % render_steps == 0) {
            if (!visFSI.Render())
                break;
        }
#endif

        // Advance both FSI and embedded MBD systems
        sysFSI.DoStepDynamics_FSI();
        t += step_size;

        frame++;
    }

    return 0;
}

// ===================================================================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& terrain_dir,
                     double& tend,
                     double& step_size,
                     double& active_box_dim,
                     double& output_major_fps,
                     double& output_minor_fps,
                     int& output_frames,
                     int& particle_output,
                     double& filter_window_vel,
                     double& filter_window_acc,
                     double& vis_output_fps,
                     bool& run_time_vis,
                     bool& run_time_vis_particles,
                     bool& run_time_vis_bce,
                     double& run_time_vis_fps,
                     bool& verbose) {
    ChCLI cli(argv[0], "Polaris SPH terrain simulation");

    cli.AddOption<std::string>("Simulation", "terrain_dir", "Directory with terrain specification data");
    cli.AddOption<double>("Simulation", "tend", "Simulation end time [s]", std::to_string(tend));
    cli.AddOption<double>("Simulation", "step_size", "Integration step size [s]", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "active_box_dim", "Active box half-size [m]",
                          std::to_string(active_box_dim));

    cli.AddOption<double>("Simulation output", "output_major_fps", "Simulation output major frequency [fps]",
                          std::to_string(output_major_fps));
    cli.AddOption<double>("Simulation output", "output_minor_fps", "Simulation output major frequency [fps]",
                          std::to_string(output_minor_fps));
    cli.AddOption<int>("Simulation output", "output_frames", "Successive output frames",
                       std::to_string(output_frames));
    cli.AddOption<int>("Simulation output", "particle_output", "Particle output (0: none, 1: pos, 2: all)",
                       std::to_string(particle_output));
    cli.AddOption<double>("Simulation output", "filter_window_vel", "Running average velocity filter window [s]",
                          std::to_string(filter_window_vel));
    cli.AddOption<double>("Simulation output", "filter_window_acc", "Running average acceleration filter window [s]",
                          std::to_string(filter_window_acc));

    cli.AddOption<bool>("", "quiet", "Disable all messages during simulation");

    cli.AddOption<double>("Visualization", "vis_output_fps", "Visualization output frequency [fps]",
                          std::to_string(vis_output_fps));
    cli.AddOption<bool>("Visualization", "run_time_vis", "Enable run-time visualization");
    cli.AddOption<bool>("Visualization", "run_time_vis_particles", "Enable run-time particle visualization");
    cli.AddOption<bool>("Visualization", "run_time_vis_bce", "Enable run-time BCE markjer visualization");
    cli.AddOption<double>("Visualization", "run_time_vis_fps", "Run-time visualization frequency [fps]",
                          std::to_string(run_time_vis_fps));

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    try {
        terrain_dir = cli.Get("terrain_dir").as<std::string>();
    } catch (std::domain_error&) {
        cout << "\nERROR: Missing terrain specification directory!\n\n" << endl;
        cli.Help();
        return false;
    }

    output_major_fps = cli.GetAsType<double>("output_major_fps");
    output_minor_fps = cli.GetAsType<double>("output_minor_fps");
    output_frames = cli.GetAsType<int>("output_frames");
    particle_output = cli.GetAsType<int>("particle_output");

    filter_window_vel = cli.GetAsType<double>("filter_window_vel");
    filter_window_acc = cli.GetAsType<double>("filter_window_acc");

    vis_output_fps = cli.GetAsType<double>("vis_output_fps");
    run_time_vis = cli.GetAsType<bool>("run_time_vis");
    run_time_vis_particles = cli.GetAsType<bool>("run_time_vis_particles");
    run_time_vis_bce = cli.GetAsType<bool>("run_time_vis_bce");
    run_time_vis_fps = cli.GetAsType<double>("run_time_vis_fps");

    tend = cli.GetAsType<double>("tend");
    step_size = cli.GetAsType<double>("step_size");
    active_box_dim = cli.GetAsType<double>("active_box_dim");

    verbose = !cli.GetAsType<bool>("quiet");

    return true;
}
