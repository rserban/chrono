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
// Author: Radu Serban
// =============================================================================
//
// Polaris wheeled vehicle on SPH terrain
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/visualization/ChFsiVisualizationGL.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

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

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& terrain_dir,
                     double& density,
                     double& cohesion,
                     double& friction,
                     double& youngs_modulus,
                     double& poisson_ratio,
                     double& ramp_length,
                     double& target_speed,
                     double& tend,
                     double& step_size,
                     double& active_box_dim,
                     double& output_major_fps,
                     double& output_minor_fps,
                     int& output_frames,
                     int& particle_output,
                     bool& wheel_output,
                     double& filter_window_vel,
                     double& filter_window_acc,
                     double& vis_output_fps,
                     bool& run_time_vis,
                     bool& run_time_vis_particles,
                     bool& run_time_vis_bce,
                     double& run_time_vis_fps,
                     bool& chase_cam,
                     bool& verbose);

// ===================================================================================================================

#ifdef CHRONO_OPENGL
class PolarisStats : public opengl::ChOpenGLStats {
  public:
    PolarisStats(const WheeledVehicle& vehicle)
        : ChOpenGLStats(), m_vehicle(vehicle), m_steering(0), m_throttle(0), m_braking(0) {}
    void UpdateDriverInputs(const DriverInputs& inputs) {
        m_steering = inputs.m_steering;
        m_throttle = inputs.m_throttle;
        m_braking = inputs.m_braking;
    }
    virtual void GenerateStats(ChSystem& sys) override {
        char buffer[150];
        sprintf(buffer, "TIME:     %.4f s", m_vehicle.GetChTime());
        text.Render(buffer, screen.LEFT, screen.TOP - 1 * screen.SPACING, screen.SX, screen.SY);

        sprintf(buffer, "Speed:    %.2f m/s (%.1f km/h)", m_vehicle.GetSpeed(), m_vehicle.GetSpeed() * 3.6);
        text.Render(buffer, screen.LEFT, screen.TOP - 3 * screen.SPACING, screen.SX, screen.SY);
        sprintf(buffer, "Steering: %.2f", m_steering);
        text.Render(buffer, screen.LEFT, screen.TOP - 4 * screen.SPACING, screen.SX, screen.SY);
        sprintf(buffer, "Throttle: %.2f", m_throttle);
        text.Render(buffer, screen.LEFT, screen.TOP - 5 * screen.SPACING, screen.SX, screen.SY);
        sprintf(buffer, "Braking:  %.2f", m_braking);
        text.Render(buffer, screen.LEFT, screen.TOP - 6 * screen.SPACING, screen.SX, screen.SY);
    }

    const WheeledVehicle& m_vehicle;
    double m_steering;
    double m_throttle;
    double m_braking;
};
#endif

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string terrain_dir;

    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;
    
    double ramp_length = 0.0;
    double target_speed = 7.0;
    double tend = 30;
    double step_size = 5e-4;
    double active_box_dim = 0.5;

    double output_major_fps = 20;
    double output_minor_fps = 1000;
    int output_frames = 5;
    int particle_output = 2;       // output all particle info
    bool wheel_output = true;      // save individual wheel output files
    double filter_window_vel = 0;  // do not filter velocity data
    double filter_window_acc = 0;  // do not filter acceleration data
    double vis_output_fps = 0;     // no post-processing visualization output

    bool run_time_vis = false;            // no run-time visualization
    double run_time_vis_fps = 0;          // render every simulation frame
    bool run_time_vis_particles = false;  // render only terrain surface mesh
    bool run_time_vis_bce = false;        // render vehicle meshes
    bool chase_cam = false;               // fixed camera

    bool verbose = true;

    if (!GetProblemSpecs(argc, argv,                                                                        //
                         terrain_dir, density, cohesion, friction, youngs_modulus, poisson_ratio,           //
                         ramp_length, target_speed, tend, step_size, active_box_dim,                        //
                         output_major_fps, output_minor_fps, output_frames, particle_output, wheel_output,  //
                         filter_window_vel, filter_window_acc,                                              //
                         vis_output_fps,                                                                    //
                         run_time_vis, run_time_vis_particles, run_time_vis_bce, run_time_vis_fps,          //
                         chase_cam, verbose)) {
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
    if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/path.txt")).exists()) {
        std::cout << "Input file path.txt not found in directory " << terrain_dir << std::endl;
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
    ////if (!filesystem::path(vehicle::GetDataFile(terrain_dir + "/sph_params.json")).exists()) {
    ////    std::cout << "Input file sph_params.json not found in directory " << terrain_dir << std::endl;
    ////    return 1;
    ////}

    // Create the Chrono systems
    ChSystemNSC sys;
    ChSystemFsi sysFSI(&sys);

    // Set SPH parameters and soil material properties

    const ChVector3d gravity(0, 0, -9.81);
    sysFSI.SetGravitationalAcceleration(gravity);
    sys.SetGravitationalAcceleration(gravity);

    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.stress = 0;  // default
    mat_props.viscosity_alpha = 0.5;
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.friction_angle = CH_PI / 10;  // default
    mat_props.dilation_angle = CH_PI / 10;  // default
    mat_props.cohesion_coeff = 0;             // default
    mat_props.kernel_threshold = 0.8;

    sysFSI.SetElasticSPH(mat_props);
    sysFSI.SetDensity(density);
    sysFSI.SetCohesionForce(cohesion);

    double init_spacing = 0.02;
    double kernel_length = 0.03;
    sysFSI.SetInitialSpacing(init_spacing);
    sysFSI.SetKernelLength(kernel_length);

    ////cout << "Load SPH parameter file..." << endl;
    ////sysFSI.ReadParametersFromFile(vehicle::GetDataFile(terrain_dir + "/sph_params.json"));

    sysFSI.SetActiveDomain(ChVector3d(active_box_dim, active_box_dim, 1));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);
    sysFSI.SetVerbose(verbose);

    // Set simulation data output and FSI information output
    std::string out_dir = GetChronoOutputPath() + "POLARIS_SPH/";
    std::string vis_dir = out_dir + "Visualization/";
    sysFSI.SetOutputLength(0);
    ////sysFSI.SetOutputDirectory(out_dir);

    // Create terrain
    cout << "Create terrain..." << endl;
    bool terrain_mesh_contact = false;
    auto init_pos = CreateTerrain(sys, sysFSI, terrain_dir, ramp_length, !run_time_vis_particles, terrain_mesh_contact);

    // Create vehicle
    cout << "Create vehicle..." << endl;
    auto vehicle = CreateVehicle(sys, init_pos);

    // Create driver
    cout << "Create path..." << endl;
    auto path = CreatePath(terrain_dir, ramp_length);
    double x_max = path->GetPoint(path->GetNumPoints() - 2).x() - 3.0;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.Initialize();

    cout << "  Num points: " << path->GetNumPoints() << endl;
    for (int i = 0; i < path->GetNumPoints(); i++) {
        cout << "  [" << i << "]   " << path->GetPoint(i) << endl;
    }

#ifdef CHRONO_OPENGL
    // Create run-time visualization
    opengl::ChVisualSystemOpenGL vis;
    ChFsiVisualizationGL visFSI(&sysFSI, &vis);
    std::shared_ptr<ChBody> sentinel;
    std::shared_ptr<PolarisStats> stats;
    if (run_time_vis) {
        visFSI.SetTitle("Chrono::FSI single wheel demo");
        visFSI.SetSize(1280, 720);
        ////visFSI.SetCameraPosition(init_pos.pos + ChVector3d(-7, 0, 6), init_pos.pos + ChVector3d(1, 0, 0.5));
        visFSI.UpdateCamera(ChVector3d(0, 4, 1), ChVector3d(0, -1, 0));
        visFSI.SetCameraMoveScale(1.0f);
        visFSI.EnableFluidMarkers(run_time_vis_particles);
        visFSI.EnableRigidBodyMarkers(run_time_vis_bce);
        visFSI.EnableBoundaryMarkers(false);
        visFSI.SetRenderMode(ChFsiVisualizationGL::RenderMode::SOLID);
        ////visFSI.SetParticleRenderMode(sysFSI.GetInitialSpacing() / 2, ChFsiVisualizationGL::RenderMode::SOLID);

        sentinel = CreateSentinel(sys, init_pos);

        stats = chrono_types::make_shared<PolarisStats>(*vehicle);
        vis.AttachStatsRenderer(stats);
        vis.EnableStats(true);

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

    DataWriterVehicle data_writer(sysFSI, vehicle);
    data_writer.SetVerbose(verbose);
    data_writer.SetParticleOutput(output_level);
    data_writer.SetMBSOutput(wheel_output);
    data_writer.UseFilteredVelData(use_filter_vel, filter_window_vel);
    data_writer.UseFilteredAccData(use_filter_acc, filter_window_acc);
    data_writer.Initialize(sim_dir, output_major_fps, output_minor_fps, output_frames, step_size);
    cout << "Simulation output data saved in: " << sim_dir << endl;
    cout << "===============================================================================" << endl;

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 0};
    ChTerrain terrain;

    int vis_output_steps = (int)std::round((1.0 / vis_output_fps) / step_size);
    int render_steps = (run_time_vis_fps > 0) ? (int)std::round((1.0 / run_time_vis_fps) / step_size) : 1;
    int vis_output_frame = 0;

    bool on_ramp = true;

    double t = 0;
    int frame = 0;

    while (t < tend) {
        const auto& veh_loc = vehicle->GetPos();

        // Check if vehicle approaching SPH terrain patch
        if (on_ramp && veh_loc.x() > -0.5) {
            // Create the wheel BCE markers at current wheel body locations
            CreateWheelBCEMarkers(vehicle, sysFSI);

            // Complete construction of FSI system
            sysFSI.Initialize();
#ifdef CHRONO_OPENGL
            visFSI.Initialize();
            if (run_time_vis_bce)
                vehicle->SetTireVisualizationType(VisualizationType::NONE);
#endif

            on_ramp = false;
        }

        // Stop before end of patch
        if (veh_loc.x() > x_max)
            break;

        // Simulation data output
        if (sim_output && !on_ramp)
            data_writer.Process(frame, t);

        // Visualization data output
        if (vis_output && frame % vis_output_steps == 0) {
            if (verbose)
                cout << "Visualization output frame = " << vis_output_frame << endl;
            sysFSI.PrintParticleToFile(vis_dir);
            std::string vehicle_file = vis_dir + "/vehicle_" + std::to_string(vis_output_frame) + ".csv";
            chrono::utils::WriteVisualizationAssets(&sys, vehicle_file);

            vis_output_frame++;
        }

        // Set current driver inputs
        driver_inputs = driver.GetInputs();

        if (t < 0.5) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (t - 0.5) / 0.5);
        }

        ////if (verbose)
        ////    cout << std::fixed << std::setprecision(3) << "t = " << t << "  STB = " << driver_inputs.m_steering << " "
        ////         << driver_inputs.m_throttle << " " << driver_inputs.m_braking << "  spd = " << vehicle->GetSpeed()
        ////         << endl;

        // Run-time visualization
#ifdef CHRONO_OPENGL
        if (run_time_vis && frame % render_steps == 0) {
            if (chase_cam) {
                ChVector3d cam_loc = veh_loc + ChVector3d(-3, 3, 2);
                ChVector3d cam_point = veh_loc;
                visFSI.UpdateCamera(cam_loc, cam_point);
            }
            sentinel->SetPos(driver.GetSteeringController().GetTargetLocation());
            stats->UpdateDriverInputs(driver_inputs);
            if (!visFSI.Render())
                break;
        }
#endif

        // Synchronize systems
        driver.Synchronize(t);
        vehicle->Synchronize(t, driver_inputs, terrain);

        // Advance system state
        if (on_ramp) {
            // While vehicle on acceleration ramp, only advance MBD system
            double step = std::max(step_size, 1e-3);
            driver.Advance(step);
            sys.DoStepDynamics(step);
            t += step;
        } else {
            // Advance both FSI and embedded MBD systems
            driver.Advance(step_size);
            sysFSI.DoStepDynamics_FSI();
            t += step_size;
        }

        frame++;
    }

    return 0;
}

// ===================================================================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& terrain_dir,
                     double& density,
                     double& cohesion,
                     double& friction,
                     double& youngs_modulus,
                     double& poisson_ratio,
                     double& ramp_length,
                     double& target_speed,
                     double& tend,
                     double& step_size,
                     double& active_box_dim,
                     double& output_major_fps,
                     double& output_minor_fps,
                     int& output_frames,
                     int& particle_output,
                     bool& wheel_output,
                     double& filter_window_vel,
                     double& filter_window_acc,
                     double& vis_output_fps,
                     bool& run_time_vis,
                     bool& run_time_vis_particles,
                     bool& run_time_vis_bce,
                     double& run_time_vis_fps,
                     bool& chase_cam,
                     bool& verbose) {
    ChCLI cli(argv[0], "Polaris SPH terrain simulation");

    cli.AddOption<std::string>("Problem setup", "terrain_dir", "Directory with terrain specification data");
    cli.AddOption<double>("Problem setup", "density", "Material density [kg/m3]", std::to_string(density));
    cli.AddOption<double>("Problem setup", "cohesion", "Cohesion [Pa]", std::to_string(cohesion));
    cli.AddOption<double>("Problem setup", "friction", "Coefficient of friction", std::to_string(friction));
    cli.AddOption<double>("Problem setup", "youngs_modulus", "Young's modulus [Pa]", std::to_string(youngs_modulus));
    cli.AddOption<double>("Problem setup", "poisson_ratio", "Poission ratio", std::to_string(poisson_ratio));

    cli.AddOption<double>("Simulation", "ramp_length", "Acceleration ramp length", std::to_string(ramp_length));
    cli.AddOption<double>("Simulation", "target_speed", "Target speed [m/s]", std::to_string(target_speed));
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
    cli.AddOption<bool>("Simulation output", "no_wheel_output", "Disable individual wheel output files");
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
    cli.AddOption<bool>("Visualization", "chase_cam", "Enable vehicle-following camera");
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

    density = cli.GetAsType<double>("density");
    cohesion = cli.GetAsType<double>("cohesion");
    friction = cli.GetAsType<double>("friction");
    youngs_modulus = cli.GetAsType<double>("youngs_modulus");
    poisson_ratio = cli.GetAsType<double>("poisson_ratio");

    ramp_length = cli.GetAsType<double>("ramp_length");
    target_speed = cli.GetAsType<double>("target_speed");

    output_major_fps = cli.GetAsType<double>("output_major_fps");
    output_minor_fps = cli.GetAsType<double>("output_minor_fps");
    output_frames = cli.GetAsType<int>("output_frames");
    particle_output = cli.GetAsType<int>("particle_output");
    wheel_output = !cli.GetAsType<bool>("no_wheel_output");

    filter_window_vel = cli.GetAsType<double>("filter_window_vel");
    filter_window_acc = cli.GetAsType<double>("filter_window_acc");

    vis_output_fps = cli.GetAsType<double>("vis_output_fps");
    run_time_vis = cli.GetAsType<bool>("run_time_vis");
    run_time_vis_particles = cli.GetAsType<bool>("run_time_vis_particles");
    run_time_vis_bce = cli.GetAsType<bool>("run_time_vis_bce");
    run_time_vis_fps = cli.GetAsType<double>("run_time_vis_fps");
    chase_cam = cli.GetAsType<bool>("chase_cam");

    tend = cli.GetAsType<double>("tend");
    step_size = cli.GetAsType<double>("step_size");
    active_box_dim = cli.GetAsType<double>("active_box_dim");

    verbose = !cli.GetAsType<bool>("quiet");

    return true;
}
