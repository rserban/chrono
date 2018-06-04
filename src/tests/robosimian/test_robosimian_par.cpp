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
// Authors: Radu Serban
// =============================================================================
//
// RoboSimian on granular terrain
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_opengl/ChOpenGLWindow.h"

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#include "robosimian.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// =============================================================================

// ID values to identify command line arguments
enum {
    OPT_HELP,
    OPT_MODE,
    OPT_SIM_TIME,
    OPT_STEP_SIZE,
    OPT_OUTPUT_FPS,
    OPT_POVRAY_FPS,
    OPT_NUM_THREADS,
    OPT_NO_RELEASE,
    OPT_NO_OUTPUT,
    OPT_NO_POVRAY_OUTPUT,
    OPT_NO_RENDERING,
    OPT_SUFFIX
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_NUM_THREADS, "--num-threads", SO_REQ_CMB},
                                    {OPT_MODE, "-m", SO_REQ_CMB},
                                    {OPT_MODE, "--mode", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "-t", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "--simulation-time", SO_REQ_CMB},
                                    {OPT_STEP_SIZE, "-s", SO_REQ_CMB},
                                    {OPT_STEP_SIZE, "--step-size", SO_REQ_CMB},
                                    {OPT_OUTPUT_FPS, "--output-FPS", SO_REQ_CMB},
                                    {OPT_POVRAY_FPS, "--povray-FPS", SO_REQ_CMB},
                                    {OPT_NO_RELEASE, "--no-release", SO_NONE},
                                    {OPT_NO_OUTPUT, "--no-output", SO_NONE},
                                    {OPT_NO_POVRAY_OUTPUT, "--no-povray-output", SO_NONE},
                                    {OPT_NO_RENDERING, "--no-rendering", SO_NONE},
                                    {OPT_SUFFIX, "--suffix", SO_REQ_CMB},
                                    {OPT_HELP, "-?", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS};

void ShowUsage();
bool GetProblemSpecs(int argc,
                     char** argv,
                     robosimian::LocomotionMode& mode,
                     double& sim_time,
                     double& step_size,
                     double& out_fps,
                     double& pov_fps,
                     int& nthreads,
                     bool& drop,
                     bool& output,
                     bool& pov_output,
                     bool& render,
                     std::string& suffix);

// =============================================================================

class RobotDriverCallback : public robosimian::Driver::PhaseChangeCallback {
  public:
    RobotDriverCallback(robosimian::RoboSimian* robot) : m_robot(robot), m_start_x(0), m_start_time(0) {}
    virtual void OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) override;

    double GetDistance() const { return m_robot->GetChassisPos().x() - m_start_x; }
    double GetDuration() const { return m_robot->GetSystem()->GetChTime() - m_start_time; }
    double GetAvgSpeed() const { return GetDistance() / GetDuration(); }

    double m_start_x;
    double m_start_time;

  private:
    robosimian::RoboSimian* m_robot;
};

void RobotDriverCallback::OnPhaseChange(robosimian::Driver::Phase old_phase, robosimian::Driver::Phase new_phase) {
    if (new_phase == robosimian::Driver::CYCLE && old_phase != robosimian::Driver::CYCLE) {
        m_start_x = m_robot->GetChassisPos().x();
        m_start_time = m_robot->GetSystem()->GetChTime();
    }
}

// =============================================================================

double CreateTerrain(ChSystemParallel& sys, double x, double z, double step_size) {
    std::cout << "x = " << x << "  z = " << z << std::endl;

    double r_g = 0.0075;
    double rho_g = 2000;
    double coh = 400e3;
    double area = CH_C_PI * r_g * r_g;
    double coh_force = area * coh;
    double coh_g = coh_force * step_size;

    double length = 6;
    double width = 3;

    unsigned int num_layers = 5;

    // Height of container surface
    double r = 1.01 * r_g;
    z -= (2 * r) * num_layers;

    // Create contact material
    std::shared_ptr<ChMaterialSurface> material;
    switch (sys.GetContactMethod()) {
        case ChMaterialSurface::SMC: {
            auto mat = std::make_shared<ChMaterialSurfaceSMC>();
            mat->SetFriction(0.9f);
            mat->SetRestitution(0.0f);
            mat->SetYoungModulus(8e5f);
            mat->SetPoissonRatio(0.3f);
            mat->SetAdhesion(static_cast<float>(coh_force));
            mat->SetKn(1.0e6f);
            mat->SetGn(6.0e1f);
            mat->SetKt(4.0e5f);
            mat->SetGt(4.0e1f);
            material = mat;
            break;
        }
        case ChMaterialSurface::NSC: {
            auto mat = std::make_shared<ChMaterialSurfaceNSC>();
            mat->SetFriction(0.9f);
            mat->SetRestitution(0.0f);
            mat->SetCohesion(static_cast<float>(coh_force));
            material = mat;
            break;
        }
    }

    // Create container
    auto ground = std::shared_ptr<ChBody>(sys.NewBody());
    ground->SetIdentifier(-1);
    ground->SetPos(ChVector<>(length / 2 + x - 1.5, 0, z));
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    ground->SetMaterialSurface(material);

    ChVector<> hdim(length / 2, width / 2, 0.4);
    double hthick = 0.1;
    ground->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick), QUNIT, true);
    utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(-hdim.x() - hthick, 0, hdim.z()), QUNIT, false);
    utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(hdim.x() + hthick, 0, hdim.z()), QUNIT, false);
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, -hdim.y() - hthick, hdim.z()), QUNIT, false);
    utils::AddBoxGeometry(ground.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, hdim.y() + hthick, hdim.z()), QUNIT, false);
    ground->GetCollisionModel()->BuildModel();

    sys.AddBody(ground);

    // Create particles
    utils::Generator gen(&sys);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
    m1->setDefaultMaterial(material);
    m1->setDefaultDensity(rho_g);
    m1->setDefaultSize(r_g);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(10000);

    // Create particles in layers until reaching the desired number of particles
    ChVector<> hdims(length/2 - r, width/2 - r, 0);
    ChVector<> center(length / 2 + x - 1.5, 0, z + r);

    for (unsigned int il = 0; il < num_layers; il++) {
        std::cout << "Create layer at " << center.z() << std::endl;
        gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
        center.z() += 2 * r;
    }

    std::cout << "Generated " << gen.getTotalNumBodies() << " particles" << std::endl;

    // Estimate number of bins for collision detection
    int factor = 2;
    int binsX = (int)std::ceil((0.5 * length) / r_g) / factor;
    int binsY = (int)std::ceil((0.5 * width) / r_g) / factor;
    int binsZ = 1;
    sys.GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

    // Set collision envelope
    sys.GetSettings()->collision.collision_envelope = 0.1 * r_g / 5;

    return (length + x - 2 * 1.5);
}

double CreateTerrainPatch(ChSystemParallel& sys, double x, double z, double step_size) {
    double r_g = 0.0075;
    double rho_g = 2000;
    double coh = 400e3;
    double area = CH_C_PI * r_g * r_g;
    double coh_force = area * coh;
    double coh_g = coh_force * step_size;

    std::cout << "x = " << x << "  z = " << z << std::endl;

    double length = 6;
    double width = 3;
    unsigned int num_layers = 5;
    ChVector<> center(length / 2 + x - 1.5, 0, z - num_layers * 2.1 * r_g);

    vehicle::GranularTerrain terrain(&sys);
    terrain.SetContactFrictionCoefficient(0.7f);
    terrain.SetContactCohesion((float)coh_g);
    terrain.SetCollisionEnvelope(0.1 * r_g / 5);

    terrain.EnableVisualization(true);
    terrain.Initialize(center, length, width, num_layers, r_g, rho_g);

    std::cout << "Generated " << terrain.GetNumParticles() << " particles" << std::endl;

    // Estimate number of bins for collision detection
    int factor = 2;
    int binsX = (int)std::ceil((0.5 * length) / r_g) / factor;
    int binsY = (int)std::ceil((0.5 * width) / r_g) / factor;
    int binsZ = 1;
    sys.GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

    // Set collision envelope
    sys.GetSettings()->collision.collision_envelope = 0.1 * r_g / 5;

    return (length + x - 2 * 1.5);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // ----------------------------
    // Parse command line arguments
    // ----------------------------

    robosimian::LocomotionMode mode = robosimian::LocomotionMode::WALK;
    double step_size = 1e-4;
    double duration_sim = 10;
    double out_fps = 100;
    double pov_fps = 60;
    int nthreads = 2;
    bool drop = true;
    bool render = true;
    bool pov_output = true;
    bool output = true;
    std::string suffix = "";

    if (!GetProblemSpecs(argc, argv, mode, duration_sim, step_size, out_fps, pov_fps, nthreads, drop, output,
                         pov_output, render, suffix)) {
        return 1;
    }

    // ------------
    // Timed events
    // ------------

    double duration_pose = 1.0;            // Interval to assume initial pose
    double duration_settle_terrain = 1.0;  // Interval to allow granular material settling
    double duration_settle_robot = 0.5;    // Interval to allow robot settling on terrain
    double duration_hold = duration_settle_terrain + duration_settle_robot;

    double time_create_terrain = duration_pose;  // create terrain after robot assumes initial pose
    double time_release = time_create_terrain + duration_settle_terrain;  // release robot after terrain settling
    double time_start = time_release + duration_settle_robot;  // start actual simulation after robot settling
    double time_end = time_start + duration_sim;               // end simulation after specified duration

    // -----------------------------
    // Initialize output directories
    // -----------------------------

    const std::string dir = "../ROBOSIMIAN_PAR";
    std::string pov_dir = dir + "/POVRAY" + suffix;
    std::string out_dir = dir + "/RESULTS" + suffix;

    if (ChFileutils::MakeDirectory(dir.c_str()) < 0) {
        std::cout << "Error creating directory " << dir << std::endl;
        return 1;
    }
    if (output) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    if (pov_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // ----------------------
    // Write problem settings
    // ----------------------

    std::ofstream outf;
    outf.open(dir + "/settings" + suffix + ".txt", std::ios::out);
    outf << "Locomotion mode: ";
    switch (mode) {
        case robosimian::LocomotionMode::WALK:
            outf << "WALK" << endl;
            break;
        case robosimian::LocomotionMode::SCULL:
            outf << "SCULL" << endl;
            break;
        case robosimian::LocomotionMode::INCHWORM:
            outf << "INCHWORM" << endl;
            break;
        case robosimian::LocomotionMode::DRIVE:
            outf << "DRIVE" << endl;
            break;
    }
    outf << "Release robot? " << (drop ? "YES" : "NO") << endl;
    outf << endl;
    outf << "Time terrain settling: " << duration_settle_terrain << endl;
    outf << "Time robot settling: " << duration_settle_robot << endl;
    outf << "Time locomotion: " << duration_sim << endl;
    outf << "Total time: " << time_end << endl;
    outf << endl;
    outf << "Step size: " << step_size << endl;
    outf << "Number threads: " << nthreads << endl;
    outf << endl;
    outf << "Result output?" << (output ? "YES" : "NO") << endl;
    if (output) {
        outf << "  Output frequency (FPS): " << out_fps << endl;
        outf << "  Output directory:       " << out_dir << endl;
    }
    outf << "POV-Ray output? " << (pov_output ? "YES" : "NO") << endl;
    if (pov_output) {
        outf << "  Output frequency (FPS): " << pov_fps << endl;
        outf << "  Output directory:       " << pov_dir << endl;
    }
    outf.close();

    // -------------
    // Create system
    // -------------

    ////ChSystemParallelSMC my_sys;
    ChSystemParallelNSC my_sys;
    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////my_sys.Set_G_acc(ChVector<double>(0, 0, 0));

    int max_threads = CHOMPfunctions::GetNumProcs();
    if (nthreads > max_threads)
        nthreads = max_threads;
    my_sys.SetParallelThreadNumber(nthreads);
    CHOMPfunctions::SetNumThreads(nthreads);

    my_sys.GetSettings()->solver.tolerance = 1e-3;
    my_sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    my_sys.GetSettings()->solver.max_iteration_normal = 0;
    my_sys.GetSettings()->solver.max_iteration_sliding = 100;
    my_sys.GetSettings()->solver.max_iteration_spinning = 0;
    my_sys.GetSettings()->solver.max_iteration_bilateral = 100;
    my_sys.GetSettings()->solver.compute_N = false;
    my_sys.GetSettings()->solver.alpha = 0;
    my_sys.GetSettings()->solver.cache_step_length = true;
    my_sys.GetSettings()->solver.use_full_inertia_tensor = false;
    my_sys.GetSettings()->solver.contact_recovery_speed = 1000;
    my_sys.GetSettings()->solver.bilateral_clamp_speed = 1e8;
    my_sys.GetSettings()->min_threads = nthreads;

    my_sys.GetSettings()->collision.collision_envelope = 0.01;
    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    my_sys.ChangeSolverType(SolverType::APGD);

    // -----------------------
    // Create RoboSimian robot
    // -----------------------

    robosimian::RoboSimian robot(&my_sys, true, true);

    // Set output directory
    robot.SetOutputDirectory(out_dir);

    // Ensure wheels are actuated in ANGLE mode (required for Chrono::Parallel)
    robot.SetMotorActuationMode(robosimian::ActuationMode::ANGLE);

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // -----------------------------------
    // Create a driver and attach to robot
    // -----------------------------------

    std::shared_ptr<robosimian::Driver> driver;

    switch (mode) {
        case robosimian::LocomotionMode::WALK: {
            auto drv = std::make_shared<robosimian::Driver>(
                "",                                                           // start input file
                GetChronoDataFile("robosimian/actuation/walking_cycle.txt"),  // cycle input file
                "",                                                           // stop input file
                true);
            driver = drv;
            std::cout << "Locomotion mode: WALK" << std::endl;
            break;
        }
        case robosimian::LocomotionMode::SCULL: {
            auto drv = std::make_shared<robosimian::Driver>(
                GetChronoDataFile("robosimian/actuation/sculling_start.txt"),  // start input file
                GetChronoDataFile("robosimian/actuation/sculling_cycle.txt"),  // cycle input file
                GetChronoDataFile("robosimian/actuation/sculling_stop.txt"),   // stop input file
                true);
            driver = drv;
            std::cout << "Locomotion mode: SCULL" << std::endl;
            break;
        }
        case robosimian::LocomotionMode::INCHWORM: {
            auto drv = std::make_shared<robosimian::Driver>(
                GetChronoDataFile("robosimian/actuation/inchworming_start.txt"),  // start input file
                GetChronoDataFile("robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
                GetChronoDataFile("robosimian/actuation/inchworming_stop.txt"),   // stop input file
                true);
            driver = drv;
            std::cout << "Locomotion mode: INCHWORM" << std::endl;
            break;
        }
        case robosimian::LocomotionMode::DRIVE: {
            auto drv = std::make_shared<robosimian::Driver>(
                GetChronoDataFile("robosimian/actuation/driving_start.txt"),  // start input file
                GetChronoDataFile("robosimian/actuation/driving_cycle.txt"),  // cycle input file
                GetChronoDataFile("robosimian/actuation/driving_stop.txt"),   // stop input file
                true);
            driver = drv;
            std::cout << "Locomotion mode: DRIVE" << std::endl;
            break;
        }
    }

    RobotDriverCallback cbk(&robot);
    driver->RegisterPhaseChangeCallback(&cbk);

    driver->SetTimeOffsets(duration_pose, duration_hold);
    robot.SetDriver(driver);

    // -----------------
    // Initialize OpenGL
    // -----------------

    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "RoboSimian", &my_sys);
        gl_window.SetCamera(ChVector<>(2, -2, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }

    // ---------------------------------
    // Run simulation for specified time
    // ---------------------------------

    int out_steps = (int)std::ceil((1.0 / out_fps) / step_size);
    int pov_steps = (int)std::ceil((1.0 / pov_fps) / step_size);
    int sim_frame = 0;
    int pov_frame = 0;

    bool terrain_created = false;
    bool robot_released = false;
    double x_max = 0;

    while (true) {
        double time = my_sys.GetChTime();
        double x = robot.GetChassisPos().x();

        if (time >= time_end) {
            std::cout << "Reached final time: " << time << std::endl;
            break;
        }

        if (drop) {
            if (!terrain_created && time > time_create_terrain) {
                // Robot position and bottom point
                double z = robot.GetWheelPos(robosimian::FR).z() - 0.15;
                // Create terrain
                std::cout << "Time: " << time << "  CREATE TERRAIN" << std::endl;
                x_max = CreateTerrain(my_sys, x, z, step_size);
                ////x_max = CreateTerrainPatch(my_sys, x, z, step_size);
                terrain_created = true;
            }
            if (!robot_released && time > time_release) {
                std::cout << "Time: " << time << "  RELEASE ROBOT" << std::endl;
                robot.GetChassis()->GetBody()->SetBodyFixed(false);
                robot_released = true;
            }

            if (robot_released && x > x_max) {
                std::cout << "Time: " << time << " Reached maximum distance" << std::endl;
                break;
            }
        }

        // Output results
        if (output && sim_frame % out_steps == 0) {
            robot.Output();
        }

        // Output POV-Ray data
        if (pov_output && sim_frame % pov_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), pov_frame + 1);
            utils::WriteShapesPovray(&my_sys, filename);
            pov_frame++;
        }

        ////double time = my_sys.GetChTime();
        ////double A = CH_C_PI / 6;
        ////double freq = 2;
        ////double val = 0.5 * A * (1 - std::cos(CH_C_2PI * freq * time));
        ////robot.Activate(robosimian::FR, "joint2", time, val);
        ////robot.Activate(robosimian::RL, "joint5", time, val);
        ////robot.Activate(robosimian::FL, "joint8", time, -0.4 * time);

        robot.DoStepDynamics(step_size);

        ////if (my_sys.GetNcontacts() > 0) {
        ////    robot.ReportContacts();
        ////}

        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active()) {
                gl_window.Render();
            } else {
                break;
            }
        }

        sim_frame++;
    }

    return 0;
}

// =============================================================================

void ShowUsage() {
    cout << "Usage:  test_robosimian_par [OPTIONS]" << endl;
    cout << endl;
    cout << " --num-threads=NUM_THREADS_TIRE" << endl;
    cout << "        Specify number of OpenMP threads [default: 2]" << endl;
    cout << " -m=MODE" << endl;
    cout << " --mode=MODE" << endl;
    cout << "        Specify locomotion mode [default: 0]" << endl;
    cout << "          0: walk" << endl;
    cout << "          1: scull" << endl;
    cout << "          2: inchworm" << endl;
    cout << "          3: drive" << endl;
    cout << " -t=SIM_TIME" << endl;
    cout << " --simulation-time=SIM_TIME" << endl;
    cout << "        Specify simulation length (after robot release) in seconds [default: 10]" << endl;
    cout << " -s=STEP_SIZE" << endl;
    cout << " --step-size=STEP_SIZE" << endl;
    cout << "        Specify integration step size in seconds [default: 1e-4]" << endl;
    cout << " --output-FPS=FPS" << endl;
    cout << "        Specify frequency of results output [default: 100]" << endl;
    cout << " --povray-FPS=FPS" << endl;
    cout << "        Specify frequency of POV-Ray output [default: 60]" << endl;
    cout << " --no-release" << endl;
    cout << "        Do not release the robot (no terrain created)" << endl;
    cout << " --no-output" << endl;
    cout << "        Disable generation of result output files" << endl;
    cout << " --no-povray-output" << endl;
    cout << "        Disable generation of POV-Ray output files" << endl;
    cout << " --no-rendering" << endl;
    cout << "        Disable OpenGL rendering" << endl;
    cout << " --suffix=SUFFIX" << endl;
    cout << "        Specify suffix for output directory names [default: \"\"]" << endl;
    cout << " -? -h --help" << endl;
    cout << "        Print this message and exit." << endl;
    cout << endl;
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     robosimian::LocomotionMode& mode,
                     double& sim_time,
                     double& step_size,
                     double& out_fps,
                     double& pov_fps,
                     int& nthreads,
                     bool& drop,
                     bool& output,
                     bool& pov_output,
                     bool& render,
                     std::string& suffix) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Default locomotion mode: WALK
    int imode = 0;

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            cout << "Invalid argument: " << args.OptionText() << endl;
            ShowUsage();
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                ShowUsage();
                return false;
            case OPT_NUM_THREADS:
                nthreads = std::stoi(args.OptionArg());
                break;
            case OPT_MODE:
                imode = std::stoi(args.OptionArg());
                break;
            case OPT_SIM_TIME:
                sim_time = std::stod(args.OptionArg());
                break;
            case OPT_STEP_SIZE:
                step_size = std::stod(args.OptionArg());
                break;
            case OPT_OUTPUT_FPS:
                out_fps = std::stod(args.OptionArg());
                break;
            case OPT_POVRAY_FPS:
                pov_fps = std::stod(args.OptionArg());
                break;
            case OPT_NO_RELEASE:
                drop = false;
                break;
            case OPT_NO_OUTPUT:
                output = false;
                break;
            case OPT_NO_POVRAY_OUTPUT:
                pov_output = false;
                break;
            case OPT_NO_RENDERING:
                render = false;
                break;
            case OPT_SUFFIX:
                suffix = args.OptionArg();
                break;
        }
    }

    switch (imode) {
        case 0:
            mode = robosimian::LocomotionMode::WALK;
            break;
        case 1:
            mode = robosimian::LocomotionMode::SCULL;
            break;
        case 2:
            mode = robosimian::LocomotionMode::INCHWORM;
            break;
        case 3:
            mode = robosimian::LocomotionMode::DRIVE;
            break;
    }

    return true;
}
