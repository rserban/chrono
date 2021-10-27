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
// Chrono::Vehicle + Chrono::Multicore program for simulating a HMMWV vehicle
// on granular terrain using the mobing patch feature.
//
// Contact uses non-smooth (DVI) formulation.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdlib>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "chrono/ChConfig.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "driver_model.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

// Patch half-dimensions
double hdimX = 3.5;
double hdimY = 1.75;

// Fixed base layer?
bool rough = false;

// Depth of granular material (mm)
double depth = 140;
int num_layers_min = 4;
int num_layers_max = 8;

// Moving patch parameters
bool moving_patch = true;
double buffer_distance = 4.0;
double shift_distance = 0.25;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Vehicle horizontal offset
double horizontal_offset = 2.5;
double horizontal_pos = hdimX - horizontal_offset;

// Initial vehicle position, orientation, and forward velocity
ChVector<> initLoc(-horizontal_pos, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);
double initSpeed = 0;

// Contact material properties for tires
float mu_t = 0.8f;  // 1.0f;
float cr_t = 0;

// -----------------------------------------------------------------------------
// Timed events
// -----------------------------------------------------------------------------

// Total simulation duration.
double time_end = 50;

// Time when the vehicle is created (allows for granular material settling)
double time_create_vehicle = 0.25;

// Delay before starting the engine (before setting the target speed)
double delay_start_engine = 0.25;
double time_start_engine = time_create_vehicle + delay_start_engine;

// Delay before throttle reaches maximum (linear ramp)
double delay_max_throttle = 0.5;
double time_max_throttle = time_start_engine + delay_max_throttle;

// Delays before checking for slow-down and steady-state
double filter_interval = 3.0;
double delay_start_check_slow = 15.0;
double delay_start_check_steady = 30.0;
double time_start_check_slow = time_max_throttle + delay_start_check_slow;
double time_start_check_steady = time_max_throttle + delay_start_check_steady;

// Time when terrain is pitched (rotate gravity)
double time_pitch = time_start_engine;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Enable thread tuning?
bool thread_tuning = false;

// Integration step size
double time_step = 1e-3;

// Solver settings
int max_iteration_bilateral = 100;
int max_iteration_normal = 0;
int max_iteration_sliding = 50;
int max_iteration_spinning = 0;

double tolerance = 1e-3;
double alpha = 0;
float contact_recovery_speed = 1000;

// Output
bool output = true;
double output_frequency = 100.0;
std::string out_dir = "../GONOGO_HMMWV_MP";

// =============================================================================

// Custom material composition law.
// Use the maximum coefficient of friction.
class CustomCompositionStrategy : public ChMaterialCompositionStrategy {
  public:
    virtual float CombineFriction(float a1, float a2) const override { return std::max<float>(a1, a2); }
};

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& file,
                     int& line,
                     int& threads,
                     bool& render,
                     bool& copy,
                     bool& pov_output);

HMMWV_Full* CreateVehicle(ChSystem* system, double vertical_offset);
GONOGO_Driver* CreateDriver(ChVehicle& vehicle);

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50);
void TimingOutput(chrono::ChSystem* mSys, chrono::ChStreamOutAsciiFile* ofile = NULL);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ----------------------------
    // Parse command line arguments
    // ----------------------------

    std::string input_file = "";  // Name of input file
    int line_number = 0;          // Line of inputs
    int threads = 0;              // Number of threads
    bool render = true;           // Render?
    bool copy = true;             // Copy input file?
    bool pov_output = false;      // Generate data for POV-Ray?

    // Extract arguments
    if (!GetProblemSpecs(argc, argv, input_file, line_number, threads, render, copy, pov_output)) {
        return 1;
    }

    // Check that input file exists
    filesystem::path inpath(input_file);
    if (!inpath.exists()) {
        cout << "Input file " << input_file << " does not exist" << endl;
        return 1;
    } else if (!inpath.is_file()) {
        cout << "Input file " << input_file << " is not a regular file" << endl;
        return 1;
    }

    // Check that a line number was specified
    if (line_number <= 0) {
        cout << "Incorrect line number." << endl;
        return 1;
    }

    // Check that the number of threads was specified
    if (threads <= 0) {
        cout << "Incorrect number of threads." << endl;
        return 1;
    }

    // ----------------
    // Parse input file
    // ----------------

    // Extract the filename, the basename, and extension of the input file
    std::string filename = inpath.filename();
    std::string stem = inpath.stem();
    std::string extension = inpath.extension();

    // Open input file
    std::ifstream ifile;
    ifile.open(input_file.c_str());

    std::string line;
    for (int i = 0; i < line_number; i++) {
        if (!std::getline(ifile, line) || line.length() == 0) {
            cout << "Incorrect line number." << endl;
            return 1;
        }
    }
    ifile.clear();
    ifile.seekg(0);

    // Extract input data
    std::istringstream iss(line);
    double slope_val, r_val, rho_val, mu_val, coh_val;
    iss >> slope_val >> r_val >> rho_val >> mu_val >> coh_val;

    double slope = slope_val * (CH_C_PI / 180);
    double r_g = r_val / 1000;
    double rho_g = rho_val;
    double mu_g = mu_val;
    double area = CH_C_PI * r_g * r_g;
    double coh_force = area * (coh_val * 1e3);
    double coh_g = coh_force * time_step;
    double envelope = 0.1 * r_g;

    int num_layers = static_cast<int>(depth / (2 * r_val));
    ChClampValue(num_layers, num_layers_min, num_layers_max);

    cout << "Set up" << endl;
    cout << "  File:        " << input_file << "  Line: " << line_number << endl;
    cout << "  Slope:       " << slope_val << "  " << slope << endl;
    cout << "  Radius:      " << r_val << "  " << r_g << endl;
    cout << "  Density:     " << rho_val << "  " << rho_g << endl;
    cout << "  Friction:    " << mu_val << "  " << mu_g << endl;
    cout << "  Cohesion:    " << coh_val << "  " << coh_g << endl;
    cout << "  Num. layers: " << num_layers << endl;

    // ---------------------------------
    // Create output directory and files
    // ---------------------------------

    std::ofstream ofile;
    std::string del("  ");
    std::string pov_dir;

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    out_dir += "/" + stem;

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Copy input file to output directory
    if (copy) {
        std::ofstream dst(out_dir + "/" + filename, std::ios::binary);
        dst << ifile.rdbuf();
    }

    if (output) {
        // Open the output file stream
        ofile.open(out_dir + "/results_" + std::to_string(line_number) + ".out", std::ios::out);
    }

    if (pov_output) {
        // Create directory for POV-Ray output files
        pov_dir = out_dir + "/POVRAY_" + std::to_string(line_number);
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // -------------
    // Create system
    // -------------

    // Prepare rotated acceleration vector
    ChVector<> gravity(0, 0, -9.81);
    ChVector<> gravityR = ChMatrix33<>(slope, ChVector<>(0, 1, 0)) * gravity;

    ChSystemMulticoreNSC* system = new ChSystemMulticoreNSC();
    system->Set_G_acc(gravity);

    // Use a custom material property composition strategy.
    // This ensures that tire-terrain interaction always uses the same coefficient of friction.
    std::unique_ptr<CustomCompositionStrategy> strategy(new CustomCompositionStrategy);
    system->SetMaterialCompositionStrategy(std::move(strategy));

    // Set number of threads
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    if (thread_tuning)
        system->SetNumThreads(1, 1, threads);
    else
        system->SetNumThreads(threads);

    // --------------------
    // Edit system settings
    // --------------------

    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.compute_N = false;
    system->GetSettings()->solver.alpha = alpha;
    system->GetSettings()->solver.cache_step_length = true;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system->ChangeSolverType(SolverType::BB);

    system->GetSettings()->collision.collision_envelope = envelope;
    system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    system->GetSettings()->collision.bins_per_axis = vec3(100, 30, 2);

    // Specify active box.
    // NOTE: this would need to be moved with the patch!
    ////system->GetSettings()->collision.use_aabb_active = false;
    ////system->GetSettings()->collision.aabb_min = real3(-1.1 * hdimX, -1.1 * hdimY, 0);
    ////system->GetSettings()->collision.aabb_max = real3(+1.1 * hdimX, +1.1 * hdimY, 10);

    // ------------------
    // Create the terrain
    // ------------------

    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    material->SetFriction((float)mu_g);
    material->SetCohesion((float)coh_g);
    GranularTerrain terrain(system);
    terrain.SetContactMaterial(material);
    terrain.SetCollisionEnvelope(envelope / 5);
    if (rough) {
        int nx = (int)std::round((2 * hdimX) / (4 * r_g));
        int ny = (int)std::round((2 * hdimY) / (4 * r_g));
        terrain.EnableRoughSurface(nx, ny);
    }
    terrain.EnableVisualization(true);
    terrain.EnableVerbose(true);

    terrain.Initialize(ChVector<>(0, 0, 0), 2 * hdimX, 2 * hdimY, num_layers, r_g, rho_g);
    uint actual_num_particles = terrain.GetNumParticles();

    std::cout << "Number of particles: " << actual_num_particles << std::endl;

    // Save parameters and problem setup to output file
    if (output) {
        ofile << "# File: " << filename << endl;
        ofile << "# Line: " << line_number << endl;
        ofile << "# " << endl;
        ofile << "# " << line << endl;
        ofile << "# " << endl;
        ofile << "# Slope (deg):     " << slope_val << endl;
        ofile << "# Radius (mm):     " << r_val << endl;
        ofile << "# Density (kg/m3): " << rho_val << endl;
        ofile << "# Friction:        " << mu_val << endl;
        ofile << "# Cohesion (kPa):  " << coh_val << endl;
        ofile << "# " << endl;
        ofile << "# Num threads:     " << threads << endl;
        ofile << "# Num layers:      " << num_layers << endl;
        ofile << "# Num particles:   " << actual_num_particles << endl;
        ofile << "# " << endl;

        ofile.precision(7);
        ofile << std::scientific;
    }

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "HMMWV go/no-go", system);
        gl_window.SetCamera(ChVector<>(-horizontal_pos, -5, 0), ChVector<>(-horizontal_pos, 0, 0), ChVector<>(0, 0, 1));
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------

    HMMWV_Full* hmmwv = nullptr;
    GONOGO_Driver* driver = nullptr;

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil((1 / output_frequency) / time_step);
    int pov_steps = (int)std::ceil((1 / 60.0) / time_step);

    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int pov_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;

    bool is_pitched = false;

    int filter_steps = (int)std::ceil(filter_interval / time_step);
    utils::ChRunningAverage fwd_vel_filter(filter_steps);
    utils::ChRunningAverage fwd_acc_filter(filter_steps);

    while (true) {
        if (time >= time_end) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Reached maximum time" << endl;
            }
            break;
        }

        // Create the vehicle
        if (!hmmwv && time > time_create_vehicle) {
            cout << time << "    Create vehicle" << endl;

            double max_height = terrain.GetHeight(ChVector<>(0, 0, 0));
            hmmwv = CreateVehicle(system, max_height);
            driver = CreateDriver(hmmwv->GetVehicle());

            if (pov_output) {
                driver->ExportPathPovray(out_dir);
            }

            // Enable moving patch, based on vehicle location
            if (moving_patch)
                terrain.EnableMovingPatch(hmmwv->GetChassisBody(), buffer_distance, shift_distance);

            next_out_frame = sim_frame;
        }

        // Rotate gravity vector
        if (!is_pitched && time > time_pitch) {
            cout << time << "    Pitch: " << gravityR.x() << " " << gravityR.y() << " " << gravityR.z() << endl;
            system->Set_G_acc(gravityR);
            is_pitched = true;
        }

        // Output POV-Ray data
        if (pov_output && sim_frame % pov_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), pov_frame + 1);
            utils::WriteVisualizationAssets(system, filename);
            pov_frame++;
        }

        // Synchronize terrain system
        terrain.Synchronize(time);

        if (hmmwv) {
            // Extract current driver inputs
            ChDriver::Inputs driver_inputs = driver->GetInputs();

            // Extract chassis state
            ChVector<> pv = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos();
            ChVector<> vv = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dt();
            ChVector<> av = hmmwv->GetChassisBody()->GetFrame_REF_to_abs().GetPos_dtdt();

            // Filtered forward velocity and acceleration
            double fwd_vel_mean = fwd_vel_filter.Add(vv.x());
            double fwd_vel_std = fwd_vel_filter.GetStdDev();
            double fwd_acc_mean = fwd_acc_filter.Add(av.x());
            double fwd_acc_std = fwd_acc_filter.GetStdDev();

            cout << fwd_acc_mean << "  " << fwd_vel_std << endl;

            // Check if vehicle at maximum position
            if (!moving_patch && pv.x() >= horizontal_pos) {
                if (output) {
                    ofile << "# " << endl;
                    ofile << "# Reached maximum x position" << endl;
                }
                break;
            }

            // Check if vehicle is sliding backward
            if (pv.x() <= -hdimX) {
                if (output) {
                    ofile << "# " << endl;
                    ofile << "# Vehicle sliding backward" << endl;
                }
                break;
            }

            // Check if vehicle is slowing down
            if (time > time_start_check_slow && fwd_acc_mean < 0.1) {
                if (output) {
                    ofile << "# " << endl;
                    ofile << "# Vehicle slowing down" << endl;
                }
                break;
            }

            // Check if vehicle reached steady-state speed
            if (time > time_start_check_steady && std::abs(fwd_acc_mean) < 0.05 && fwd_vel_std < 0.03) {
                if (output) {
                    ofile << "# " << endl;
                    ofile << "# Vehicle reached steady state" << endl;
                }
                break;
            }

            // Save output
            if (output && sim_frame == next_out_frame) {
                ofile << system->GetChTime() << del;
                ofile << driver_inputs.m_throttle << del << driver_inputs.m_steering << del;

                ofile << pv.x() << del << pv.y() << del << pv.z() << del;
                ofile << vv.x() << del << vv.y() << del << vv.z() << del;

                ofile << fwd_vel_mean << del << fwd_vel_std << del;
                ofile << fwd_acc_mean << del << fwd_acc_std << del;

                ofile << endl;

                out_frame++;
                next_out_frame += output_steps;
            }

            // Synchronize subsystems
            driver->Synchronize(time);
            hmmwv->Synchronize(time, driver_inputs, terrain);

            // Advance subsystems
            driver->Advance(time_step);
            hmmwv->Advance(time_step);
        } else {
            // Advance system state (no vehicle created yet)
            system->DoStepDynamics(time_step);
        }

#ifdef CHRONO_OPENGL
        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active())
                gl_window.Render();
            else
                break;
        }
#endif

        // Display performance metrics
        TimingOutput(system);

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;

    if (output) {
        ofile << "# " << endl;
        ofile << "# Simulation time (s): " << exec_time << endl;
        ofile.close();
    }

    delete hmmwv;
    delete driver;

    return 0;
}

// =============================================================================

HMMWV_Full* CreateVehicle(ChSystem* system, double vertical_offset) {
    auto hmmwv = new HMMWV_Full(system);

    hmmwv->SetContactMethod(ChContactMethod::NSC);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(ChCoordsys<>(initLoc + ChVector<>(0, 0, vertical_offset), initRot));
    hmmwv->SetInitFwdVel(initSpeed);
    hmmwv->SetTireType(TireModelType::RIGID);
    hmmwv->SetPowertrainType(PowertrainModelType::SIMPLE_MAP);
    hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    hmmwv->SetTireStepSize(time_step);

    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv->SetTireVisualizationType(VisualizationType::NONE);

    return hmmwv;
}

GONOGO_Driver* CreateDriver(ChVehicle& vehicle) {
    double height = initLoc.z();
    auto path = StraightLinePath(ChVector<>(-2 * hdimX, 0, height), ChVector<>(200 * hdimX, 0, height));

    auto driver = new GONOGO_Driver(vehicle, path, time_start_engine, time_max_throttle);
    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;
    driver->SetLookAheadDistance(look_ahead_dist);
    driver->SetGains(Kp_steering, Ki_steering, Kd_steering);
    driver->Initialize();

    return driver;
}
