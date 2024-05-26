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
// Authors: Radu Serban
// =============================================================================
//
// 
//
// =============================================================================

#include <cstdlib>
#include <cstdio>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

#include "chrono_models/vehicle/m113/M113.h"

////#undef CHRONO_IRRLICHT
#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Specification of terrain
// -----------------------------------------------------------------------------

// Slope scaling factor (in degrees)
double max_slope = 40;

// Patch half-dimensions
double hdimX = 100;//// 250;
double hdimY = 2.5;

// Vehicle horizontal offset
double horizontal_offset = 12;
double horizontal_pos = hdimX - horizontal_offset;

// Initial vehicle position, orientation, and forward velocity
ChVector3d initLoc(-horizontal_pos, 0, 0.8);
ChQuaternion<> initRot(1, 0, 0, 0);
double initSpeed = 0;

// Point on chassis tracked by the camera (Irrlicht only)
ChVector3d trackPoint(-1.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// Timed events
// -----------------------------------------------------------------------------

// Total simulation duration.
double time_end = 50;

// Delay before starting the engine
double time_start_engine = 0.25;

// Delay before throttle reaches maximum (linear ramp)
double delay_max_throttle = 0.5;
double time_max_throttle = time_start_engine + delay_max_throttle;

// Delays before checking for slow-down and steady-state
double filter_interval = 3.0;
double delay_start_check_slow = 5.0;
double delay_start_check_steady = 5.0;
double time_start_check_slow = time_max_throttle + delay_start_check_slow;
double time_start_check_steady = time_max_throttle + delay_start_check_steady;

// Time when terrain is pitched (rotate gravity)
double time_pitch = time_start_engine;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double time_step = 1e-3;

// Output
bool output = true;
double output_frequency = 100.0;
std::string out_dir = "../GONOGO_M113_SCM";

// =============================================================================

class GONOGO_Driver : public chrono::vehicle::ChDriver {
  public:
    GONOGO_Driver(chrono::vehicle::ChVehicle& vehicle,          // associated vehicle
                  std::shared_ptr<chrono::ChBezierCurve> path,  // target path
                  bool render_path,                             // add curve visualization asset?
                  double time_start,                            // time throttle start
                  double time_max                               // time throttle max
    );

    void SetGains(double Kp, double Ki, double Kd) { m_steeringPID.SetGains(Kp, Ki, Kd); }
    void SetLookAheadDistance(double dist) { m_steeringPID.SetLookAheadDistance(dist); }

    void Reset() { m_steeringPID.Reset(m_vehicle.GetRefFrame()); }

    virtual void Synchronize(double time) override;
    virtual void Advance(double step) override;

    void ExportPathPovray(const std::string& out_dir);

  private:
    chrono::vehicle::ChPathSteeringController m_steeringPID;
    double m_start;
    double m_end;
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ----------------------------
    // Parse command line arguments
    // ----------------------------

    std::string input_file = "";  // Name of input file
    int line_number = 0;          // Line of inputs
    bool copy = true;             // Copy input file?

    chrono::ChCLI cli(argv[0]);

    cli.AddOption<std::string>("Demo", "f,filename", "Name of input file");
    cli.AddOption<int>("Demo", "l,line", "Line in input file");
    cli.AddOption<bool>("Demo", "copy", "Copy input file to output directory", "true");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return 1;
    }

    input_file = cli.GetAsType<std::string>("filename");
    line_number = cli.GetAsType<int>("line");
    copy = cli.GetAsType<bool>("copy");

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
    int model_index;
    double slope_val, saturation_val;
    double Bekker_n, Bekker_Kphi, Bekker_Kc, Mohr_coh, Mohr_phi, Janosi_k;
    std::istringstream iss(line);
    iss >> model_index >> slope_val >> saturation_val >> Bekker_n >> Bekker_Kphi >> Bekker_Kc >> Mohr_coh >> Mohr_phi >> Janosi_k;
    
    double slope_deg = slope_val * max_slope;
    double slope = slope_deg * (CH_PI / 180);
    Bekker_Kphi *= 1000;
    Bekker_Kc *= 1000;
    Mohr_coh *= 1000;
    Janosi_k *= 1e-2;

    cout << "Set up" << endl;
    cout << "  File:          " << input_file << "  Line: " << line_number << endl;
    cout << "  Parameters:    " << model_index << " " << slope_val << " " << saturation_val << endl;
    cout << "  Slope:         " << slope_deg << endl;
    cout << "  Bekker Kphi:   " << Bekker_Kphi << endl;
    cout << "  Bekker Kc:     " << Bekker_Kc << endl;
    cout << "  Bekker n:      " << Bekker_n << endl;
    cout << "  Mohr cohesion: " << Mohr_coh << endl;
    cout << "  Mohr phi:      " << Mohr_phi << endl;
    cout << "  Janosi k:      " << Janosi_k << endl;

    // ---------------------------------
    // Create output directory and files
    // ---------------------------------

    std::ofstream ofile;
    std::string del("  ");

    if (output) {
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

        // Open the output file stream
        char buf[10];
        std::sprintf(buf, "%03d", line_number);
        ofile.open(out_dir + "/results_" + std::string(buf) + ".out", std::ios::out);
    }

    // -------------
    // Create system
    // -------------

    // Prepare rotated acceleration vector
    ChVector3d gravity(0, 0, -9.81);
    ChVector3d gravityR = ChMatrix33<>(slope, ChVector3d(0, 1, 0)) * gravity;

    ChSystemSMC* system = new ChSystemSMC();
    system->SetGravitationalAcceleration(gravity);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetTolerance(1e-10);
    solver->EnableWarmStart(true);
    system->SetSolver(solver);

    // -------------------------------------
    // Create the vehicle
    // -------------------------------------

    M113 m113;
    m113.SetContactMethod(ChContactMethod::SMC);
    m113.SetChassisFixed(false);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetDrivelineType(DrivelineTypeTV::SIMPLE);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);

    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();
    auto& vehicle = m113.GetVehicle();
    auto engine = vehicle.GetEngine();

    // Set visualization type for subsystems
    m113.SetChassisVisualizationType(VisualizationType::NONE);
    m113.SetSprocketVisualizationType(VisualizationType::MESH);
    m113.SetIdlerVisualizationType(VisualizationType::MESH);
    m113.SetSuspensionVisualizationType(VisualizationType::MESH);
    m113.SetRoadWheelVisualizationType(VisualizationType::MESH);
    m113.SetTrackShoeVisualizationType(VisualizationType::MESH);

    // Control steering type (enable crossdrive capability).
    m113.GetDriveline()->SetGyrationMode(true);

    // -------------------------------------
    // Create the driver
    // -------------------------------------

    double height = initLoc.z();
    auto path = StraightLinePath(ChVector3d(-2 * hdimX, 0, height), ChVector3d(200 * hdimX, 0, height));

    GONOGO_Driver driver(vehicle, path, false, time_start_engine, time_max_throttle);
    double look_ahead_dist = 5;
    double Kp_steering = 0.5;
    double Ki_steering = 0;
    double Kd_steering = 0;
    driver.SetLookAheadDistance(look_ahead_dist);
    driver.SetGains(Kp_steering, Ki_steering, Kd_steering);
    driver.Initialize();

    // ------------------
    // Create the terrain
    // ------------------

    // Deformable terrain properties (LETE sand)
    double Kphi = 5301e3;    // Bekker Kphi
    double Kc = 102e3;       // Bekker Kc
    double n = 0.793;        // Bekker n exponent
    double c = 1.3e3;        // Mohr cohesive limit (Pa)
    double phi = 31.1;       // Mohr friction limit (degrees)
    double K = 1.2e-2;       // Janosi shear coefficient (m)
    double E_elastic = 2e8;  // Elastic stiffness (Pa/m), before plastic yeld
    double damping = 3e4;    // Damping coefficient (Pa*s/m)

    SCMTerrain terrain(system);
    terrain.SetPlane(ChCoordsys<>(VNULL, QuatFromAngleX(CH_PI_2)));
    terrain.SetSoilParameters(Bekker_Kphi, Bekker_Kc, Bekker_n, Mohr_coh, Mohr_phi, K, E_elastic, damping);
    ////terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);
    terrain.Initialize(2 * hdimX, 2 * hdimY, 0.02);

    // Enable moving patch feature
    terrain.AddMovingPatch(m113.GetChassisBody(), ChVector3d(-2, 0, 0), ChVector3d(6.5, 3.5, 1.0));

#ifdef CHRONO_IRRLICHT

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("M113 go/no-go");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    ////vis->SetChaseCameraAngle(CH_PI / 5);
    ////vis->EnableStats(false);
#endif

    // Save parameters and problem setup to output file
    if (output) {
        ofile << "# File: " << filename << endl;
        ofile << "# Line: " << line_number << endl;
        ofile << "# " << endl;
        ofile << "# " << line << endl;
        ofile << "# " << endl;
        ofile << "# Model index:      " << model_index << endl;
        ofile << "# Slope level:      " << slope_val << endl;
        ofile << "# Saturation level: " << saturation_val << endl;
        ofile << "# Slope (deg):      " << slope_deg << endl;
        ofile << "# Bekker Kphi:      " << Bekker_Kphi << endl;
        ofile << "# Bekker Kc:        " << Bekker_Kc << endl;
        ofile << "# Bekker n:         " << Bekker_n << endl;
        ofile << "# Mohr cohesion:    " << Mohr_coh << endl;
        ofile << "# Mohr phi:         " << Mohr_phi << endl;
        ofile << "# Janosi k:         " << Janosi_k << endl;
        ofile << "# " << endl;

        ofile.precision(7);
        ofile << std::scientific;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil((1 / output_frequency) / time_step);

    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;

    bool is_pitched = false;

    int filter_steps = (int)std::ceil(filter_interval / time_step);
    utils::ChRunningAverage fwd_vel_filter(filter_steps);
    utils::ChRunningAverage fwd_acc_filter(filter_steps);

    while (true) {
        // Rotate gravity vector
        if (!is_pitched && time > time_pitch) {
            cout << time << "    Pitch: " << gravityR.x() << " " << gravityR.y() << " " << gravityR.z() << endl;
            system->SetGravitationalAcceleration(gravityR);
            is_pitched = true;
        }

        // Check if reached maximum simulation time
        if (time >= time_end) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Reached maximum time" << endl;
            }
            break;
        }

#ifdef CHRONO_IRRLICHT
        if (!vis->Run())
            break;

        vis->BeginScene();
        vis->Render();
#endif

        // Extract chassis state
        ChVector3d pv = m113.GetChassisBody()->GetFrameRefToAbs().GetPos();
        ChVector3d vv = m113.GetChassisBody()->GetFrameRefToAbs().GetPosDt();
        ChVector3d av = m113.GetChassisBody()->GetFrameRefToAbs().GetPosDt2();

        // Check if vehicle at maximum position
        if (pv.x() >= horizontal_pos) {
            if (output) {
                ofile << "# " << endl;
                ofile << "# Reached maximum x position" << endl;
            }
            break;
        }

        // Filtered forward velocity and acceleration
        double fwd_vel_mean = fwd_vel_filter.Add(vv.x());
        double fwd_vel_std = fwd_vel_filter.GetStdDev();
        double fwd_acc_mean = fwd_acc_filter.Add(av.x());
        double fwd_acc_std = fwd_acc_filter.GetStdDev();

        // Check if vehicle is sliding backward
        if (pv.x() <= -horizontal_pos - 7) {
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

        // Collect output data from modules (for inter-module communication)
        DriverInputs driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Save output
        if (output && sim_frame == next_out_frame) {
            cout << system->GetChTime() << " pos: " << pv.x() - 6 << " vel: " << vv.x() << " Wtime: " << exec_time << endl;

            ofile << system->GetChTime() << del;
            ofile << driver_inputs.m_throttle << del << driver_inputs.m_steering << del;

            ofile << pv.x() - 6 << del << pv.y() << del << pv.z() << del;
            ofile << vv.x() << del << vv.y() << del << vv.z() << del;

            ofile << fwd_vel_mean << del << fwd_vel_std << del;
            ofile << fwd_acc_mean << del << fwd_acc_std << del;

            ofile << endl;

            out_frame++;
            next_out_frame += output_steps;
        }

        // Synchronize subsystems
        terrain.Synchronize(time);
        driver.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
#ifdef CHRONO_IRRLICHT
        vis->Synchronize(time, driver_inputs);
#endif

        // Advance systems
        driver.Advance(time_step);
        terrain.Advance(time_step);
        m113.Advance(time_step);
#ifdef CHRONO_IRRLICHT
        vis->Advance(time_step);
#endif

        ////terrain.PrintStepStatistics(cout);

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();

#ifdef CHRONO_IRRLICHT
        vis->EndScene();
#endif
    }

    // Final stats
    cout << "==================================" << endl;
    cout << "Simulation time:   " << exec_time << endl;

    if (output) {
        ofile << "# " << endl;
        ofile << "# Simulation time (s): " << exec_time << endl;
        ofile.close();
    }

    return 0;
}

// =============================================================================

GONOGO_Driver::GONOGO_Driver(chrono::vehicle::ChVehicle& vehicle,
                             std::shared_ptr<chrono::ChBezierCurve> path,
                             bool render_path,
                             double time_start,
                             double time_max)
    : chrono::vehicle::ChDriver(vehicle), m_steeringPID(path), m_start(time_start), m_end(time_max) {
    m_steeringPID.Reset(m_vehicle.GetRefFrame());

    if (render_path) {
        auto road = chrono_types::make_shared<ChBody>();
        road->SetFixed(true);
        m_vehicle.GetSystem()->AddBody(road);

        auto path_asset = chrono_types::make_shared<chrono::ChVisualShapeLine>();
        path_asset->SetLineGeometry(chrono_types::make_shared<chrono::ChLineBezier>(m_steeringPID.GetPath()));
        path_asset->SetColor(chrono::ChColor(0.0f, 0.8f, 0.0f));
        path_asset->SetName("straight_path");
        road->AddVisualShape(path_asset);
    }
}

void GONOGO_Driver::Synchronize(double time) {
    m_braking = 0;
    if (time < m_start) {
        m_throttle = 0;
    } else if (time < m_end) {
        m_throttle = (time - m_start) / (m_end - m_start);
    } else {
        m_throttle = 1;
    }
}

void GONOGO_Driver::Advance(double step) {
    double out_steering = m_steeringPID.Advance(m_vehicle.GetRefFrame(), m_vehicle.GetChTime(), step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

void GONOGO_Driver::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steeringPID.GetPath(), "straight_path", out_dir, 0.04,
                                    chrono::ChColor(0.8f, 0.5f, 0.0f));
}
