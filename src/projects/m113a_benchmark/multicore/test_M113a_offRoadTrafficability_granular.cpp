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
// =============================================================================

#include <iostream>

// Chrono::Engine header files
#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

// Chrono::Multicore header files
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

// Chrono::Multicore OpenGL header files
//#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

// Chrono utility header files
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono vehicle header files
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

// M113 model header files
#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Comment the following line to use Chrono::Multicore
//#define USE_SEQ

// Comment the following line to use NSC contact
//#define USE_SMC

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

enum TerrainType { RIGID_TERRAIN, GRANULAR_TERRAIN };

// Type of terrain
TerrainType terrain_type = GRANULAR_TERRAIN;

// Control visibility of containing bin walls
bool visible_walls = false;

int Id_g = 100;

// Dimensions
double hdimX = 5.5;  //// 2.5;
double hdimY = 2.5;
double hdimZ = 0.5;
double hthick = 0.25;

double r_g = 0.02;
double rho_g = 1389.4;
float mu_g = 0.5f;
unsigned int num_particles = 1;  // 200000; //// 40000;
double sphereRatio = 0.4;

double sprocketRadius = 0.2605 + 0.5 * 0.06;
double rotSpeed = 1;

// Create material that will be used for all surfaces
#ifdef USE_SMC
auto mat_g = chrono_types::make_shared<ChContactMaterialSMC>();
#else
auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
#endif

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Initial vehicle position and orientation
ChVector3d initLoc(-hdimX + 4.5, 0, 1.2);
ChQuaternion<> initRot(1, 0, 0, 0);

// Simple powertrain model
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 8;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Total simulation duration.
double time_end = 10;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.2;

int max_iteration_bilateral = 4000;  // 1000;
int max_iteration_normal = 0;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// Output directories
bool povray_output = true;

// Output
std::string out_dir = "../M113_MULTICORE";
std::string pov_dir = out_dir + "/POVRAY";
std::string stats_file = out_dir + "/stats.dat";

int out_fps = 60;

// =============================================================================

double CreateParticles(ChSystem* system, double sphRatio) {
    // Create a particle generator and a mixture entirely made out of spheres
    double r = 1.01 * r_g;
    chrono::utils::ChPDSampler<double> sampler(2 * r);
    chrono::utils::ChGenerator gen(system);
    std::shared_ptr<chrono::utils::ChMixtureIngredient> m1;
    if (sphRatio == 1) {
        m1 = gen.AddMixtureIngredient(chrono::utils::MixtureType::SPHERE, 1.0);
        m1->SetDefaultMaterial(mat_g);
        m1->SetDefaultDensity(rho_g);
        m1->SetDefaultSize(r_g);
    } else {
        if (sphRatio < 1) {
            m1 = gen.AddMixtureIngredient(chrono::utils::MixtureType::ELLIPSOID, 1.0);
            m1->SetDefaultMaterial(mat_g);
            m1->SetDefaultDensity(rho_g);
            m1->SetDefaultSize(ChVector3d(r_g, sphRatio * r_g, r_g));
        } else {
            m1 = gen.AddMixtureIngredient(chrono::utils::MixtureType::ELLIPSOID, 1.0);
            m1->SetDefaultMaterial(mat_g);
            m1->SetDefaultDensity(rho_g);
            m1->SetDefaultSize(ChVector3d(r_g / sphRatio, r_g, r_g / sphRatio));
        }
    }

    // Set starting value for body identifiers
    gen.SetStartTag(Id_g);

    // Create particles in layers until reaching the desired number of particles
    ChVector3d hdims(hdimX - r, hdimY - r, 0);
    ChVector3d center(0, 0, 2 * r);

    while (gen.GetTotalNumBodies() < num_particles) {
        gen.CreateObjectsBox(sampler, center, hdims);
        center.z() += 2 * r;
    }

    std::cout << "Created " << gen.GetTotalNumBodies() << " particles." << std::endl;

    return center.z();
}

// =============================================================================
// Utility function to print to console a few important step statistics

static inline void TimingOutput(chrono::ChSystem* mSys,
                                double drawbarPull,
                                chrono::ChStreamOutAsciiFile* ofile = NULL) {
    double TIME = mSys->GetChTime();
    double STEP = mSys->GetTimerStep();
    double BROD = mSys->GetTimerCollisionBroad();
    double NARR = mSys->GetTimerCollisionNarrow();
    double LCP = mSys->GetTimerAdvance();
    double UPDT = drawbarPull;
    double RESID = 0;
    int REQ_ITS = 0;
    int BODS = mSys->GetNumBodies();
    int CNTC = mSys->GetNumContacts();
    if (chrono::ChSystemMulticore* mc_sys = dynamic_cast<chrono::ChSystemMulticore*>(mSys)) {
        auto msolver = std::static_pointer_cast<ChIterativeSolverMulticore>(mSys->GetSolver());
        RESID = msolver->GetResidual();
        REQ_ITS = msolver->GetIterations();
        BODS = mc_sys->GetNumBodies();
        CNTC = mc_sys->GetNumContacts();
    }

    if (ofile) {
        char buf[200];
        sprintf(buf, "%8.5f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7d  %7d  %7d  %7.9f\n", TIME, STEP, BROD, NARR, LCP,
                UPDT, BODS, CNTC, REQ_ITS, RESID);
        *ofile << buf;
    }

    printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.9f\n", TIME, STEP, BROD, NARR, LCP,
           UPDT, BODS, CNTC, REQ_ITS, RESID);
}

// =============================================================================
// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50) {
    if ((x != n) && (x % (n / 100 + 1) != 0))
        return;

    float ratio = x / (float)n;
    unsigned int c = (unsigned int)(ratio * w);

    std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
    for (unsigned int xx = 0; xx < c; xx++)
        std::cout << "=";
    for (unsigned int xx = c; xx < w; xx++)
        std::cout << " ";
    std::cout << "]\r" << std::flush;
}

// =============================================================================
int main(int argc, char* argv[]) {
    // Parameters for granular material
    double time_step = 1e-3;  // 2e-4;
    double tolerance = 0.01;
    int max_iteration_sliding = 500;  // 2000;
    double slip = 0.3;

    if (argc > 1) {
        r_g = atof(argv[1]);
        rho_g = atof(argv[2]);
        sphereRatio = atof(argv[3]);
        mu_g = atof(argv[4]);
        slip = atof(argv[5]);
        time_step = atof(argv[6]);
        tolerance = atof(argv[7]);

        std::stringstream dataFolderStream;
        dataFolderStream << "../M113_MULTICORE_" << r_g << "_" << rho_g << "_" << sphereRatio << "_" << mu_g << "_"
                         << slip << "_" << time_step << "_" << tolerance << "/";
        out_dir = dataFolderStream.str();
        pov_dir = out_dir + "/POVRAY";

        std::cout << out_dir << std::endl;
        std::cout << pov_dir << std::endl;
    }

    double vol_g = (4.0 / 3) * CH_PI * r_g * r_g * r_g;
    double mass_g = rho_g * vol_g;
    ChVector3d inertia_g = 0.4 * mass_g * r_g * r_g * ChVector3d(1, 1, 1);

    // -----------------
    // Initialize output
    // -----------------

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    // Initialize the output file
    std::stringstream statsFileStream;
    statsFileStream << out_dir << "/M113_MULTICORE_" << r_g << "_" << rho_g << "_" << sphereRatio << "_" << mu_g << "_"
                    << slip << "_" << time_step << "_" << tolerance << ".csv";
    std::cout << statsFileStream.str() << std::endl;
    ChStreamOutAsciiFile statsStream(statsFileStream.str().c_str());
    statsStream.SetNumFormat("%16.4e");

    // --------------
    // Create system.
    // --------------

#ifdef USE_SEQ
    // ----  Sequential
    #ifdef USE_SMC
    std::cout << "Create SMC system" << std::endl;
    ChSystemSMC* system = new ChSystemSMC();
    #else
    std::cout << "Create NSC system" << std::endl;
    ChSystemNSC* system = new ChSystemNSC();
    #endif

#else
    // ----  Multicore
    #ifdef USE_SMC
    std::cout << "Create multicore SMC system" << std::endl;
    ChSystemMulticoreSMC* system = new ChSystemMulticoreSMC();
    #else
    std::cout << "Create multicore NSC system" << std::endl;
    ChSystemMulticoreNSC* system = new ChSystemMulticoreNSC();
    #endif

#endif

    system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // ---------------------
    // Edit system settings.
    // ---------------------

#ifdef USE_SEQ

    system->GetSolver()->AsIterative()->SetMaxIterations(50);

#else

    // Set number of threads
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    if (thread_tuning)
        system->SetNumThreads(1, 1, threads);
    else
        system->SetNumThreads(threads);

    // Set solver parameters
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = tolerance;

    #ifndef USE_SMC
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    system->ChangeSolverType(SolverType::APGD);
    system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
    #endif

    system->GetSettings()->collision.bins_per_axis = vec3(20, 20, 20);

#endif

// -------------------
// Create the terrain.
// -------------------

// Contact material
#ifdef USE_SMC
    mat_g->SetYoungModulus(1e9f);
    mat_g->SetFriction(0);
    mat_g->SetRestitution(0);
#else
    mat_g->SetFriction(0);  // set to zero during settling
#endif

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);

    // Bottom box
    if (terrain_type == GRANULAR_TERRAIN) {
        ChVector3d r_e = ChVector3d(r_g, r_g, sphereRatio * r_g);
        ChVector3d pos = ChVector3d(0, 0, -hthick);
        ChVector3d size = ChVector3d(hdimX, hdimY, hthick);
        for (int i = 0; i <= 2 * (int)(size.x() / r_g); i++) {
            for (int j = 0; j <= 2 * (int)(size.y() / r_g); j++) {
                chrono::utils::AddEllipsoidGeometry(
                    ground.get(), mat_g, r_e,                                                //
                    pos + size - ChVector3d((double)i * r_e.x(), (double)j * r_e.y(), 0.0),  //
                    ChQuaternion<>(1, 0, 0, 0),                                              //
                    true);
            }
        }

        // Front box
        pos = ChVector3d(hdimX - hthick, 0, hdimZ - hthick);
        size = ChVector3d(hthick, hdimY, hdimZ + hthick);
        // utils::AddBoxGeometry(ground.get(), mat_g, size, pos, ChQuaternion<>(1, 0, 0, 0), visible_walls);
        r_e = ChVector3d(sphereRatio * r_g, r_g, r_g);
        for (int i = 0; i <= 2 * (int)(size.y() / r_g); i++) {
            for (int j = 0; j <= 2 * (int)(size.z() / r_g); j++) {
                chrono::utils::AddEllipsoidGeometry(
                    ground.get(), mat_g, r_e,                                                //
                    pos + size - ChVector3d(0.0, (double)i * r_e.y(), (double)j * r_e.z()),  //
                    ChQuaternion<>(1, 0, 0, 0),                                              //
                    visible_walls);
            }
        }

        // Rear box
        pos = ChVector3d(-hdimX - hthick, 0, hdimZ - hthick);
        size = ChVector3d(hthick, hdimY, hdimZ + hthick);
        // utils::AddBoxGeometry(ground.get(), mat_g, size, pos, ChQuaternion<>(1, 0, 0, 0), visible_walls);
        r_e = ChVector3d(sphereRatio * r_g, r_g, r_g);
        for (int i = 0; i <= 2 * (int)(size.y() / r_g); i++) {
            for (int j = 0; j <= 2 * (int)(size.z() / r_g); j++) {
                chrono::utils::AddEllipsoidGeometry(
                    ground.get(), mat_g, r_e,                                                //
                    pos + size - ChVector3d(0.0, (double)i * r_e.y(), (double)j * r_e.z()),  //
                    ChQuaternion<>(1, 0, 0, 0),                                              //
                    visible_walls);
            }
        }

        // Left box
        pos = ChVector3d(0, hdimY - hthick, hdimZ - hthick);
        size = ChVector3d(hdimX, hthick, hdimZ + hthick);
        // utils::AddBoxGeometry(ground.get(), mat_g, size, pos, ChQuaternion<>(1, 0, 0, 0), visible_walls);
        r_e = ChVector3d(r_g, sphereRatio * r_g, r_g);
        for (int i = 0; i <= 2 * (int)(size.x() / r_g); i++) {
            for (int j = 0; j <= 2 * (int)(size.z() / r_g); j++) {
                chrono::utils::AddEllipsoidGeometry(
                    ground.get(), mat_g, r_e,                                                //
                    pos + size - ChVector3d((double)i * r_e.x(), 0.0, (double)j * r_e.z()),  //
                    ChQuaternion<>(1, 0, 0, 0),                                              //
                    visible_walls);
            }
        }

        // Right box
        pos = ChVector3d(0, -hdimY - hthick, hdimZ - hthick);
        size = ChVector3d(hdimX, hthick, hdimZ + hthick);
        // utils::AddBoxGeometry(ground.get(), mat_g, size, pos, ChQuaternion<>(1, 0, 0, 0), visible_walls);
        r_e = ChVector3d(r_g, sphereRatio * r_g, r_g);
        for (int i = 0; i <= 2 * (int)(size.x() / r_g); i++) {
            for (int j = 0; j <= 2 * (int)(size.z() / r_g); j++) {
                chrono::utils::AddEllipsoidGeometry(
                    ground.get(), mat_g, r_e,                                                //
                    pos + size - ChVector3d((double)i * r_e.x(), 0.0, (double)j * r_e.z()),  //
                    ChQuaternion<>(1, 0, 0, 0),                                              //
                    visible_walls);
            }
        }
    } else {
        chrono::utils::AddBoxGeometry(ground.get(), mat_g, ChVector3d(hdimX, hdimY, hthick),  //
                                      ChVector3d(0, 0, -hthick), ChQuaternion<>(1, 0, 0, 0),  //
                                      true);
    }

    system->AddBody(ground);

    // Create the granular material.
    double vertical_offset = 0;

    if (terrain_type == GRANULAR_TERRAIN) {
        vertical_offset = CreateParticles(system, sphereRatio);
    }

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    // Create the vehicle system
    M113 m113(system);
    m113.SetContactMethod(ChContactMethod::SMC);
    m113.SetChassisFixed(true);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetDrivelineType(DrivelineTypeTV::SIMPLE);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);

    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();
    auto& vehicle = m113.GetVehicle();
    auto engine = vehicle.GetEngine();

    // Control steering type (enable crossdrive capability).
    m113.GetDriveline()->SetGyrationMode(true);

    ////vehicle.EnableCollision(TrackCollide::NONE);
    ////vehicle.EnableCollision(TrackCollide::WHEELS_LEFT | TrackCollide::WHEELS_RIGHT);
    ////vehicle.EnableCollision(TrackCollide::ALL & (~TrackCollide::SPROCKET_LEFT) & (~TrackCollide::SPROCKET_RIGHT));

    // Create the driver system
    ChDataDriver driver(vehicle, vehicle::GetDataFile("M113a_benchmark/driver/Acceleration.txt"));
    driver.Initialize();
    double drawbarPull = 0;

    // Control the slip of the vehicle
    auto slipRig = chrono_types::make_shared<ChBody>();
    slipRig->SetName("slipRig");
    slipRig->SetPos(initLoc);
    slipRig->SetFixed(false);
    slipRig->EnableCollision(false);
    system->AddBody(slipRig);

    auto box = chrono_types::make_shared<ChVisualShapeBox>(0.4, 0.2, 0.2);
    slipRig->AddVisualShape(box);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.05, 0.2);
    slipRig->AddVisualShape(cyl, ChFrame<>(ChVector3d(0, 0, -0.1)));

    auto transJoint = chrono_types::make_shared<ChLinkLockPrismatic>();
    transJoint->SetName("_transJoint");
    transJoint->Initialize(slipRig, m113.GetChassisBody(), ChFrame<>(initLoc, QUNIT));
    system->AddLink(transJoint);

    auto lockJoint = chrono_types::make_shared<ChLinkLockLock>();
    lockJoint->SetName("_lockJoint");
    lockJoint->Initialize(slipRig, ground, ChCoordsys<>(initLoc, QUNIT));
    system->AddLink(lockJoint);

    double transSpeed = (1.0 - slip) * sprocketRadius * rotSpeed;
    auto motion = chrono_types::make_shared<ChFunctionRamp>(-transSpeed * time_hold, transSpeed);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(
        vehicle.GetTrackAssembly(chrono::vehicle::RIGHT)->GetSprocket()->GetGearBody(), m113.GetChassisBody(),
        ChFrame<>(vehicle.GetTrackAssembly(chrono::vehicle::RIGHT)->GetSprocket()->GetGearBody()->GetPos(),
                  chrono::QuatFromAngleAxis(CH_PI / 2, VECT_X)));
    auto speedFunc = chrono_types::make_shared<ChFunctionRamp>(0, -rotSpeed);
    motor->SetAngleFunction(speedFunc);
    system->AddLink(motor);

    auto motor2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor2->Initialize(
        vehicle.GetTrackAssembly(chrono::vehicle::LEFT)->GetSprocket()->GetGearBody(), m113.GetChassisBody(),
        ChFrame<>(vehicle.GetTrackAssembly(chrono::vehicle::LEFT)->GetSprocket()->GetGearBody()->GetPos(),
                  chrono::QuatFromAngleAxis(CH_PI / 2, VECT_X)));
    auto speedFunc2 = chrono_types::make_shared<ChFunctionRamp>(0, -rotSpeed);
    motor2->SetAngleFunction(speedFunc2);
    system->AddLink(motor2);

    // ---------------
    // Simulation loop
    // ---------------

#ifdef CHRONO_OPENGL
    // Initialize OpenGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(system);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, -10, 0), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);
#endif

    // Number of simulation steps between two 3D view render frames
    int out_steps = (int)std::ceil((1.0 / time_step) / out_fps);

    // Run simulation for specified time.
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;
    double exec_time = 0;
    int num_contacts = 0;

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    while (time < time_end) {
        if (sim_frame == next_out_frame) {
            cout << endl;
            cout << "---- Frame:          " << out_frame + 1 << endl;
            cout << "     Sim frame:      " << sim_frame << endl;
            cout << "     Time:           " << time << endl;
            cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
            cout << "     Execution time: " << exec_time << endl;

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame + 1);
                chrono::utils::WriteVisualizationAssets(system, filename);
            }

            out_frame++;
            next_out_frame += out_steps;
            num_contacts = 0;
        }

        // Release the vehicle chassis at the end of the hold time.
        if (m113.GetChassis()->IsFixed() && time > time_hold) {
            std::cout << std::endl << "Release vehicle t = " << time << std::endl;
            m113.GetChassisBody()->SetFixed(false);
            mat_g->SetFriction(mu_g);  // set friction of soil to true value
            lockJoint->SetMotion_X(motion);
        }

        // Collect output data from modules
        DriverInputs driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance simulation for one timestep for all modules
        driver.Advance(time_step);
        vehicle.Advance(time_step);

#ifdef CHRONO_OPENGL
        if (vis.Run())
            vis.Render();
        else
            break;
#endif

        // progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);

        // Periodically display maximum constraint violation
        if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
            vehicle.LogConstraintViolations();
        }

        if (time > time_hold)
            drawbarPull = lockJoint->GetReaction2().force.x();
        TimingOutput(system, drawbarPull, &statsStream);

        // Update counters.
        time += time_step;
        sim_frame++;
        exec_time += system->GetTimerStep();
        num_contacts += system->GetNumContacts();
    }

    // Final stats
    std::cout << "==================================" << std::endl;
    std::cout << "Simulation time:   " << exec_time << std::endl;
    std::cout << "Number of threads: " << threads << std::endl;

    return 0;
}
