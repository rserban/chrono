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
// Chrono::Multicore test program for settling process of granular material.
//
// The global reference frame has Z up.
// All units SI (CGS, i.e., centimeter - gram - second)
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <valarray>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// --------------------------------------------------------------------------

void TimingHeader() {
    printf("    TIME    |");
    printf("    STEP |");
    printf("   BROAD |");
    printf("  NARROW |");
    printf("  SOLVER |");
    printf("  UPDATE |");
    printf("# BODIES |");
    printf("# CONTACT|");
    printf(" # ITERS |");
    printf("\n\n");
}

void TimingOutput(chrono::ChSystem* mSys) {
    double TIME = mSys->GetChTime();
    double STEP = mSys->GetTimerStep();
    double BROD = mSys->GetTimerCollisionBroad();
    double NARR = mSys->GetTimerCollisionNarrow();
    double SOLVER = mSys->GetTimerAdvance();
    double UPDT = mSys->GetTimerUpdate();
    int REQ_ITS = 0;
    int BODS = mSys->GetNumBodies();
    int CNTC = mSys->GetNumContacts();
    if (chrono::ChSystemMulticore* mc_sys = dynamic_cast<chrono::ChSystemMulticore*>(mSys)) {
        REQ_ITS = std::static_pointer_cast<ChIterativeSolverMulticore>(mSys->GetSolver())->GetIterations();
    }

    printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d\n", TIME, STEP, BROD, NARR,
           SOLVER, UPDT, BODS, CNTC, REQ_ITS);
}

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
    int num_threads = 4;
    ChContactMethod method = ChContactMethod::SMC;
    bool render = true;
    double time_end = 1;
    std::string out_dir = "../TRACK_TEST";

    // ----------------
    // Model parameters
    // ----------------

    // Container properties
    double hdimX = 2.0;
    double hdimY = 0.25;
    double hdimZ = 0.5;
    double hthick = 0.25;

    float mu_c = 0.9f;
    float cr_c = 0.0f;
    float Y_c = 1e8f;
    float nu_c = 0.3f;
    float kn_c = 1.0e7f;
    float gn_c = 1.0e3f;
    float kt_c = 3.0e6f;
    float gt_c = 1.0e3f;
    float coh_c = 10.0f;

    // Granular material properties (SPHERE or BISPHERE)
    auto shape = utils::MixtureType::SPHERE;
    bool randomized_size = true;

    double radius_g = 10e-3;
    int Id_g = 100;
    double rho_g = 2500;
    double vol_g = (4.0 / 3) * CH_PI * radius_g * radius_g * radius_g;
    double mass_g = rho_g * vol_g;
    ChVector3d inertia_g = 0.4 * mass_g * radius_g * radius_g * ChVector3d(1, 1, 1);

    int num_layers = 20;

    float mu_g = 0.9f;
    float cr_g = 0.0f;
    float Y_g = 1e8f;
    float nu_g = 0.3f;
    float kn_g = 1.0e7f;
    float gn_g = 1.0e3f;
    float kt_g = 3.0e6f;
    float gt_g = 1.0e3f;
    float coh_pressure = 1e4f;
    float coh_g = (float)(CH_PI * radius_g * radius_g) * coh_pressure;

    // Track properties
    double hX = 0.11 / 2;
    double hY = 0.19 / 2;
    double hZ = 0.06 / 2;
    double pitch = 0.154;
    int npads = 20;

    double mass_t = 10000;
    ChVector3d inertia_t(1786.92, 10449.67, 10721.22);

    float mu_t = 0.8f;
    float cr_t = 0.1f;
    float Y_t = 1e7f;
    float nu_t = 0.3f;
    float kn_t = 2.0e5f;
    float gn_t = 4.0e1f;
    float kt_t = 2.0e5f;
    float gt_t = 2.0e1f;
    float coh_t = 0.0f;

    // Estimates for number of bins for broad-phase
    int factor = 2;
    int binsX = (int)std::ceil(hdimX / radius_g) / factor;
    int binsY = (int)std::ceil(hdimY / radius_g) / factor;
    int binsZ = 10;
    std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

    // ---------------------------
    // Create the multicore system
    // ---------------------------

    // Create system and set method-specific solver settings
    chrono::ChSystemMulticore* system;
    double time_step;

    switch (method) {
        case ChContactMethod::SMC: {
            ChSystemMulticoreSMC* sys = new ChSystemMulticoreSMC;
            auto contact_force_model = ChSystemSMC::PlainCoulomb;
            bool use_mat_properties = true;
            sys->GetSettings()->solver.contact_force_model = contact_force_model;
            sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
            sys->GetSettings()->solver.use_material_properties = use_mat_properties;
            system = sys;
            time_step = 1e-5;
            out_dir += "_DEM_" + std::to_string(contact_force_model) + "_" + std::to_string(use_mat_properties);
            break;
        }
        case ChContactMethod::NSC: {
            ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC;
            sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 200;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->GetSettings()->solver.alpha = 0;
            sys->GetSettings()->solver.contact_recovery_speed = -1;
            sys->GetSettings()->collision.collision_envelope = 0.1 * radius_g;
            sys->ChangeSolverType(SolverType::BB);
            system = sys;
            time_step = 1e-3;
            out_dir += "_DVI";
            break;
        }
    }

    system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = 0.1;
    system->GetSettings()->solver.max_iteration_bilateral = 100;
    system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);

    // Set number of threads
    system->SetNumThreads(num_threads);

// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master
    { std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

    // ---------------------
    // Create terrain bodies
    // ---------------------

    // Create contact material for container
    std::shared_ptr<ChContactMaterial> material_c;

    switch (method) {
        case ChContactMethod::SMC: {
            auto mat_c = chrono_types::make_shared<ChContactMaterialSMC>();
            mat_c->SetFriction(mu_g);
            mat_c->SetRestitution(cr_g);
            mat_c->SetYoungModulus(Y_g);
            mat_c->SetPoissonRatio(nu_g);
            mat_c->SetAdhesion(coh_g);
            mat_c->SetKn(kn_g);
            mat_c->SetGn(gn_g);
            mat_c->SetKt(kt_g);
            mat_c->SetGt(gt_g);

            material_c = mat_c;

            break;
        }
        case ChContactMethod::NSC: {
            auto mat_c = chrono_types::make_shared<ChContactMaterialNSC>();
            mat_c->SetFriction(mu_g);
            mat_c->SetRestitution(cr_g);
            mat_c->SetCohesion(coh_g);

            material_c = mat_c;

            break;
        }
    }

    // Create container body
    auto container = chrono_types::make_shared<ChBody>();
    system->AddBody(container);
    container->SetTag(-1);
    container->SetMass(1);
    container->SetFixed(true);
    container->EnableCollision(true);

    // Bottom box
    utils::AddBoxGeometry(container.get(), material_c, ChVector3d(hdimX, hdimY, hthick), ChVector3d(0, 0, -hthick),
                          ChQuaternion<>(1, 0, 0, 0), true);
    // Front box
    utils::AddBoxGeometry(container.get(), material_c, ChVector3d(hthick, hdimY, hdimZ + hthick),
                          ChVector3d(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Rear box
    utils::AddBoxGeometry(container.get(), material_c, ChVector3d(hthick, hdimY, hdimZ + hthick),
                          ChVector3d(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Left box
    utils::AddBoxGeometry(container.get(), material_c, ChVector3d(hdimX, hthick, hdimZ + hthick),
                          ChVector3d(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Right box
    utils::AddBoxGeometry(container.get(), material_c, ChVector3d(hdimX, hthick, hdimZ + hthick),
                          ChVector3d(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);

    // ----------------
    // Create particles
    // ----------------

    // Create contact material for granular material
    std::shared_ptr<ChContactMaterial> material_g;

    switch (method) {
        case ChContactMethod::SMC: {
            auto mat_g = chrono_types::make_shared<ChContactMaterialSMC>();
            mat_g->SetFriction(mu_g);
            mat_g->SetRestitution(cr_g);
            mat_g->SetYoungModulus(Y_g);
            mat_g->SetPoissonRatio(nu_g);
            mat_g->SetAdhesion(coh_g);
            mat_g->SetKn(kn_g);
            mat_g->SetGn(gn_g);
            mat_g->SetKt(kt_g);
            mat_g->SetGt(gt_g);

            material_g = mat_g;

            break;
        }
        case ChContactMethod::NSC: {
            auto mat_g = chrono_types::make_shared<ChContactMaterialNSC>();
            mat_g->SetFriction(mu_g);
            mat_g->SetRestitution(cr_g);
            mat_g->SetCohesion(coh_g);

            material_g = mat_g;

            break;
        }
    }

    // Create a particle generator and a mixture entirely made out of spheres
    utils::ChGenerator gen(system);
    std::shared_ptr<utils::ChMixtureIngredient> m1 = gen.AddMixtureIngredient(shape, 1.0);
    m1->SetDefaultMaterial(material_g);
    m1->SetDefaultDensity(rho_g);

    double v_sep;
    double h_sep;

    switch (shape) {
        case utils::MixtureType::SPHERE:
            if (randomized_size) {
                m1->setDistributionSize(0.75 * radius_g, 0.5 * radius_g, 0.1 * radius_g, radius_g);
            } else {
                m1->SetDefaultSize(radius_g);
            }
            v_sep = 2.001 * radius_g;
            h_sep = v_sep;
            break;
        case utils::MixtureType::BISPHERE:
            m1->SetDefaultSize(ChVector3d(radius_g, 0.75 * radius_g, 0));
            v_sep = 2.001 * radius_g;
            h_sep = 2.751 * radius_g;
            break;
    }

    utils::ChPDSampler<double> sampler(h_sep);

    // Set starting value for body identifiers
    gen.SetStartTag(Id_g);

    // Create particles in layers until reaching the desired number of particles
    ChVector3d hdims(hdimX - h_sep / 2, hdimY - h_sep / 2, 0);
    ChVector3d center(0, 0, v_sep);

    for (int il = 0; il < num_layers; il++) {
        gen.CreateObjectsBox(sampler, center, hdims);
        center.z() += v_sep;
    }

    unsigned int num_particles = gen.GetTotalNumBodies();
    std::cout << "Generated particles:  " << num_particles << std::endl;

    // -------------------------------
    // Create track body
    // -------------------------------

    // Create contact material for track body
    std::shared_ptr<ChContactMaterial> material_t;

    switch (method) {
        case ChContactMethod::SMC: {
            auto mat_t = chrono_types::make_shared<ChContactMaterialSMC>();
            mat_t->SetFriction(mu_t);
            mat_t->SetRestitution(cr_t);
            mat_t->SetYoungModulus(Y_t);
            mat_t->SetPoissonRatio(nu_t);
            mat_t->SetAdhesion(coh_t);
            mat_t->SetKn(kn_t);
            mat_t->SetGn(gn_t);
            mat_t->SetKt(kt_t);
            mat_t->SetGt(gt_t);

            material_t = mat_t;

            break;
        }
        case ChContactMethod::NSC: {
            auto mat_t = chrono_types::make_shared<ChContactMaterialNSC>();
            mat_t->SetFriction(mu_t);
            mat_t->SetRestitution(cr_t);
            mat_t->SetCohesion(coh_t);

            material_t = mat_t;

            break;
        }
    }

    auto track = chrono_types::make_shared<ChBody>();
    system->AddBody(track);

    track->SetTag(0);
    track->SetFixed(false);
    track->SetMass(mass_t);
    track->SetInertiaXX(inertia_t);
    track->SetPos(ChVector3d(0, 0, hZ + center.z()));
    track->SetRot(ChQuaternion<>(1, 0, 0, 0));
    track->EnableCollision(true);

    for (int i = -npads / 2; i < npads / 2; i++) {
        utils::AddBoxGeometry(track.get(), material_t, ChVector3d(hX, hY, hZ), ChVector3d(i * pitch, 0, 0));
    }

    // ---------------------------
    // Set active Bbox
    // ---------------------------

    system->GetSettings()->collision.use_aabb_active = true;
    real3 bmin(-hdimX - 0.5, -hdimY - 1.0, -hthick - 0.5);
    real3 bmax(+hdimX + 0.5, +hdimY + 1.0, +hdimZ + 0.5);
    system->GetSettings()->collision.aabb_min = bmin;
    system->GetSettings()->collision.aabb_max = bmax;

    real3 bsize = bmax - bmin;
    real3 bpos = 0.5 * (bmax + bmin);
    auto bbox = chrono_types::make_shared<ChVisualShapeBox>(bsize.x, bsize.y, bsize.z);
    container->AddVisualShape(bbox, ChFrame<>(ChVector3d(bpos.x, bpos.y, bpos.z)));

    // -------------------------------
    // Create output directories
    // -------------------------------

    std::string pov_dir = out_dir + "/POVRAY";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return 1;
    }

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    opengl::ChVisualSystemOpenGL vis;
    if (render) {
        vis.AttachSystem(system);
        vis.SetWindowTitle("Test");
        vis.SetWindowSize(1280, 720);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.AddCamera(ChVector3d(0, -4, center.z()), ChVector3d(0, 0, center.z()));
        vis.SetCameraVertical(CameraVerticalDir::Z);
    }

    // ---------------
    // Simulate system
    // ---------------

    int out_fps = 250;
    int out_steps = (int)std::ceil((1 / time_step) / out_fps);
    int sim_frame = 0;
    int out_frame = 0;
    int next_out_frame = 0;

    TimingHeader();

    while (system->GetChTime() < time_end) {
        if (sim_frame == next_out_frame) {
            char filename[100];

            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame);
            utils::WriteVisualizationAssets(system, filename);

            out_frame++;
            next_out_frame += out_steps;

            double height = track->GetPos().z() - hZ;
            std::cout << "---  " << height << std::endl;
        }

        system->DoStepDynamics(time_step);
        sim_frame++;

        TimingOutput(system);

        if (render) {
            if (vis.Run()) {
                vis.Render();
            } else {
                return 1;
            }
        }
    }

    return 0;
}