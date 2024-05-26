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
// Authors: Cecily Sunday, Radu Serban
// =============================================================================
//
// This project simulates 5 spheres dropping on top of on another and ending in
// a stacked state. See Asmar et al. (2002) DEM Validation Test 4.
//
// =============================================================================

#include "./test_SMC.h"

int main(int argc, char* argv[]) {
    // Update accordingly. The rest of main should remain largely the same between projects.
    const bool archive = true;
    const std::string projname = "test_stacking";

    if (CreateOutputPath(projname) != 0) {
        fprintf(stderr, "Error creating output data directory\n");
        return -1;
    }

    // Print the sim set - up parameters to userlog
    std::cout << "\nCopyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << ".VCMS\n";
    std::cout << "\nTesting SMC multicore stacking behavior....\n";

    // Execute test for each force model
    std::vector<std::string> fmodels = {"hooke", "hertz", "plaincoulomb", "flores"};

    for (int f = 0; f < fmodels.size(); ++f) {
        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  /// Default 2e5
        float p_ratio = 0.3f;      /// Default 0.3f
        float s_frict = 0.3f;      /// Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.3f;      /// Default 0.6f
        float roll_frict = 0.0f;   /// Usually around 1E-3
        float spin_frict = 0.0f;   /// Usually around 1E-3
        float cor_in = 0.3f;       /// Default 0.4f
        float ad = 0.0f;           /// Magnitude of the adhesion in the Constant adhesion model
        float adDMT = 0.0f;        /// Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;     /// Magnitude of the adhesion in the SPerko adhesion model

        auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
        mat->SetYoungModulus(y_modulus);
        mat->SetPoissonRatio(p_ratio);
        mat->SetStaticFriction(s_frict);
        mat->SetSlidingFriction(k_frict);
        mat->SetRollingFriction(roll_frict);
        mat->SetSpinningFriction(spin_frict);
        mat->SetRestitution(cor_in);
        mat->SetAdhesion(ad);
        mat->SetAdhesionMultDMT(adDMT);
        mat->SetAdhesionSPerko(adSPerko);

        // Create a multicore SMC system and set the system parameters
        double time_step = 1.0E-4;
        double out_step = 1.0E-2;
        double time_sim = 10.0;

        ChVector3d gravity(0, -9.81, 0);

        ChSystemMulticoreSMC msystem;
        SetSimParameters(&msystem, gravity, force_to_enum(fmodels[f]));
        msystem.SetNumThreads(2);

        // Add the wall to the system
        double wmass = 10.0;
        ChVector3d wsize(8, 1, 8);
        ChVector3d wpos(0, -wsize.y() / 2 - 3, 0);

        auto wall = AddWall(-1, &msystem, mat, wsize, wmass, wpos, ChVector3d(0, 0, 0), true);

        // Add the spheres to the system
        double srad = 1.0;
        double smass = 1.0;
        ChVector3d spos(0, 11, 0);

        for (int sid = 0; sid < 5; ++sid) {
            auto body = AddSphere(sid, &msystem, mat, srad, smass, spos + ChVector3d(0, srad * 3.0 * sid, 0),
                                  ChVector3d(0, 0, 0));
        }

        // Create the Irrlicht visualization.
#ifdef CHRONO_IRRLICHT
        auto vis = SetSimVis(&msystem, time_step);
#endif

        // Print the sim set - up parameters to userlog once
        if (f == 0) {
            std::cout << "\ntime_step, " << time_step << "\nout_step, " << out_step << "\ntotal_sim_time, " << time_sim
                     << "\nadhesion_model, " << static_cast<int>(msystem.GetSettings()->solver.adhesion_force_model)
                     << "\ntangential_displ_model, "
                     << static_cast<int>(msystem.GetSettings()->solver.tangential_displ_mode) << "\ntimestepper, "
                     << static_cast<int>(msystem.GetTimestepperType()) << "\ngravity_x, " << gravity.x()
                     << "\ngravity_y, " << gravity.y() << "\ngravity_z, " << gravity.z() << "\nyoungs_modulus, "
                     << y_modulus << "\npoissons_ratio, " << p_ratio << "\nstatic_friction, " << s_frict
                     << "\nkinetic_friction, " << k_frict << "\nrolling_friction, " << roll_frict
                     << "\nspinning_friction, " << spin_frict << "\ncor, " << cor_in << "\nconstant_adhesion, " << ad
                     << "\nDMT_adhesion_multiplier, " << adDMT << "\nperko_adhesion_multiplier, " << adSPerko << "\n";
        }
        std::cout << "\nModel #" << f << " (" << fmodels[f].c_str() << ")\n";

        // Set the soft-real-time cycle parameters
        double time = 0.0;
        double out_time = 0.0;

        // Iterate through simulation. Calculate resultant forces and motion for each timestep
        while (time < time_sim) {
#ifdef CHRONO_IRRLICHT
            vis->BeginScene();
            vis->Render();
#endif

            while (time == 0 || time < out_time) {
                msystem.DoStepDynamics(time_step);
                time += time_step;
            }

            out_time = time - time_step + out_step;

#ifdef CHRONO_IRRLICHT
            vis->EndScene();
#endif

            // Calculate the average velocity of all particles and exit the loop if KE < threshold
            bool KEthresh = CalcAverageKE(&msystem, 1.0E-9);
            if (KEthresh) {
                std::cout << "[simulation] KE exceeds threshold at t = " << time << "\n";
                break;
            }
        }

        // Check results. The spheres should be in a stack with their (x,y) positions at (0,0).
        // The rotation quaternions should be (1, 0, 0, 0)
        for (int i = 0; i < msystem.GetBodies().size(); ++i) {
            const std::shared_ptr<ChBody> body = msystem.GetBodies().at(i);

            ChQuaternion<> ref = ChQuaternion<>(1, 0, 0, 0);
            ChQuaternion<> rot = body->GetRot();
            ChVector3d pos = body->GetPos();
        }
    }

    return 0;
}
