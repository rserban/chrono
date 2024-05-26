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
// Create an off-angle collision of a sphere on a plate and measure the
// sphere's final rotational velocity and tangential coefficient of restitution
//
// =============================================================================

#include "./test_SMC.h"

int main(int argc, char* argv[]) {
    // Update accordingly. The rest of main should remain largely the same between projects.
    const std::string projname = "test_cor_oblique";

    if (CreateOutputPath(projname) != 0) {
        fprintf(stderr, "Error creating output data directory\n");
        return -1;
    }

    // Print the sim set - up parameters to userlog
    std::cout << "\nCopyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << ".VCMS\n";
    std::cout << "\nTesting SMC multicore oblique impacts....\n";

    // Create a file for logging data
    const std::string fname = GetChronoOutputPath() + "/dat.csv";
    std::ofstream dat(fname, std::ofstream::out);
    dat << "force_model, theta, cor_in, cor_out, cor_out_n, cor_out_t, cor_out_t_thy, w_out_act, w_out_thy, "
           "w_out_err\n";

    // Execute test for each force model
    std::vector<std::string> fmodels = {"hooke", "hertz", "plaincoulomb", "flores"};
    for (int f = 0; f < fmodels.size(); ++f) {
        bool pass = false;
        for (int var = 1; var <= 44; var += 4) {
            // Create a shared material to be used by the all bodies
            float y_modulus = 2.0e5f;  /// Default 2e5
            float p_ratio = 0.3f;      /// Default 0.3f
            float s_frict = 0.3f;      /// Usually in 0.1 range, rarely above. Default 0.6f
            float k_frict = 0.3f;      /// Default 0.6f
            float roll_frict = 0.0f;   /// Usually around 1E-3
            float spin_frict = 0.0f;   /// Usually around 1E-3
            float cor_in = 1.0f;       /// Default 0.4f
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
            double time_step = 1.0E-5;
            double out_step = 1.0E-2;
            double time_sim = 0.5;

            ChVector3d gravity(0, 0, 0);

            ChSystemMulticoreSMC msystem;
            SetSimParameters(&msystem, gravity, force_to_enum(fmodels[f]));
            msystem.SetNumThreads(2);

            // Add the wall to the system
            double wmass = 10.0;
            ChVector3d wsize(20, 1, 4);
            ChVector3d wpos(0, -wsize.y() / 2, 0);

            auto wall = AddWall(-1, &msystem, mat, wsize, wmass, wpos, ChVector3d(0, 0, 0), true);

            // Add the sphere to the system
            double srad = 0.5;
            double smass = 1.0;
            ChVector3d spos(0, srad * 1.25, 0);
            ChVector3d init_v(tan((var * 2.0 * CH_PI) / 180), -1, 0);

            auto body = AddSphere(0, &msystem, mat, srad, smass, spos, init_v);

            // Create the Irrlicht visualization.
#ifdef CHRONO_IRRLICHT
            auto vis = SetSimVis(&msystem, time_step);
#endif

            // Print the sim set - up parameters to userlog once
            if (f == 0 && var == 1) {
                std::cout << "\ntime_step, " << time_step << "\nout_step, " << out_step << "\ntotal_sim_time, "
                         << time_sim << "\nadhesion_model, "
                         << static_cast<int>(msystem.GetSettings()->solver.adhesion_force_model)
                         << "\ntangential_displ_model, "
                         << static_cast<int>(msystem.GetSettings()->solver.tangential_displ_mode) << "\ntimestepper, "
                         << static_cast<int>(msystem.GetTimestepperType()) << "\ngravity_x, " << gravity.x()
                         << "\ngravity_y, " << gravity.y() << "\ngravity_z, " << gravity.z() << "\nyoungs_modulus, "
                         << y_modulus << "\npoissons_ratio, " << p_ratio << "\nstatic_friction, " << s_frict
                         << "\nkinetic_friction, " << k_frict << "\nrolling_friction, " << roll_frict
                         << "\nspinning_friction, " << spin_frict << "\ncor, " << cor_in << "\nconstant_adhesion, "
                         << ad << "\nDMT_adhesion_multiplier, " << adDMT << "\nperko_adhesion_multiplier, " << adSPerko
                         << "\n";
            }
            std::cout << "\nModel #" << f << " (" << fmodels[f].c_str() << ") var setting #" << var << "\n";

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
            }

            // Calculate and log COR values for each impact angle
            // Check results. Passes test if w_out_err < 1.0e-3 when theta >~ 66 deg
            real theta = atan(abs(init_v.x() / init_v.y()));
            real cor_out_t_thy = 1 - (k_frict * (1 + cor_in) / tan(theta));

            real cor_out_n = abs(body->GetPosDt().y() / init_v.y());
            real cor_out_t = abs(body->GetPosDt().x() / init_v.x());
            real cor_out = body->GetPosDt().Length() / init_v.Length();

            real w_out_thy = (5.0 / 2) * (k_frict / srad) * (1.0 + cor_in) * init_v.y();
            real w_out_act = body->GetAngVelParent().z();
            real w_out_err = abs((w_out_thy - w_out_act) / w_out_thy) * 100;

            dat << fmodels[f].c_str() << "," << theta * 180 / CH_PI << "," << cor_in << "," << cor_out << ","
                << cor_out_n << "," << cor_out_t << "," << cor_out_t_thy << "," << w_out_act << "," << w_out_thy << ","
                << w_out_err << "\n";
        }
    }

    dat.close();

    return 0;
}
