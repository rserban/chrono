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
// This project simulates the collision of a sphere against another sphere and
// compares the measured coefficient of restitution against the input
// coefficient of restitution
//
// =============================================================================

#include "./test_SMC.h"

int main(int argc, char* argv[]) {
    // Update accordingly. The rest of main should remain largely the same between projects.
    const std::string projname = "test_cor_normal";

    if (CreateOutputPath(projname) != 0) {
        fprintf(stderr, "Error creating output data directory\n");
        return -1;
    }

    // Print the sim set - up parameters to userlog
    std::cout << "\nCopyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << ".VCMS\n";
    std::cout << "\nTesting SMC multicore coefficient of restitution....\n";

    // Create a file for logging data
    const std::string fname = GetChronoOutputPath() + "/dat.csv";
    std::ofstream dat(fname, std::ofstream::out);
    dat << "force_model, cor_in, cor_out, e\n";

    // Execute test for each force model
    std::vector<std::string> fmodels = {"hooke", "hertz", "plaincoulomb", "flores"};

    for (int f = 0; f < fmodels.size(); ++f) {
        for (int var = 0; var <= 100; var += 10) {
            // Create a shared material to be used by the all bodies
            float y_modulus = 2.0e5f;         /// Default 2e5
            float p_ratio = 0.3f;             /// Default 0.3f
            float s_frict = 0.3f;             /// Usually in 0.1 range, rarely above. Default 0.6f
            float k_frict = 0.3f;             /// Default 0.6f
            float roll_frict = 0.0f;          /// Usually around 1E-3
            float spin_frict = 0.0f;          /// Usually around 1E-3
            float cor_in = (float)var / 100;  /// Default 0.4f
            float ad = 0.0f;                  /// Magnitude of the adhesion in the Constant adhesion model
            float adDMT = 0.0f;               /// Magnitude of the adhesion in the DMT adhesion model
            float adSPerko = 0.0f;            /// Magnitude of the adhesion in the SPerko adhesion model

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

            // Add the colliding spheres to the system
            double rad = 0.5;
            double mass = 1.0;

            ChVector3d pos(0, rad * 1.25, 0);
            ChVector3d init_v(0, -1, 0);

            auto body1 = AddSphere(0, &msystem, mat, rad, mass, pos, init_v);
            auto body2 = AddSphere(1, &msystem, mat, rad, mass, pos * -1, init_v * -1);

            // Create the Irrlicht visualization.
#ifdef CHRONO_IRRLICHT
            auto vis = SetSimVis(&msystem, time_step);
#endif

            // Calculate motion parameters prior to collision
            ChVector3d rel_v_in = body2->GetPosDt() - body1->GetPosDt();
            real rel_vm_in = rel_v_in.Length();

            // Print the sim set - up parameters to userlog once
            if (f == 0 && var == 0) {
                std::cout << "\ntime_step, " << time_step << "\nout_step, " << out_step << "\ntotal_sim_time, "
                         << time_sim << "\nadhesion_model, "
                         << static_cast<int>(msystem.GetSettings()->solver.adhesion_force_model)
                         << "\ntangential_displ_model, "
                         << static_cast<int>(msystem.GetSettings()->solver.tangential_displ_mode) << "\ntimestepper, "
                         << static_cast<int>(msystem.GetTimestepperType()) << "\ngravity_x, " << gravity.x()
                         << "\ngravity_y, " << gravity.y() << "\ngravity_z, " << gravity.z() << "\nyoungs_modulus, "
                         << y_modulus << "\npoissons_ratio, " << p_ratio << "\nstatic_friction, " << s_frict
                         << "\nkinetic_friction, " << k_frict << "\nrolling_friction, " << roll_frict
                         << "\nspinning_friction, " << spin_frict << "\ncor, "
                         << "0 - 1"
                         << "\nconstant_adhesion, " << ad << "\nDMT_adhesion_multiplier, " << adDMT
                         << "\nperko_adhesion_multiplier, " << adSPerko << "\n";
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

            // Check results. Should fail test if abs(cor_dif) >= 1.0e-3
            ChVector3d rel_v_out = body2->GetPosDt() - body1->GetPosDt();
            real rel_vm_out = rel_v_out.Length();

            real cor_out = rel_vm_out / rel_vm_in;
            real cor_dif = cor_in - cor_out;

            dat << fmodels[f].c_str() << "," << cor_in << "," << cor_out << "," << abs(cor_dif) << "\n";
        }
    }

    dat.close();

    return 0;
}