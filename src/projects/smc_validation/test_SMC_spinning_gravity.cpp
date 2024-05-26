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
// Simulation of a sphere spinning on a fixed plate. The sphere
// eventually comes to a rest due spinning friction.
//
// =============================================================================

#include "./test_SMC.h"

std::ofstream CreateDataFile() {
    const std::string fname = GetChronoOutputPath() + "/dat.csv";
    std::ofstream dat(fname, std::ofstream::out);

    // Write header
    dat << "force_model, body_id, time, radius, collision_map,"
        << "pos_x, pos_y, pos_z,"
        << "rot_0, rot_1, rot_2, rot_3,"
        << "vel_x, vel_y, vel_z,"
        << "rot_vel_x, rot_vel_y, rot_vel_z,"
        << "acc_x, acc_y, acc_z,"
        << "rot_acc_x, rot_acc_y, rot_acc_z,"
        << "force_x, force_y, force_z,"
        << "torque_x, torque_y, torque_z\n";

    return dat;
}

void WriteData(std::ofstream& dat, ChSystemMulticoreSMC* msystem, const std::string& str) {
    for (int i = 0; i < msystem->GetBodies().size(); ++i) {
        const std::shared_ptr<ChBody> body = msystem->GetBodies().at(i);

        // Get the radius of the object
        auto shape = std::static_pointer_cast<ChCollisionShapeChrono>(body->GetCollisionModel()->GetShape(0));
        double radius = 0;
        // I'm assuming that there is only one object per collision model. Fix this later.
        switch (shape->GetType()) {
            case ChCollisionShape::Type::SPHERE:
                radius = shape->B.x;
                break;
            case ChCollisionShape::Type::BOX:
                radius = 1.0e8;
                break;
            case ChCollisionShape::Type::CYLINDER:
                radius = shape->B.x;
                break;
        }

        // Add all other object information to the data structure
        dat << str.c_str() << "," << body->GetIdentifier() << "," << msystem->GetChTime() << "," << radius << ","
            << msystem->data_manager->host_data.ct_body_map[i] << "," << body->GetPos().x() << "," << body->GetPos().y()
            << "," << body->GetPos().z() << "," << body->GetRot().e0() << "," << body->GetRot().e1() << ","
            << body->GetRot().e2() << "," << body->GetRot().e3() << "," << body->GetPosDt().x() << ","
            << body->GetPosDt().y() << "," << body->GetPosDt().z() << "," << body->GetAngVelParent().x() << ","
            << body->GetAngVelParent().y() << "," << body->GetAngVelParent().z() << "," << body->GetPosDt2().x() << ","
            << body->GetPosDt2().y() << "," << body->GetPosDt2().z() << "," << body->GetWacc_par().x() << ","
            << body->GetWacc_par().y() << "," << body->GetWacc_par().z() << "," << msystem->GetBodyContactForce(body).x
            << "," << msystem->GetBodyContactForce(body).y << "," << msystem->GetBodyContactForce(body).z << ","
            << msystem->GetBodyContactTorque(body).x << "," << msystem->GetBodyContactTorque(body).y << ","
            << msystem->GetBodyContactTorque(body).z << "\n";
    }
}

int main(int argc, char* argv[]) {
    // Update accordingly. The rest of main should remain largely the same between projects.
    const bool archive = true;
    const std::string projname = "test_spinning_gravity";

    if (CreateOutputPath(projname) != 0) {
        fprintf(stderr, "Error creating output data directory\n");
        return -1;
    }

    auto dat = CreateDataFile();

    // Print the sim set - up parameters to userlog
    std::cout << "\nCopyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n";
    std::cout << "\nTesting SMC multicore spinning with gravity behavior....\n";

    // Execute test for each force model
    std::vector<std::string> fmodels = {"hooke", "hertz", "flores"};
    for (int f = 0; f < fmodels.size(); ++f) {
        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  /// Default 2e5
        float p_ratio = 0.3f;      /// Default 0.3f
        float s_frict = 0.3f;      /// Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.3f;      /// Default 0.6f
        float roll_frict = 0.0f;   /// Usually around 1E-3
        float spin_frict = 0.2f;   /// Usually around 1E-3
        float cor_in = 0.0f;       /// Default 0.4f
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
        double time_sim = 2.0;

        ChVector3d gravity(0, -9.81, 0);

        ChSystemMulticoreSMC msystem;
        SetSimParameters(&msystem, gravity, force_to_enum(fmodels[f]));
        msystem.SetNumThreads(2);

        // Add the wall to the system
        int id = -1;
        double wmass = 10.0;
        ChVector3d wsize(4, 1, 4);
        ChVector3d wpos(0, -wsize.y() / 2, 0);

        auto wall = AddWall(id, &msystem, mat, wsize, wmass, wpos, ChVector3d(0, 0, 0), true);

        // Add the sphere to the system
        double srad = 0.5;
        double smass = 1.0;
        ChVector3d spos(0, srad + 1e-2, 0);
        ChVector3d init_v(0, 0, 0);

        auto body1 = AddSphere(++id, &msystem, mat, srad, smass, spos, init_v);

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

        // Let the block settle of the plate before giving it a push
        while (time < time_sim) {
            while (time == 0 || time < out_time) {
                msystem.DoStepDynamics(time_step);
                time += time_step;
            }

            out_time = time - time_step + out_step;

            bool KEthresh = CalcKE(&msystem, 1.0E-9);
            if (KEthresh) {
                std::cout << "[settling] KE exceeds threshold at t = " << time << "\n";
                break;
            }
        }

        // Write the initial state date and give the sphere an initial spin
        WriteData(dat, &msystem, fmodels[f].c_str());
        ChVector3d init_w = ChVector3d(0, 1, 0);
        body1->SetAngVelParent(init_w);

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

            // Print state data for the block only
            WriteData(dat, &msystem, fmodels[f].c_str());

            // Calculate the average velocity of all particles and exit the loop if KE < threshold
            bool KEthresh = CalcKE(&msystem, 1.0E-9);
            if (KEthresh) {
                std::cout << "[simulation] KE exceeds threshold at t = " << time << "\n";
                break;
            }
        }

        // Check results. The sphere's rotational velocity should be < 1e-3.
        double wvel_body1 = body1->GetAngVelParent().Length();
    }

    dat.close();

    return 0;
}