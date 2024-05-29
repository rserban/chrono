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
// Author: Wei Hu, Radu Serban
//
// Chrono::FSI demo to show usage of tracked vehicle M113 on SPH terrain
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <iostream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// ----------------------------------------------------------------------------------------

// Pointer to store the vehicle instance
std::shared_ptr<M113> track;

double rock_scale = 0.5;

// Simulation time and stepsize
double total_time = 30.0;
double dT = 5e-4;

// Output
const std::string out_dir = GetChronoOutputPath() + "FSI_M113/";
bool output = true;
int out_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// ----------------------------------------------------------------------------------------

void CreateMeshMarkers(std::shared_ptr<ChTriangleMeshConnected> mesh,
                       double delta,
                       std::vector<ChVector3d>& point_cloud);

std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method);

void AddWall(std::shared_ptr<ChBody> body,
             const ChVector3d& dim,
             std::shared_ptr<ChContactMaterial> mat,
             const ChVector3d& loc);

void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI, const ChVector3d& box_dims);

// ----------------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    /// Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    /// Read parameters from JSON file
    std::string inputJson = GetChronoDataFile("fsi/input_json/test_VEH_SPH_M113.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_Granular_Viper <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    sysFSI.SetStepSize(dT);

    // Set the periodic boundary condition
    ChVector3d box_dims(10, 10, 0.24);
    double bxDim = box_dims.x();
    double byDim = box_dims.y();
    double bzDim = box_dims.z();

    double initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 * 1.5, -byDim / 2 * 1.5, -bzDim * 20);
    ChVector3d cMax(bxDim / 2 * 1.5, byDim / 2 * 1.5, bzDim * 20);
    sysFSI.SetBoundaries(cMin, cMax);

    // Setup the output directory for FSI data
    sysFSI.SetOutputLength(0);

    // Create an initial box for the terrain patch
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(0, 0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);
    /// Add SPH particles from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * abs(gravity.z()) * (-points[i].z() + bzDim);
        sysFSI.AddSPHParticle(points[i], sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
                              ChVector3d(0),         // initial velocity
                              ChVector3d(-pre_ini),  // tauxxyyzz
                              ChVector3d(0)          // tauxyxzyz
        );
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI, box_dims);
    ChDataDriver driver(track->GetVehicle(), vehicle::GetDataFile("M113/driver/Acceleration.txt"));
    driver.Initialize();

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Create oputput directories and files
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::vector<std::ofstream> outfiles;
    for (int i = 0; i < sysMBS.GetBodies().size(); i++) {
        std::cout << "Body " << i << ": " << sysMBS.GetBodies()[i]->GetName() << std::endl;

        std::string filename = out_dir + "/body" + std::to_string(i) + ".csv";
        outfiles.push_back(std::ofstream(filename));
        outfiles.back() << "Time, x, y, z, q0, q1, q2, q3, Vx, Vy, Vz" << std::endl;
    }

    // Add a force to the chassis to push the vehicle
    std::shared_ptr<ChBody> m113_chassis = track->GetVehicle().GetChassisBody();
    double force_base = 5000;
    ChVector3d force_on_chassis(force_base * cos(0.25 * CH_PI), force_base * sin(0.25 * CH_PI), 0);

    // Simulation loop
    double time = 0.0;
    int current_step = 0;

    unsigned int output_steps = (unsigned int)(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * dT));

    TerrainForces shoe_forces_left(track->GetVehicle().GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(track->GetVehicle().GetNumTrackShoes(RIGHT));

    std::string delim = ",";

    ChTimer timer;
    timer.start();
    while (time < total_time) {
        printf("\nstep : %d, time= : %f (s) \n", current_step, time);

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;

            sysFSI.PrintParticleToFile(out_dir);

            for (int i = 1; i < sysMBS.GetBodies().size(); i++) {
                auto body = sysMBS.GetBodies()[i];
                ChFrame<> ref_frame = body->GetFrameRefToAbs();
                ChVector3d pos = ref_frame.GetPos();
                ChVector3d vel = body->GetPosDt();
                ChQuaternion<> rot = ref_frame.GetRot();

                outfiles[i] << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
                            << rot.e0() << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim
                            << vel.x() << delim << vel.y() << delim << vel.z() << std::endl;
            }
        }

        // output inf to screen
        auto bbody = sysMBS.GetBodies()[0];
        auto vbody = sysMBS.GetBodies()[1];
        printf("bin=%f,%f,%f\n", bbody->GetPos().x(), bbody->GetPos().y(), bbody->GetPos().z());
        printf("M113 pos = %f,%f,%f\n", vbody->GetPos().x(), vbody->GetPos().y(), vbody->GetPos().z());
        printf("M113 vel = %f,%f,%f\n", vbody->GetPosDt().x(), vbody->GetPosDt().y(), vbody->GetPosDt().z());

        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        track->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance simulation for one timestep for all modules
        // driver.Advance(paramsH->dT_Max);
        // track->Advance(paramsH->dT_Max);

        // Do step dynamics
        timer.start();
        sysFSI.DoStepDynamics_FSI();
        timer.stop();

        time += dT;
        current_step++;
    }

    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    for (int i = 0; i < sysMBS.GetBodies().size(); i++) {
        outfiles[i].close();
    }

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI, const ChVector3d& box_dims) {
    // Set common material Properties
    auto cmaterial = chrono_types::make_shared<ChContactMaterialNSC>();
    cmaterial->SetFriction(0.9);
    cmaterial->SetRestitution(0.4);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(100, 100, 0.02, 1000, false, true, cmaterial);
    box->SetPos(ChVector3d(0, 0, 0));
    box->SetFixed(true);
    sysMBS.Add(box);

    // Fluid-Solid Coupling at the walls via BCE particles
    sysFSI.AddBoxContainerBCE(box, ChFrame<>(), 2.0 * box_dims + ChVector3d(0, 0, 2.0 * box_dims.z()), ChVector3<int>(2, 0, -1));

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------
    track = chrono_types::make_shared<M113>(&sysMBS);
    track->SetContactMethod(ChContactMethod::NSC);
    track->SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    track->SetBrakeType(BrakeType::SIMPLE);
    // track->SetDrivelineType(DrivelineTypeTV::BDS);
    track->SetEngineType(EngineModelType::SIMPLE_MAP);
    track->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    track->SetChassisCollisionType(CollisionType::NONE);

    // Initialize the vehicle at the specified position
    ChVector3d initLoc(-1, -1, 1.2);
    auto initRot = QuatFromAngleSet({RotRepresentation::CARDAN_ANGLES_XYZ, ChVector3d(0, 0, 0.25 * CH_PI)});
    track->SetInitPosition(ChCoordsys<>(initLoc, initRot));
    track->Initialize();

    // Get the number of shoes: left and right
    int num_shoe_L = track->GetVehicle().GetNumTrackShoes(LEFT);
    int num_shoe_R = track->GetVehicle().GetNumTrackShoes(RIGHT);
    std::cout << "number of shoes left = " << num_shoe_L << std::endl;
    std::cout << "number of shoes right = " << num_shoe_R << std::endl;

    // Add BCE for each shoe
    for (int i = 0; i < num_shoe_L; i++) {
        auto track_L = track->GetVehicle().GetTrackAssembly(LEFT);
        auto shoe_body = track_L->GetTrackShoe(i)->GetShoeBody();
        sysFSI.AddFsiBody(shoe_body);
        sysFSI.AddBoxBCE(shoe_body, ChFrame<>(VNULL, QUNIT), ChVector3d(0.06, 0.18, 0.04), true);
    }
    for (int i = 0; i < num_shoe_R; i++) {
        auto track_R = track->GetVehicle().GetTrackAssembly(RIGHT);
        auto shoe_body = track_R->GetTrackShoe(i)->GetShoeBody();
        sysFSI.AddFsiBody(shoe_body);
        sysFSI.AddBoxBCE(shoe_body, ChFrame<>(VNULL, QUNIT), ChVector3d(0.06, 0.18, 0.04), true);
    }

    /// Add some rocks on the terrain
    /*std::vector<ChVector3d> BCE_par_rock;
    int n_r = 0;
    for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
        std::string rock_name = "rock" + std::to_string(n_r+1);
        // Load mesh from obj file
        auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "./rock3.obj";
        trimesh->LoadWavefrontMesh(obj_path, true, true);
        double scale_ratio = rock_scale;
        // trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(body_rot));    // rotate the mesh if needed
        trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));    // scale to a different size
        // trimesh->RepairDuplicateVertexes(1e-9);                             // if meshes are not watertight

        // Compute mass inertia from mesh
        double mmass;// = 5.0;
        double mdensity = paramsH->bodyDensity;
        ChVector3d mcog;// = ChVector3d(0.0, 0.0, 0.0);
        ChMatrix33<> minertia;
        trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChVector3d principal_I;// = ChVector3d(1.0, 1.0, 1.0);
        ChMatrix33<> principal_inertia_rot;// = ChMatrix33<>(1.0);
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // Set the abs orientation, position
        auto rock_body = chrono_types::make_shared<ChBodyAuxRef>();
        rock_body->SetName(rock_name);
        double rot_ang_x, rot_ang_y, rot_ang_z;
        ChVector3d rock_rel_pos;

        // Set initial pos and rot
        rot_ang_x = i * 30.0 / 180.0 * CH_PI;
        rot_ang_y = j * 45.0 / 180.0 * CH_PI;
        rot_ang_z = i * 60.0 / 180.0 * CH_PI;
        double det_x = 0.1 * pow(-1.0, i + j);
        double det_y = 0.1 * pow(-1.0, i + j + 1);
        ChVector3d rock_pos = ChVector3d(-2.0 + i * 1.0 + det_x, -2.0 + j * 1.0 + det_y, 0.8);
        ChQuaternion<> rock_rot = QuatFromAngleSet({RotRepresentation::CARDAN_ANGLES_XYZ, 
                                                    ChVector3d(rot_ang_x, rot_ang_y, rot_ang_z)});

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        rock_body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        // Set inertia
        std::cout << "\n" << "The mass of the rock is " << mmass * mdensity << "\n" << std::endl;
        rock_body->SetMass(mmass * mdensity);
        rock_body->SetInertiaXX(mdensity * principal_I);

        // Set the absolute position of the body:
        rock_body->SetFrameRefToAbs(ChFrame<>(ChVector3d(rock_pos),ChQuaternion<>(rock_rot)));
        sysMBS.Add(rock_body);

        // Set collision
        rock_body->SetFixed(false);
        rock_body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial, trimesh, false, false, 0.005));
        rock_body->EnableCollision(true);

        // Create BCE particles associated with mesh
        if(i==0){
            CreateMeshMarkers(trimesh, (double)initSpace0, BCE_par_rock);
        }
        sysFSI.AddFsiBody(rock_body);
        sysFSI.AddBceFromPoints(paramsH, rock_body, BCE_par_rock, ChVector3d(0.0), QUNIT);
        n_r++;
    }
    }*/
}

//------------------------------------------------------------------

void AddWall(std::shared_ptr<ChBody> body,
             const ChVector3d& dim,
             std::shared_ptr<ChContactMaterial> mat,
             const ChVector3d& loc) {
    body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(mat, 2 * dim.x(), 2 * dim.y(), 2 * dim.z()), loc);
    auto box = chrono_types::make_shared<ChVisualShapeBox>(2 * dim.x(), 2 * dim.y(), 2 * dim.z());
    body->AddVisualShape(box, ChFrame<>(loc));
}

void CreateMeshMarkers(std::shared_ptr<ChTriangleMeshConnected> mesh,
                       double delta,
                       std::vector<ChVector3d>& point_cloud) {
    mesh->RepairDuplicateVertexes(1e-9);  // if meshes are not watertight

    ChVector3d minV = mesh->m_vertices[0];
    ChVector3d maxV = mesh->m_vertices[0];
    ChVector3d currV = mesh->m_vertices[0];
    for (unsigned int i = 1; i < mesh->m_vertices.size(); ++i) {
        currV = mesh->m_vertices[i];
        if (minV.x() > currV.x())
            minV.x() = currV.x();
        if (minV.y() > currV.y())
            minV.y() = currV.y();
        if (minV.z() > currV.z())
            minV.z() = currV.z();
        if (maxV.x() < currV.x())
            maxV.x() = currV.x();
        if (maxV.y() < currV.y())
            maxV.y() = currV.y();
        if (maxV.z() < currV.z())
            maxV.z() = currV.z();
    }
    ////printf("start coords: %f, %f, %f\n", minV.x(), minV.y(), minV.z());
    ////printf("end coords: %f, %f, %f\n", maxV.x(), maxV.y(), maxV.z());

    const double EPSI = 1e-6;

    ChVector3d ray_origin;
    for (double x = minV.x(); x < maxV.x(); x += delta) {
        ray_origin.x() = x + 1e-9;
        for (double y = minV.y(); y < maxV.y(); y += delta) {
            ray_origin.y() = y + 1e-9;
            for (double z = minV.z(); z < maxV.z(); z += delta) {
                ray_origin.z() = z + 1e-9;

                ChVector3d ray_dir[2] = {ChVector3d(5, 0.5, 0.25), ChVector3d(-3, 0.7, 10)};
                int intersectCounter[2] = {0, 0};

                for (unsigned int i = 0; i < mesh->m_face_v_indices.size(); ++i) {
                    auto& t_face = mesh->m_face_v_indices[i];
                    auto& v1 = mesh->m_vertices[t_face.x()];
                    auto& v2 = mesh->m_vertices[t_face.y()];
                    auto& v3 = mesh->m_vertices[t_face.z()];

                    // Find vectors for two edges sharing V1
                    auto edge1 = v2 - v1;
                    auto edge2 = v3 - v1;

                    bool t_inter[2] = {false, false};

                    for (unsigned int j = 0; j < 2; j++) {
                        // Begin calculating determinant - also used to calculate uu parameter
                        auto pvec = Vcross(ray_dir[j], edge2);
                        // if determinant is near zero, ray is parallel to plane of triangle
                        double det = Vdot(edge1, pvec);
                        // NOT CULLING
                        if (det > -EPSI && det < EPSI) {
                            t_inter[j] = false;
                            continue;
                        }
                        double inv_det = 1.0 / det;

                        // calculate distance from V1 to ray origin
                        auto tvec = ray_origin - v1;

                        // Calculate uu parameter and test bound
                        double uu = Vdot(tvec, pvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (uu < 0.0 || uu > 1.0) {
                            t_inter[j] = false;
                            continue;
                        }

                        // Prepare to test vv parameter
                        auto qvec = Vcross(tvec, edge1);

                        // Calculate vv parameter and test bound
                        double vv = Vdot(ray_dir[j], qvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (vv < 0.0 || ((uu + vv) > 1.0)) {
                            t_inter[j] = false;
                            continue;
                        }

                        double tt = Vdot(edge2, qvec) * inv_det;
                        if (tt > EPSI) {  // ray intersection
                            t_inter[j] = true;
                            continue;
                        }

                        // No hit, no win
                        t_inter[j] = false;
                    }

                    intersectCounter[0] += t_inter[0] ? 1 : 0;
                    intersectCounter[1] += t_inter[1] ? 1 : 0;
                }

                if (((intersectCounter[0] % 2) == 1) && ((intersectCounter[1] % 2) == 1))  // inside mesh
                    point_cloud.push_back(ChVector3d(x, y, z));
            }
        }
    }
}

std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.9f;   // coefficient of friction
    float cr = 0.4f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}
