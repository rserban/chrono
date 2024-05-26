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
// Chrono test for box-triangle collision
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/multicore_math/utility.h"
#include "chrono/collision/multicore/ChCollisionUtils.h"
#include "chrono/collision/multicore/ChNarrowphase.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;

// --------------------------------------------------------------------------

class EvRec : public irr::IEventReceiver {
  public:
    EvRec(std::shared_ptr<ChBody> box) : m_box(box) {}
    virtual bool OnEvent(const irr::SEvent& event) override {
        if (event.EventType != irr::EET_KEY_INPUT_EVENT)
            return false;

        auto p = m_box->GetPos();
        auto q = m_box->GetRot();

        double dp = 0.01;
        double da = 1 * CH_DEG_TO_RAD;

        if (event.KeyInput.PressedDown) {
            bool translate = !event.KeyInput.Shift;
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_A:
                    if (translate)
                        m_box->SetPos(p + ChVector3d(-dp, 0, 0));
                    else
                        m_box->SetRot(q * QuatFromAngleX(-da));
                    return true;
                case irr::KEY_KEY_D:
                    if (translate)
                        m_box->SetPos(p + ChVector3d(dp, 0, 0));
                    else
                        m_box->SetRot(q * QuatFromAngleX(da));
                    return true;
                case irr::KEY_KEY_Q:
                    if (translate)
                        m_box->SetPos(p + ChVector3d(0, -dp, 0));
                    else
                        m_box->SetRot(q * QuatFromAngleY(-da));
                    return true;
                case irr::KEY_KEY_E:
                    if (translate)
                        m_box->SetPos(p + ChVector3d(0, dp, 0));
                    else
                        m_box->SetRot(q * QuatFromAngleY(da));
                    return true;
                case irr::KEY_KEY_W:
                    if (translate)
                        m_box->SetPos(p + ChVector3d(0, 0, dp));
                    else
                        m_box->SetRot(q * QuatFromAngleZ(da));
                    return true;
                case irr::KEY_KEY_S:
                    if (translate)
                        m_box->SetPos(p + ChVector3d(0, 0, -dp));
                    else
                        m_box->SetRot(q * QuatFromAngleZ(-da));
                    return true;
                default:
                    break;
            }
        }

        return false;
    }

    std::shared_ptr<ChBody> m_box;
};

// --------------------------------------------------------------------------

std::shared_ptr<ChTriangleMeshConnected> TriangleMesh(std::vector<ChVector3d> vertices) {
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    std::vector<ChVector3d>& n = trimesh->GetCoordsNormals();
    std::vector<ChVector2<>>& uv = trimesh->GetCoordsUV();
    std::vector<ChVector3<int>>& iv = trimesh->GetIndicesVertexes();
    std::vector<ChVector3<int>>& in = trimesh->GetIndicesNormals();

    trimesh->GetCoordsVertices() = vertices;
    n.resize(3);
    uv.resize(3);

    iv.resize(1);
    in.resize(1);

    n[0] = ChVector3d(0, 0, 1);
    n[1] = ChVector3d(0, 0, 1);
    n[2] = ChVector3d(0, 0, 1);

    uv[0] = ChVector2<>(0, 0);
    uv[1] = ChVector2<>(1, 0);
    uv[2] = ChVector2<>(0, 1);

    iv[0] = ChVector3<int>(0, 1, 2);
    in[0] = ChVector3<int>(0, 1, 2);

    return trimesh;
}

// --------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // ----------------
    // Parameters
    // ----------------

    double collision_envelope = .001;
    ChVector3d hdims(0.25, 0.25, 0.25);
    std::vector<ChVector3d> vertices = {ChVector3d(0, 0, 0), ChVector3d(1.5, 0, 0), ChVector3d(0, 1.5, 0)};
    double separation = 0.0;

    // -----------------
    // Create the system
    // -----------------

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    system.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // ----------
    // Add bodies
    // ----------
    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    auto triangle = chrono_types::make_shared<ChBody>();
    system.Add(triangle);
    triangle->SetPos(ChVector3d(0, 0, 0));
    triangle->SetFixed(true);

    auto trimesh = TriangleMesh(vertices);

    triangle->EnableCollision(true);
    triangle->GetCollisionModel()->SetEnvelope(collision_envelope);
    triangle->GetCollisionModel()->AddTriangleMesh(contact_mat, trimesh, false, false, ChVector3d(0), ChMatrix33<>(1));

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetColor(ChColor(0.6f, 0.6f, 0.6f));
    triangle->AddVisualShape(trimesh_shape);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0, 0, 0.5));
    box->EnableCollision(false);
    box->GetCollisionModel()->SetEnvelope(collision_envelope);
    utils::AddBoxGeometry(box.get(), contact_mat, hdims);

    box->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.3f, 0.4f));

    system.AddBody(box);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Box-triangle test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(2, 2, 1));
    vis->AttachSystem(&system);

    EvRec er(box);
    vis->AddUserEventReceiver(&er);

    // ---------------
    // Simulate system
    // ---------------

    auto hdims_r = FromChVector(hdims);

    ConvexShapeCustom shape_box;
    shape_box.type = ChCollisionShape::Type::BOX;
    shape_box.dimensions = hdims_r;
    auto A = FromChVector(vertices[0]);
    auto B = FromChVector(vertices[1]);
    auto C = FromChVector(vertices[2]);
    ConvexShapeTriangle shape_tri(A, B, C);

    while (vis->Run()) {
        system.DoStepDynamics(1e-3);

        // Express triangle vertices in current box frame
        auto v0 = FromChVector(box->TransformPointParentToLocal(vertices[0]));
        auto v1 = FromChVector(box->TransformPointParentToLocal(vertices[1]));
        auto v2 = FromChVector(box->TransformPointParentToLocal(vertices[2]));

        // Perform box-triangle collision test
        shape_box.position = FromChVector(box->GetPos());
        shape_box.rotation = FromChQuaternion(box->GetRot());
        real3 norm[6];
        real3 pt1[6];
        real3 pt2[6];
        real depth[6];
        real eff_rad[6];
        int nC = 0;
        ChNarrowphase::PRIMSCollision(&shape_box, &shape_tri, (real)separation, norm, pt1, pt2, depth,
                                                 eff_rad, nC);

        vis->BeginScene();
        vis->Render();
        irrlicht::tools::drawAllCOGs(vis.get(), 1.0);
        if (nC > 0) {
            assert(nC <= 6);
            for (int i = 0; i < nC; i++) {
                assert(separation > 0 || depth[i] < 0);
                irrlicht::tools::drawSegment(vis.get(), ToChVector(pt1[i]), ToChVector(pt2[i]), ChColor(1, 0, 1));
            }
        }
        vis->EndScene();
    }

    return 0;
}
