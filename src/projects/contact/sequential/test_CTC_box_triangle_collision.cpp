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
#include "chrono/collision/chrono/ChCollisionUtils.h"
#include "chrono/collision/chrono/ChNarrowphase.h"

#include "chrono_irrlicht/ChIrrApp.h"

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
        double da = 1 * CH_C_DEG_TO_RAD;

        if (event.KeyInput.PressedDown) {
            bool translate = !event.KeyInput.Shift;
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_A:
                    if (translate)
                        m_box->SetPos(p + ChVector<>(-dp, 0, 0));
                    else
                        m_box->SetRot(q * Q_from_AngX(-da));
                    return true;
                case irr::KEY_KEY_D:
                    if (translate)
                        m_box->SetPos(p + ChVector<>(dp, 0, 0));
                    else
                        m_box->SetRot(q * Q_from_AngX(da));
                    return true;
                case irr::KEY_KEY_Q:
                    if (translate)
                        m_box->SetPos(p + ChVector<>(0, -dp, 0));
                    else
                        m_box->SetRot(q * Q_from_AngY(-da));
                    return true;
                case irr::KEY_KEY_E:
                    if (translate)
                        m_box->SetPos(p + ChVector<>(0, dp, 0));
                    else
                        m_box->SetRot(q * Q_from_AngY(da));
                    return true;
                case irr::KEY_KEY_W:
                    if (translate)
                        m_box->SetPos(p + ChVector<>(0, 0, dp));
                    else
                        m_box->SetRot(q * Q_from_AngZ(da));
                    return true;
                case irr::KEY_KEY_S:
                    if (translate)
                        m_box->SetPos(p + ChVector<>(0, 0, -dp));
                    else
                        m_box->SetRot(q * Q_from_AngZ(-da));
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

std::shared_ptr<geometry::ChTriangleMeshConnected> TriangleMesh(std::vector<ChVector<>> vertices) {
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    std::vector<ChVector<>>& n = trimesh->getCoordsNormals();
    std::vector<ChVector<>>& uv = trimesh->getCoordsUV();
    std::vector<ChVector<int>>& iv = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& in = trimesh->getIndicesNormals();

    trimesh->getCoordsVertices() = vertices;
    n.resize(3);
    uv.resize(3);

    iv.resize(1);
    in.resize(1);

    n[0] = ChVector<>(0, 0, 1);
    n[1] = ChVector<>(0, 0, 1);
    n[2] = ChVector<>(0, 0, 1);

    uv[0] = ChVector<>(0, 0, 0);
    uv[1] = ChVector<>(1, 0, 0);
    uv[2] = ChVector<>(0, 1, 0);

    iv[0] = ChVector<int>(0, 1, 2);
    in[0] = ChVector<int>(0, 1, 2);

    return trimesh;
}


// --------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // ----------------
    // Parameters
    // ----------------

    double collision_envelope = .001;
    ChVector<> hdims(0.25, 0.25, 0.25);
    std::vector<ChVector<>> vertices = {ChVector<>(0, 0, 0), ChVector<>(1.5, 0, 0), ChVector<>(0, 1.5, 0)};
    double separation = 0.0;

    // -----------------
    // Create the system
    // -----------------

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));
    system.SetCollisionSystemType(collision::ChCollisionSystemType::CHRONO);

    // ----------
    // Add bodies
    // ----------
    auto contact_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto triangle = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(triangle);
    triangle->SetPos(ChVector<>(0, 0, 0));
    triangle->SetBodyFixed(true);

    auto trimesh = TriangleMesh(vertices);

    triangle->SetCollide(true);
    triangle->GetCollisionModel()->SetEnvelope(collision_envelope);
    triangle->GetCollisionModel()->ClearModel();
    triangle->GetCollisionModel()->AddTriangleMesh(contact_mat, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1));
    triangle->GetCollisionModel()->BuildModel();

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    triangle->AddAsset(trimesh_shape);
    triangle->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.6f, 0.6f, 0.6f)));

    auto box = std::shared_ptr<ChBody>(system.NewBody());
    box->SetPos(ChVector<>(0, 0, 0.5));
    box->SetCollide(false);
    box->GetCollisionModel()->SetEnvelope(collision_envelope);
    box->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box.get(), contact_mat, hdims);
    box->GetCollisionModel()->BuildModel();

    box->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.2f, 0.3f, 0.4f)));

    system.AddBody(box);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    irrlicht::ChIrrApp application(&system, L"Box-triangle test", irr::core::dimension2d<irr::u32>(800, 600),
                                   irrlicht::VerticalDir::Z);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(irr::core::vector3df(30.f, 50.f, 40.f), irr::core::vector3df(-50.f, -30.f, -40.f));
    application.AddTypicalCamera(irr::core::vector3df(2, 2, 1));
    application.AssetBindAll();
    application.AssetUpdateAll();

    EvRec er(box);
    application.SetUserEventReceiver(&er);

    // ---------------
    // Simulate system
    // ---------------

    auto hdims_r = FromChVector(hdims);

    collision::ConvexShapeCustom shape_box;
    shape_box.type = collision::ChCollisionShape::Type::BOX;
    shape_box.dimensions = hdims_r;
    auto A = FromChVector(vertices[0]);
    auto B = FromChVector(vertices[1]);
    auto C = FromChVector(vertices[2]);
    collision::ConvexShapeTriangle shape_tri(A, B, C);

    while (application.GetDevice()->run()) {
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
        collision::ChNarrowphase::PRIMSCollision(&shape_box, &shape_tri, (real)separation, norm, pt1, pt2, depth,
                                                 eff_rad, nC);

        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();
        irrlicht::tools::drawAllCOGs(system, application.GetVideoDriver(), 1.0);
        if (nC > 0) {
            assert(nC <= 6);
            for (int i = 0; i < nC; i++) {
                assert(separation > 0 || depth[i] < 0);
                irrlicht::tools::drawSegment(application.GetVideoDriver(), ToChVector(pt1[i]), ToChVector(pt2[i]),
                                             irr::video::SColor(255, 255, 0, 255));
            }
        }
        application.EndScene();
    }

    return 0;
}
