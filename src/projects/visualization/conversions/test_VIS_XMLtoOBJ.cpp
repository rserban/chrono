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
// Authors: Radu Serban
// =============================================================================
//
// Utility program to convert a LandXML file to a Wavefront mesh OBJ.
//
// =============================================================================

#include <cstdio>
#include <string>
#include <vector>

#include "chrono_thirdparty/rapidxml/rapidxml_print.hpp"
#include "chrono_thirdparty/rapidxml/rapidxml_utils.hpp"

#include "chrono/geometry/ChTriangleMeshConnected.h"

using namespace rapidxml;
using namespace chrono;

int main(int argc, char* argv[]) {
    std::string xml_filename = "in_terrain.xml";
    std::string obj_filename = "out_mesh_2.obj";

    const double ft2m = 1200.0 / 3937.0;

    ////ChTriangleMeshConnected trimesh;
    ////std::vector<ChVector3d >& vertices = trimesh.GetCoordsVertices();
    ////std::vector<ChVector3<int> >& idx_vertices = trimesh.GetIndicesVertexes();

    FILE* fp = std::fopen(obj_filename.c_str(), "w");

    rapidxml::file<char> file(xml_filename.c_str());
    xml_document<> doc;
    doc.parse<0>(file.data());
    xml_node<>* tin =
        doc.first_node("LandXML")->first_node("Surfaces")->first_node("Surface")->first_node("Definition");
    xml_node<>* pnts = tin->first_node("Pnts");
    xml_node<>* faces = tin->first_node("Faces");

    xml_node<>* pnt_node = pnts->first_node();
    int num_vert = 0;
    while (pnt_node != NULL) {
        double x, y, z;
        auto foo = pnt_node->value();
        int n = sscanf(foo, "%lf %lf %lf", &x, &y, &z);
        if (n != 3) {
            std::cout << "Error reading point\n" << std::endl;
            return 1;
        }

        ////vertices.push_back(ChVector3d(x, y, z) * ft2m);
        fprintf(fp, "v %.12lf %.12lf %.12lf\n", x * ft2m, y * ft2m, z * ft2m);
        num_vert++;

        pnt_node = pnt_node->next_sibling();
    }
    
    xml_node<>* face_node = faces->first_node();
    int num_tri = 0;
    while (face_node != NULL) {
        int x, y, z;
        auto foo = face_node->value();
        int n = sscanf(foo, "%d %d %d", &x, &y, &z);
        if (n != 3) {
            std::cout << "Error reading face\n" << std::endl;
            return 1;
        }

        ////idx_vertices.push_back(ChVector3<int>(x - 1, y - 1, z - 1));
        fprintf(fp, "f %d %d %d\n", x, y, z);
        num_tri++;

        face_node = face_node->next_sibling();
    }

    fclose(fp);

    std::cout << "Num vert: " << num_vert << std::endl;
    std::cout << "Num tri:  " << num_tri << std::endl;

    ////std::vector<ChTriangleMeshConnected> meshes = {trimesh};
    ////std::cout << "Exporting to " << obj_filename << std::endl;
    ////trimesh.WriteWavefront(obj_filename, meshes);

    return 0;
}
