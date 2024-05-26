#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <glm/glm.hpp>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChGlobal.h"
#include "chrono_opengl/ChVisualSystemOpenGL.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

using namespace chrono;

// =============================================================================

#define EPSILON 0.000001

struct triangle {
    int a;
    int b;
    int c;
    void print() { printf("	Triangle: %d, %d, %d\n", a, b, c); }
};

struct vertex {
    glm::dvec3 position;
    glm::dvec3 normal;
    void print() {
        printf("	Vertex Position: %f, %f, %f\n", position.x, position.y, position.z);
        printf("	Vertex Normal: %f, %f, %f\n", normal.x, normal.y, normal.z);
    }
};

struct mesh {
    std::vector<triangle> triangles;
    std::vector<vertex> vertices;

    void printInfo() {
        std::cout << "Num Triangles: " << triangles.size() << std::endl;
        std::cout << "Num Vertices: " << vertices.size() << std::endl;
    }
    void printVertices() {
        std::cout << "Vertices: " << std::endl;

        for (unsigned int i = 0; i < vertices.size(); ++i) {
            printf("Vertices[%d]: \n", i);
            vertices[i].print();
        }
    }
    void printTriangles() {
        std::cout << "Triangles: " << std::endl;
        for (unsigned int i = 0; i < triangles.size(); ++i) {
            printf("Triangle[%d]: \n", i);
            triangles[i].print();
        }
    }
};

glm::dvec3 getMinVert(std::vector<vertex> vertices) {
    glm::dvec3 minV = vertices[0].position;
    glm::dvec3 currV = vertices[0].position;
    for (unsigned int i = 0; i < vertices.size(); ++i) {
        currV = vertices[i].position;
        if (minV.x > currV.x)
            minV.x = currV.x;
        if (minV.y > currV.y)
            minV.y = currV.y;
        if (minV.z > currV.z)
            minV.z = currV.z;
    }
    return minV;
}

glm::dvec3 getMaxVert(std::vector<vertex> vertices) {
    glm::dvec3 maxV = vertices[0].position;
    glm::dvec3 currV = vertices[0].position;
    for (unsigned int i = 0; i < vertices.size(); ++i) {
        currV = vertices[i].position;
        if (maxV.x < currV.x)
            maxV.x = currV.x;
        if (maxV.y < currV.y)
            maxV.y = currV.y;
        if (maxV.z < currV.z)
            maxV.z = currV.z;
    }
    return maxV;
}

mesh loadMesh(const std::string& filename) {
    mesh m;

    std::vector<tinyobj::shape_t> shapes;
    tinyobj::attrib_t att;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    bool success = tinyobj::LoadObj(&att, &shapes, &materials, &warn, &err, filename.c_str());
    if (!success) {
        std::cerr << "Error loading OBJ file " << filename << std::endl;
        std::cerr << "   tiny_obj warning message: " << warn << std::endl;
        std::cerr << "   tiny_obj error message:   " << err << std::endl;
        return m;
    }

    for (size_t i = 0; i < att.vertices.size() / 3; i++) {
        vertex v;
        v.position = glm::dvec3(att.vertices[3 * i + 0], att.vertices[3 * i + 1], att.vertices[3 * i + 2]);

        ////if (att.normals.size() > 0) {
        ////    v.normal = glm::dvec3(att.normals[3 * i + 0], att.normals[3 * i + 1], att.normals[3 * i + 2]);
        ////}
        m.vertices.push_back(std::move(v));
    }
    for (size_t i = 0; i < shapes.size(); i++) {
        for (size_t j = 0; j < shapes[i].mesh.indices.size() / 3; j++) {
            m.triangles.push_back(triangle{shapes[i].mesh.indices[3 * j + 0].vertex_index,
                                           shapes[i].mesh.indices[3 * j + 1].vertex_index,
                                           shapes[i].mesh.indices[3 * j + 2].vertex_index});
        }
    }

    return m;
}

int triangle_intersection(const glm::dvec3 v1,  // Triangle vertices
                          const glm::dvec3 v2,
                          const glm::dvec3 v3,
                          const glm::dvec3 orig,  // Ray origin
                          const glm::dvec3 dir,   // Ray direction
                          double* out) {
    // printf("Starting intersection\n");
    glm::dvec3 edge1, edge2;  // Edgedge1, Edgedge2
    glm::dvec3 pvec, qvec, tvec;
    double det, inv_det, u, v;
    double t;

    // Find vectors for two edges sharing V1
    edge1 = v2 - v1;
    edge2 = v3 - v1;

    // Begin calculating determinant - also used to calculate u parameter
    pvec = glm::cross(dir, edge2);
    // if determinant is near zero, ray lies in plane of triangle or ray is parallel to plane of triangle
    det = glm::dot(edge1, pvec);
    // NOT CULLING
    if (det > -EPSILON && det < EPSILON) {
        // printf("det = %f , first IF!\n", det);
        return 0;
    }
    inv_det = 1.0 / det;

    // calculate distance from V1 to ray origin
    tvec = orig - v1;

    // Calculate u parameter and test bound
    u = glm::dot(tvec, pvec) * inv_det;
    // The intersection lies outside of the triangle
    if (u < 0.0 || u > 1.0) {
        // printf("u = %f , second IF!\n", u);
        return 0;
    }

    // Prepare to test v parameter
    qvec = glm::cross(tvec, edge1);

    // Calculate V parameter and test bound
    v = glm::dot(dir, qvec) * inv_det;
    // The intersection lies outside of the triangle
    if (v < 0.0 || ((u + v) > 1.0)) {
        // printf("v = %f , v + u = %f, third IF!\n", u,u+v);
        return 0;
    }

    t = glm::dot(edge2, qvec) * inv_det;
    if (t > EPSILON) {  // ray intersection
        *out = t;
        return 1;
    }

    // No hit, no win
    return 0;
}

int isInsideMesh(mesh m, glm::dvec3 ray_origin) {
    glm::dvec3 ray_dir1 = glm::dvec3(5, 0.5, 0.25);
    glm::dvec3 ray_dir2 = glm::dvec3(-3, 0.7, 10);
    triangle t;
    vertex v1, v2, v3;
    int t_inter1, t_inter2;
    double out;
    int intersectCounter1 = 0;
    int intersectCounter2 = 0;
    for (unsigned int i = 0; i < m.triangles.size(); ++i) {
        t = m.triangles[i];
        v1 = m.vertices[t.a];
        v2 = m.vertices[t.b];
        v3 = m.vertices[t.c];

        t_inter1 = triangle_intersection(v1.position, v2.position, v3.position, ray_origin, ray_dir1, &out);
        t_inter2 = triangle_intersection(v1.position, v2.position, v3.position, ray_origin, ray_dir2, &out);
        if (t_inter1 == 1) {  //|| t_inter2 == 1) {
            intersectCounter1 += 1;
        }
        if (t_inter2 == 1) {  //|| t_inter2 == 1) {
            intersectCounter2 += 1;
        }
    }
    // printf("IntersectCounter = %d\n",intersectCounter);
    // if (intersectCounter == 8)
    // {
    // 	pri
    // }

    if (((intersectCounter1 % 2) == 1) && ((intersectCounter2 % 2) == 1)) {
        return 1;
    }

    return 0;
}

// =============================================================================

bool GetProblemSpecs(int argc, char** argv, std::string& obj_file, double& delta) {
    delta = 0.01;

    ChCLI cli(argv[0], "BCE generator");

    cli.AddOption<std::string>("", "obj_file", "Input OBJ file (assumed relative to data/ directory)");
    cli.AddOption<double>("", "delta", "Particle spacing", std::to_string(delta));

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    try {
        obj_file = cli.Get("obj_file").as<std::string>();
    } catch (std::domain_error&) {
        std::cout << "\nERROR: Missing input OBJ file!\n\n" << std::endl;
        cli.Help();
        return false;
    }
    delta = cli.GetAsType<double>("delta");

    return true;
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::string obj_file;
    double delta;

    if (!GetProblemSpecs(argc, argv, obj_file, delta)) {
        return 1;
    }

    struct mesh m = loadMesh(GetChronoDataFile(obj_file));
    std::cout << "Loaded mesh" << std::endl;
    ////m.printInfo();
    ////m.printVertices();
    ////m.printTriangles();

    glm::dvec3 ray_origin = glm::dvec3(0.0, 0.0, 0.0);
    glm::dvec3 startCoords = getMinVert(m.vertices);
    glm::dvec3 endCoords = getMaxVert(m.vertices);
    std::vector<glm::dvec3> point_cloud;

    printf("start coords: %f, %f, %f\n", startCoords.x, startCoords.y, startCoords.z);
    printf("end coords: %f, %f, %f\n", endCoords.x, endCoords.y, endCoords.z);
    int total_num = 0;
    for (double x = startCoords.x; x < endCoords.x; x += delta) {
        ray_origin.x = x + 1e-9;
        for (double y = startCoords.y; y < endCoords.y; y += delta) {
            ray_origin.y = y + 1e-9;
            for (double z = startCoords.z; z < endCoords.z; z += delta) {
                ray_origin.z = z + 1e-9;
                if (isInsideMesh(m, ray_origin)) {
                    point_cloud.push_back(glm::dvec3(x, y, z));
                    total_num++;
                    // printf("current number of particles is: %d \n", total_num);
                }
            }
        }
        printf("finished: %f \n", double(x - startCoords.x) / double(endCoords.x - startCoords.x));
        printf("current number of particles is: %d \n", total_num);
    }

    // Create a Chrono system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(VNULL);
    double radius = delta;
    for (const auto& v : point_cloud) {
        auto body = chrono_types::make_shared<ChBodyEasySphere>(radius, 1, true, false);
        body->SetPos(ChVector3d(v.x, v.y, v.z));
        sys.AddBody(body);
    }

    // Initialize OpenGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(-1, -1, 0.75), ChVector3d(0, 0, 0.5));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    while (vis.Run()) {
        vis.Render();
    }

    // Create output directory
    const std::string out_dir = "../TEST_BCE_GEN";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Write particles to file
    std::ofstream myFile(out_dir + "/" + filesystem::path(obj_file).stem() + ".txt", std::ios::trunc);
    myFile << "x"
           << ","
           << "y"
           << ","
           << "z"
           << "\n";
    for (long unsigned i = 0; i < point_cloud.size(); ++i) {
        ////printf("point_cloud[%d]: %f, %f, %f\n", i, point_cloud[i].x, point_cloud[i].y, point_cloud[i].z);
        ////printf("%f, %f, %f\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z);
        myFile << point_cloud[i].x << ","
               << "\t" << point_cloud[i].y << ","
               << "\t" << point_cloud[i].z << ","
               << "\n";
    }
    myFile.close();

    return 0;
}
