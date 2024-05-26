#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

using namespace chrono;

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& obj_file,
                     double& delta,
                     double& min_depth,
                     int& num_bce,
                     bool& flat_bottom,
                     bool& flat_top) {
    delta = 0.1;
    min_depth = 0.3;
    num_bce = 3;

    ChCLI cli(argv[0], "BCE generator");

    cli.AddOption<std::string>("", "obj_file", "Input OBJ file (assumed relative to data/ directory)");
    cli.AddOption<double>("", "delta", "Particle spacing", std::to_string(delta));
    cli.AddOption<double>("", "min_depth", "Minimum soil depth", std::to_string(min_depth));
    cli.AddOption<int>("", "num_bce", "Number of BCE layers", std::to_string(num_bce));
    cli.AddOption<bool>("", "flat_bottom", "Flat container bottom");
    cli.AddOption<bool>("", "flat_top", "Flat container top");
    
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
    min_depth = cli.GetAsType<double>("min_depth");
    num_bce = cli.GetAsType<int>("num_bce");
    flat_bottom = cli.GetAsType<bool>("flat_bottom");
    flat_top = cli.GetAsType<bool>("flat_top");

    return true;
}

// =============================================================================
// Hash function for a pair of integer grid coordinates
struct CoordHash {
  public:
    // 31 is just a decently-sized prime number to reduce bucket collisions
    std::size_t operator()(const ChVector2<int>& p) const { return p.x() * 31 + p.y(); }
};

// Calculate barycentric coordinates of point'v'
bool calcBarycentricCoordinates(const ChVector3d& v1,
                                const ChVector3d& v2,
                                const ChVector3d& v3,
                                const ChVector3d& v,
                                double& a1,
                                double& a2,
                                double& a3) {
    double denom = (v2.y() - v3.y()) * (v1.x() - v3.x()) + (v3.x() - v2.x()) * (v1.y() - v3.y());
    a1 = ((v2.y() - v3.y()) * (v.x() - v3.x()) + (v3.x() - v2.x()) * (v.y() - v3.y())) / denom;
    a2 = ((v3.y() - v1.y()) * (v.x() - v3.x()) + (v1.x() - v3.x()) * (v.y() - v3.y())) / denom;
    a3 = 1 - a1 - a2;

    return (0 <= a1) && (a1 <= 1) && (0 <= a2) && (a2 <= 1) && (0 <= a3) && (a3 <= 1);
}

int main(int argc, char* argv[]) {
    std::string obj_file;
    double delta;
    double min_depth;
    int num_bce;
    bool flat_bottom = false;
    bool flat_top = false;

    if (!GetProblemSpecs(argc, argv, obj_file, delta, min_depth, num_bce, flat_bottom, flat_top)) {
        return 1;
    }

    // Load triangular mesh
    ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(GetChronoDataFile(obj_file), false, false);

    const auto& vertices = trimesh.GetCoordsVertices();
    const auto& faces = trimesh.GetIndicesVertexes();

    // Find x, y, and z ranges of vertex data
    auto minmaxX = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector3d& v1, const ChVector3d& v2) { return v1.x() < v2.x(); });
    auto minmaxY = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector3d& v1, const ChVector3d& v2) { return v1.y() < v2.y(); });
    auto minmaxZ = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector3d& v1, const ChVector3d& v2) { return v1.z() < v2.z(); });
    auto minX = minmaxX.first->x() + delta;
    auto maxX = minmaxX.second->x() - delta;
    auto minY = minmaxY.first->y() + delta;
    auto maxY = minmaxY.second->y() - delta;
    auto minZ = minmaxZ.first->z();
    auto maxZ = minmaxZ.second->z();

    auto sizeX = (maxX - minX);
    auto sizeY = (maxY - minY);
    auto sizeZ = (maxZ - minZ);
    ChVector3d center((maxX + minX) / 2, (maxY + minY) / 2, 0);

    int nx = static_cast<int>(std::ceil((sizeX / 2) / delta));  // half number of divisions in X direction
    int ny = static_cast<int>(std::ceil((sizeY / 2) / delta));  // number of divisions in Y direction
    delta = sizeX / (2.0 * nx);                                 // actual grid spacing

    std::cout << "Search grid size: " << nx << " x " << ny << std::endl;

    // Parse all mesh faces and generate a height map 
    std::unordered_map<ChVector2<int>, double, CoordHash> h_map;
    double a1, a2, a3;
    for (const auto& f : faces) {
        // Find bounds of (shifted) face projection
        const auto& v1 = vertices[f[0]] - center;
        const auto& v2 = vertices[f[1]] - center;
        const auto& v3 = vertices[f[2]] - center;
        auto x_min = std::min(std::min(v1.x(), v2.x()), v3.x());
        auto x_max = std::max(std::max(v1.x(), v2.x()), v3.x());
        auto y_min = std::min(std::min(v1.y(), v2.y()), v3.y());
        auto y_max = std::max(std::max(v1.y(), v2.y()), v3.y());
        int i_min = static_cast<int>(std::floor(x_min / delta));
        int j_min = static_cast<int>(std::floor(y_min / delta));
        int i_max = static_cast<int>(std::ceil(x_max / delta));
        int j_max = static_cast<int>(std::ceil(y_max / delta));
        ChClampValue(i_min, -nx, +nx);
        ChClampValue(i_max, -nx, +nx);
        ChClampValue(j_min, -ny, +ny);
        ChClampValue(j_max, -ny, +ny);
        // Loop over all grid nodes within bounds
        for (int i = i_min; i <= i_max; i++) {
            for (int j = j_min; j <= j_max; j++) {
                ChVector3d v(i * delta, j * delta, 0);
                if (calcBarycentricCoordinates(v1, v2, v3, v, a1, a2, a3)) {
                    double h = minZ + a1 * v1.z() + a2 * v2.z() + a3 * v3.z();
                    h_map.insert({ChVector2<int>(i, j), h});
                }
            }
        }
    }
    std::cout << "Height map size: " <<  h_map.size() << std::endl;

    // Parse the grid and identify side boundaries
    std::unordered_map<ChVector2<int>, double, CoordHash> s_map;

    for (int i = -nx; i <= nx; i++) {
        int jmin = -ny;
        int jmax = +ny;
        while (h_map.find(ChVector2<int>(i, jmin)) == h_map.end())
            jmin++;
        while (h_map.find(ChVector2<int>(i, jmax)) == h_map.end())
            jmax--;
        for (int j = jmin - num_bce; j < jmin; j++)
            s_map.insert({ChVector2<int>(i,j), h_map[ChVector2<int>(i, jmin)]});
        for (int j = jmax + num_bce; j > jmax; j--)
            s_map.insert({ChVector2<int>(i, j), h_map[ChVector2<int>(i, jmax)]});
    }

    for (int j = -ny; j <= ny; j++) {
        int imin = -nx;
        int imax = +nx;
        while (h_map.find(ChVector2<int>(imin, j)) == h_map.end())
            imin++;
        while (h_map.find(ChVector2<int>(imax, j)) == h_map.end())
            imax--;
        for (int i = imin - num_bce; i < imin; i++)
            s_map.insert({ChVector2<int>(i, j), h_map[ChVector2<int>(imin, j)]});
        for (int i = imax + num_bce; i > imax; i--)
            s_map.insert({ChVector2<int>(i, j), h_map[ChVector2<int>(imax, j)]});
    }

    std::cout << "Side map size: " << s_map.size() << std::endl;

    // Create a Chrono system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(VNULL);

    double radius = delta / 2;
    double side = 0.9 * delta;

    // Parse the hash map and create points & bce markers in the corresponding vertical segment
    std::vector<ChVector3d> points;
    std::vector<ChVector3d> bce;
    for (const auto& p : h_map) {
        const auto& ij = p.first;
        auto v = ChVector3d(ij.x() * delta, ij.y() * delta, 0) + center;
        double h;

        // SPH markers
        h = flat_bottom ? -min_depth : p.second - min_depth;
        while (h <= p.second) {
            auto w = v + ChVector3d(0, 0, h);
            // Cache the point
            points.push_back(w);
            // Create and add a body for each marker
            auto body = chrono_types::make_shared<ChBodyEasySphere>(radius, 1, true, false);
            body->SetPos(w);
            sys.AddBody(body);
            // Move up
            h += delta;
        }

        // BCE markers (bottom)
        h = (flat_bottom ? -min_depth : p.second - min_depth) - delta;
        for (int i = 0; i < num_bce; i++, h -= delta) {
            auto w = v + ChVector3d(0, 0, h);
            // Cache the point
            bce.push_back(w);
            // Create and add a body for each marker
            auto body = chrono_types::make_shared<ChBodyEasyBox>(side, side, side, 1, true, false);
            body->SetPos(w);
            sys.AddBody(body);
        }
    }

    // Parse the side map and create bce markers in the corresponding vertical segment
    for (const auto& p : s_map) {
        const auto& ij = p.first;
        auto v = ChVector3d(ij.x() * delta, ij.y() * delta, 0) + center;
        double h = (flat_bottom ? -min_depth : p.second - min_depth) - num_bce * delta;
        double h_max = (flat_top ? sizeZ : p.second) + num_bce * delta;

        // BCE markers (sides)
        while (h <= h_max) {
            auto w = v + ChVector3d(0, 0, h);
            // Cache the point
            bce.push_back(w);
            // Create and add a body for each marker
            auto body = chrono_types::make_shared<ChBodyEasyBox>(side, side, side, 1, true, false);
            body->SetPos(w);
            sys.AddBody(body);
            // Move up
            h += delta;
        }
    }

    sys.Setup();
    std::cout << "Num particles: " << points.size() << std::endl;
    std::cout << "Num BCE markers: " << bce.size() << std::endl;

    // Initialize OpenGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(maxX, maxY, maxZ) + ChVector3d(3), ChVector3d(0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    while (vis.Run()) {
        vis.Render();
    }

    // Create output directory
    const std::string out_dir = "../TEST_PARTICLE_GEN";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Write particles to file
    std::ofstream myFile(out_dir + "/" + filesystem::path(obj_file).stem() + ".txt", std::ios::trunc);
    myFile << "x, y, z\n";
    for (const auto& p : points) {
        myFile << p.x() << ", " << p.y() << ", " << p.z() << ",\n";
    }
    myFile.close();

    return 0;
}
