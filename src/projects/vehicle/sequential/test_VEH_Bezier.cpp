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
// Test for TNB frame on Bezier curves
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace vehicle;

const std::string out_dir = "../TEST_BEZIER";

void plot(std::shared_ptr<ChBezierCurve> path, int n, const char* name) {
    std::ofstream outp(out_dir + "/" + name + "_P.dat", std::ios::out);
    std::ofstream outf(out_dir + "/" + name + "_F.dat", std::ios::out);

    // Generate points on the path
    std::vector<double> x(n);
    std::vector<double> y(n);
    std::vector<double> z(n);
    for (int i = 0; i < n; i++) {
        ChVector3d pos = path->eval(0.01 * i);
        x[i] = pos.x();
        y[i] = pos.y();
        z[i] = pos.z();
        outp << x[i] << " " << y[i] << " " << z[i] << std::endl;
    }

    // Create a tracker
    ChBezierCurveTracker tracker(path);
    tracker.reset(ChVector3d(x[5], y[5], z[5]));

    // Find TNB frame and curvature at points on path
    ChFrame<> tnb;
    double curvature;
    for (int i = 2; i < n; i++) {
        ChVector3d loc(x[i], y[i], z[i]);
        tracker.calcClosestPoint(loc, tnb, curvature);
        auto r = tnb.GetPos();
        auto T = tnb.GetRotMat().GetAxisX();
        auto N = tnb.GetRotMat().GetAxisY();
        auto B = tnb.GetRotMat().GetAxisZ();
        outf << r.x() << " " << r.y() << " " << r.z() << " ";
        outf << T.x() << " " << T.y() << " " << T.z() << " ";
        outf << N.x() << " " << N.y() << " " << N.z() << " ";
        outf << B.x() << " " << B.y() << " " << B.z() << " ";
        outf << curvature << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Circle path (left)
    auto path1 = CirclePath(ChVector3d(1, 2, 0), 3.0, 5.0, true, 1);
    plot(path1, 100, "left_circle");

    // Circle path (right)
    auto path2 = CirclePath(ChVector3d(1, 2, 0), 3.0, 5.0, false, 1);
    plot(path2, 100, "right_circle");

    // NATO double lane change path (left)
    auto path3 = DoubleLaneChangePath(ChVector3d(-100, 0, 0.1), 28.93, 3.6105, 25.0, 100.0, true);
    plot(path3, 600, "left_DLC");

    // double lane change path (right)
    auto path4 = DoubleLaneChangePath(ChVector3d(-10, 0, 0.1), 5, 3, 5.0, 10.0, false);
    plot(path4, 600, "right_DLC");

    return 0;
}
