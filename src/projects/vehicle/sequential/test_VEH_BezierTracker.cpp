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
// Test for tracker on Bezier curve
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_postprocess/ChGnuPlot.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace vehicle;
using namespace postprocess;

const std::string out_dir = "../TEST_BEZIER_TRACKER";

int main(int argc, char* argv[]) {
    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Bezier curve from given points
    std::vector<ChVector3d> pts = {ChVector3d(0, 0, 0),    ChVector3d(50, 0, 0),    ChVector3d(100, 0, 0),
                                   ChVector3d(100, 50, 0), ChVector3d(100, 100, 0), ChVector3d(50, 100, 0),
                                   ChVector3d(0, 100, 0),  ChVector3d(0, 50, 0),    ChVector3d(0, 0, 0)};

    {
        // Save points to file
        std::ofstream out(out_dir + "/" + "points.dat", std::ios::out);
        for (auto p : pts)
            out << p.x() << " " << p.y() << " " << p.z() << std::endl;
    }

    auto path = chrono_types::make_shared<ChBezierCurve>(pts);

    {
        // Save path to file
        std::ofstream out(out_dir + "/" + "path.dat", std::ios::out);
        int n = 100;
        double delta = 1.0 / n;
        for (int i = 0; i < n; i++) {
            ChVector3d p = path->Eval(delta * i);
            out << p.x() << " " << p.y() << " " << p.z() << std::endl;
        }
    }

    // Create a new tracker every time to check correctness of first closest point calculation.
    // Save "sentinel" pont and "target" (closest point on path to sentinel)
    std::ofstream out(out_dir + "/" + "track.dat", std::ios::out);
    for (int i = 0; i < 50; i++) {
        ChBezierCurveTracker tracker(path);        
        ChVector3d s = ChVector3d(110.0 * (i / 50.0), 0, 0);
        ChVector3d t;
        tracker.CalcClosestPoint(s, t);
        out << s.x() << " " << s.y() << " " << s.z() << " ";
        out << t.x() << " " << t.y() << " " << t.z() << std::endl;
    }
    for (int i = 0; i < 50; i++) {
        ChBezierCurveTracker tracker(path);
        ChVector3d s = ChVector3d(110, 110.0 * (i / 50.0), 0);
        ChVector3d t;
        tracker.CalcClosestPoint(s, t);
        out << s.x() << " " << s.y() << " " << s.z() << " ";
        out << t.x() << " " << t.y() << " " << t.z() << std::endl;
    }

    return 0;
}

/*
p = load('points.dat');
c = load('path.dat');
s = load('track.dat');

plot(p(:,1),p(:,2),'*', 'markerfacecolor', 'b')
hold on
plot(c(:,1),c(:,2),'k-')

sh = plot(s(:,1), s(:,2), 'o', 'markerfacecolor', 'r');
th = plot(s(:,4), s(:,5), 'o', 'markerfacecolor', 'g');

for i = 1:size(s,1)
   set(sh, 'XData', s(i,1), 'YData', s(i,2))
   set(th, 'XData', s(i,4), 'YData', s(i,5))
   pause(0.4)
end
*/