#include <vector>
#include "chrono/core/ChCubicSpline.h"
#include "chrono_postprocess/ChGnuPlot.h"

using namespace chrono;
using namespace postprocess;

void test1() {
    std::vector<double> x_val;
    std::vector<double> y_val;
    int nv = 21;
    for (int i = 0; i < nv; i++) {
        double x = i * CH_2PI / (nv - 1);
        x_val.push_back(x);
        y_val.push_back(std::sin(x));
    }
    ChVectorDynamic<> xv(nv);
    ChVectorDynamic<> yv(nv);

    for (int i = 0; i < nv; i++) {
        xv(i) = x_val[i];
        yv(i) = y_val[i];
    }

    ChCubicSpline spline(x_val, y_val);

    double x_min = 0;
    double x_max = CH_2PI;
    int n = 300;

    ChVectorDynamic<> x(n);
    ChVectorDynamic<> y(n);
    ChVectorDynamic<> yd(n);
    ChVectorDynamic<> ydd(n);

    for (int i = 0; i < n; i++) {
        x(i) = x_min + i * (x_max - x_min) / (n - 1);
        spline.Evaluate(x(i), y(i), yd(i), ydd(i));
    }

    ChGnuPlot mplot("__tmp_gnuplot_spline1.gpl");
    mplot.SetGrid();
    mplot.SetLabelX("x");
    mplot.Plot(xv, yv, "data", " with points ");
    mplot.Plot(x, y, "y", " with lines lt -1 lc rgb'#EEAAAA' ");
    mplot.Plot(x, yd, "yd", " with lines lt -1 lc rgb'#AAEEAA' ");
    mplot.Plot(x, ydd, "ydd", " with lines lt -1 lc rgb'#AAAAEE' ");

}

void test2() {
    std::vector<double> x_val({-1.2700, -0.2540, -0.1524, -0.1270, -0.1016, -0.0762, -0.0508, -0.0254, 0.0, 0.0254, 0.0508,
                           0.0762, 0.1016, 0.1270, 0.1524, 0.2540, 1.2700});
    std::vector<double> y_val({1495.5, 809.5, 654.8, 587.1, 533.8, 455.5, 370.1, 206.4, 0.0, -462.6, -695.4, -854.0, -966.4,
                           -1085.1, -1171.4, -1423.4, -3218.1});
    int nv = x_val.size();
    ChVectorDynamic<> xv(nv);
    ChVectorDynamic<> yv(nv);

    for (int i = 0; i < nv; i++) {
        xv(i) = x_val[i];
        yv(i) = y_val[i];
    }

    ChCubicSpline spline(x_val, y_val);

    double x_min = -1.27;
    double x_max = +1.27;
    int n = 300;

    ChVectorDynamic<> x(n);
    ChVectorDynamic<> y(n);
    ChVectorDynamic<> yd(n);
    ChVectorDynamic<> ydd(n);

    for (int i = 0; i < n; i++) {
        x(i) = x_min + i * (x_max - x_min) / (n - 1);
        spline.Evaluate(x(i), y(i), yd(i), ydd(i));
    }

    ChGnuPlot mplot("__tmp_gnuplot_spline2.gpl");
    mplot.SetGrid();
    mplot.SetLabelX("x");
    mplot.Plot(xv, yv, "data", " with points ");
    mplot.Plot(x, y, "y", " with lines lt -1 lc rgb'#EEAAAA' ");
    //mplot.Plot(x, yd, "yd", " with lines lt -1 lc rgb'#AAEEAA' ");
    //mplot.Plot(x, ydd, "ydd", " with lines lt -1 lc rgb'#AAAAEE' ");
}


int main(int argc, char* argv[]) {
    test1();
    test2();
    return 0;
}