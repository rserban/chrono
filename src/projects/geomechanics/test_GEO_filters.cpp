#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "chrono/utils/ChFilters.h"

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " filename  nlines" << endl;
        return 1;
    }

    std::string in_filename(argv[1]);
    int nlines = std::stoi(argv[2]);
    std::ifstream ifs(in_filename);

    std::string out_filename("filtered_data.txt");
    std::ofstream ofs(out_filename);

    double dt = 1e-4;
    chrono::utils::ChButterworthLowpass lowpass1(1, dt, 1.0);
    chrono::utils::ChButterworthLowpass lowpass5(1, dt, 5.0);
    chrono::utils::ChButterworthLowpass lowpass10(1, dt, 10.0);

    double time, data;
    for (auto i = 0; i < nlines; i++) {
        ifs >> time >> data;
        double f1 = lowpass1.Filter(data);
        double f5 = lowpass5.Filter(data);
        double f10 = lowpass10.Filter(data);
        ofs << time << " " << data << " " << f1 << " " << f5 << " " << f10 << endl;
    }

    ifs.close();
    ofs.close();

    return 0;
}
