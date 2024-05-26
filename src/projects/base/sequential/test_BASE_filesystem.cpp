#include <iostream>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using namespace filesystem;
using namespace chrono;
using std::cout;
using std::endl;

int main(int argc, char **argv) {
#if !defined(WIN32)
    path path1("/dir 1/dir 2/");
#else
    path path1("C:\\dir 1\\dir 2\\");
#endif
    path path2("dir 3");

    cout << path1.exists() << endl;
    cout << path1 << endl;
    cout << (path1/path2) << endl;
    cout << (path1/path2).parent_path() << endl;
    cout << (path1/path2).parent_path().parent_path() << endl;
    cout << (path1/path2).parent_path().parent_path().parent_path() << endl;
    cout << (path1/path2).parent_path().parent_path().parent_path().parent_path() << endl;
    cout << path().parent_path() << endl;

    // Absolute path of current directory 
    cout << endl;
    cout << "Current directory = " << path(".").make_absolute() << endl;

    // Create output directory and output file
    std::string out_dir = "../TEST_filesystem";
    std::string out_file = out_dir + "/foo.txt";
    bool out_dir_exists = path(out_dir).exists();
    if (out_dir_exists) {
        cout << "Output directory already exists" << endl;
    } else if (create_directory(path(out_dir))) {
        cout << "Create directory = " << path(out_dir).make_absolute() << endl;
    } else {
        cout << "Error creating output directory" << endl;
        return 1;
    }
    cout << "Output directory exists = " << path(out_dir).exists() << endl;
    cout << "             dir name: " << out_dir << endl;
    cout << "          path of dir: " << path(out_dir) << endl;
    cout << "     abs. path of dir: " << path(out_dir).make_absolute() << endl;

    cout << "Create output file" << endl;
    cout << "            file name: " << out_file << endl;
    cout << "         path of file: " << path(out_file) << endl;
    cout << "    abs. path of file: " << path(out_file).make_absolute() << endl;
    utils::ChWriterCSV csv(",");
    csv << ChVector3d(1, 2, 3) << ChQuaternion<>(1, 0, 0, 0) << endl;
    csv.WriteToFile(out_file);
    cout << "Output file exists = " << path(out_file).exists() << endl;
    cout << "Output file is file = " << path(out_file).is_file() << endl;
    cout << endl;

    // Other tests

    cout << "some/path.ext:operator==() = " << (path("some/path.ext") == path("some/path.ext")) << endl;
    cout << "some/path.ext:operator==() (unequal) = " << (path("some/path.ext") == path("another/path.ext")) << endl;

    cout << "nonexistant:exists = " << path("nonexistant").exists() << endl;
    cout << "nonexistant:is_file = " << path("nonexistant").is_file() << endl;
    cout << "nonexistant:is_directory = " << path("nonexistant").is_directory() << endl;
    cout << "nonexistant:filename = " << path("nonexistant").filename() << endl;
    cout << "nonexistant:extension = " << path("nonexistant").extension() << endl;

    // Note: results of the following depend on current value of "Working Directory"

    cout << "filesystem/path.h:exists = " << path("filesystem/path.h").exists() << endl;
    cout << "filesystem/path.h:is_file = " << path("filesystem/path.h").is_file() << endl;
    cout << "filesystem/path.h:is_directory = " << path("filesystem/path.h").is_directory() << endl;
    cout << "filesystem/path.h:filename = " << path("filesystem/path.h").filename() << endl;
    cout << "filesystem/path.h:stem = " << path("filesystem/path.h").stem() << endl;
    cout << "filesystem/path.h:extension = " << path("filesystem/path.h").extension() << endl;
    cout << "filesystem/path.h:make_absolute = " << path("filesystem/path.h").make_absolute() << endl;

    cout << "../filesystem:exists = " << path("../filesystem").exists() << endl;
    cout << "../filesystem:is_file = " << path("../filesystem").is_file() << endl;
    cout << "../filesystem:is_directory = " << path("../filesystem").is_directory() << endl;
    cout << "../filesystem:filename = " << path("../filesystem").filename() << endl;
    cout << "../filesystem:stem = " << path("../filesystem").stem() << endl;
    cout << "../filesystem:extension = " << path("../filesystem").extension() << endl;
    cout << "../filesystem:make_absolute = " << path("../filesystem").make_absolute() << endl;

    cout << "resolve(filesystem/path.h) = " << resolver().resolve("filesystem/path.h") << endl;
    cout << "resolve(nonexistant) = " << resolver().resolve("nonexistant") << endl;
    return 0;
}
