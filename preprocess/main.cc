#include "bin.h"

using namespace std;

int main() {
    Bin bin;
    Bin::build_matrix();
    Bin::build_elevation_matrix();

    return 0;
}


// #include "lib.h"

// using namespace std;

// int main() {
//     VTM tool = VTM();
//     cout << tool.conf.output.matrix_file << endl;
//     auto x = VTM::get_min_max_coordiantes("./maps/rails/IND_rails.shp",1);
//     for (double coord : x) {
//         cout << coord << " ";
//     }
//     cout << "rail" << endl;
//     x = VTM::get_min_max_coordiantes("./maps/roads/IND_roads.shp",1);
//     for (double coord : x) {
//         cout << coord << " ";
//     }
//     cout<< "road" << endl;
//     x = VTM::get_min_max_coordiantes(  "./maps/water-areas/IND_water_areas_dcw.shp",1);
//     for (double coord : x) {
//         cout << coord << " ";
//     }
//     cout << "waterarea" << endl;
//     x = VTM::get_min_max_coordiantes(  "./maps/water-lines/IND_water_lines_dcw.shp",1);
//     for (double coord : x) {
//         cout << coord << " ";
//     }
//     cout << "waterliens" << endl;
//     cout<< VTM::calc_cols(30) << " \n";
//     cout <<VTM::calc_rows(30)<< "\n";
//     return 0;
// }




// #include "./bin.h"
// #include <iostream>
// #include <cassert>
// using namespace std;
// int main() {

//     Bin bin = Bin();

//     cout << "resolution: " << bin.getTerrianResolution() <<endl;
//     // auto ar = bin.getFeasibleGrid(WRITE).cells;
//     // for (auto val :ar) {
//     // }
//     // Bin::verifyMatrixBin<FeasibleCell>("./feasable.bin",1,ar.size());
//     cout<< bin.conf.input.road << " " << bin.conf.input.rail << " " << bin.conf.input.water_area << " " << bin.conf.input.water_lines << " " << bin.conf.input.elevation << endl;
//     cout << bin.conf.output.matrix_file << " " << bin.conf.output.elevation_file << " " << bin.conf.output.feasible << endl;
//     return 0;
// }


// #include "bin.h"
// #include <iostream>
// #include <fstream>
// #include <vector>


// int main() {
//     std::ifstream f("./feasable.bin", std::ios::binary);
//     if (!f.is_open()) {
//         std::cerr << "could not open feasable.bin\n";
//         return -1;
//     }

//     // get file size and derive cell count
//     f.seekg(0, std::ios::end);
//     size_t file_bytes = f.tellg();
//     f.seekg(0, std::ios::beg);

//     size_t cell_count = file_bytes / sizeof(FeasibleCell);
//     std::cout << "file size  : " << file_bytes << " bytes\n";
//     std::cout << "cell size  : " << sizeof(FeasibleCell) << " bytes\n";
//     std::cout << "cell count : " << cell_count << "\n";
//     std::cout << "─────────────────────────────────\n";

//     std::vector<FeasibleCell> cells(cell_count);
//     f.read(reinterpret_cast<char*>(cells.data()), file_bytes);
//     f.close();

//     // print first 20 and last 20
//     size_t preview = std::min(cell_count, (size_t)20);
//     std::cout << "first " << preview << " cells:\n";
//     for (size_t i = 0; i < preview; i++) {
//         std::cout << "  [" << i << "] r=" << cells[i].r
//                   << " c="   << cells[i].c
//                   << " val=" << (int)cells[i].value
//                   << " bits=";
//         for (int b = 3; b >= 0; b--)
//             std::cout << ((cells[i].value >> b) & 1);
//         std::cout << "\n";
//     }

//     if (cell_count > 20) {
//         std::cout << "...\nlast " << preview << " cells:\n";
//         for (size_t i = cell_count - preview; i < cell_count; i++) {
//             std::cout << "  [" << i << "] r=" << cells[i].r
//                       << " c="   << cells[i].c
//                       << " val=" << (int)cells[i].value
//                       << " bits=";
//             for (int b = 3; b >= 0; b--)
//                 std::cout << ((cells[i].value >> b) & 1);
//             std::cout << "\n";
//         }
//     }

//     return 0;
// }
