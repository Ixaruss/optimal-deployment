// #include "bin.h"
// using namespace std;
// int main() {
//     Bin b;
//     FeasibleGrid grid = b.getFeasibleGrid(WRITE);
//     // find cells with any bit set
//     int found = 0;
//     for (auto& cell : grid.cells) {
//         if (cell.value > 0 && found < 10) {
//             cout << "cell r=" << cell.r << " c=" << cell.c
//                  << " road="  << ((cell.value >> BIT_ROAD)       & 1)
//                  << " rail="  << ((cell.value >> BIT_RAIL)       & 1)
//                  << " wline=" << ((cell.value >> BIT_WATER_LINE) & 1)
//                  << " warea=" << ((cell.value >> BIT_WATER_AREA) & 1)
//                  << " ib="    << ((cell.value >> BIT_IB)         & 1)
//                  << " raw="   << (int)cell.value
//                  << "\n";
//             found++;
//         }
//     }
//     cout << "total non-zero cells: " <<
//         count_if(grid.cells.begin(), grid.cells.end(),
//                  [](const FeasibleCell& c){ return c.value > 0; }) << "\n";
//     Bin::verifyMatrixBin<FeasibleCell>("feasable.bin", 1, grid.cells.size());
//     return 0;
// }

#include "bin.h"

using namespace lib;
int main() {
    Bin::verifyMatrixBin<uint8_t>("map.bin", calc_rows(RESOLUTION), calc_cols(RESOLUTION));
    Bin::verifyMatrixBin<uint8_t>("elev.bin", calc_rows(RESOLUTION), calc_cols(RESOLUTION));
    return 0;
}
