// #include "../bin.h"
// #include <iostream>
// #include <cassert>
// #include <string>

// // Structure to hold coordinates and expected test outcomes
// struct TestLocation {
//     std::string name;
//     double lat;
//     double lng;
//     bool expected_to_be_inside;
// };

// int main() {
//     std::cout << "==================================================\n";
//     std::cout << "       STARTING Bin SYSTEM TESTING           \n";
//     std::cout << "==================================================\n\n";

//     // 1. Initialize the query object
//     // (This does not load data yet; files stay on disk until the first query)
//     Bin query_engine;
//     std::cout << "[test] Query engine initialized successfully.\n";

//     // 2. Define a diverse set of test coordinates across India
//     std::vector<TestLocation> targets = {
//         {"New Delhi (Capital Region)", 28.6139, 77.2090, true},
//         {"Mumbai Coastal Strip",        19.0760, 72.8777, true},
//         {"Khardung La Pass (High Alt)", 34.2790, 77.6053, true},
//         {"Guwahati (Northeast)",        26.1445, 91.7362, true},
//         {"Invalid Out of Bounds Point", 45.0000, 12.0000, false} // Far outside India
//     };

//     // 3. Execute testing pipeline
//     int passed_tests = 0;
//     int total_tests = targets.size();

//     for (const auto& loc : targets) {
//         std::cout << "--------------------------------------------------\n";
//         std::cout << "[test] Target Location: " << loc.name << "\n";
//         std::cout << "[test] GPS Coordinates: Lat=" << loc.lat << ", Lng=" << loc.lng << "\n";

//         // Test A: Coordinate Translation Bounds Check
//         int r = -1, c = -1;
//         bool inside_grid = query_engine.getGridCoords(loc.lat, loc.lng, r, c);

//         // This first iteration automatically handles lazy-loading your binary matrix files
//         if (!inside_grid) {
//             std::cout << "[result] Status: Location falls outside matrix coverage grid.\n";
//             if (!loc.expected_to_be_inside) {
//                 std::cout << "[pass] Correctly caught out-of-bounds boundary point.\n";
//                 passed_tests++;
//             } else {
//                 std::cerr << "[FAIL] Expected location to fall inside map boundaries!\n";
//             }
//             continue;
//         }

//         std::cout << "[result] Grid Mapping: Row=" << r << ", Col=" << c << "\n";

//         // Test B: Verify Elevation Reader
//         int16_t altitude = query_engine.getElevation(loc.lat, loc.lng);
//         if (altitude == -9999) {
//             std::cout << "[result] Elevation : NoData / Maritime Mask\n";
//         } else {
//             std::cout << "[result] Elevation : " << altitude << " meters\n";
//             // Sanity verification check: Khardung La must track high
//             if (loc.name.find("Khardung La") != std::string::npos) {
//                 if (altitude > 3000) {
//                     std::cout << "[pass] Alpine altitude validation succeeded.\n";
//                 } else {
//                     std::cerr << "[WARN] Elevation looks unexpectedly low for a mountain pass!\n";
//                 }
//             }
//         }

//         // Test C: Extrapolate Boolean Layer Variables
//         bool road   = query_engine.getStatusRoad(loc.lat, loc.lng);
//         bool rail   = query_engine.getStatusRail(loc.lat, loc.lng);
//         bool w_area = query_engine.getStatusWaterArea(loc.lat, loc.lng);
//         bool w_line = query_engine.getStatusWaterLine(loc.lat, loc.lng);

//         std::cout << "[result] Vector Flags: "
//                   << "Road=" << (road ? "YES" : "no") << " | "
//                   << "Rail=" << (rail ? "YES" : "no") << " | "
//                   << "WaterArea=" << (w_area ? "YES" : "no") << " | "
//                   << "WaterLine=" << (w_line ? "YES" : "no") << "\n";

//         // Test D: Verify Comprehensive Overview String Format
//         std::string overview = query_engine.getStatusAll(loc.lat, loc.lng);
//         std::cout << "[result] String Dump : " << overview << "\n";

//         if (!overview.empty() && overview != "Coordinates out of map coverage boundaries.") {
//             passed_tests++;
//         }
//     }

//     // 4. Print Summary Diagnostics Report
//     std::cout << "\n==================================================\n";
//     std::cout << "               TEST RUN SUMMARY                   \n";
//     std::cout << "==================================================\n";
//     std::cout << "Total Cases Processed : " << total_tests << "\n";
//     std::cout << "Successful Operations : " << passed_tests << " / " << total_tests << "\n";

//     if (passed_tests == total_tests) {
//         std::cout << "STATUS                : ALL TESTS PASSED SUCCESSFULLY! 🎉\n";
//     } else {
//         std::cout << "STATUS                : FAILURE RECORDED DURING SUITE CHECK\n";
//     }
//     std::cout << "==================================================\n";

//     return (passed_tests == total_tests) ? 0 : 1;
// }
