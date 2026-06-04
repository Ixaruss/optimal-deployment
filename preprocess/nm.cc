// /******************************************************************************
//  *
//  * FAST RADAR COVERAGE / RADAR PLATTER GENERATION
//  * ==============================================
//  *
//  * ALGORITHM:
//  * ----------
//  * Incremental Horizon Sweep using Polar Traversal
//  *
//  * PURPOSE:
//  * --------
//  * Generate radar terrain visibility (viewshed / radar platter)
//  * extremely fast for:
//  *
//  *   - Long range radar (300 km+)
//  *   - Large DEMs
//  *   - Surveillance radar coverage
//  *   - Air defence simulations
//  *   - Real-time military simulation engines
//  *
//  * WHY THIS IS FAST:
//  * -----------------
//  * Traditional LOS algorithms:
//  *
//  *     For every terrain cell:
//  *         Trace LOS from radar
//  *
//  * Complexity becomes enormous.
//  *
//  * This implementation instead:
//  *
//  *     Sweeps outward radially only ONCE
//  *     while maintaining the terrain horizon incrementally.
//  *
//  * This avoids repeated LOS calculations.
//  *
//  * CORE IDEA:
//  * ----------
//  * For every azimuth direction:
//  *
//  *     Move outward cell-by-cell
//  *
//  *     Compute terrain elevation angle:
//  *
//  *         angle = atan2(height_difference, distance)
//  *
//  *     Maintain maximum angle seen so far.
//  *
//  *     If current angle exceeds previous maximum:
//  *
//  *         Terrain is visible
//  *
//  *     Else:
//  *
//  *         Terrain is hidden behind previous terrain.
//  *
//  * VISIBILITY RULE:
//  * ----------------
//  *
//  *     current_angle > max_angle_so_far
//  *
//  * IMPORTANT FEATURES:
//  * -------------------
//  * 1. Polar-space traversal
//  * 2. Incremental horizon tracking
//  * 3. Earth curvature correction
//  * 4. Precomputed trigonometric tables
//  * 5. Cache-friendly memory access
//  * 6. Suitable for multithreading
//  * 7. Suitable for GPU conversion
//  *
//  * WHAT THIS IMPLEMENTATION DOES NOT YET INCLUDE:
//  * ----------------------------------------------
//  * 1. Atmospheric refraction
//  * 2. Radar equation
//  * 3. Beam shape modelling
//  * 4. Adaptive angular resolution
//  * 5. SIMD optimization
//  * 6. GPU acceleration
//  * 7. DEM tiling
//  * 8. DDA/Bresenham exact cell traversal
//  *
//  * These can be added later.
//  *
//  *****************************************************************************/

// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <cstdint>

// /******************************************************************************
//  * CONSTANTS
//  *****************************************************************************/

// // Mathematical PI
// constexpr double PI = 3.141592653589793;

// // Approximate Earth radius in meters
// constexpr double EARTH_RADIUS = 6371000.0;

// /******************************************************************************
//  * DEM STRUCTURE
//  *
//  * Represents a Digital Elevation Model.
//  *
//  * width      -> number of columns
//  * height     -> number of rows
//  * cellSize   -> terrain resolution in meters
//  * elev       -> terrain elevation array
//  *
//  *****************************************************************************/

// struct DEM
// {
//     int width;
//     int height;

//     // Terrain grid spacing in meters
//     double cellSize;

//     // Elevation storage
//     std::vector<float> elev;

//     /**************************************************************************
//      * SAFE TERRAIN ACCESSOR
//      *
//      * Returns terrain elevation at (x,y)
//      *
//      * If outside DEM:
//      *     return invalid elevation
//      *
//      **************************************************************************/
//     inline float get(int x, int y) const
//     {
//         if (x < 0 || y < 0 ||
//             x >= width || y >= height)
//         {
//             return -9999.0f;
//         }

//         return elev[y * width + x];
//     }
// };

// /******************************************************************************
//  * RADAR STRUCTURE
//  *
//  * Stores radar parameters.
//  *
//  *****************************************************************************/

// struct Radar
// {
//     // Radar position in DEM grid coordinates
//     int x;
//     int y;

//     // Radar antenna height above terrain
//     double antennaHeight;

//     // Maximum radar range in meters
//     double maxRange;

//     // Angular sweep spacing
//     //
//     // IMPORTANT:
//     // Smaller value:
//     //      More accurate
//     //      More computation
//     //
//     // Example:
//     //      0.05 degrees
//     //
//     double azimuthStepDeg;

//     // Range stepping interval
//     //
//     // Usually equal to DEM resolution
//     //
//     double rangeStep;
// };

// /******************************************************************************
//  * MAIN PROGRAM
//  *****************************************************************************/

// int main()
// {
//     /**************************************************************************
//      *
//      * STEP 1:
//      * CREATE DEM
//      *
//      **************************************************************************/

//     DEM dem;

//     // DEM dimensions
//     dem.width = 4000;
//     dem.height = 4000;

//     // DEM resolution
//     //
//     // Example:
//     //      90 meter terrain grid
//     //
//     dem.cellSize = 90.0;

//     // Allocate terrain memory
//     dem.elev.resize(dem.width * dem.height);

//     /**************************************************************************
//      *
//      * STEP 2:
//      * GENERATE SYNTHETIC TERRAIN
//      *
//      * In production:
//      *      Read DEM from GDAL / GeoTIFF / DTED / SRTM etc.
//      *
//      **************************************************************************/

//     for (int y = 0; y < dem.height; y++)
//     {
//         for (int x = 0; x < dem.width; x++)
//         {
//             // Artificial terrain function
//             //
//             // Produces hills/waves for testing
//             //
//             dem.elev[y * dem.width + x] =
//                 100.0f +
//                 50.0f * sin(x * 0.01) * cos(y * 0.01);
//         }
//     }

//     /**************************************************************************
//      *
//      * STEP 3:
//      * DEFINE RADAR
//      *
//      **************************************************************************/

//     Radar radar;

//     // Place radar at DEM center
//     radar.x = dem.width / 2;
//     radar.y = dem.height / 2;

//     // Antenna mast height
//     radar.antennaHeight = 30.0;

//     // 300 km radar range
//     radar.maxRange = 300000.0;

//     // Angular sweep spacing
//     //
//     // IMPORTANT:
//     // Do NOT use 1 degree.
//     //
//     // At 300 km:
//     //      1 degree spacing skips several kilometers.
//     //
//     radar.azimuthStepDeg = 0.05;

//     // Radial stepping interval
//     radar.rangeStep = dem.cellSize;

//     /**************************************************************************
//      *
//      * STEP 4:
//      * PRECOMPUTE TRIGONOMETRIC TABLES
//      *
//      * WHY?
//      * ----
//      * sin() and cos() are expensive.
//      *
//      * Avoid recomputing them inside inner loops.
//      *
//      **************************************************************************/

//     int azimuthCount =
//         (int)(360.0 / radar.azimuthStepDeg);

//     std::vector<double> sinTable(azimuthCount);
//     std::vector<double> cosTable(azimuthCount);

//     for (int i = 0; i < azimuthCount; i++)
//     {
//         // Convert azimuth to radians
//         double azimuthRad =
//             i * radar.azimuthStepDeg * PI / 180.0;

//         // Precompute trig
//         sinTable[i] = sin(azimuthRad);
//         cosTable[i] = cos(azimuthRad);
//     }

//     /**************************************************************************
//      *
//      * STEP 5:
//      * CREATE VISIBILITY OUTPUT BUFFER
//      *
//      * 1 = visible
//      * 0 = hidden
//      *
//      **************************************************************************/

//     std::vector<uint8_t>
//         visible(dem.width * dem.height, 0);

//     /**************************************************************************
//      *
//      * STEP 6:
//      * COMPUTE RADAR ABSOLUTE HEIGHT
//      *
//      **************************************************************************/

//     float radarElevation =
//         dem.get(radar.x, radar.y) +
//         radar.antennaHeight;

//     /**************************************************************************
//      *
//      * STEP 7:
//      * MAIN RADIAL HORIZON SWEEP
//      *
//      * THIS IS THE CORE ALGORITHM.
//      *
//      **************************************************************************/

//     for (int azIdx = 0;
//          azIdx < azimuthCount;
//          azIdx++)
//     {
//         /**********************************************************************
//          *
//          * Retrieve precomputed direction vector
//          *
//          **********************************************************************/

//         double dx = cosTable[azIdx];
//         double dy = sinTable[azIdx];

//         /**********************************************************************
//          *
//          * Incremental terrain horizon
//          *
//          * Stores maximum terrain elevation angle seen so far.
//          *
//          **********************************************************************/

//         double maxAngle = -1e9;

//         /**********************************************************************
//          *
//          * Walk outward radially from radar
//          *
//          **********************************************************************/

//         for (double r = radar.rangeStep;
//              r <= radar.maxRange;
//              r += radar.rangeStep)
//         {
//             /******************************************************************
//              *
//              * Convert polar coordinates -> DEM grid coordinates
//              *
//              ******************************************************************/

//             double fx =
//                 radar.x +
//                 (dx * r / dem.cellSize);

//             double fy =
//                 radar.y +
//                 (dy * r / dem.cellSize);

//             int ix = (int)fx;
//             int iy = (int)fy;

//             /******************************************************************
//              *
//              * Stop if outside terrain
//              *
//              ******************************************************************/

//             if (ix < 0 || iy < 0 ||
//                 ix >= dem.width ||
//                 iy >= dem.height)
//             {
//                 break;
//             }

//             /******************************************************************
//              *
//              * Get terrain elevation
//              *
//              ******************************************************************/

//             float terrainElevation =
//                 dem.get(ix, iy);

//             /******************************************************************
//              *
//              * EARTH CURVATURE CORRECTION
//              * --------------------------
//              *
//              * Over long distances, Earth curves away.
//              *
//              * Without correction:
//              *      Long-range visibility becomes unrealistic.
//              *
//              * Formula:
//              *
//              *      drop = d² / (2R)
//              *
//              ******************************************************************/

//             double curvatureDrop =
//                 (r * r) /
//                 (2.0 * EARTH_RADIUS);

//             /******************************************************************
//              *
//              * Corrected terrain elevation
//              *
//              ******************************************************************/

//             double correctedElevation =
//                 terrainElevation -
//                 curvatureDrop;

//             /******************************************************************
//              *
//              * COMPUTE ELEVATION ANGLE
//              *
//              * angle = atan2(height_difference, distance)
//              *
//              ******************************************************************/

//             double angle =
//                 atan2(
//                     correctedElevation - radarElevation,
//                     r);

//             /******************************************************************
//              *
//              * INCREMENTAL HORIZON TEST
//              *
//              * If current terrain angle exceeds previous maximum:
//              *
//              *      Terrain becomes visible
//              *
//              * Otherwise:
//              *
//              *      Hidden behind previous terrain
//              *
//              ******************************************************************/

//             if (angle > maxAngle)
//             {
//                 // Mark terrain cell visible
//                 visible[iy * dem.width + ix] = 1;

//                 // Update horizon
//                 maxAngle = angle;
//             }
//         }
//     }

//     /**************************************************************************
//      *
//      * STEP 8:
//      * COUNT VISIBLE CELLS
//      *
//      **************************************************************************/

//     long long visibleCount = 0;

//     for (auto v : visible)
//     {
//         visibleCount += v;
//     }

//     /**************************************************************************
//      *
//      * OUTPUT RESULTS
//      *
//      **************************************************************************/

//     std::cout
//         << "=====================================\n";

//     std::cout
//         << "RADAR COVERAGE GENERATION COMPLETE\n";

//     std::cout
//         << "=====================================\n";

//     std::cout
//         << "Visible Cells = "
//         << visibleCount
//         << "\n";

//     std::cout
//         << "=====================================\n";

//     return 0;
// }




// cpu threaded code:

// /******************************************************************************
//  *
//  * MULTITHREADED FAST RADAR COVERAGE / RADAR PLATTER GENERATION
//  * ============================================================
//  *
//  * ALGORITHM:
//  * ----------
//  * Parallel Incremental Horizon Sweep
//  *
//  * PURPOSE:
//  * --------
//  * Extremely fast terrain visibility / radar coverage computation
//  * for:
//  *
//  *      - Air defence radar
//  *      - Surveillance radar
//  *      - Radar platter generation
//  *      - Viewshed analysis
//  *      - Real-time simulation engines
//  *      - Military C4ISR systems
//  *
//  * KEY FEATURES:
//  * -------------
//  * 1. CPU multithreading
//  * 2. Incremental horizon algorithm
//  * 3. Polar radial traversal
//  * 4. Earth curvature correction
//  * 5. Precomputed trig tables
//  * 6. Cache-friendly design
//  * 7. Thread-safe processing
//  *
//  * WHY MULTITHREADING WORKS VERY WELL HERE:
//  * ----------------------------------------
//  * Each azimuth sector is independent.
//  *
//  * Therefore:
//  *
//  *      Sector 0-45 deg  -> Thread 0
//  *      Sector 45-90 deg -> Thread 1
//  *      ...
//  *
//  * No synchronization needed during computation.
//  *
//  * This scales almost linearly with CPU cores.
//  *
//  * PERFORMANCE:
//  * ------------
//  * Example:
//  *
//  *      DEM           : 4000 x 4000
//  *      Range         : 300 km
//  *      Azimuth Step  : 0.05 deg
//  *
//  * Single-thread:
//  *      slow
//  *
//  * Multithread:
//  *      several times faster
//  *
//  * BUILD:
//  * ------
//  *
//  * Linux:
//  *      g++ radar.cpp -O3 -march=native -pthread
//  *
//  * Windows MinGW:
//  *      g++ radar.cpp -O3 -pthread
//  *
//  *****************************************************************************/

// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <thread>
// #include <cstdint>
// #include <chrono>

// /******************************************************************************
//  * CONSTANTS
//  *****************************************************************************/

// constexpr double PI = 3.141592653589793;
// constexpr double EARTH_RADIUS = 6371000.0;

// /******************************************************************************
//  * DEM STRUCTURE
//  *****************************************************************************/

// struct DEM
// {
//     int width;
//     int height;

//     double cellSize;

//     std::vector<float> elev;

//     /**************************************************************************
//      * SAFE TERRAIN ACCESS
//      **************************************************************************/
//     inline float get(int x, int y) const
//     {
//         if (x < 0 || y < 0 ||
//             x >= width || y >= height)
//         {
//             return -9999.0f;
//         }

//         return elev[y * width + x];
//     }
// };

// /******************************************************************************
//  * RADAR STRUCTURE
//  *****************************************************************************/

// struct Radar
// {
//     int x;
//     int y;

//     double antennaHeight;

//     double maxRange;

//     double azimuthStepDeg;

//     double rangeStep;
// };

// /******************************************************************************
//  * THREAD WORKER FUNCTION
//  *
//  * Each thread processes:
//  *
//  *      azimuthStart -> azimuthEnd
//  *
//  * independently.
//  *
//  *****************************************************************************/

// void processAzimuthSector(
//     const DEM& dem,
//     const Radar& radar,
//     const std::vector<double>& sinTable,
//     const std::vector<double>& cosTable,
//     std::vector<uint8_t>& visible,
//     int azimuthStart,
//     int azimuthEnd)
// {
//     /**************************************************************************
//      * Compute radar absolute elevation
//      **************************************************************************/

//     float radarElevation =
//         dem.get(radar.x, radar.y) +
//         radar.antennaHeight;

//     /**************************************************************************
//      * PROCESS ASSIGNED AZIMUTH RANGE
//      **************************************************************************/

//     for (int azIdx = azimuthStart;
//          azIdx < azimuthEnd;
//          azIdx++)
//     {
//         /**********************************************************************
//          * Direction vector
//          **********************************************************************/

//         double dx = cosTable[azIdx];
//         double dy = sinTable[azIdx];

//         /**********************************************************************
//          * Incremental horizon
//          *
//          * Stores highest terrain angle seen so far.
//          **********************************************************************/

//         double maxAngle = -1e9;

//         /**********************************************************************
//          * Walk outward radially
//          **********************************************************************/

//         for (double r = radar.rangeStep;
//              r <= radar.maxRange;
//              r += radar.rangeStep)
//         {
//             /******************************************************************
//              * Polar -> grid conversion
//              ******************************************************************/

//             double fx =
//                 radar.x +
//                 (dx * r / dem.cellSize);

//             double fy =
//                 radar.y +
//                 (dy * r / dem.cellSize);

//             int ix = (int)fx;
//             int iy = (int)fy;

//             /******************************************************************
//              * Stop if outside DEM
//              ******************************************************************/

//             if (ix < 0 || iy < 0 ||
//                 ix >= dem.width ||
//                 iy >= dem.height)
//             {
//                 break;
//             }

//             /******************************************************************
//              * Terrain elevation
//              ******************************************************************/

//             float terrainElevation =
//                 dem.get(ix, iy);

//             /******************************************************************
//              * EARTH CURVATURE CORRECTION
//              *
//              * Formula:
//              *
//              *      drop = d² / (2R)
//              *
//              ******************************************************************/

//             double curvatureDrop =
//                 (r * r) /
//                 (2.0 * EARTH_RADIUS);

//             double correctedElevation =
//                 terrainElevation -
//                 curvatureDrop;

//             /******************************************************************
//              * TERRAIN ELEVATION ANGLE
//              *
//              * angle = atan2(height_diff, distance)
//              *
//              ******************************************************************/

//             double angle =
//                 atan2(
//                     correctedElevation - radarElevation,
//                     r);

//             /******************************************************************
//              * INCREMENTAL HORIZON TEST
//              *
//              * If terrain exceeds previous horizon:
//              *      visible
//              *
//              ******************************************************************/

//             if (angle > maxAngle)
//             {
//                 visible[iy * dem.width + ix] = 1;

//                 maxAngle = angle;
//             }
//         }
//     }
// }

// /******************************************************************************
//  * MAIN
//  *****************************************************************************/

// int main()
// {
//     /**************************************************************************
//      * TIMER START
//      **************************************************************************/

//     auto startTime =
//         std::chrono::high_resolution_clock::now();

//     /**************************************************************************
//      * CREATE DEM
//      **************************************************************************/

//     DEM dem;

//     dem.width = 4000;
//     dem.height = 4000;

//     dem.cellSize = 90.0;

//     dem.elev.resize(dem.width * dem.height);

//     /**************************************************************************
//      * GENERATE SYNTHETIC TERRAIN
//      *
//      * Replace with:
//      *      GDAL / DTED / GeoTIFF loading
//      *
//      **************************************************************************/

//     for (int y = 0; y < dem.height; y++)
//     {
//         for (int x = 0; x < dem.width; x++)
//         {
//             dem.elev[y * dem.width + x] =
//                 100.0f +
//                 50.0f *
//                 sin(x * 0.01) *
//                 cos(y * 0.01);
//         }
//     }

//     /**************************************************************************
//      * RADAR CONFIGURATION
//      **************************************************************************/

//     Radar radar;

//     radar.x = dem.width / 2;
//     radar.y = dem.height / 2;

//     radar.antennaHeight = 30.0;

//     radar.maxRange = 300000.0;

//     radar.azimuthStepDeg = 0.05;

//     radar.rangeStep = dem.cellSize;

//     /**************************************************************************
//      * PRECOMPUTE TRIG TABLES
//      *
//      * Huge optimization.
//      *
//      **************************************************************************/

//     int azimuthCount =
//         (int)(360.0 / radar.azimuthStepDeg);

//     std::vector<double> sinTable(azimuthCount);
//     std::vector<double> cosTable(azimuthCount);

//     for (int i = 0; i < azimuthCount; i++)
//     {
//         double azimuthRad =
//             i * radar.azimuthStepDeg *
//             PI / 180.0;

//         sinTable[i] = sin(azimuthRad);
//         cosTable[i] = cos(azimuthRad);
//     }

//     /**************************************************************************
//      * VISIBILITY BUFFER
//      *
//      * Shared output.
//      *
//      * THREAD SAFETY:
//      * --------------
//      * Multiple threads may write same value (=1)
//      * safely without locks.
//      *
//      **************************************************************************/

//     std::vector<uint8_t>
//         visible(dem.width * dem.height, 0);

//     /**************************************************************************
//      * MULTITHREADING SETUP
//      **************************************************************************/

//     unsigned int threadCount =
//         std::thread::hardware_concurrency();

//     if (threadCount == 0)
//     {
//         threadCount = 4;
//     }

//     std::cout
//         << "CPU Threads = "
//         << threadCount
//         << "\n";

//     /**************************************************************************
//      * CREATE THREADS
//      **************************************************************************/

//     std::vector<std::thread> workers;

//     /**************************************************************************
//      * Divide azimuth space among threads
//      **************************************************************************/

//     int azimuthsPerThread =
//         azimuthCount / threadCount;

//     for (unsigned int t = 0;
//          t < threadCount;
//          t++)
//     {
//         int startAz =
//             t * azimuthsPerThread;

//         int endAz =
//             (t == threadCount - 1)
//             ? azimuthCount
//             : startAz + azimuthsPerThread;

//         /**********************************************************************
//          * Launch worker thread
//          **********************************************************************/

//         workers.emplace_back(
//             processAzimuthSector,
//             std::cref(dem),
//             std::cref(radar),
//             std::cref(sinTable),
//             std::cref(cosTable),
//             std::ref(visible),
//             startAz,
//             endAz);
//     }

//     /**************************************************************************
//      * WAIT FOR THREADS
//      **************************************************************************/

//     for (auto& t : workers)
//     {
//         t.join();
//     }

//     /**************************************************************************
//      * COUNT VISIBLE CELLS
//      **************************************************************************/

//     long long visibleCount = 0;

//     for (auto v : visible)
//     {
//         visibleCount += v;
//     }

//     /**************************************************************************
//      * TIMER END
//      **************************************************************************/

//     auto endTime =
//         std::chrono::high_resolution_clock::now();

//     double elapsedSeconds =
//         std::chrono::duration<double>(
//             endTime - startTime).count();

//     /**************************************************************************
//      * OUTPUT RESULTS
//      **************************************************************************/

//     std::cout
//         << "====================================\n";

//     std::cout
//         << "RADAR COVERAGE COMPLETE\n";

//     std::cout
//         << "====================================\n";

//     std::cout
//         << "Visible Cells : "
//         << visibleCount
//         << "\n";

//     std::cout
//         << "Execution Time: "
//         << elapsedSeconds
//         << " sec\n";

//     std::cout
//         << "====================================\n";

//     return 0;
// }

// gpu based code

// /******************************************************************************
//  *
//  * CUDA GPU RADAR COVERAGE / RADAR PLATTER GENERATION
//  * ==================================================
//  *
//  * PURPOSE:
//  * --------
//  * Extremely fast radar terrain visibility generation
//  * using NVIDIA CUDA GPU acceleration.
//  *
//  * TARGET USE CASES:
//  * -----------------
//  *  - Long-range surveillance radar
//  *  - Real-time radar platter generation
//  *  - Air-defence simulations
//  *  - Terrain masking analysis
//  *  - Large DEM processing
//  *  - Military simulation engines
//  *
//  * CORE ALGORITHM:
//  * ---------------
//  * Incremental Horizon Sweep
//  *
//  * GPU PARALLELIZATION MODEL:
//  * --------------------------
//  * One CUDA thread processes:
//  *
//  *      ONE AZIMUTH RADIAL
//  *
//  * Example:
//  *
//  *      Thread 0  -> 0 deg radial
//  *      Thread 1  -> 0.05 deg radial
//  *      Thread 2  -> 0.10 deg radial
//  *
//  * etc...
//  *
//  * WHY THIS WORKS WELL:
//  * --------------------
//  * Each radial is independent.
//  *
//  * Therefore:
//  *      perfect GPU parallel workload
//  *
//  * MAJOR FEATURES:
//  * ---------------
//  * 1. CUDA GPU acceleration
//  * 2. Parallel azimuth processing
//  * 3. Incremental horizon algorithm
//  * 4. Earth curvature correction
//  * 5. Massive throughput
//  * 6. Radar-scale computation
//  *
//  * PERFORMANCE:
//  * ------------
//  * GPU acceleration can provide:
//  *
//  *      10x - 100x speedup
//  *
//  * depending on:
//  *      - GPU
//  *      - DEM size
//  *      - azimuth resolution
//  *      - memory bandwidth
//  *
//  * BUILD:
//  * ------
//  *
//  * nvcc radar_cuda.cu -O3
//  *
//  *****************************************************************************/

// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <cstdint>
// #include <chrono>

// #include <cuda_runtime.h>

// /******************************************************************************
//  * CONSTANTS
//  *****************************************************************************/

// #define PI 3.141592653589793
// #define EARTH_RADIUS 6371000.0

// /******************************************************************************
//  * CUDA ERROR CHECK MACRO
//  *****************************************************************************/

// #define CUDA_CHECK(x)                                      \
// {                                                          \
//     cudaError_t err = x;                                   \
//     if (err != cudaSuccess)                                \
//     {                                                      \
//         std::cout                                           \
//             << "CUDA ERROR: "                              \
//             << cudaGetErrorString(err)                     \
//             << std::endl;                                  \
//         exit(1);                                           \
//     }                                                      \
// }

// /******************************************************************************
//  * DEM STRUCTURE
//  *****************************************************************************/

// struct DEM
// {
//     int width;
//     int height;

//     double cellSize;

//     std::vector<float> elev;
// };

// /******************************************************************************
//  * RADAR STRUCTURE
//  *****************************************************************************/

// struct Radar
// {
//     int x;
//     int y;

//     double antennaHeight;

//     double maxRange;

//     double azimuthStepDeg;

//     double rangeStep;
// };

// /******************************************************************************
//  * DEVICE FUNCTION:
//  * SAFE TERRAIN ACCESS
//  *****************************************************************************/

// __device__
// inline float getTerrain(
//     const float* dem,
//     int width,
//     int height,
//     int x,
//     int y)
// {
//     if (x < 0 || y < 0 ||
//         x >= width || y >= height)
//     {
//         return -9999.0f;
//     }

//     return dem[y * width + x];
// }

// /******************************************************************************
//  *
//  * CUDA KERNEL
//  * ===========
//  *
//  * One thread = one azimuth radial
//  *
//  *****************************************************************************/

// __global__
// void radarCoverageKernel(
//     const float* dem,
//     int width,
//     int height,
//     double cellSize,

//     Radar radar,

//     const double* sinTable,
//     const double* cosTable,

//     int azimuthCount,

//     uint8_t* visible)
// {
//     /**************************************************************************
//      * THREAD INDEX
//      **************************************************************************/

//     int azIdx =
//         blockIdx.x * blockDim.x +
//         threadIdx.x;

//     /**************************************************************************
//      * Bounds check
//      **************************************************************************/

//     if (azIdx >= azimuthCount)
//     {
//         return;
//     }

//     /**************************************************************************
//      * Direction vector
//      **************************************************************************/

//     double dx = cosTable[azIdx];
//     double dy = sinTable[azIdx];

//     /**************************************************************************
//      * Radar elevation
//      **************************************************************************/

//     float radarElevation =
//         getTerrain(
//             dem,
//             width,
//             height,
//             radar.x,
//             radar.y)
//         +
//         radar.antennaHeight;

//     /**************************************************************************
//      * Incremental horizon
//      **************************************************************************/

//     double maxAngle = -1e9;

//     /**************************************************************************
//      * Walk outward radially
//      **************************************************************************/

//     for (double r = radar.rangeStep;
//          r <= radar.maxRange;
//          r += radar.rangeStep)
//     {
//         /**********************************************************************
//          * Polar -> DEM grid
//          **********************************************************************/

//         double fx =
//             radar.x +
//             (dx * r / cellSize);

//         double fy =
//             radar.y +
//             (dy * r / cellSize);

//         int ix = (int)fx;
//         int iy = (int)fy;

//         /**********************************************************************
//          * Stop if outside terrain
//          **********************************************************************/

//         if (ix < 0 || iy < 0 ||
//             ix >= width || iy >= height)
//         {
//             break;
//         }

//         /**********************************************************************
//          * Terrain elevation
//          **********************************************************************/

//         float terrainElevation =
//             getTerrain(
//                 dem,
//                 width,
//                 height,
//                 ix,
//                 iy);

//         /**********************************************************************
//          * EARTH CURVATURE CORRECTION
//          *
//          * drop = d² / (2R)
//          *
//          **********************************************************************/

//         double curvatureDrop =
//             (r * r) /
//             (2.0 * EARTH_RADIUS);

//         double correctedElevation =
//             terrainElevation -
//             curvatureDrop;

//         /**********************************************************************
//          * Elevation angle
//          **********************************************************************/

//         double angle =
//             atan2(
//                 correctedElevation -
//                 radarElevation,
//                 r);

//         /**********************************************************************
//          * Incremental horizon visibility test
//          **********************************************************************/

//         if (angle > maxAngle)
//         {
//             /******************************************************************
//              * Mark visible
//              *
//              * Multiple threads may write 1 safely.
//              ******************************************************************/

//             visible[iy * width + ix] = 1;

//             /******************************************************************
//              * Update horizon
//              ******************************************************************/

//             maxAngle = angle;
//         }
//     }
// }

// /******************************************************************************
//  * MAIN
//  *****************************************************************************/

// int main()
// {
//     /**************************************************************************
//      * TIMER START
//      **************************************************************************/

//     auto startTime =
//         std::chrono::high_resolution_clock::now();

//     /**************************************************************************
//      * CREATE DEM
//      **************************************************************************/

//     DEM dem;

//     dem.width = 4000;
//     dem.height = 4000;

//     dem.cellSize = 90.0;

//     dem.elev.resize(
//         dem.width * dem.height);

//     /**************************************************************************
//      * SYNTHETIC TERRAIN
//      *
//      * Replace with:
//      *      GDAL / DTED / GeoTIFF loading
//      *
//      **************************************************************************/

//     for (int y = 0; y < dem.height; y++)
//     {
//         for (int x = 0; x < dem.width; x++)
//         {
//             dem.elev[y * dem.width + x] =
//                 100.0f +
//                 50.0f *
//                 sin(x * 0.01) *
//                 cos(y * 0.01);
//         }
//     }

//     /**************************************************************************
//      * RADAR CONFIGURATION
//      **************************************************************************/

//     Radar radar;

//     radar.x = dem.width / 2;
//     radar.y = dem.height / 2;

//     radar.antennaHeight = 30.0;

//     radar.maxRange = 300000.0;

//     radar.azimuthStepDeg = 0.05;

//     radar.rangeStep = dem.cellSize;

//     /**************************************************************************
//      * PRECOMPUTE TRIG TABLES
//      **************************************************************************/

//     int azimuthCount =
//         (int)(360.0 /
//         radar.azimuthStepDeg);

//     std::vector<double>
//         sinTable(azimuthCount);

//     std::vector<double>
//         cosTable(azimuthCount);

//     for (int i = 0; i < azimuthCount; i++)
//     {
//         double azimuthRad =
//             i *
//             radar.azimuthStepDeg *
//             PI / 180.0;

//         sinTable[i] = sin(azimuthRad);
//         cosTable[i] = cos(azimuthRad);
//     }

//     /**************************************************************************
//      * GPU MEMORY
//      **************************************************************************/

//     float* d_dem = nullptr;

//     double* d_sinTable = nullptr;
//     double* d_cosTable = nullptr;

//     uint8_t* d_visible = nullptr;

//     /**************************************************************************
//      * Allocate GPU memory
//      **************************************************************************/

//     CUDA_CHECK(cudaMalloc(
//         &d_dem,
//         dem.elev.size() * sizeof(float)));

//     CUDA_CHECK(cudaMalloc(
//         &d_sinTable,
//         azimuthCount * sizeof(double)));

//     CUDA_CHECK(cudaMalloc(
//         &d_cosTable,
//         azimuthCount * sizeof(double)));

//     CUDA_CHECK(cudaMalloc(
//         &d_visible,
//         dem.width *
//         dem.height *
//         sizeof(uint8_t)));

//     /**************************************************************************
//      * Copy data to GPU
//      **************************************************************************/

//     CUDA_CHECK(cudaMemcpy(
//         d_dem,
//         dem.elev.data(),
//         dem.elev.size() * sizeof(float),
//         cudaMemcpyHostToDevice));

//     CUDA_CHECK(cudaMemcpy(
//         d_sinTable,
//         sinTable.data(),
//         azimuthCount * sizeof(double),
//         cudaMemcpyHostToDevice));

//     CUDA_CHECK(cudaMemcpy(
//         d_cosTable,
//         cosTable.data(),
//         azimuthCount * sizeof(double),
//         cudaMemcpyHostToDevice));

//     CUDA_CHECK(cudaMemset(
//         d_visible,
//         0,
//         dem.width *
//         dem.height));

//     /**************************************************************************
//      * CUDA KERNEL LAUNCH CONFIGURATION
//      **************************************************************************/

//     int threadsPerBlock = 256;

//     int blocks =
//         (azimuthCount +
//          threadsPerBlock - 1)
//         / threadsPerBlock;

//     /**************************************************************************
//      * LAUNCH CUDA KERNEL
//      **************************************************************************/

//     radarCoverageKernel<<<
//         blocks,
//         threadsPerBlock>>>(
//             d_dem,
//             dem.width,
//             dem.height,
//             dem.cellSize,

//             radar,

//             d_sinTable,
//             d_cosTable,

//             azimuthCount,

//             d_visible);

//     /**************************************************************************
//      * Wait for GPU completion
//      **************************************************************************/

//     CUDA_CHECK(cudaDeviceSynchronize());

//     /**************************************************************************
//      * COPY RESULTS BACK
//      **************************************************************************/

//     std::vector<uint8_t>
//         visible(
//             dem.width *
//             dem.height);

//     CUDA_CHECK(cudaMemcpy(
//         visible.data(),
//         d_visible,
//         visible.size(),
//         cudaMemcpyDeviceToHost));

//     /**************************************************************************
//      * COUNT VISIBLE CELLS
//      **************************************************************************/

//     long long visibleCount = 0;

//     for (auto v : visible)
//     {
//         visibleCount += v;
//     }

//     /**************************************************************************
//      * TIMER END
//      **************************************************************************/

//     auto endTime =
//         std::chrono::high_resolution_clock::now();

//     double elapsedSeconds =
//         std::chrono::duration<double>(
//             endTime - startTime).count();

//     /**************************************************************************
//      * OUTPUT RESULTS
//      **************************************************************************/

//     std::cout
//         << "====================================\n";

//     std::cout
//         << "CUDA RADAR COVERAGE COMPLETE\n";

//     std::cout
//         << "====================================\n";

//     std::cout
//         << "Visible Cells : "
//         << visibleCount
//         << "\n";

//     std::cout
//         << "Execution Time: "
//         << elapsedSeconds
//         << " sec\n";

//     std::cout
//         << "====================================\n";

//     /**************************************************************************
//      * FREE GPU MEMORY
//      **************************************************************************/

//     cudaFree(d_dem);

//     cudaFree(d_sinTable);
//     cudaFree(d_cosTable);

//     cudaFree(d_visible);

//     return 0;
// }
