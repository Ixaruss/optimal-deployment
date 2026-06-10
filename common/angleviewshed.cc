/**
 * @file Viewshed.cc
 *
 * Correct LOS algorithm: max-angle horizon sweep.
 *
 * For each ray from source outward:
 *   - Track running maximum slope seen so far
 *   - Cell is visible if its slope >= maxSlope
 *   - Update maxSlope when a higher slope is found
 *
 * This is physically correct. A cell is visible if the line from
 * the radar to that cell is not obstructed by any closer terrain.
 */

#include "../operations.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

    // ------------------------------------------------------------------
    // CORRECT MAX-ANGLE HORIZON RAY
    //
    // Walk outward from source toward (xEnd, yEnd).
    // Track maxSlope = max(dh/dist) seen so far.
    // Cell visible <=> its slope >= maxSlope.
    //
    // dist is accumulated correctly:
    //   axis step     -> RESOLUTION
    //   diagonal step -> RESOLUTION * sqrt(2)
    //
    // Earth curvature: subtract d²/2R from terrain elevation.
    // At 300km this is ~7km — significant for radar.
    // ------------------------------------------------------------------
    void ViewshedEngine::castRay(
        int xEnd, int yEnd,
        double srcElev,
        const short* __restrict demPtr,
        uint8_t*     __restrict maskPtr)
    {
        int adx = std::abs(xEnd - centerX);
        int ady = std::abs(yEnd - centerY);
        int sx  = (centerX < xEnd) ? 1 : -1;
        int sy  = (centerY < yEnd) ? 1 : -1;
        int err = adx - ady;

        // centerX = row, centerY = col
        // layout: elev_matrix[row * COLS + col]
        // stepping row (centerX direction) -> ±COLS
        // stepping col (centerY direction) -> ±1
        const std::ptrdiff_t strideX =
            static_cast<std::ptrdiff_t>(sx) * gridCols;  // row step
        const std::ptrdiff_t strideY = sy;                // col step

        std::ptrdiff_t off =
            static_cast<std::ptrdiff_t>(centerX) * gridCols + centerY;

        const short* dem  = demPtr  + off;
        uint8_t*     mask = maskPtr + off;

        int    x = centerX, y = centerY;
        double dist     = 0.0;
        double maxSlope = -1e18;

        // incremental curvature: drop = dist² / 2R
        // updated as: drop(d+step) = drop(d) + (2d*step + step²) / 2R
        double curvDrop = 0.0;

        const double maxDist =
            static_cast<double>(maxRadiusCells) * RESOLUTION;

        while (true)
        {
            if (x == xEnd && y == yEnd) break;

            int e2     = 2 * err;
            bool moveX = (e2 > -ady);
            bool moveY = (e2 <  adx);

            if (moveX) { err -= ady; x += sx; dem += strideX; mask += strideX; }
            if (moveY) { err += adx; y += sy; dem += strideY; mask += strideY; }

            // exact step length
            double step = (moveX && moveY)
                ? RESOLUTION * 1.41421356237
                : RESOLUTION;

            // incremental curvature update
            if (useEarthCurvature)
                curvDrop += (2.0 * dist * step + step * step) * INV_2EARTHR;

            dist += step;

            if (dist > maxDist) break;

            short rawElev = *dem;
            if (rawElev < -9000) break;

            // corrected terrain elevation
            double terrainElev = static_cast<double>(rawElev) - curvDrop;

            // slope = tan(elevation angle) — no trig needed, ordering preserved
            double slope = (terrainElev - srcElev) / dist;

            if (slope >= maxSlope)
            {
                *mask = 1;
                if (slope > maxSlope)
                    maxSlope = slope;
            }
        }
    }

    // ------------------------------------------------------------------
    void ViewshedEngine::precomputeCirclePerimeter()
    {
        int x = 0, y = maxRadiusCells, d = 3 - 2 * maxRadiusCells;

        auto push8 = [&](int px, int py) {
            circlePerimeter.push_back({centerX+px, centerY+py});
            circlePerimeter.push_back({centerX-px, centerY+py});
            circlePerimeter.push_back({centerX+px, centerY-py});
            circlePerimeter.push_back({centerX-px, centerY-py});
            circlePerimeter.push_back({centerX+py, centerY+px});
            circlePerimeter.push_back({centerX-py, centerY+px});
            circlePerimeter.push_back({centerX+py, centerY-px});
            circlePerimeter.push_back({centerX-py, centerY-px});
        };

        while (y >= x) {
            push8(x, y);
            x++;
            if (d > 0) { y--; d += 4*(x-y)+10; }
            else        {      d += 4*x+6;      }
        }

        // deduplicate
        std::sort(circlePerimeter.begin(), circlePerimeter.end(),
            [](const Point2D& a, const Point2D& b){
                return a.x < b.x || (a.x == b.x && a.y < b.y);
            });
        auto last = std::unique(circlePerimeter.begin(), circlePerimeter.end(),
            [](const Point2D& a, const Point2D& b){
                return a.x == b.x && a.y == b.y;
            });
        circlePerimeter.erase(last, circlePerimeter.end());
    }



    // ------------------------------------------------------------------
    void ViewshedEngine::computeViewshed()
    {
        auto t0 = std::chrono::high_resolution_clock::now();

        // centerX = row, centerY = col
        size_t srcIdx =
            static_cast<size_t>(centerX) * gridCols + centerY;

        short srcTerrain = g.bin.elev_matrix[srcIdx];
        if (srcTerrain < -9000) {
            std::cerr << "[ERROR] No-data elevation at radar cell\n";
            return;
        }

        double srcElev =
            static_cast<double>(srcTerrain) + radarHeightMeters;

        // source cell always visible
        viewshedMask[srcIdx] = 1;

        const size_t N       = circlePerimeter.size();
        const short* demPtr  = g.bin.elev_matrix.data();
        uint8_t*     maskPtr = viewshedMask.data();
        const int    COLS    = gridCols;
        const int    ROWS    = gridRows;

        // rays are independent — safe to parallelize
        // each ray only writes cells it visits (no two rays share a cell
        // on the same Bresenham path), and both write value=1 if they do
        #pragma omp parallel for schedule(dynamic, 64) default(none) \
            shared(demPtr, maskPtr) \
            firstprivate(N, COLS, ROWS, srcElev)
        for (size_t i = 0; i < N; ++i)
        {
            int tx = circlePerimeter[i].x;
            int ty = circlePerimeter[i].y;
            if (tx >= 0 && tx < COLS && ty >= 0 && ty < ROWS)
                castRay(tx, ty, srcElev, demPtr, maskPtr);
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Viewshed compute             : "
                  << std::chrono::duration<double,std::milli>(t1-t0).count()
                  << " ms\n";

        // stats
        int vis = 0;
        for (auto v : viewshedMask) vis += v;
        std::cout << "[INFO]   Visible cells: " << vis << "\n";
    }





/******************************************************************************
 * MAIN
 ******************************************************************************/

// int main()
// {
//     auto t0 = std::chrono::high_resolution_clock::now();

//     Global g;

//     // Srinagar airport
//     auto engine = std::make_unique<ViewshedEngine>(
//         g,
//         33.9871, 74.7742,  // lon
//         300.0f,       // range km
//         30.0f,        // antenna height m
//         true          // earth curvature
//     );

//     engine->computeViewshed();
//     engine->writeViewshedToSVG("viewshed_timed.svg");

//     auto t1 = std::chrono::high_resolution_clock::now();
//     std::cout << "\n=======================================================\n"
//               << ">> TOTAL: "
//               << std::chrono::duration<double,std::milli>(t1-t0).count()/1000.0
//               << " s\n=======================================================\n";
// }
