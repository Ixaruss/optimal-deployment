/**
 * @file ViewshedTimedMetrics.cpp
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

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <algorithm>
#include "../common.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

constexpr double EARTH_R     = 6371000.0;
constexpr double INV_2EARTHR = 1.0 / (2.0 * EARTH_R);

struct Point2D { int x, y; };

class ViewshedMetricEngine
{
private:
    Global&  g;
    int      gridCols;
    int      gridRows;
    int      centerX;
    int      centerY;
    int      maxRadiusCells;
    float    radarHeightMeters;
    bool     useEarthCurvature;

    std::vector<uint8_t> viewshedMask;
    std::vector<Point2D> circlePerimeter;

    // ------------------------------------------------------------------
    std::string base64Encode(const uint8_t* data, size_t length)
    {
        static const char lut[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        out.reserve(((length + 2) / 3) * 4);
        size_t i = 0;
        for (; i + 2 < length; i += 3) {
            uint32_t v = ((uint32_t)data[i]<<16)|((uint32_t)data[i+1]<<8)|data[i+2];
            out.push_back(lut[(v>>18)&0x3F]);
            out.push_back(lut[(v>>12)&0x3F]);
            out.push_back(lut[(v>> 6)&0x3F]);
            out.push_back(lut[ v     &0x3F]);
        }
        if (i < length) {
            uint32_t v = (uint32_t)data[i] << 16;
            if (i+1 < length) v |= (uint32_t)data[i+1] << 8;
            out.push_back(lut[(v>>18)&0x3F]);
            out.push_back(lut[(v>>12)&0x3F]);
            out.push_back(i+1 < length ? lut[(v>>6)&0x3F] : '=');
            out.push_back('=');
        }
        return out;
    }

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
    inline void castRay(
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
    void precomputeCirclePerimeter()
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

public:
    ViewshedMetricEngine(
        Global& globalInstance,
        double  srcLat,
        double  srcLon,
        float   rangeKM,
        float   txHeightMeters,
        bool    earthCurvature = true)
        : g(globalInstance)
        , radarHeightMeters(txHeightMeters)
        , useEarthCurvature(earthCurvature)
    {
        auto t0 = std::chrono::high_resolution_clock::now();

        gridCols = g.conf.grid.cols;
        gridRows = g.conf.grid.rows;

        if (!g.bin.getGridCoords(srcLat, srcLon, centerX, centerY)) {
            std::cerr << "[ERROR] Source outside grid\n";
            centerX = gridCols / 2;
            centerY = gridRows / 2;
        }

        maxRadiusCells = static_cast<int>(
            std::round(rangeKM * 1000.0f / (float)RESOLUTION));

        int maxPossible = std::min(
            std::min(centerX, gridCols - centerX - 1),
            std::min(centerY, gridRows - centerY - 1));
        if (maxRadiusCells > maxPossible) {
            std::cout << "[WARN] Radius clamped from "
                      << maxRadiusCells << " to " << maxPossible << "\n";
            maxRadiusCells = maxPossible;
        }

        viewshedMask.assign((size_t)gridCols * gridRows, 0);
        precomputeCirclePerimeter();

        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Init                         : "
                  << std::chrono::duration<double,std::milli>(t1-t0).count()
                  << " ms\n";
        std::cout << "[INFO]   Center  (" << centerX << ", " << centerY << ")\n";
        std::cout << "[INFO]   Radius  " << maxRadiusCells << " cells = "
                  << (maxRadiusCells * RESOLUTION / 1000.0) << " km\n";
        std::cout << "[INFO]   Rays    " << circlePerimeter.size() << "\n";
        std::cout << "[INFO]   Curv    " << (useEarthCurvature ? "ON" : "OFF") << "\n";
    }

    // ------------------------------------------------------------------
    void computeViewshed()
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

    // ------------------------------------------------------------------
    void writeViewshedToSVG(const std::string& filename)
    {
        auto tAll = std::chrono::high_resolution_clock::now();

        // centerX = row (north-south, increases southward)
        // centerY = col (east-west,  increases eastward)
        //
        // Image convention:
        //   px (horizontal) = east-west  = column axis
        //   py (vertical)   = north-south = row axis  (0=north at top)

        int rowMin = std::max(0,        centerX - maxRadiusCells - 2);
        int rowMax = std::min(gridRows, centerX + maxRadiusCells + 2);
        int colMin = std::max(0,        centerY - maxRadiusCells - 2);
        int colMax = std::min(gridCols, centerY + maxRadiusCells + 2);

        int W = colMax - colMin;   // image width  = number of columns (E-W)
        int H = rowMax - rowMin;   // image height = number of rows    (N-S)

        size_t total = (size_t)W * H;
        std::vector<uint8_t> rgba(total * 4, 0);

        long long rSq =
            static_cast<long long>(maxRadiusCells) * maxRadiusCells;

        auto t0 = std::chrono::high_resolution_clock::now();

        #pragma omp parallel for schedule(static, 4096)
        for (size_t i = 0; i < total; ++i)
        {
            int px  = (int)(i % W);          // horizontal → column
            int py  = (int)(i / W);          // vertical   → row
            int col = colMin + px;
            int row = rowMin + py;

            int dcol = col - centerY;        // column offset from source
            int drow = row - centerX;        // row offset from source

            if ((long long)dcol*dcol + (long long)drow*drow > rSq)
                continue;

            // [row * COLS + col] — standard row-major
            size_t gridIdx = (size_t)row * gridCols + col;
            size_t pix     = i * 4;

            if (viewshedMask[gridIdx]) {
                rgba[pix+0] = 46;  rgba[pix+1] = 204;
                rgba[pix+2] = 113; rgba[pix+3] = 255;
            } else {
                rgba[pix+0] = 231; rgba[pix+1] = 76;
                rgba[pix+2] = 60;  rgba[pix+3] = 255;
            }
        }

        // BGRA swap for BMP
        #pragma omp parallel for schedule(static, 4096)
        for (size_t i = 0; i < total; ++i)
            std::swap(rgba[i*4+0], rgba[i*4+2]);

        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] RGBA map + BGRA swap         : "
                  << std::chrono::duration<double,std::milli>(t1-t0).count()
                  << " ms\n";

        t0 = std::chrono::high_resolution_clock::now();
        uint32_t fs = 54 + (uint32_t)(total * 4);
        uint8_t hdr[54] = {
            'B','M',
            (uint8_t)fs,(uint8_t)(fs>>8),(uint8_t)(fs>>16),(uint8_t)(fs>>24),
            0,0,0,0, 54,0,0,0, 40,0,0,0,
            (uint8_t)W,(uint8_t)(W>>8),(uint8_t)(W>>16),(uint8_t)(W>>24),
            (uint8_t)(-H),(uint8_t)(-H>>8),(uint8_t)(-H>>16),(uint8_t)(-H>>24),
            1,0, 32,0, 0,0,0,0, 0,0,0,0,
            0,0,0,0, 0,0,0,0, 0,0,0,0
        };
        std::vector<uint8_t> payload;
        payload.reserve(54 + total * 4);
        payload.insert(payload.end(), hdr, hdr + 54);
        payload.insert(payload.end(), rgba.begin(), rgba.end());
        std::string b64 = base64Encode(payload.data(), payload.size());

        t1 = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Base64 encode                : "
                  << std::chrono::duration<double,std::milli>(t1-t0).count()
                  << " ms\n";

        t0 = std::chrono::high_resolution_clock::now();
        // center in image coords: col offset = horizontal, row offset = vertical
        int cx = (centerY - colMin);   // east-west  → image x
        int cy = (centerX - rowMin);   // north-south → image y
        std::ofstream f(filename);
        f << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n"
          << "<svg xmlns=\"http://www.w3.org/2000/svg\""
          << " viewBox=\"0 0 " << W << " " << H << "\""
          << " width=\"100%\" height=\"100%\""
          << " style=\"background-color:#0a0a0a;\">\n"
          << "  <style>"
          << ".rm{fill:#FFF;stroke:#1abc9c;stroke-width:8}"
          << ".rr{fill:none;stroke:#FFF;stroke-width:3;"
          << "stroke-dasharray:20,15;opacity:.4}"
          << "</style>\n"
          << "  <image width=\""<<W<<"\" height=\""<<H
          << "\" x=\"0\" y=\"0\""
          << " href=\"data:image/bmp;base64,"<<b64<<"\"/>\n"
          << "  <circle cx=\""<<cx<<"\" cy=\""<<cy
          << "\" r=\""<<maxRadiusCells<<"\" class=\"rr\"/>\n"
          << "  <circle cx=\""<<cx<<"\" cy=\""<<cy
          << "\" r=\""<<maxRadiusCells/2<<"\" class=\"rr\"/>\n"
          << "  <circle cx=\""<<cx<<"\" cy=\""<<cy
          << "\" r=\"10\" class=\"rm\"/>\n"
          << "</svg>\n";

        t1 = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] SVG write                    : "
                  << std::chrono::duration<double,std::milli>(t1-t0).count()
                  << " ms\n";

        auto tEnd = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Total SVG generation         : "
                  << std::chrono::duration<double,std::milli>(tEnd-tAll).count()
                  << " ms\n";
    }

    const std::vector<uint8_t>& getMask() const { return viewshedMask; }
};

/******************************************************************************
 * MAIN
 ******************************************************************************/

int main()
{
    auto t0 = std::chrono::high_resolution_clock::now();

    Global g;

    // Srinagar airport
    auto engine = std::make_unique<ViewshedMetricEngine>(
        g,
        33.9871, 74.7742,  // lon
        300.0f,       // range km
        30.0f,        // antenna height m
        true          // earth curvature
    );

    engine->computeViewshed();
    engine->writeViewshedToSVG("viewshed_timed.svg");

    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "\n=======================================================\n"
              << ">> TOTAL: "
              << std::chrono::duration<double,std::milli>(t1-t0).count()/1000.0
              << " s\n=======================================================\n";
}
