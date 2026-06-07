/**
 * @file ViewshedTimedMetrics.cpp
 * @brief Zero-Allocation Viewshed Engine with Granular Runtime Micro-Timing.
 * Ported to use Global struct, bin, and conf.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include "../common.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

struct Point2D { int x, y; };

class ViewshedMetricEngine
{
private:
    Global&  g;               // reference to loaded Global instance

    int      gridCols;        // g.conf.grid.cols
    int      gridRows;        // g.conf.grid.rows
    int      centerX;
    int      centerY;
    int      maxRadiusCells;
    float    metersPerCell;
    float    radarHeightMeters;
    float    targetHeightMeters;

    std::vector<uint8_t>  viewshedMask;
    std::vector<Point2D>  circlePerimeter;

    // ------------------------------------------------------------------
    std::string base64Encode(const uint8_t* data, size_t length)
    {
        static const char lookup[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        out.reserve(((length + 2) / 3) * 4);
        size_t i = 0;
        for (; i + 2 < length; i += 3) {
            uint32_t val = (data[i] << 16) | (data[i+1] << 8) | data[i+2];
            out.push_back(lookup[(val >> 18) & 0x3F]);
            out.push_back(lookup[(val >> 12) & 0x3F]);
            out.push_back(lookup[(val >>  6) & 0x3F]);
            out.push_back(lookup[ val        & 0x3F]);
        }
        if (i < length) {
            uint32_t val = data[i] << 16;
            if (i + 1 < length) val |= data[i+1] << 8;
            out.push_back(lookup[(val >> 18) & 0x3F]);
            out.push_back(lookup[(val >> 12) & 0x3F]);
            if (i + 1 < length) out.push_back(lookup[(val >> 6) & 0x3F]);
            else                 out.push_back('=');
            out.push_back('=');
        }
        return out;
    }

    // ------------------------------------------------------------------
    inline void castRayMetricMatrix(
        int xEnd, int yEnd,
        float radarElevation,
        const short* __restrict demPtr,   // elev_matrix is short
        uint8_t*     __restrict maskPtr)
    {
        int x = centerX, y = centerY;
        int dx = std::abs(xEnd - centerX);
        int dy = std::abs(yEnd - centerY);
        int sx = (centerX < xEnd) ? 1 : -1;
        int sy = (centerY < yEnd) ? 1 : -1;
        int err = dx - dy;

        // strides in the flat [row-major: y*COLS+x] layout
        std::ptrdiff_t strideX = sx;
        std::ptrdiff_t strideY = static_cast<std::ptrdiff_t>(sy) * gridCols;
        std::ptrdiff_t currentOffset =
            static_cast<std::ptrdiff_t>(y) * gridCols + x;

        const short* currentDemCell  = demPtr  + currentOffset;
        uint8_t*     currentMaskCell = maskPtr + currentOffset;

        float maxRadiusSq =
            static_cast<float>(maxRadiusCells) * maxRadiusCells;

        size_t endCellIdx =
            static_cast<size_t>(yEnd) * gridCols + xEnd;

        float targetDeltaH =
            (demPtr[endCellIdx] + targetHeightMeters) - radarElevation;
        float targetDistSq =
            static_cast<float>(dx * dx + dy * dy);
        float targetDeltaHSq = targetDeltaH * targetDeltaH;

        bool rayIsBlocked = false;

        while (true)
        {
            int   deltaX = x - centerX;
            int   deltaY = y - centerY;
            float distSq = static_cast<float>(deltaX*deltaX + deltaY*deltaY);
            if (distSq > maxRadiusSq) break;

            if (x != centerX || y != centerY)
            {
                float intermediateDeltaH =
                    static_cast<float>(*currentDemCell) - radarElevation;
                float intermediateDeltaHSq =
                    intermediateDeltaH * intermediateDeltaH;

                bool localBlock = false;
                if (intermediateDeltaH > 0 && targetDeltaH <= 0) {
                    localBlock = true;
                } else if (intermediateDeltaH > 0 && targetDeltaH > 0) {
                    if ((intermediateDeltaHSq * targetDistSq) >
                        (targetDeltaHSq * distSq))
                        localBlock = true;
                } else if (intermediateDeltaH <= 0 && targetDeltaH <= 0) {
                    if ((intermediateDeltaHSq * targetDistSq) <
                        (targetDeltaHSq * distSq))
                        localBlock = true;
                }

                if (localBlock) rayIsBlocked = true;

                if (!rayIsBlocked)
                    if (*currentMaskCell == 0) *currentMaskCell = 1;
            }

            if (x == xEnd && y == yEnd) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy; x += sx;
                currentDemCell  += strideX;
                currentMaskCell += strideX;
            }
            if (e2 < dx) {
                err += dx; y += sy;
                currentDemCell  += strideY;
                currentMaskCell += strideY;
            }
        }
    }

    // ------------------------------------------------------------------
    void precomputeTrueCirclePerimeter()
    {
        int x = 0, y = maxRadiusCells, d = 3 - 2 * maxRadiusCells;

        auto push8 = [&](int cx, int cy, int px, int py) {
            circlePerimeter.push_back({cx+px, cy+py});
            circlePerimeter.push_back({cx-px, cy+py});
            circlePerimeter.push_back({cx+px, cy-py});
            circlePerimeter.push_back({cx-px, cy-py});
            circlePerimeter.push_back({cx+py, cy+px});
            circlePerimeter.push_back({cx-py, cy+px});
            circlePerimeter.push_back({cx+py, cy-px});
            circlePerimeter.push_back({cx-py, cy-px});
        };

        while (y >= x) {
            push8(centerX, centerY, x, y);
            x++;
            if (d > 0) { y--; d = d + 4*(x-y) + 10; }
            else        {      d = d + 4*x     +  6; }
        }
    }

public:
    // ------------------------------------------------------------------
    // Constructor
    //
    // Parameters:
    //   globalInstance   -> loaded Global (DEM already in bin.elev_matrix)
    //   srcLat, srcLon   -> radar position
    //   rangeKM          -> radar range in km
    //   txHeightMeters   -> radar antenna height above terrain
    //   rxHeightMeters   -> target height above terrain
    // ------------------------------------------------------------------
    ViewshedMetricEngine(
        Global& globalInstance,
        double  srcLat,
        double  srcLon,
        float   rangeKM,
        float   txHeightMeters,
        float   rxHeightMeters)
        : g(globalInstance)
        , radarHeightMeters(txHeightMeters)
        , targetHeightMeters(rxHeightMeters)
    {
        auto tStart = std::chrono::high_resolution_clock::now();

        gridCols      = g.conf.grid.cols;
        gridRows      = g.conf.grid.rows;
        metersPerCell = static_cast<float>(RESOLUTION);

        // convert lat/lon to grid coords
        if (!g.bin.getGridCoords(srcLat, srcLon, centerX, centerY)) {
            std::cerr << "[ERROR] Source coords outside grid\n";
            centerX = gridCols / 2;
            centerY = gridRows / 2;
        }

        float rangeMeters  = rangeKM * 1000.0f;
        maxRadiusCells     = static_cast<int>(
            std::round(rangeMeters / metersPerCell));

        // clamp to grid bounds from center
        int maxPossible = std::min(
            std::min(centerX, gridCols - centerX - 1),
            std::min(centerY, gridRows - centerY - 1));
        if (maxRadiusCells > maxPossible)
            maxRadiusCells = maxPossible;

        viewshedMask.assign(
            static_cast<size_t>(gridCols) * gridRows, 0);

        precomputeTrueCirclePerimeter();

        auto tEnd = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Initialization & Perimeter Precompute : "
                  << std::chrono::duration<double, std::milli>(tEnd-tStart).count()
                  << " ms\n";
        std::cout << "[INFO]   Center: (" << centerX << ", " << centerY << ")"
                  << "  Radius: " << maxRadiusCells << " cells"
                  << "  Perimeter: " << circlePerimeter.size() << " rays\n";
    }

    // ------------------------------------------------------------------
    void computeViewshed()
    {
        auto tStart = std::chrono::high_resolution_clock::now();

        size_t radarCenterIdx =
            static_cast<size_t>(centerY) * gridCols + centerX;

        float srcTerrain =
            static_cast<float>(g.bin.elev_matrix[radarCenterIdx]);

        if (srcTerrain < -9000.0f) {
            std::cerr << "[ERROR] Radar cell has no-data elevation\n";
            return;
        }

        float radarElevation = srcTerrain + radarHeightMeters;
        viewshedMask[radarCenterIdx] = 1;

        size_t       perimeterSize = circlePerimeter.size();
        const short* demPtr        = g.bin.elev_matrix.data();
        uint8_t*     maskPtr       = viewshedMask.data();

        #pragma omp parallel for schedule(guided, 32)
        for (size_t i = 0; i < perimeterSize; ++i)
        {
            int tx = circlePerimeter[i].x;
            int ty = circlePerimeter[i].y;

            if (tx >= 0 && tx < gridCols &&
                ty >= 0 && ty < gridRows)
            {
                castRayMetricMatrix(
                    tx, ty,
                    radarElevation,
                    demPtr, maskPtr);
            }
        }

        auto tEnd = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Core Viewshed Engine Calculations     : "
                  << std::chrono::duration<double, std::milli>(tEnd-tStart).count()
                  << " ms\n";
    }

    // ------------------------------------------------------------------
    void writeViewshedToSVG(const std::string& filename)
    {
        auto tStartOverall = std::chrono::high_resolution_clock::now();
        auto tStartPass1   = std::chrono::high_resolution_clock::now();

        // crop to circle bounding box to keep SVG manageable
        int x0 = std::max(0,         centerX - maxRadiusCells - 2);
        int y0 = std::max(0,         centerY - maxRadiusCells - 2);
        int x1 = std::min(gridCols,  centerX + maxRadiusCells + 2);
        int y1 = std::min(gridRows,  centerY + maxRadiusCells + 2);
        int W  = x1 - x0;
        int H  = y1 - y0;

        size_t totalPixels = static_cast<size_t>(W) * H;
        std::vector<uint8_t> rgbaBuffer(totalPixels * 4, 0);

        long long maxRadiusSq =
            static_cast<long long>(maxRadiusCells) * maxRadiusCells;

        // Pass 1: color mapping
        #pragma omp parallel for schedule(static, 4096)
        for (size_t i = 0; i < totalPixels; ++i)
        {
            int px  = i % W;
            int py  = i / W;
            int gx  = x0 + px;
            int gy  = y0 + py;
            int ddx = gx - centerX;
            int ddy = gy - centerY;
            long long distSq =
                static_cast<long long>(ddx)*ddx +
                static_cast<long long>(ddy)*ddy;

            size_t pixOff = i * 4;

            if (distSq <= maxRadiusSq)
            {
                size_t gridIdx =
                    static_cast<size_t>(gy) * gridCols + gx;

                if (viewshedMask[gridIdx] == 1) {
                    // visible: green
                    rgbaBuffer[pixOff+0] = 46;
                    rgbaBuffer[pixOff+1] = 204;
                    rgbaBuffer[pixOff+2] = 113;
                    rgbaBuffer[pixOff+3] = 255;
                } else {
                    // hidden: red
                    rgbaBuffer[pixOff+0] = 231;
                    rgbaBuffer[pixOff+1] = 76;
                    rgbaBuffer[pixOff+2] = 60;
                    rgbaBuffer[pixOff+3] = 255;
                }
            }
            // outside radius: transparent (already 0)
        }

        // Pass 2: RGBA -> BGRA swap for BMP
        #pragma omp parallel for schedule(static, 4096)
        for (size_t i = 0; i < totalPixels; ++i) {
            size_t idx = i * 4;
            std::swap(rgbaBuffer[idx+0], rgbaBuffer[idx+2]);
        }

        auto tEndPass1 = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] RGBA Mapping & BGRA Swap              : "
                  << std::chrono::duration<double, std::milli>(tEndPass1-tStartPass1).count()
                  << " ms\n";

        // Pass 3: BMP header + base64
        auto tStartB64 = std::chrono::high_resolution_clock::now();

        uint32_t fileSize = 54 + static_cast<uint32_t>(totalPixels * 4);
        uint8_t header[54] = {
            'B','M',
            static_cast<uint8_t>(fileSize),
            static_cast<uint8_t>(fileSize >> 8),
            static_cast<uint8_t>(fileSize >> 16),
            static_cast<uint8_t>(fileSize >> 24),
            0,0,0,0, 54,0,0,0, 40,0,0,0,
            static_cast<uint8_t>(W),
            static_cast<uint8_t>(W >> 8),
            static_cast<uint8_t>(W >> 16),
            static_cast<uint8_t>(W >> 24),
            static_cast<uint8_t>(-H),
            static_cast<uint8_t>(-H >> 8),
            static_cast<uint8_t>(-H >> 16),
            static_cast<uint8_t>(-H >> 24),
            1,0, 32,0, 0,0,0,0, 0,0,0,0,
            0,0,0,0, 0,0,0,0, 0,0,0,0
        };

        std::vector<uint8_t> payload;
        payload.reserve(54 + totalPixels * 4);
        payload.insert(payload.end(), header, header + 54);
        payload.insert(payload.end(), rgbaBuffer.begin(), rgbaBuffer.end());

        std::string b64 = base64Encode(payload.data(), payload.size());

        auto tEndB64 = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Base64 Encoding                       : "
                  << std::chrono::duration<double, std::milli>(tEndB64-tStartB64).count()
                  << " ms\n";

        // Pass 4: write SVG
        auto tStartIO = std::chrono::high_resolution_clock::now();

        std::ofstream svgFile(filename);
        svgFile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
        svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\""
                << " viewBox=\"0 0 " << W << " " << H << "\""
                << " width=\"100%\" height=\"100%\""
                << " style=\"background-color: #2c3e50;\">\n";
        svgFile << "  <style>\n";
        svgFile << "    .radar-marker { fill: #FFFFFF; stroke: #1abc9c; stroke-width: 12; }\n";
        svgFile << "    .radar-ring   { fill: none; stroke: #FFFFFF; stroke-width: 4;"
                << " stroke-dasharray: 20,15; opacity: 0.35; }\n";
        svgFile << "  </style>\n";
        svgFile << "  <image width=\"" << W << "\" height=\"" << H << "\""
                << " x=\"0\" y=\"0\""
                << " href=\"data:image/bmp;base64," << b64 << "\"/>\n";

        // circles in crop-relative coords
        int cx = centerX - x0;
        int cy = centerY - y0;
        svgFile << "  <circle cx=\"" << cx << "\" cy=\"" << cy
                << "\" r=\"" << maxRadiusCells
                << "\" class=\"radar-ring\"/>\n";
        svgFile << "  <circle cx=\"" << cx << "\" cy=\"" << cy
                << "\" r=\"" << maxRadiusCells / 2
                << "\" class=\"radar-ring\"/>\n";
        svgFile << "  <circle cx=\"" << cx << "\" cy=\"" << cy
                << "\" r=\"15\" class=\"radar-marker\"/>\n";
        svgFile << "</svg>\n";

        auto tEndIO = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] SVG File Write                        : "
                  << std::chrono::duration<double, std::milli>(tEndIO-tStartIO).count()
                  << " ms\n";

        auto tEndOverall = std::chrono::high_resolution_clock::now();
        std::cout << "[METRIC] Total SVG Generation                  : "
                  << std::chrono::duration<double, std::milli>(tEndOverall-tStartOverall).count()
                  << " ms\n";
    }

    // expose mask if needed by other systems
    const std::vector<uint8_t>& getMask() const { return viewshedMask; }
};

/******************************************************************************
 * MAIN
 ******************************************************************************/

int main()
{
    auto globalStart = std::chrono::high_resolution_clock::now();

    // load data once
    Global g;

    float radarRangeKM           = 300.0f;
    float radarTowerHeightMeters = 30.0f;
    float targetHeightMeters     = 2.0f;

    auto engine = std::make_unique<ViewshedMetricEngine>(
        g,
        34.017346, 74.50587999,  // src lat/lon
        radarRangeKM,
        radarTowerHeightMeters,
        targetHeightMeters
    );

    engine->computeViewshed();
    engine->writeViewshedToSVG("viewshed_timed.svg");

    auto globalEnd = std::chrono::high_resolution_clock::now();
    std::cout << "\n=======================================================\n";
    std::cout << ">> TOTAL PIPELINE: "
              << std::chrono::duration<double, std::milli>(globalEnd-globalStart).count() / 1000.0
              << " seconds\n";
    std::cout << "=======================================================\n";

    return 0;
}
