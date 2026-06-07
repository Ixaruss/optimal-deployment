#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include "../common.h"

// ============================================================================
// UPDATED HIGH-PERFORMANCE LINE OF SIGHT METHOD
// ============================================================================
std::vector<LOSResult> lineOfVisibilityOptimized(double lat0, double long0, int h0, double lat1, double long1, int h1)
{
    Global g;
    int x0, y0;
    int x1, y1;

    if (!g.bin.getGridCoords(lat0, long0, x0, y0) || !g.bin.getGridCoords(lat1, long1, x1, y1)) {
        std::cout << "no" << std::endl;
        return {};
    }

    auto s = std::chrono::steady_clock::now();

    // Absolute source elevation
    float srcTerrain = g.bin.getElevation(lat0, long0);
    if (srcTerrain < -9000.0f) return {};

    double srcElev = srcTerrain + h0;

    // 1. Get cells along the ray via Bresenham
    auto cells = Global::bresenham(x0, y0, x1, y1);
    size_t N = cells.size();
    if (N == 0) return {};

    // 2. ALLOCATE FLAT ALIGNED ARRAYS (Emulating Rust's continuous buffers)
    // Allocating contiguous vectors allows the CPU to stream memory sequentially
    std::vector<double> distances(N);
    std::vector<double> angles(N);
    std::vector<double> prefix_max(N);
    std::vector<float> elevations(N);

    std::vector<LOSResult> results;
    results.reserve(N); // Crucial: Prevents internal vector re-allocations

    // 3. STEP 1: PRE-CALCULATE DISTANCES & EXTRACT ELEVATIONS
    // This loop isolates memory lookups so the CPU can pipeline them smoothly
    for (size_t i = 0; i < N; ++i) {
        int cx = cells[i].first;
        int cy = cells[i].second;

        if (cx < 0 || cy < 0 || (size_t)cx >= g.conf.grid.cols || (size_t)cy >= g.conf.grid.rows) {
            N = i; // Clamp array size if we stepped out of bounds
            break;
        }

        float terrainElev = g.bin.elev_matrix[(size_t)cx * g.conf.grid.cols + cy];
        if (terrainElev < -9000.0f) {
            N = i;
            break;
        }

        elevations[i] = terrainElev;

        // Mathematical Translation: Instead of doing slow std::sqrt and std::atan2,
        // we can model the slope ratio directly if we want pure speed.
        // To maintain your exact angle format, we calculate distances cleanly:
        double ddx = (cx - x0) * RESOLUTION;
        double ddy = (cy - y0) * RESOLUTION;
        distances[i] = std::sqrt(ddx * ddx + ddy * ddy);
    }

    // If out-of-bounds triggered an immediate break
    if (N == 0) return {};

    // 4. STEP 2: VECTORIZABLE ANGLE CALCULATIONS (Rust's Angle Trait)
    // No branches inside this loop. The compiler can easily unroll this loop
    // or convert it to auto-SIMD hardware steps.
    angles[0] = 0.0;
    for (size_t i = 1; i < N; ++i) {
        // angle = atan2(delta_height, distance)
        angles[i] = std::atan2(elevations[i] - srcElev, distances[i]);
    }

    // 5. STEP 3: PREFIX MAXIMUM HORIZON PROCESSING (Rust's PrefixMax Trait)
    // Eliminates conditional branch logic blocks using std::max, preventing
    // pipeline flushes caused by branch mispredictions.
    double current_max = -1e18;
    prefix_max[0] = current_max;

    for (size_t i = 1; i < N; ++i) {
        prefix_max[i] = current_max;
        current_max = std::max(current_max, angles[i]); // Compiled to a native hardware MAX instruction
    }

    // 6. STEP 4: ACCUMULATING MATERIALIZED RESULTS (Rust's Accumulate Trait)
    for (size_t i = 0; i < N; ++i) {
        LOSResult r;
        r.x = cells[i].first;
        r.y = cells[i].second;
        r.dist = distances[i];
        r.angle = angles[i];
        r.maxAngle = prefix_max[i];
        r.elev = elevations[i];

        // Point is visible if its native viewing angle matches or beats the running horizon
        r.visible = (i == 0) ? true : (angles[i] >= prefix_max[i]);

        results.push_back(r);
    }

    auto e = std::chrono::steady_clock::now();
    auto total = std::chrono::duration_cast<std::chrono::microseconds>(e - s).count();
    std::cout << "done. Processing time: " << total << " us" << std::endl;

    return results;
}
