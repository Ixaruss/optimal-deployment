#define _USE_MATH_DEFINES // Ensures M_PI is loaded on all compilers
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include "../common.h"
constexpr int16_t INVALID_ELEV = -32768;



struct BinData {
    std::vector<int16_t> elev_matrix;
};


struct LocalViewshedResult {
    std::vector<uint8_t> visibility_grid; // Sized exactly to the (2*max_los + 1)^2 footprint
    int width;
};

class FastLocalRotator {
private:
    double alpha;
    double beta;
    double center;
    int width;

    double invert_angle(double angle) {
        return (angle > 0.0) ? (360.0 - angle) : 0.0;
    }

public:
    FastLocalRotator(double angle_degrees, int local_width) : width(local_width) {
        center = (local_width - 1) / 2.0;
        double angle_inverted = invert_angle(std::fmod(angle_degrees, 360.0));
        double theta = angle_inverted * M_PI / 180.0;
        alpha = -std::tan(theta / 2.0);
        beta = std::sin(theta);
    }

    inline std::pair<int, int> rotate(int x, int y) const {
        double x_rel = x - center;
        double y_rel = y - center;
        x_rel = std::round(x_rel + alpha * y_rel);
        y_rel = std::round(y_rel + beta * x_rel);
        x_rel = std::round(x_rel + alpha * y_rel);
        return {
            std::clamp(static_cast<int>(x_rel + center), 0, width - 1),
            std::clamp(static_cast<int>(y_rel + center), 0, width - 1)
        };
    }
};

struct ReusableBuffers {
    std::vector<double> distances;
    std::vector<double> slopes;
    std::vector<double> prefix_max;
    std::vector<float> elevations;
    std::vector<int> local_indices;

    void resize(size_t size) {
        distances.resize(size);
        slopes.resize(size);
        prefix_max.resize(size);
        elevations.resize(size);
        local_indices.resize(size);
    }
};

// ============================================================================
// THE LINE OF SIGHT CORE KERNEL
// ============================================================================
void processHorizontalCacheRay(
    int max_los,
    double resolution,
    float srcElev,
    const std::vector<int16_t>& local_dem_cache,
    const FastLocalRotator& rotator,
    ReusableBuffers& bufs,
    std::vector<uint8_t>& local_visibility,
    int max_dim)
{
    size_t N = max_los;
    bufs.resize(N);

    int local_center_x = max_dim / 2;
    int center_row_y = max_dim / 2;

    // --- STEP 1: Contiguous SAMPLING ---
    for (size_t i = 0; i < N; ++i) {
        int x_rotated_space = local_center_x + static_cast<int>(i);

        auto [rx, ry] = rotator.rotate(x_rotated_space, center_row_y);

        int cache_idx = ry * max_dim + rx;
        bufs.local_indices[i] = cache_idx;

        int16_t elev = local_dem_cache[cache_idx];
        if (elev <= INVALID_ELEV || elev < -9000) {
            bufs.elevations[i] = (i > 0) ? bufs.elevations[i-1] : 0.0f;
        } else {
            bufs.elevations[i] = static_cast<float>(elev);
        }

        bufs.distances[i] = i * resolution;
    }

    // --- STEP 2: SLOPE CALCULATION (Rise / Run) ---
    bufs.slopes[0] = 0.0;
    for (size_t i = 1; i < N; ++i) {
        bufs.slopes[i] = (bufs.elevations[i] - srcElev) / (bufs.distances[i] + 0.0001);
    }

    // --- STEP 3: HORIZON PROCESSING (PrefixMax) ---
    double current_max = -1e18;
    for (size_t i = 1; i < N; ++i) {
        bufs.prefix_max[i] = current_max;
        current_max = std::max(current_max, bufs.slopes[i]);
    }

    // --- STEP 4: WRITEBACK TO LOCAL BITMAP ---
    // FIXED: Writes out individual array positions explicitly using a standard loop element pass
    for (size_t i = 1; i < N; ++i) {
        if (bufs.slopes[i] >= bufs.prefix_max[i]) {
            local_visibility[bufs.local_indices[i]] = 1;
        }
    }
}

// ============================================================================
// THE CACHED MASTER VIEWSHED ENGINE
// ============================================================================
LocalViewshedResult computeTotalViewshed(
    int center_x, int center_y, int h0,
    int max_los_cells, int num_angle_subdivisions,
    const Global& g)
{
    auto start_time = std::chrono::steady_clock::now();

    size_t cols = static_cast<size_t>(g.conf.grid.cols);
    size_t rows = static_cast<size_t>(g.conf.grid.rows);
    double resolution = static_cast<double>(g.conf.grid.resolution);

    int max_dim = (2 * max_los_cells) + 1;
    size_t cache_box_size = static_cast<size_t>(max_dim) * max_dim;

    // Extract the "Chocolate Bar" cache slice
    std::vector<int16_t> local_dem_cache(cache_box_size, INVALID_ELEV);
    std::vector<uint8_t> local_visibility(cache_box_size, 0);

    int start_x = center_x - max_los_cells;
    int start_y = center_y - max_los_cells;

    for (int y = 0; y < max_dim; ++y) {
        int target_y = start_y + y;
        if (target_y < 0 || (size_t)target_y >= rows) continue;

        size_t giant_row_offset = static_cast<size_t>(target_y) * cols;
        int cache_row_offset = y * max_dim;

        for (int x = 0; x < max_dim; ++x) {
            int target_x = start_x + x;
            if (target_x < 0 || (size_t)target_x >= cols) continue;

            local_dem_cache[cache_row_offset + x] = g.bin.elev_matrix[giant_row_offset + target_x];
        }
    }

    int local_center_coord = max_dim / 2;
    int16_t srcTerrainRaw = local_dem_cache[local_center_coord * max_dim + local_center_coord];
    float srcElev = static_cast<float>(srcTerrainRaw) + h0;

    // Flag the center radar location itself as visible
    local_visibility[local_center_coord * max_dim + local_center_coord] = 1;

    ReusableBuffers scratch_buffers;
    scratch_buffers.resize(max_los_cells);

    double angle_step = 360.0 / num_angle_subdivisions;

    for (int a = 0; a < num_angle_subdivisions; ++a) {
        double current_angle = a * angle_step;
        FastLocalRotator rotator(current_angle, max_dim);

        processHorizontalCacheRay(
            max_los_cells, resolution, srcElev,
            local_dem_cache, rotator, scratch_buffers, local_visibility, max_dim
        );
    }

    auto end_time = std::chrono::steady_clock::now();
    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << ">>> Total Cached 300km Viewshed Grid Computed in: " << total_ms << " ms <<<\n";

    return LocalViewshedResult{ local_visibility, max_dim };
}

// ============================================================================
// VALIDATION TEST DRIVER
// ============================================================================
int main() {
    Global g;

    std::cout << "Starting optimized engine validation step...\n";
    // Compute a full 300 km range radar footprint using 64 angles
    LocalViewshedResult output = computeTotalViewshed(3000, 3000, 15, 3000, 64, g);

    std::cout << "Calculated Footprint array dimension: " << output.width << " x " << output.width << " elements.\n";
    return 0;
}
