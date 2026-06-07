// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <chrono>
// #include <memory>
// #include <cstdint>

// // Configuration Constants
// constexpr double RESOLUTION = 100.0; // 100 meters per cell
// constexpr uint16_t INVALID_ELEV = 0xFFFF; // Standard uint16 data void sentinel

// struct ViewshedResult {
//     std::vector<bool> visibility_grid; // Flat 1D grid representation
//     size_t width;
// };

// // Simulated Global/Bin structure to hold your 1D uint16 elevation matrix matching your exact setup
// struct BinData {
//     std::vector<uint16_t> elev_matrix;
// };
// struct Global {
//     BinData bin;
// };

// // ============================================================================
// // 1. THE 3-SHEAR (PAETH) ROTATOR
// // Ensures a perfect 1:1 (bijective) pixel mapping without holes or duplication.
// // ============================================================================
// class PaethRotator {
// private:
//     double alpha;
//     double beta;
//     double center_x;
//     double center_y;
//     int width;

//     double invert_angle(double angle) {
//         return (angle > 0.0) ? (360.0 - angle) : 0.0;
//     }

// public:
//     PaethRotator(double angle_degrees, int grid_width) : width(grid_width) {
//         double middle = (grid_width - 1) / 2.0;
//         center_x = middle;
//         center_y = middle;

//         double angle_inverted = invert_angle(std::fmod(angle_degrees, 360.0));
//         double theta = angle_inverted * M_PI / 180.0;
//         alpha = -std::tan(theta / 2.0);
//         beta = std::sin(theta);
//     }

//     std::pair<int, int> rotate(int x, int y) const {
//         double x_rel = x - center_x;
//         double y_rel = y - center_y;

//         x_rel = std::round(x_rel + alpha * y_rel);
//         y_rel = std::round(y_rel + beta * x_rel);
//         x_rel = std::round(x_rel + alpha * y_rel);

//         int nx = std::clamp(static_cast<int>(x_rel + center_x), 0, width - 1);
//         int ny = std::clamp(static_cast<int>(y_rel + center_y), 0, width - 1);
//         return {nx, ny};
//     }
// };

// // ============================================================================
// // 2. SCRATCHPAD BUFFERS (Hoisted allocations to avoid OS Heap overhead)
// // ============================================================================
// struct VectorBuffers {
//     std::vector<double> distances;
//     std::vector<double> angles;
//     std::vector<double> prefix_max;
//     std::vector<float> elevations;
//     std::vector<size_t> rotated_indices;

//     void resize(size_t size) {
//         distances.resize(size);
//         angles.resize(size);
//         prefix_max.resize(size);
//         elevations.resize(size);
//         rotated_indices.resize(size);
//     }
// };

// // ============================================================================
// // 3. THE LINE OF SIGHT KERNEL (Rust's Trait Logic Pipeline)
// // ============================================================================
// void processSingleHorizontalRow(
//     int center_offset,
//     int max_los,
//     int width,
//     float srcElev,
//     const std::vector<uint16_t>& source_elev_matrix,
//     const PaethRotator& rotator,
//     VectorBuffers& bufs,
//     std::vector<uint8_t>& global_visibility,
//     int current_y)
// {
//     size_t N = max_los;
//     bufs.resize(N);

//     int center_x = width / 2;

//     // --- STEP 1: LINEAR SAMPLING ---
//     for (size_t i = 0; i < N; ++i) {
//         int x_rotated_space = center_x + static_cast<int>(i);

//         auto [orig_x, orig_y] = rotator.rotate(x_rotated_space, current_y);

//         // Critical Fix: Used explicit 64-bit size_t multiplication to prevent coordinate overflow on a 300,000 grid
//         size_t orig_idx = static_cast<size_t>(orig_y) * static_cast<size_t>(width) + static_cast<size_t>(orig_x);
//         bufs.rotated_indices[i] = orig_idx;

//         uint16_t elev = source_elev_matrix[orig_idx];

//         // Forward-fill missing data errors (Rust's fill_line_elevations)
//         if (elev == INVALID_ELEV) {
//             bufs.elevations[i] = (i > 0) ? bufs.elevations[i-1] : 0.0f;
//         } else {
//             bufs.elevations[i] = static_cast<float>(elev);
//         }

//         bufs.distances[i] = i * RESOLUTION;
//     }

//     // --- STEP 2: ANGLE CALCULATION (Rust's Angle Trait) ---
//     bufs.angles = 0.0;
//     for (size_t i = 1; i < N; ++i) {
//         bufs.angles[i] = std::atan2(bufs.elevations[i] - srcElev, bufs.distances[i]);
//     }

//     // --- STEP 3: HORIZON PROCESSING (Rust's PrefixMax Trait) ---
//     double current_max = -1e18;
//     for (size_t i = 1; i < N; ++i) {
//         bufs.prefix_max[i] = current_max;
//         current_max = std::max(current_max, bufs.angles[i]);
//     }

//     // --- STEP 4: ACCUMULATE & WRITEBACK ---
//     global_visibility[bufs.rotated_indices] = 1;
//     for (size_t i = 1; i < N; ++i) {
//         if (bufs.angles[i] >= bufs.prefix_max[i]) {
//             global_visibility[bufs.rotated_indices[i]] = 1;
//         }
//     }
// }

// // ============================================================================
// // 4. TOTAL VIEWSHED MASTER ENGINE
// // ============================================================================
// ViewshedResult computeTotalViewshed(
//     int center_x, int center_y, int h0,
//     int max_los_cells, int num_angle_subdivisions,
//     const std::vector<uint16_t>& elev_matrix, int width)
// {
//     auto start_time = std::chrono::steady_clock::now();

//     // Limit tracking size to the bounding context of the local observer footprint area (max_los x max_los)
//     // to prevent allocating a massive 300,000 x 300,000 boolean vector which would crash RAM.
//     size_t total_cells = static_cast<size_t>(width) * static_cast<size_t>(width);

//     // For large matrices, we dynamically allocate a smaller localized buffer mapping around the target point,
//     // but to preserve exact logic compliance with the dataset structure, we track visibility directly:
//     std::vector<uint8_t> visibility_bytes(total_cells, 0);

//     size_t observer_idx = static_cast<size_t>(center_y) * static_cast<size_t>(width) + static_cast<size_t>(center_x);
//     uint16_t srcTerrainRaw = elev_matrix[observer_idx];
//     float srcElev = static_cast<float>(srcTerrainRaw) + h0;

//     VectorBuffers thread_buffers;
//     thread_buffers.resize(max_los_cells);

//     double angle_step = 360.0 / num_angle_subdivisions;

//     for (int a = 0; a < num_angle_subdivisions; ++a) {
//         double current_angle = a * angle_step;
//         PaethRotator rotator(current_angle, width);

//         // Process horizontal ray lanes parallel to the rotation alignment around the active chocolate-bar zone
//         for (int y_row = center_y - 10; y_row <= center_y + 10; ++y_row) {
//             if (y_row >= 0 && y_row < width) {
//                 processSingleHorizontalRow(
//                     center_x, max_los_cells, width, srcElev,
//                     elev_matrix, rotator, thread_buffers, visibility_bytes, y_row
//                 );
//             }
//         }
//     }

//     std::vector<bool> final_visibility(total_cells, false);
//     for(size_t i = 0; i < total_cells; ++i) {
//         if (visibility_bytes[i] == 1) final_visibility[i] = true;
//     }

//     auto end_time = std::chrono::steady_clock::now();
//     auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
//     std::cout << ">>> Total Viewshed Grid Computed in: " << total_ms << " ms <<<\n";

//     return ViewshedResult{ final_visibility, static_cast<size_t>(width) };
// }

// // ============================================================================
// // MAIN ROUTINE MATCHING YOUR PRECISE DESIGN SIGNATURE
// // ============================================================================
// int main() {
//     int grid_width = 30000;
//     Global g;

//     // Allocate mockup structure matching your data parameters safely.
//     // NOTE: A full 300,000 x 300,000 array takes 180 Gigabytes of RAM.
//     // For your real system, resize this vector or map your file streaming pointer directly.
//     g.bin.elev_matrix.assign(100000000ULL, static_cast<uint16_t>(100));

//     // Inject observer center elevations
//     size_t observer_idx = 1000ULL * static_cast<size_t>(grid_width) + 1000ULL;
//     if(observer_idx < g.bin.elev_matrix.size()) {
//         g.bin.elev_matrix[observer_idx] = static_cast<uint16_t>(120);
//     }

//     // Compute viewshed: 60 cells radius, 64 angles using Rust's vectorized model pattern
//     ViewshedResult output = computeTotalViewshed(
//         1000, 1000, 15, 60, 64, g.bin.elev_matrix, grid_width
//     );

//     return 0;
// }
