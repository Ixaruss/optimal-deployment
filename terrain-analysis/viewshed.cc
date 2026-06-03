#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>
#include <iomanip>

constexpr double PI = 3.14159265358979323846;

struct Grid
{
    int    width;
    int    height;
    double cellSize;
    std::vector<float> elev;

    inline float get(int x, int y) const
    {
        if (x < 0 || y < 0 || x >= width || y >= height)
            return -9999.0f;
        return elev[static_cast<size_t>(y) * width + x];
    }
};

// Classic Bresenham line algorithm to find cell indices along a ray path
static std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int, int>> cells;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        cells.push_back({x0, y0});
        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
    return cells;
}

/******************************************************************************
 * COMPUTE VIEWSHED
 * Calculates visibility for every cell by casting rays to the perimeter.
 * Updates a 2D boolean mask matrix representing structural visibility flags.
 ******************************************************************************/
std::vector<bool> computeViewshed(const Grid& grid, int srcX, int srcY, double srcHeight)
{
    // Initialize visibility output array (false = blocked/unseen, true = visible)
    std::vector<bool> visibleMap(static_cast<size_t>(grid.width) * grid.height, false);

    float srcTerrain = grid.get(srcX, srcY);
    if (srcTerrain < -9000.0f) return visibleMap; // Source is outside grid boundaries

    double srcElev = srcTerrain + srcHeight;
    visibleMap[static_cast<size_t>(srcY) * grid.width + srcX] = true; // Observer cell is always visible

    // Collect all boundary edge cells along the structural perimeter
    std::vector<std::pair<int, int>> perimeterCells;

    // Top and bottom rows
    for (int x = 0; x < grid.width; ++x) {
        perimeterCells.push_back({x, 0});
        if (grid.height > 1) perimeterCells.push_back({x, grid.height - 1});
    }
    // Left and right columns (skipping corners already handled)
    for (int y = 1; y < grid.height - 1; ++y) {
        perimeterCells.push_back({0, y});
        if (grid.width > 1) perimeterCells.push_back({grid.width - 1, y});
    }

    // Cast an individual ray from the center out to every perimeter cell target
    for (const auto& target : perimeterCells)
    {
        auto ray = bresenham(srcX, srcY, target.first, target.second);
        double maxAngle = -std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < ray.size(); ++i)
        {
            int cx = ray[i].first;
            int cy = ray[i].second;
            size_t linearIndex = static_cast<size_t>(cy) * grid.width + cx;

            float terrainElev = grid.get(cx, cy);
            if (terrainElev < -9000.0f) break; // Stepped off grid boundary edge

            if (i == 0) continue; // Skip source cell handling inside the loop

            double ddx = (cx - srcX) * grid.cellSize;
            double ddy = (cy - srcY) * grid.cellSize;
            double dist = std::sqrt(ddx * ddx + ddy * ddy);

            double angle = std::atan2(terrainElev - srcElev, dist);

            // Corrected strict horizon validation logic flag
            if (angle > maxAngle)
            {
                visibleMap[linearIndex] = true;
                maxAngle = angle;
            }
        }
    }

    return visibleMap;
}

int main()
{
    // Generate a 15x15 synthetic mountain hill terrain grid configuration
    Grid grid;
    grid.width = 15;
    grid.height = 15;
    grid.cellSize = 10.0;
    grid.elev.resize(static_cast<size_t>(grid.width) * grid.height);

    // Create a smooth geometric dome-shaped terrain obstacle hill layout
    int centerX = 7, centerY = 7;
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            double dx = x - centerX;
            double dy = y - centerY;
            double dist = std::sqrt(dx * dx + dy * dy);
            // Height drops off progressively moving away from center peak ridge point
            grid.elev[static_cast<size_t>(y) * grid.width + x] = static_cast<float>(std::max(0.0, 50.0 - dist * 6.0));
        }
    }

    // Introduce an artificial local ridge ring barrier wall setting to block visibility lines
    grid.elev[4 * grid.width + 7] = 65.0f;
    grid.elev[5 * grid.width + 7] = 65.0f;

    // Run viewshed processing calculation framework profile step
    int observerX = 7, observerY = 7;
    double observerOffsetHeight = 2.0; // Observer stands 2 meters over the terrain layer point

    std::vector<bool> viewshed = computeViewshed(grid, observerX, observerY, observerOffsetHeight);

    // Output calculated matrix data stream results straight into another file
    std::ofstream outFile("viewshed_output.txt");
    if (!outFile) {
        std::cerr << "CRITICAL ERROR: Failed to create/open output data stream target file.\n";
        return 1;
    }

    outFile << "=== 360-DEGREE VIEWSHED ANALYSIS OUTPUT MAP ===\n";
    outFile << "Grid Dimensions: " << grid.width << "x" << grid.height << " (Cell Spacing: " << grid.cellSize << "m)\n";
    outFile << "Observer Location: (" << observerX << ", " << observerY << ") at Elevation: "
            << grid.get(observerX, observerY) + observerOffsetHeight << "m MSL\n\n";

    outFile << "LEGEND KEY MAP VIEW:\n";
    outFile << "  [X] -> Observer Spot Position\n";
    outFile << "  [V] -> Visible Location Node\n";
    outFile << "  [.] -> Hidden / Obscured Shaded Dead Zone Area\n\n";

    // Build ASCII map visualization graphic matrix layout representation
    for (int y = 0; y < grid.height; ++y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            if (x == observerX && y == observerY) {
                outFile << " X ";
            } else {
                size_t idx = static_cast<size_t>(y) * grid.width + x;
                outFile << (viewshed[idx] ? " V " : " . ");
            }
        }
        outFile << "\n";
    }

    outFile.close();
    std::cout << "SUCCESS: Full directional viewshed data map written to 'viewshed_output.txt'\n";

    return 0;
}
