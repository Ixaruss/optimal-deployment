#include <cstddef>
#include <vector>
#include <cmath>
#include <chrono>
#include "../common.h"
#include "cpng.h"
using namespace std;

/******************************************************************************
 * BRESENHAM LINE
 *
 * Returns grid cells along the line from (x0,y0) to (x1,y1).
 *
 * Classic integer line algorithm.
 * No floating point, no gaps, no duplicates.
 *
 ******************************************************************************/

std::vector<std::pair<int,int>>
Global::bresenham(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int,int>> cells;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        cells.push_back({x0, y0});

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;

        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }

    return cells;
}

/******************************************************************************
 * LINE OF SIGHT
 *
 * Traces LOS from source to target using max-angle horizon tracking.
 *
 * Parameters:
 *      grid          -> terrain grid
 *      srcX, srcY    -> source cell coordinates
 *      srcHeight     -> observer height above terrain in meters
 *      dstX, dstY    -> target cell coordinates
 *
 * Returns:
 *      Vector of LOSResult, one per cell along the ray.
 *      Includes source and target.
 *
 *      Check result.back().visible to know if target is visible.
 *
 ******************************************************************************/

std::vector<LOSResult> Global::lineOfVisibility(double lat0, double long0,int h0, double lat1, double long1,int h1)
{
    Global g;
    int x0, y0;
    int x1, y1;
     auto start = chrono::steady_clock::now();
    if(!g.bin.getGridCoords(lat0, long0, x0, y0) || !g.bin.getGridCoords(lat1, long1, x1, y1)){
        cout << "no" << endl;
        return {};
    }

    std::cout << "src grid: " << x0 << ", " << y0 << "\n";
    std::cout << "dst grid: " << x1 << ", " << y1 << "\n";
    std::cout << "grid size: " << g.conf.grid.cols << " x " << g.conf.grid.rows << "\n";

    std::vector<LOSResult> results;

    // Absolute source elevation
    float srcTerrain = g.bin.getElevation(lat0, long0);

    if (srcTerrain < -9000.0f)
        return results;  // source outside grid

    double srcElev = srcTerrain + h0;

    // Get cells along the ray
    auto cells = Global::bresenham(x0, y0, x1, y1);

    double maxAngle = -1e18;

    for (size_t i = 0; i < cells.size(); i++)
    {
        int cx = cells[i].first;
        int cy = cells[i].second;
        if (cx < 0 || cy < 0 || cx >= g.conf.grid.cols || cy >= g.conf.grid.rows)
            break;

        float terrainElev = g.bin.elev_matrix[(size_t)cx * g.conf.grid.cols + cy];

        if (terrainElev < -9000.0f)
            break;  // stepped outside grid

        // Ground distance in meters
        double ddx  = (cx - x0) * RESOLUTION;
        double ddy  = (cy - y0) * RESOLUTION;
        double dist = std::sqrt(ddx*ddx + ddy*ddy);

        LOSResult r;
        r.x   = cx;
        r.y   = cy;
        r.dist = dist;

        // Source cell itself
        if (i == 0)
        {
            r.angle    = 0.0;
            r.maxAngle = -1e18;
            r.visible  = true;
            results.push_back(r);
            continue;
        }

        // Elevation angle from source to top of this cell
        r.angle = std::atan2(
            terrainElev - srcElev,
            dist);

        // Visible if angle clears the horizon
        r.visible  = (r.angle >= maxAngle);
        r.maxAngle = maxAngle;
        r.elev     = terrainElev;
        // Update running horizon
        if (r.angle > maxAngle)
            maxAngle = r.angle;

        results.push_back(r);
    }

    return results;
}

/******************************************************************************
 * EXAMPLE USAGE
 ******************************************************************************/

#include <iostream>

int main()
{
    Global g;
    auto results = Global::lineOfVisibility(34.017346,74.50587999,50,34.16910062,74.71267172,500);

    png::generateLOSPng(results, "los_chart.png");

    return 0;
}


// int main () {
//     cout<< Global::lineOfSight(26,77,29,33,80,2500) <<endl;
//     //cout<< Global::lineOfSight(34.017346,74.50587999,34.16910062,74.71267172) <<endl; // return true;

//     return 0;
// }
