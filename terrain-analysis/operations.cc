#include <vector>
#include <cmath>

/******************************************************************************
 * GRID
 *
 * Simple 2D height grid.
 *
 * width    -> number of columns
 * height   -> number of rows
 * cellSize -> spacing between cells in meters
 * elev     -> elevation values
 *
 ******************************************************************************/

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

        return elev[y * width + x];
    }
};

/******************************************************************************
 * LOS RESULT
 *
 * Stores result for a single cell along the ray.
 *
 * x, y       -> grid coordinates of this cell
 * dist       -> ground distance from source in meters
 * angle      -> elevation angle from source to this cell in radians
 * maxAngle   -> running maximum angle at this point
 * visible    -> true if this cell is visible from source
 *
 ******************************************************************************/

struct LOSResult
{
    int    x;
    int    y;
    double dist;
    double angle;
    double maxAngle;
    bool   visible;
};

/******************************************************************************
 * BRESENHAM LINE
 *
 * Returns grid cells along the line from (x0,y0) to (x1,y1).
 *
 * Classic integer line algorithm.
 * No floating point, no gaps, no duplicates.
 *
 ******************************************************************************/

static std::vector<std::pair<int,int>>
bresenham(int x0, int y0, int x1, int y1)
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

std::vector<LOSResult> lineOfSight(
    const Grid& grid,
    int    srcX,    int srcY,
    double srcHeight,
    int    dstX,    int dstY)
{
    std::vector<LOSResult> results;

    // Absolute source elevation
    float srcTerrain = grid.get(srcX, srcY);

    if (srcTerrain < -9000.0f)
        return results;  // source outside grid

    double srcElev = srcTerrain + srcHeight;

    // Get cells along the ray
    auto cells = bresenham(srcX, srcY, dstX, dstY);

    double maxAngle = -1e18;

    for (size_t i = 0; i < cells.size(); i++)
    {
        int cx = cells[i].first;
        int cy = cells[i].second;

        float terrainElev = grid.get(cx, cy);

        if (terrainElev < -9000.0f)
            break;  // stepped outside grid

        // Ground distance in meters
        double ddx  = (cx - srcX) * grid.cellSize;
        double ddy  = (cy - srcY) * grid.cellSize;
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
    // Build a small test grid
    Grid grid;
    grid.width    = 12;
    grid.height   = 1;
    grid.cellSize = 10.0;  // 10 meters per cell
    grid.elev     = { 3,1,1,4,1,1,2,1,5,1,1,2 };

    // Trace LOS from cell 0 to cell 11
    // Observer standing on the terrain (no extra height)
    auto results = lineOfSight(grid, 0, 0, 0.0, 11, 0);

    std::cout << "Cell  Height  Dist    Angle     MaxAngle  Visible\n";
    std::cout << "----  ------  ------  --------  --------  -------\n";

    for (auto& r : results)
    {
        std::cout
            << r.x    << "     "
            << grid.get(r.x, r.y) << "       "
            << (int)r.dist << "m    "
            << (r.angle    * 180.0 / M_PI) << "°  "
            << (r.maxAngle * 180.0 / M_PI) << "°  "
            << (r.visible ? "YES" : "NO")
            << "\n";
    }

    return 0;
}
