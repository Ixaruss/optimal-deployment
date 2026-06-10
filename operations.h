#ifndef OPERATIONS_H_
#define OPERATIONS_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <algorithm>
#include "common.h"

constexpr double EARTH_R     = 6371000.0;
constexpr double INV_2EARTHR = 1.0 / (2.0 * EARTH_R);
constexpr double K_ATMOS     = 4.0 / 3.0;
constexpr double K_EFF_R     = K_ATMOS * EARTH_R;
constexpr double INV_2_KEFFR = 1.0 / (2.0 * K_EFF_R);
constexpr double DEG2RAD     = 3.14159265358979 / 180.0;
constexpr double RAD2DEG     = 180.0 / 3.14159265358979;

enum class CoverageStatus : uint8_t
{
    OUTSIDE_RANGE  = 0,
    COVERED        = 1,
    TERRAIN_MASKED = 2,
    BELOW_MIN_ELEV = 3,
    ABOVE_MAX_ELEV = 4,
    BEYOND_HORIZON = 5,
    OUTSIDE_SECTOR = 6,  // NEW: cell not in the scanned sector
};

enum class LOSStatus : uint8_t
{
    VISIBLE        = 0,
    BEYOND_RANGE   = 1,
    BEYOND_HORIZON = 2,
    BELOW_MIN_ELEV = 3,
    ABOVE_MAX_ELEV = 4,
    TERRAIN_MASKED = 5,
    NO_DATA        = 6,   // DEM has no data along ray
    OUT_OF_GRID    = 7,
};

static const char* losStatusStr(LOSStatus s)
{
    switch(s){
        case LOSStatus::VISIBLE:        return "VISIBLE";
        case LOSStatus::BEYOND_RANGE:   return "BEYOND_RANGE";
        case LOSStatus::BEYOND_HORIZON: return "BEYOND_HORIZON";
        case LOSStatus::BELOW_MIN_ELEV: return "BELOW_MIN_ELEV";
        case LOSStatus::ABOVE_MAX_ELEV: return "ABOVE_MAX_ELEV";
        case LOSStatus::TERRAIN_MASKED: return "TERRAIN_MASKED";
        case LOSStatus::NO_DATA:        return "NO_DATA";
        case LOSStatus::OUT_OF_GRID:    return "OUT_OF_GRID";
    }
    return "UNKNOWN";
}

/******************************************************************************
 * FAST RESULT — single struct, no allocation
 ******************************************************************************/

/******************************************************************************
 * DETAILED CELL — per-cell breakdown for visualization
 ******************************************************************************/

struct LOSCell
{
    int    row, col;
    double distM;
    float  terrainElev;     // raw terrain elevation (m)
    float  correctedElev;   // terrain - curvature drop (m)
    double terrainSlope;    // (correctedElev - srcElev) / dist
    double maxSlopeSoFar;   // running max terrain slope before this cell
    double targetSlope;     // (target_abs_elev - srcElev) / targetDist (fixed)
    bool   blocks;          // terrainSlope > targetSlope at this cell
};



/******************************************************************************
 * INTERNAL: precompute all ray parameters that don't depend on DEM
 ******************************************************************************/

struct LOSParams
{
    // source
    int    srcRow, srcCol;
    double srcElev;          // absolute source elevation (m)

    // target
    int    tgtRow, tgtCol;
    double tgtAbsElev;       // target terrain + targetHeight (m)
    double tgtDist;          // ground distance to target (m)
    double targetSlope;      // (tgtAbsElev - srcElev) / tgtDist — FIXED for ray
    double elevAngleDeg;     // elevation angle to target

    // limits
    double maxRangeM;
    double horizonM;
    double minSlope;         // tan(minElevDeg)
    double maxSlope;         // tan(maxElevDeg)

    // grid
    int COLS, ROWS;
};
struct RadarLOSResult
{
    LOSStatus status;
    bool      visible;

    double distM;           // ground distance to target (m)
    double elevAngleDeg;    // elevation angle to target (degrees)
    double horizonM;        // radar+target LOS horizon distance (m)

    // if TERRAIN_MASKED: where the ray was first blocked
    int    blockRow;
    int    blockCol;
    double blockDistM;
    float  blockElevM;      // terrain elevation at block point

    // timings
    double computeUs;       // microseconds
};

struct RadarCoverageResult
{
    std::vector<CoverageStatus> status;
    int srcRow, srcCol;
    int radiusCells;
    int gridCols, gridRows;
    int sectorAzimuthDeg;     // -1 = full 360
    int countCovered, countTerrainMasked;
    int countBelowMinElev, countAboveMaxElev;
    int countBeyondHorizon, countOutsideRange;
    int countOutsideSector;
};

struct RadarLOSDetailedResult
{
    RadarLOSResult      summary;
    std::vector<LOSCell> cells;
};

struct Point2D { int x, y; };

/**
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
class ViewshedEngine
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

    void castRay(
        int xEnd, int yEnd,
        double srcElev,
        const short* __restrict demPtr,
        uint8_t*     __restrict maskPtr);

    void precomputeCirclePerimeter();

public:
    ViewshedEngine(
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

    public: void computeViewshed();
    public: inline const std::vector<uint8_t>& getMask() const { return viewshedMask; }
    public: void writeViewshedToSVG(const std::string& filename);
    public: static std::string base64Encode(const uint8_t* data, size_t length);
};

class RadarEngine
{
private:
    Global& g;
    int     gridCols, gridRows;
    int     srcRow,   srcCol;
    int     maxRadiusCells;
    int     rowMin,rowMax,colMin,colMax;

    double  antennaHeight, targetHeight;
    double  minSlopeRatio, maxSlopeRatio;
    double  maxRangeM, horizonM, srcElev;

    std::vector<uint8_t> statusBuf;
    std::vector<uint8_t> fastBuf;
    std::vector<Point2D> boundary;   // full 360 boundary

    // ------------------------------------------------------------------
    // Filter boundary cells for a sector
    // sectorAz: centre azimuth (already snapped to 45° multiple)
    // ------------------------------------------------------------------

    std::vector<Point2D> sectorBoundary(int sectorAz) const;

    void prefill(int sectorAz, uint8_t* buf, uint8_t fillInside, uint8_t fillOutside);

    /******************************************************************************
     * AZIMUTH HELPER
     *
     * North-based clockwise azimuth from source to (row,col).
     * North = 0°, East = 90°, South = 180°, West = 270°
     ******************************************************************************/

    static inline double cellAzimuth(int srcRow, int srcCol, int r, int c)
    {
        double drow = r - srcRow;
        double dcol = c - srcCol;
        // atan2(east, north) = clockwise from north
        double az = std::atan2(dcol, -drow) * RAD2DEG;
        if (az < 0.0) az += 360.0;
        return az;
    }

    /******************************************************************************
     * SECTOR HELPER
     *
     * Returns true if azimuth `az` is within `halfWidth` degrees of `centerAz`.
     * Handles 0/360 wraparound.
     ******************************************************************************/

    static inline bool inSectorAz(double az, double centerAz, double halfWidth)
    {
        double diff = std::fabs(az - centerAz);
        if (diff > 180.0) diff = 360.0 - diff;
        return diff <= halfWidth;
    }

    /******************************************************************************
     * SNAP TO NEAREST 45° MULTIPLE
     ******************************************************************************/

    static int snapTo45(int deg)
    {
        // normalise to [0,360)
        deg = ((deg % 360) + 360) % 360;
        int snapped = (int)(std::round(deg / 45.0) * 45) % 360;
        return snapped;
    }

public:
    RadarEngine(
        Global& globalInstance,
        double srcLat, double srcLon,
        double antennaHeightM,
        double targetHeightM,
        double minElevDeg, double maxElevDeg,
        double maxRangeKM)
        : g(globalInstance)
        , antennaHeight(antennaHeightM)
        , targetHeight(targetHeightM)
        , maxRangeM(maxRangeKM * 1000.0)
    {
        auto t0=std::chrono::high_resolution_clock::now();

        gridCols=g.conf.grid.cols;
        gridRows=g.conf.grid.rows;

        if(!g.bin.getGridCoords(srcLat,srcLon,srcRow,srcCol)){
            std::cerr<<"[ERROR] Source outside grid\n";
            srcRow=gridRows/2; srcCol=gridCols/2;
        }

        maxRadiusCells=(int)std::ceil(maxRangeM/RESOLUTION);
        int maxPossible=std::min(
            std::min(srcRow,gridRows-srcRow-1),
            std::min(srcCol,gridCols-srcCol-1));
        if(maxRadiusCells>maxPossible){
            std::cout<<"[WARN] Radius clamped to "<<maxPossible<<"\n";
            maxRadiusCells=maxPossible;
        }

        rowMin=std::max(0,        srcRow-maxRadiusCells-1);
        rowMax=std::min(gridRows, srcRow+maxRadiusCells+2);
        colMin=std::max(0,        srcCol-maxRadiusCells-1);
        colMax=std::min(gridCols, srcCol+maxRadiusCells+2);

        statusBuf.assign((size_t)gridRows*gridCols, 0);
        fastBuf.assign(  (size_t)gridRows*gridCols, 0);

        horizonM = std::sqrt(2.0*K_EFF_R*antennaHeight)
                 + std::sqrt(2.0*K_EFF_R*targetHeight);
        if(horizonM>maxRangeM) horizonM=maxRangeM;

        minSlopeRatio=std::tan(minElevDeg*DEG2RAD);
        maxSlopeRatio=std::tan(maxElevDeg*DEG2RAD);

        short srcTerrain=g.bin.elev_matrix[(size_t)srcRow*gridCols+srcCol];
        srcElev=static_cast<double>(srcTerrain)+antennaHeight;

        boundary=circleBoundary(srcRow,srcCol,maxRadiusCells);

        auto t1=std::chrono::high_resolution_clock::now();
        std::cout<<"[RADAR] Init          : "
                 <<std::chrono::duration<double,std::milli>(t1-t0).count()<<" ms\n";
        std::cout<<"[RADAR]   Src terrain : "<<srcTerrain<<"m  elev: "<<srcElev<<"m\n";
        std::cout<<"[RADAR]   Horizon     : "<<horizonM/1000.0<<" km\n";
        std::cout<<"[RADAR]   Full rays   : "<<boundary.size()<<"\n";
    }
    RadarCoverageResult compute(int sectorAzimuthDeg = -1);

    std::vector<uint8_t> computeFast(int sectorAzimuthDeg = -1);

    void writeToSVG(const RadarCoverageResult& res, const std::string& filename="radar_coverage.svg");

    static std::vector<Point2D> circleBoundary(int cr, int cc, int radius);

    public: static RadarLOSResult radarLOS(
        Global&  g,
        double   srcLat,  double srcLon,  double antennaHeightM,
        double   tgtLat,  double tgtLon,  double targetHeightM,
        double   minElevDeg, double maxElevDeg,
        double   maxRangeKM);

    public: static RadarLOSDetailedResult radarLOSDetailed(
        Global&  g,
        double   srcLat,  double srcLon,  double antennaHeightM,
        double   tgtLat,  double tgtLon,  double targetHeightM,
        double   minElevDeg, double maxElevDeg,
        double   maxRangeKM);

    inline static void printLOSResult(const RadarLOSResult& r,
                               double srcLat, double srcLon,
                               double tgtLat, double tgtLon)
    {
        std::cout << "======================================\n";
        std::cout << "RADAR LOS RESULT\n";
        std::cout << "  Src         : " << srcLat << ", " << srcLon << "\n";
        std::cout << "  Tgt         : " << tgtLat << ", " << tgtLon << "\n";
        std::cout << "  Distance    : " << r.distM/1000.0 << " km\n";
        std::cout << "  Elev angle  : " << r.elevAngleDeg << "°\n";
        std::cout << "  Horizon     : " << r.horizonM/1000.0 << " km\n";
        std::cout << "  Status      : " << losStatusStr(r.status) << "\n";
        std::cout << "  Visible     : " << (r.visible ? "YES" : "NO") << "\n";

        if(r.status == LOSStatus::TERRAIN_MASKED){
            std::cout << "  Block row   : " << r.blockRow << "\n";
            std::cout << "  Block col   : " << r.blockCol << "\n";
            std::cout << "  Block dist  : " << r.blockDistM/1000.0 << " km\n";
            std::cout << "  Block elev  : " << r.blockElevM << " m\n";
        }
        std::cout << "  Compute     : " << r.computeUs << " µs\n";
        std::cout << "======================================\n";
    }
};

#endif
