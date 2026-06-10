/**
 * @file RadarLOS.cpp
 *
 * RADAR LINE OF SIGHT — single point-to-point check
 *
 * Different from coverage (which checks every cell in a circle):
 *   This checks ONE specific target point against ONE radar source.
 *
 * CHECKS (in order of cost, cheapest first):
 *   1. Range           — is target within maxRangeM?
 *   2. Horizon         — is target within radar+target LOS horizon?
 *   3. Elevation angle — is target within [minElev, maxElev] cone?
 *   4. Terrain masking — does any terrain along the ray block LOS?
 *
 * FAST PATH (radarLOS):
 *   Checks 1-3 before touching the DEM at all.
 *   On terrain check, breaks immediately when blocked.
 *   Returns single RadarLOSResult in microseconds.
 *
 * DETAILED PATH (radarLOSDetailed):
 *   Returns per-cell breakdown — useful for debugging and visualization.
 *   Walks full ray, marks each cell with why it would/would not block.
 *
 * KEY ALGORITHM DIFFERENCE FROM COVERAGE:
 *   In coverage, targetSlope varies per cell (each cell is a different target).
 *   In LOS, targetSlope is FIXED (one known target) — precomputed once.
 *   So the terrain masking check is: maxTerrainSlope > fixedTargetSlope
 *   This allows immediate early termination on first blocking cell.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

#include "../common.h"
#include "../operations.h"


RadarLOSResult RadarEngine::radarLOS(
    Global&  g,
    double   srcLat,  double srcLon,  double antennaHeightM,
    double   tgtLat,  double tgtLon,  double targetHeightM,
    double   minElevDeg, double maxElevDeg,
    double   maxRangeKM)
{
    auto t0 = std::chrono::steady_clock::now();

    RadarLOSResult res{};
    res.blockRow = -1;
    res.blockCol = -1;

    const int COLS = g.conf.grid.cols;
    const int ROWS = g.conf.grid.rows;

    // -- grid coords --
    int srcRow, srcCol, tgtRow, tgtCol;
    if(!g.bin.getGridCoords(srcLat, srcLon, srcRow, srcCol) ||
       !g.bin.getGridCoords(tgtLat, tgtLon, tgtRow, tgtCol)){
        res.status  = LOSStatus::OUT_OF_GRID;
        res.visible = false;
        return res;
    }

    // -- source elevation --
    short srcTerrain = g.bin.elev_matrix[(size_t)srcRow * COLS + srcCol];
    if(srcTerrain < -9000){
        res.status  = LOSStatus::NO_DATA;
        res.visible = false;
        return res;
    }
    double srcElev = static_cast<double>(srcTerrain) + antennaHeightM;

    // -- target elevation --
    short tgtTerrain = g.bin.elev_matrix[(size_t)tgtRow * COLS + tgtCol];
    if(tgtTerrain < -9000){
        res.status  = LOSStatus::NO_DATA;
        res.visible = false;
        return res;
    }
    double tgtAbsElev = static_cast<double>(tgtTerrain) + targetHeightM;

    // -- ground distance to target --
    double ddr = (double)(tgtRow - srcRow) * RESOLUTION;
    double ddc = (double)(tgtCol - srcCol) * RESOLUTION;
    double tgtDist = std::sqrt(ddr*ddr + ddc*ddc);

    res.distM = tgtDist;

    // -- CHECK 1: range --
    double maxRangeM = maxRangeKM * 1000.0;
    if(tgtDist > maxRangeM){
        res.status  = LOSStatus::BEYOND_RANGE;
        res.visible = false;
        auto t1 = std::chrono::steady_clock::now();
        res.computeUs = std::chrono::duration<double,std::micro>(t1-t0).count();
        return res;
    }

    // -- CHECK 2: horizon (k=4/3 standard atmosphere) --
    double horizonTx = std::sqrt(2.0 * K_EFF_R * antennaHeightM);
    double horizonRx = std::sqrt(2.0 * K_EFF_R * targetHeightM);
    double horizonM  = horizonTx + horizonRx;
    res.horizonM = horizonM;

    if(tgtDist > horizonM){
        res.status  = LOSStatus::BEYOND_HORIZON;
        res.visible = false;
        auto t1 = std::chrono::steady_clock::now();
        res.computeUs = std::chrono::duration<double,std::micro>(t1-t0).count();
        return res;
    }

    // -- CHECK 3: elevation angle to target --
    // Apply curvature correction to target too
    double tgtCurvDrop = (tgtDist * tgtDist) * INV_2_KEFFR;
    double tgtCorrElev = tgtAbsElev - tgtCurvDrop;
    double dh          = tgtCorrElev - srcElev;
    double elevAngleDeg = std::atan2(dh, tgtDist) * RAD2DEG;
    res.elevAngleDeg = elevAngleDeg;

    double targetSlope = dh / tgtDist;  // tan(elevAngle) — fixed for this ray

    if(elevAngleDeg < minElevDeg){
        res.status  = LOSStatus::BELOW_MIN_ELEV;
        res.visible = false;
        auto t1 = std::chrono::steady_clock::now();
        res.computeUs = std::chrono::duration<double,std::micro>(t1-t0).count();
        return res;
    }
    if(elevAngleDeg > maxElevDeg){
        res.status  = LOSStatus::ABOVE_MAX_ELEV;
        res.visible = false;
        auto t1 = std::chrono::steady_clock::now();
        res.computeUs = std::chrono::duration<double,std::micro>(t1-t0).count();
        return res;
    }

    // -- CHECK 4: terrain masking via Bresenham ray --
    // targetSlope is FIXED — precomputed above
    // Break immediately on first blocking cell

    int adr = std::abs(tgtRow - srcRow);
    int adc = std::abs(tgtCol - srcCol);
    int sr  = (srcRow < tgtRow) ? 1 : -1;
    int sc  = (srcCol < tgtCol) ? 1 : -1;
    int err = adr - adc;

    const std::ptrdiff_t strideR =
        static_cast<std::ptrdiff_t>(sr) * COLS;
    const std::ptrdiff_t strideC = sc;
    std::ptrdiff_t off =
        static_cast<std::ptrdiff_t>(srcRow) * COLS + srcCol;

    const short* demCell = g.bin.elev_matrix.data() + off;

    int    row = srcRow, col = srcCol;
    double dist = 0.0, curvDrop = 0.0;
    double maxTerrainSlope = -1e18;
    int    cellCount = 0;
    int    maxCells  = std::max(adr, adc) + 1;

    while(true)
    {
        if(row == tgtRow && col == tgtCol) break;

        int e2 = 2*err;
        bool mr = (e2 > -adc), mc = (e2 < adr);
        if(mr){ err -= adc; row += sr; demCell += strideR; }
        if(mc){ err += adr; col += sc; demCell += strideC; }

        double step = (mr && mc) ? RESOLUTION*1.41421356237 : RESOLUTION;
        curvDrop += (2.0*dist*step + step*step) * INV_2_KEFFR;
        dist     += step;
        ++cellCount;
        if(cellCount > maxCells) break;

        short raw = *demCell;
        if(raw < -9000){
            res.status  = LOSStatus::NO_DATA;
            res.visible = false;
            auto t1 = std::chrono::steady_clock::now();
            res.computeUs = std::chrono::duration<double,std::micro>(t1-t0).count();
            return res;
        }

        double terrainElev  = static_cast<double>(raw) - curvDrop;
        double terrainSlope = (terrainElev - srcElev) / dist;

        // EARLY TERMINATION: first cell where terrain slope > target slope
        if(terrainSlope > targetSlope){
            res.status    = LOSStatus::TERRAIN_MASKED;
            res.visible   = false;
            res.blockRow  = row;
            res.blockCol  = col;
            res.blockDistM = dist;
            res.blockElevM = raw;
            auto t1 = std::chrono::steady_clock::now();
            res.computeUs = std::chrono::duration<double,std::micro>(t1-t0).count();
            return res;
        }

        if(terrainSlope > maxTerrainSlope)
            maxTerrainSlope = terrainSlope;
    }

    // reached target — visible
    res.status  = LOSStatus::VISIBLE;
    res.visible = true;

    auto t1 = std::chrono::steady_clock::now();
    res.computeUs = std::chrono::duration<double,std::micro>(t1-t0).count();
    return res;
}

/******************************************************************************
 * DETAILED RADAR LOS
 *
 * Same algorithm but walks the full ray and records per-cell state.
 * Used for debugging, visualization, and understanding why blocked.
 * Slightly slower due to vector push_back per cell.
 ******************************************************************************/

RadarLOSDetailedResult RadarEngine::radarLOSDetailed(
    Global&  g,
    double   srcLat,  double srcLon,  double antennaHeightM,
    double   tgtLat,  double tgtLon,  double targetHeightM,
    double   minElevDeg, double maxElevDeg,
    double   maxRangeKM)
{
    auto t0 = std::chrono::steady_clock::now();

    RadarLOSDetailedResult out{};
    out.summary.blockRow = -1;
    out.summary.blockCol = -1;

    const int COLS = g.conf.grid.cols;
    const int ROWS = g.conf.grid.rows;

    int srcRow, srcCol, tgtRow, tgtCol;
    if(!g.bin.getGridCoords(srcLat, srcLon, srcRow, srcCol) ||
       !g.bin.getGridCoords(tgtLat, tgtLon, tgtRow, tgtCol)){
        out.summary.status  = LOSStatus::OUT_OF_GRID;
        out.summary.visible = false;
        return out;
    }

    short srcTerrain = g.bin.elev_matrix[(size_t)srcRow*COLS+srcCol];
    short tgtTerrain = g.bin.elev_matrix[(size_t)tgtRow*COLS+tgtCol];
    if(srcTerrain < -9000 || tgtTerrain < -9000){
        out.summary.status  = LOSStatus::NO_DATA;
        out.summary.visible = false;
        return out;
    }

    double srcElev    = static_cast<double>(srcTerrain) + antennaHeightM;
    double tgtAbsElev = static_cast<double>(tgtTerrain) + targetHeightM;

    double ddr = (double)(tgtRow-srcRow)*RESOLUTION;
    double ddc = (double)(tgtCol-srcCol)*RESOLUTION;
    double tgtDist = std::sqrt(ddr*ddr + ddc*ddc);

    double maxRangeM = maxRangeKM * 1000.0;
    double horizonTx = std::sqrt(2.0*K_EFF_R*antennaHeightM);
    double horizonRx = std::sqrt(2.0*K_EFF_R*targetHeightM);
    double horizonM  = horizonTx + horizonRx;

    out.summary.distM    = tgtDist;
    out.summary.horizonM = horizonM;

    // pre-check range / horizon / elev angle
    if(tgtDist > maxRangeM){
        out.summary.status=LOSStatus::BEYOND_RANGE; out.summary.visible=false; return out;}
    if(tgtDist > horizonM){
        out.summary.status=LOSStatus::BEYOND_HORIZON; out.summary.visible=false; return out;}

    double tgtCurvDrop  = (tgtDist*tgtDist) * INV_2_KEFFR;
    double tgtCorrElev  = tgtAbsElev - tgtCurvDrop;
    double dh           = tgtCorrElev - srcElev;
    double elevAngleDeg = std::atan2(dh, tgtDist) * RAD2DEG;
    double targetSlope  = dh / tgtDist;

    out.summary.elevAngleDeg = elevAngleDeg;

    if(elevAngleDeg < minElevDeg){
        out.summary.status=LOSStatus::BELOW_MIN_ELEV; out.summary.visible=false; return out;}
    if(elevAngleDeg > maxElevDeg){
        out.summary.status=LOSStatus::ABOVE_MAX_ELEV; out.summary.visible=false; return out;}

    // ray walk — record every cell
    int adr=std::abs(tgtRow-srcRow), adc=std::abs(tgtCol-srcCol);
    int sr=(srcRow<tgtRow)?1:-1, sc=(srcCol<tgtCol)?1:-1;
    int err=adr-adc;

    const std::ptrdiff_t strideR = static_cast<std::ptrdiff_t>(sr)*COLS;
    const std::ptrdiff_t strideC = sc;
    std::ptrdiff_t off = static_cast<std::ptrdiff_t>(srcRow)*COLS+srcCol;
    const short* demCell = g.bin.elev_matrix.data() + off;

    int    row=srcRow, col=srcCol;
    double dist=0.0, curvDrop=0.0;
    double maxTerrainSlope=-1e18;
    int    cellCount=0, maxCells=std::max(adr,adc)+1;

    out.cells.reserve(maxCells);

    // source cell
    out.cells.push_back({srcRow,srcCol,0.0,(float)srcTerrain,(float)srcTerrain,
                         0.0,-1e18,targetSlope,false});

    while(true)
    {
        if(row==tgtRow && col==tgtCol) break;

        int e2=2*err;
        bool mr=(e2>-adc), mc=(e2<adr);
        if(mr){err-=adc;row+=sr;demCell+=strideR;}
        if(mc){err+=adr;col+=sc;demCell+=strideC;}

        double step=(mr&&mc)?RESOLUTION*1.41421356237:RESOLUTION;
        curvDrop+=(2.0*dist*step+step*step)*INV_2_KEFFR;
        dist+=step;
        ++cellCount;
        if(cellCount>maxCells) break;

        short raw=*demCell;
        if(raw<-9000){
            out.summary.status=LOSStatus::NO_DATA;
            out.summary.visible=false;
            auto t1=std::chrono::steady_clock::now();
            out.summary.computeUs=
                std::chrono::duration<double,std::micro>(t1-t0).count();
            return out;
        }

        double terrainElev  = static_cast<double>(raw) - curvDrop;
        double terrainSlope = (terrainElev - srcElev) / dist;
        bool   blocks       = (terrainSlope > targetSlope);

        LOSCell cell;
        cell.row             = row;
        cell.col             = col;
        cell.distM           = dist;
        cell.terrainElev     = raw;
        cell.correctedElev   = (float)terrainElev;
        cell.terrainSlope    = terrainSlope;
        cell.maxSlopeSoFar   = maxTerrainSlope;
        cell.targetSlope     = targetSlope;
        cell.blocks          = blocks;
        out.cells.push_back(cell);

        if(blocks){
            out.summary.status     = LOSStatus::TERRAIN_MASKED;
            out.summary.visible    = false;
            out.summary.blockRow   = row;
            out.summary.blockCol   = col;
            out.summary.blockDistM = dist;
            out.summary.blockElevM = raw;
            // continue walking for full visualization even after block
            // but don't update maxTerrainSlope past block point
        }

        if(terrainSlope > maxTerrainSlope)
            maxTerrainSlope = terrainSlope;
    }

    if(out.summary.status != LOSStatus::TERRAIN_MASKED){
        out.summary.status  = LOSStatus::VISIBLE;
        out.summary.visible = true;
    }

    auto t1=std::chrono::steady_clock::now();
    out.summary.computeUs=
        std::chrono::duration<double,std::micro>(t1-t0).count();
    return out;
}

/******************************************************************************
 * PRINT HELPERS
 ******************************************************************************/



/******************************************************************************
 * MAIN
 ******************************************************************************/

// int main()
// {
//     Global g;

//     double srcLat = 34.017346,  srcLon = 74.50587999;
//     double tgtLat = 34.16910062,  tgtLon = 74.71267172;  // ~50km E

//     // fast LOS
//     auto res = radarLOS(g,
//         srcLat, srcLon, 30.0,   // radar: 30m antenna
//         tgtLat, tgtLon, 1000.0, // target: 1km AGL aircraft
//         0.5, 30.0,              // elevation cone
//         300.0);                 // max range

//     printLOSResult(res, srcLat, srcLon, tgtLat, tgtLon);

//     // detailed LOS (for visualization / debugging)
//     auto det = radarLOSDetailed(g,
//         srcLat, srcLon, 30.0,
//         tgtLat, tgtLon, 1000.0,
//         0.5, 30.0, 300.0);

//     std::cout << "\nDETAILED — " << det.cells.size() << " cells along ray\n";
//     std::cout << "First blocking cell: ";
//     if(det.summary.status == LOSStatus::TERRAIN_MASKED)
//         std::cout << "row=" << det.summary.blockRow
//                   << " col=" << det.summary.blockCol
//                   << " dist=" << det.summary.blockDistM/1000.0 << "km\n";
//     else
//         std::cout << "none — " << losStatusStr(det.summary.status) << "\n";

//     // print first 10 cells
//     std::cout << "\nRow    Col    Dist(km)  TerrElev  CorrElev  "
//               << "TerrSlope   TargSlope   Blocks\n";
//     int n = std::min((int)det.cells.size(), 10);
//     for(int i=0; i<n; i++){
//         const auto& c = det.cells[i];
//         printf("%-6d %-6d %-9.2f %-9.1f %-9.1f %-11.6f %-11.6f %s\n",
//                c.row, c.col, c.distM/1000.0,
//                (double)c.terrainElev, (double)c.correctedElev,
//                c.terrainSlope, c.targetSlope,
//                c.blocks ? "BLOCK" : "ok");
//     }
// }
