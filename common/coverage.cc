/**
 * @file RadarCoverage.cpp
 *
 * RADAR COVERAGE ENGINE — with sector coverage support
 *
 * SECTOR COVERAGE:
 *   Sectors are 45° wide, aligned to 45° grid:
 *     0° = North, 45° = NE, 90° = East, 135° = SE,
 *     180° = South, 225° = SW, 270° = West, 315° = NW
 *
 *   Pass sectorAzimuthDeg to compute()/computeFast():
 *     -1  = full 360° (default)
 *     0   = North sector  (337.5° to 22.5°)
 *     45  = NE sector     (22.5° to 67.5°)
 *     90  = East sector   (67.5° to 112.5°)
 *     ... etc in 45° steps
 *
 *   Any value is accepted and snapped to nearest 45° multiple.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <algorithm>
#include <cstring>
#include "../common.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

constexpr double EARTH_R_M   = 6371000.0;
constexpr double K_ATMOS     = 4.0 / 3.0;
constexpr double K_EFF_R     = K_ATMOS * EARTH_R_M;
constexpr double INV_2_KEFFR = 1.0 / (2.0 * K_EFF_R);
constexpr double DEG2RAD     = 3.14159265358979 / 180.0;
constexpr double RAD2DEG     = 180.0 / 3.14159265358979;

/******************************************************************************
 * COVERAGE STATUS
 ******************************************************************************/

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

struct Pt { int r, c; };

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

/******************************************************************************
 * CIRCLE BOUNDARY
 ******************************************************************************/

static std::vector<Pt>
circleBoundary(int cr, int cc, int radius)
{
    std::vector<Pt> pts;
    pts.reserve(8 * radius);
    int x=0,y=radius,d=3-2*radius;
    auto push8=[&](int px,int py){
        pts.push_back({cr+px,cc+py}); pts.push_back({cr-px,cc+py});
        pts.push_back({cr+px,cc-py}); pts.push_back({cr-px,cc-py});
        pts.push_back({cr+py,cc+px}); pts.push_back({cr-py,cc+px});
        pts.push_back({cr+py,cc-px}); pts.push_back({cr-py,cc-px});
    };
    while(y>=x){
        push8(x,y); x++;
        if(d>0){y--;d+=4*(x-y)+10;}else{d+=4*x+6;}
    }
    std::sort(pts.begin(),pts.end(),[](const Pt&a,const Pt&b){
        return a.r<b.r||(a.r==b.r&&a.c<b.c);});
    pts.erase(std::unique(pts.begin(),pts.end(),
        [](const Pt&a,const Pt&b){return a.r==b.r&&a.c==b.c;}),pts.end());
    return pts;
}

/******************************************************************************
 * CORE RAY — DIAGNOSTIC
 ******************************************************************************/

static void castRadarRayDiag(
    int srcRow, int srcCol,
    int endRow, int endCol,
    double srcElev, double targetHeight,
    double minSlope, double maxSlope,
    double maxRangeM, double horizonM,
    int maxRadiusCells,
    const short*  __restrict__ dem,
    uint8_t*      __restrict__ stat,
    int COLS, int ROWS)
{
    int adr=std::abs(endRow-srcRow), adc=std::abs(endCol-srcCol);
    int sr=(srcRow<endRow)?1:-1, sc=(srcCol<endCol)?1:-1;
    int err=adr-adc;

    const std::ptrdiff_t strideR=static_cast<std::ptrdiff_t>(sr)*COLS;
    const std::ptrdiff_t strideC=sc;
    std::ptrdiff_t off=static_cast<std::ptrdiff_t>(srcRow)*COLS+srcCol;

    const short* demCell  = dem  + off;
    uint8_t*     statCell = stat + off;

    int row=srcRow,col=srcCol;
    double dist=0.0,curvDrop=0.0;
    double maxTerrainSlope=-1e18;
    int cellCount=0;

    while(true)
    {
        if(row==endRow && col==endCol) break;

        int e2=2*err;
        bool mr=(e2>-adc), mc=(e2<adr);
        if(mr){err-=adc;row+=sr;demCell+=strideR;statCell+=strideR;}
        if(mc){err+=adr;col+=sc;demCell+=strideC;statCell+=strideC;}

        double step=(mr&&mc)?RESOLUTION*1.41421356237:RESOLUTION;
        curvDrop+=(2.0*dist*step+step*step)*INV_2_KEFFR;
        dist+=step;
        ++cellCount;

        if(cellCount>maxRadiusCells) break;
        if(dist>maxRangeM){ *statCell=(uint8_t)CoverageStatus::OUTSIDE_RANGE; continue; }
        if(dist>horizonM) break;  // pre-filled as BEYOND_HORIZON

        short raw=*demCell;
        if(raw<-9000) break;

        double terrainElev  = static_cast<double>(raw) - curvDrop;
        double terrainSlope = (terrainElev - srcElev) / dist;
        double targetSlope  = terrainSlope + targetHeight / dist;

        uint8_t s;
        if     (maxTerrainSlope > targetSlope) s=(uint8_t)CoverageStatus::TERRAIN_MASKED;
        else if(targetSlope < minSlope)        s=(uint8_t)CoverageStatus::BELOW_MIN_ELEV;
        else if(targetSlope > maxSlope)        s=(uint8_t)CoverageStatus::ABOVE_MAX_ELEV;
        else                                   s=(uint8_t)CoverageStatus::COVERED;

        *statCell = s;
        if(terrainSlope > maxTerrainSlope) maxTerrainSlope = terrainSlope;
    }
}

/******************************************************************************
 * CORE RAY — FAST (binary)
 ******************************************************************************/

static void castRadarRayFast(
    int srcRow, int srcCol,
    int endRow, int endCol,
    double srcElev, double targetHeight,
    double minSlope, double maxSlope,
    double horizonM, int maxRadiusCells,
    const short* __restrict__ dem,
    uint8_t*     __restrict__ stat,
    int COLS, int ROWS)
{
    int adr=std::abs(endRow-srcRow), adc=std::abs(endCol-srcCol);
    int sr=(srcRow<endRow)?1:-1, sc=(srcCol<endCol)?1:-1;
    int err=adr-adc;

    const std::ptrdiff_t strideR=static_cast<std::ptrdiff_t>(sr)*COLS;
    const std::ptrdiff_t strideC=sc;
    std::ptrdiff_t off=static_cast<std::ptrdiff_t>(srcRow)*COLS+srcCol;

    const short* demCell  = dem  + off;
    uint8_t*     statCell = stat + off;

    int row=srcRow,col=srcCol;
    double dist=0.0,curvDrop=0.0;
    double maxTerrainSlope=-1e18;
    int cellCount=0;

    while(true)
    {
        if(row==endRow && col==endCol) break;

        int e2=2*err;
        bool mr=(e2>-adc), mc=(e2<adr);
        if(mr){err-=adc;row+=sr;demCell+=strideR;statCell+=strideR;}
        if(mc){err+=adr;col+=sc;demCell+=strideC;statCell+=strideC;}

        double step=(mr&&mc)?RESOLUTION*1.41421356237:RESOLUTION;
        curvDrop+=(2.0*dist*step+step*step)*INV_2_KEFFR;
        dist+=step;
        ++cellCount;

        if(cellCount>maxRadiusCells) break;
        if(dist>horizonM) break;

        short raw=*demCell;
        if(raw<-9000) break;

        double terrainElev  = static_cast<double>(raw) - curvDrop;
        double terrainSlope = (terrainElev - srcElev) / dist;
        double targetSlope  = terrainSlope + targetHeight / dist;

        if(maxTerrainSlope<=targetSlope && targetSlope>=minSlope && targetSlope<=maxSlope)
            *statCell = 1;

        if(terrainSlope > maxTerrainSlope) maxTerrainSlope = terrainSlope;
    }
}

/******************************************************************************
 * RADAR COVERAGE ENGINE
 ******************************************************************************/

class RadarCoverageEngine
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
    std::vector<Pt>      boundary;   // full 360 boundary

    // ------------------------------------------------------------------
    // Filter boundary cells for a sector
    // sectorAz: centre azimuth (already snapped to 45° multiple)
    // ------------------------------------------------------------------
    std::vector<Pt> sectorBoundary(int sectorAz) const
    {
        if(sectorAz < 0) return boundary;          // full 360

        const double halfWidth = 22.5;
        std::vector<Pt> out;
        out.reserve(boundary.size() / 8 + 1);

        for(const auto& pt : boundary){
            double az = cellAzimuth(srcRow, srcCol, pt.r, pt.c);
            if(inSectorAz(az, (double)sectorAz, halfWidth))
                out.push_back(pt);
        }
        return out;
    }

    // ------------------------------------------------------------------
    // Pre-fill the active area with BEYOND_HORIZON.
    // Cells outside the sector get OUTSIDE_SECTOR.
    // sectorAz < 0 → fill full circle, no OUTSIDE_SECTOR
    // ------------------------------------------------------------------
    void prefill(int sectorAz, uint8_t* buf, uint8_t fillInside, uint8_t fillOutside)
    {
        const long long rSq = (long long)maxRadiusCells * maxRadiusCells;
        const double halfWidth = 22.5;

        #pragma omp parallel for schedule(static,32)
        for(int r=rowMin; r<rowMax; r++){
            int dr = r - srcRow;
            uint8_t* rowPtr = buf + (size_t)r * gridCols + colMin;

            for(int c=colMin; c<colMax; c++){
                int dc = c - srcCol;
                if((long long)dr*dr + (long long)dc*dc > rSq){
                    rowPtr[c - colMin] = (uint8_t)CoverageStatus::OUTSIDE_RANGE;
                    continue;
                }
                // inside circle
                if(sectorAz < 0){
                    // full 360
                    rowPtr[c - colMin] = fillInside;
                } else {
                    double az = cellAzimuth(srcRow, srcCol, r, c);
                    if(inSectorAz(az, (double)sectorAz, halfWidth))
                        rowPtr[c - colMin] = fillInside;
                    else
                        rowPtr[c - colMin] = fillOutside;
                }
            }
        }
    }

    // ------------------------------------------------------------------
    std::string base64Encode(const uint8_t* data, size_t length)
    {
        static const char lut[]=
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out; out.reserve(((length+2)/3)*4);
        size_t i=0;
        for(;i+2<length;i+=3){
            uint32_t v=((uint32_t)data[i]<<16)|((uint32_t)data[i+1]<<8)|data[i+2];
            out.push_back(lut[(v>>18)&0x3F]); out.push_back(lut[(v>>12)&0x3F]);
            out.push_back(lut[(v>>6) &0x3F]); out.push_back(lut[ v     &0x3F]);
        }
        if(i<length){
            uint32_t v=(uint32_t)data[i]<<16;
            if(i+1<length) v|=(uint32_t)data[i+1]<<8;
            out.push_back(lut[(v>>18)&0x3F]); out.push_back(lut[(v>>12)&0x3F]);
            out.push_back(i+1<length?lut[(v>>6)&0x3F]:'='); out.push_back('=');
        }
        return out;
    }

public:
    RadarCoverageEngine(
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

    // ------------------------------------------------------------------
    // DIAGNOSTIC COMPUTE
    //
    // sectorAzimuthDeg: -1 = full 360
    //                    0, 45, 90 ... 315 = 45° sector centred on that azimuth
    //                    any value snapped to nearest 45°
    // ------------------------------------------------------------------
    RadarCoverageResult compute(int sectorAzimuthDeg = -1)
    {
        int sectorAz = (sectorAzimuthDeg < 0) ? -1 : snapTo45(sectorAzimuthDeg);

        auto activeBoundary = sectorBoundary(sectorAz);

        std::cout<<"[RADAR] Sector        : ";
        if(sectorAz < 0) std::cout<<"360° (full)\n";
        else std::cout<<sectorAz<<"° ± 22.5°  rays="<<activeBoundary.size()<<"\n";

        auto t0=std::chrono::high_resolution_clock::now();

        // pre-fill: inside sector → BEYOND_HORIZON, outside → OUTSIDE_SECTOR
        prefill(sectorAz, statusBuf.data(),
                (uint8_t)CoverageStatus::BEYOND_HORIZON,
                (uint8_t)CoverageStatus::OUTSIDE_SECTOR);

        statusBuf[(size_t)srcRow*gridCols+srcCol]=(uint8_t)CoverageStatus::COVERED;

        const size_t N      = activeBoundary.size();
        const short* demPtr = g.bin.elev_matrix.data();
        uint8_t*     statPtr= statusBuf.data();
        const int COLS=gridCols,ROWS=gridRows;
        const double SE=srcElev,TH=targetHeight;
        const double MNS=minSlopeRatio,MXS=maxSlopeRatio;
        const double MXR=maxRangeM,HOR=horizonM;
        const int    MRC=maxRadiusCells;

        #pragma omp parallel for schedule(dynamic,64) default(none) \
            shared(demPtr,statPtr,activeBoundary) \
            firstprivate(N,COLS,ROWS,SE,TH,MNS,MXS,MXR,HOR,MRC)
        for(size_t i=0;i<N;++i){
            int er=activeBoundary[i].r, ec=activeBoundary[i].c;
            if(er>=0&&ec>=0&&er<ROWS&&ec<COLS)
                castRadarRayDiag(
                    srcRow,srcCol,er,ec,
                    SE,TH,MNS,MXS,MXR,HOR,MRC,
                    demPtr,statPtr,COLS,ROWS);
        }

        auto t1=std::chrono::high_resolution_clock::now();

        // stats — only count cells in active area
        RadarCoverageResult res;
        res.srcRow=srcRow; res.srcCol=srcCol;
        res.radiusCells=maxRadiusCells;
        res.gridCols=gridCols; res.gridRows=gridRows;
        res.sectorAzimuthDeg=sectorAz;
        res.countCovered=res.countTerrainMasked=0;
        res.countBelowMinElev=res.countAboveMaxElev=0;
        res.countBeyondHorizon=res.countOutsideRange=res.countOutsideSector=0;

        long long rSq=(long long)maxRadiusCells*maxRadiusCells;
        for(int r=rowMin;r<rowMax;r++)
        for(int c=colMin;c<colMax;c++){
            int dr=r-srcRow,dc=c-srcCol;
            if((long long)dr*dr+(long long)dc*dc>rSq) continue;
            switch((CoverageStatus)statusBuf[(size_t)r*gridCols+c]){
                case CoverageStatus::COVERED:        res.countCovered++;        break;
                case CoverageStatus::TERRAIN_MASKED: res.countTerrainMasked++;  break;
                case CoverageStatus::BELOW_MIN_ELEV: res.countBelowMinElev++;   break;
                case CoverageStatus::ABOVE_MAX_ELEV: res.countAboveMaxElev++;   break;
                case CoverageStatus::BEYOND_HORIZON: res.countBeyondHorizon++;  break;
                case CoverageStatus::OUTSIDE_SECTOR: res.countOutsideSector++;  break;
                default:                             res.countOutsideRange++;   break;
            }
        }

        res.status.resize((size_t)gridRows*gridCols);
        std::memcpy(res.status.data(),statusBuf.data(),(size_t)gridRows*gridCols);

        int total=res.countCovered+res.countTerrainMasked+
                  res.countBelowMinElev+res.countAboveMaxElev+
                  res.countBeyondHorizon;
        double ms=std::chrono::duration<double,std::milli>(t1-t0).count();

        std::cout<<"[RADAR] Compute       : "<<ms<<" ms\n";
        std::cout<<"  COVERED        : "<<res.countCovered<<"\n";
        std::cout<<"  TERRAIN_MASKED : "<<res.countTerrainMasked<<"\n";
        std::cout<<"  BELOW_MIN_ELEV : "<<res.countBelowMinElev<<"\n";
        std::cout<<"  ABOVE_MAX_ELEV : "<<res.countAboveMaxElev<<"\n";
        std::cout<<"  BEYOND_HORIZON : "<<res.countBeyondHorizon<<"\n";
        std::cout<<"  OUTSIDE_SECTOR : "<<res.countOutsideSector<<"\n";
        if(total>0) std::cout<<"  Coverage %     : "
                             <<(100.0*res.countCovered/total)<<"%\n";
        return res;
    }

    // ------------------------------------------------------------------
    // FAST COMPUTE — binary, no diagnostic
    // ------------------------------------------------------------------
    std::vector<uint8_t> computeFast(int sectorAzimuthDeg = -1)
    {
        int sectorAz=(sectorAzimuthDeg<0)?-1:snapTo45(sectorAzimuthDeg);
        auto activeBoundary=sectorBoundary(sectorAz);

        auto t0=std::chrono::high_resolution_clock::now();

        // reset bounding box to 0
        for(int r=rowMin;r<rowMax;r++){
            uint8_t* p=fastBuf.data()+(size_t)r*gridCols+colMin;
            std::memset(p,0,colMax-colMin);
        }
        fastBuf[(size_t)srcRow*gridCols+srcCol]=1;

        const size_t N      = activeBoundary.size();
        const short* demPtr = g.bin.elev_matrix.data();
        uint8_t*     statPtr= fastBuf.data();
        const int COLS=gridCols,ROWS=gridRows;
        const double SE=srcElev,TH=targetHeight;
        const double MNS=minSlopeRatio,MXS=maxSlopeRatio;
        const double HOR=horizonM;
        const int    MRC=maxRadiusCells;

        #pragma omp parallel for schedule(dynamic,64) default(none) \
            shared(demPtr,statPtr,activeBoundary) \
            firstprivate(N,COLS,ROWS,SE,TH,MNS,MXS,HOR,MRC)
        for(size_t i=0;i<N;++i){
            int er=activeBoundary[i].r,ec=activeBoundary[i].c;
            if(er>=0&&ec>=0&&er<ROWS&&ec<COLS)
                castRadarRayFast(
                    srcRow,srcCol,er,ec,
                    SE,TH,MNS,MXS,HOR,MRC,
                    demPtr,statPtr,COLS,ROWS);
        }

        auto t1=std::chrono::high_resolution_clock::now();
        int vis=0;
        for(int r=rowMin;r<rowMax;r++)
        for(int c=colMin;c<colMax;c++)
            vis+=fastBuf[(size_t)r*gridCols+c];

        std::cout<<"[RADAR] Fast ("
                 <<(sectorAz<0?"360°":std::to_string(sectorAz)+"°")
                 <<")  : "
                 <<std::chrono::duration<double,std::milli>(t1-t0).count()
                 <<" ms  covered="<<vis<<"\n";

        return fastBuf;
    }

    // ------------------------------------------------------------------
    // SVG OUTPUT
    //
    // Draws sector arc lines if in sector mode.
    // ------------------------------------------------------------------
    void writeToSVG(
        const RadarCoverageResult& res,
        const std::string& filename="radar_coverage.svg")
    {
        auto tAll=std::chrono::high_resolution_clock::now();

        int W=colMax-colMin, H=rowMax-rowMin;
        size_t total=(size_t)W*H;
        std::vector<uint8_t> rgba(total*4,0);
        long long rSq=(long long)maxRadiusCells*maxRadiusCells;

        #pragma omp parallel for schedule(static,4096)
        for(size_t i=0;i<total;++i){
            int px=(int)(i%W), py=(int)(i/W);
            int col=colMin+px, row=rowMin+py;
            int dc=col-srcCol, dr=row-srcRow;
            if((long long)dc*dc+(long long)dr*dr>rSq) continue;
            size_t gi=(size_t)row*gridCols+col;
            size_t pix=i*4;
            switch((CoverageStatus)res.status[gi]){
                case CoverageStatus::COVERED:
                    rgba[pix+0]=46; rgba[pix+1]=204;rgba[pix+2]=113;rgba[pix+3]=255; break;
                case CoverageStatus::TERRAIN_MASKED:
                    rgba[pix+0]=231;rgba[pix+1]=76; rgba[pix+2]=60; rgba[pix+3]=255; break;
                case CoverageStatus::BELOW_MIN_ELEV:
                    rgba[pix+0]=52; rgba[pix+1]=152;rgba[pix+2]=219;rgba[pix+3]=255; break;
                case CoverageStatus::ABOVE_MAX_ELEV:
                    rgba[pix+0]=230;rgba[pix+1]=126;rgba[pix+2]=34; rgba[pix+3]=255; break;
                case CoverageStatus::BEYOND_HORIZON:
                    rgba[pix+0]=155;rgba[pix+1]=89; rgba[pix+2]=182;rgba[pix+3]=255; break;
                case CoverageStatus::OUTSIDE_SECTOR:
                    rgba[pix+0]=40; rgba[pix+1]=40; rgba[pix+2]=40; rgba[pix+3]=255; break;
                default: break;
            }
        }

        #pragma omp parallel for schedule(static,4096)
        for(size_t i=0;i<total;++i)
            std::swap(rgba[i*4+0],rgba[i*4+2]);

        uint32_t fs=54+(uint32_t)(total*4);
        uint8_t hdr[54]={'B','M',
            (uint8_t)fs,(uint8_t)(fs>>8),(uint8_t)(fs>>16),(uint8_t)(fs>>24),
            0,0,0,0,54,0,0,0,40,0,0,0,
            (uint8_t)W,(uint8_t)(W>>8),(uint8_t)(W>>16),(uint8_t)(W>>24),
            (uint8_t)(-H),(uint8_t)(-H>>8),(uint8_t)(-H>>16),(uint8_t)(-H>>24),
            1,0,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        std::vector<uint8_t> payload;
        payload.reserve(54+total*4);
        payload.insert(payload.end(),hdr,hdr+54);
        payload.insert(payload.end(),rgba.begin(),rgba.end());
        std::string b64=base64Encode(payload.data(),payload.size());

        int cx=srcCol-colMin, cy=srcRow-rowMin;
        int rr=maxRadiusCells;

        std::ofstream f(filename);
        f<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
         <<"<svg xmlns=\"http://www.w3.org/2000/svg\""
         <<" viewBox=\"0 0 "<<W<<" "<<H<<"\" width=\"100%\" height=\"100%\""
         <<" style=\"background:#0a0a0a\">\n"
         // legend
         <<"  <rect x=\"10\" y=\"10\" width=\"12\" height=\"12\" fill=\"#2ecc71\"/>"
         <<"<text x=\"26\" y=\"21\" fill=\"#eee\" font-size=\"12\">COVERED</text>\n"
         <<"  <rect x=\"10\" y=\"28\" width=\"12\" height=\"12\" fill=\"#e74c3c\"/>"
         <<"<text x=\"26\" y=\"39\" fill=\"#eee\" font-size=\"12\">TERRAIN MASKED</text>\n"
         <<"  <rect x=\"10\" y=\"46\" width=\"12\" height=\"12\" fill=\"#3498db\"/>"
         <<"<text x=\"26\" y=\"57\" fill=\"#eee\" font-size=\"12\">BELOW MIN ELEV</text>\n"
         <<"  <rect x=\"10\" y=\"64\" width=\"12\" height=\"12\" fill=\"#e67e22\"/>"
         <<"<text x=\"26\" y=\"75\" fill=\"#eee\" font-size=\"12\">ABOVE MAX ELEV</text>\n"
         <<"  <rect x=\"10\" y=\"82\" width=\"12\" height=\"12\" fill=\"#9b59b6\"/>"
         <<"<text x=\"26\" y=\"93\" fill=\"#eee\" font-size=\"12\">BEYOND HORIZON</text>\n"
         <<"  <rect x=\"10\" y=\"100\" width=\"12\" height=\"12\" fill=\"#282828\""
         <<" stroke=\"#555\" stroke-width=\"1\"/>"
         <<"<text x=\"26\" y=\"111\" fill=\"#eee\" font-size=\"12\">OUTSIDE SECTOR</text>\n"
         // image
         <<"  <image width=\""<<W<<"\" height=\""<<H
         <<"\" x=\"0\" y=\"0\" href=\"data:image/bmp;base64,"<<b64<<"\"/>\n"
         // range circle
         <<"  <circle cx=\""<<cx<<"\" cy=\""<<cy<<"\" r=\""<<rr
         <<"\" fill=\"none\" stroke=\"white\" stroke-width=\"2\""
         <<" stroke-dasharray=\"20,15\" opacity=\"0.4\"/>\n"
         // horizon circle
         <<"  <circle cx=\""<<cx<<"\" cy=\""<<cy
         <<"\" r=\""<<(int)(horizonM/RESOLUTION)
         <<"\" fill=\"none\" stroke=\"#9b59b6\" stroke-width=\"2\""
         <<" stroke-dasharray=\"10,8\" opacity=\"0.5\"/>\n";

        // sector boundary lines
        if(res.sectorAzimuthDeg >= 0){
            double az  = res.sectorAzimuthDeg * DEG2RAD;
            double hw  = 22.5 * DEG2RAD;
            double len = (double)rr;

            // left edge: az - halfWidth (SVG: x=east=col, y=south=row)
            double az1 = az - hw;
            double x1  = cx + len * std::sin(az1);
            double y1  = cy - len * std::cos(az1);
            // right edge
            double az2 = az + hw;
            double x2  = cx + len * std::sin(az2);
            double y2  = cy - len * std::cos(az2);

            f<<"  <line x1=\""<<cx<<"\" y1=\""<<cy
             <<"\" x2=\""<<(int)x1<<"\" y2=\""<<(int)y1
             <<"\" stroke=\"white\" stroke-width=\"2\" opacity=\"0.7\""
             <<" stroke-dasharray=\"12,8\"/>\n"
             <<"  <line x1=\""<<cx<<"\" y1=\""<<cy
             <<"\" x2=\""<<(int)x2<<"\" y2=\""<<(int)y2
             <<"\" stroke=\"white\" stroke-width=\"2\" opacity=\"0.7\""
             <<" stroke-dasharray=\"12,8\"/>\n"
             // sector label
             <<"  <text x=\""<<(int)(cx+len*0.55*std::sin(az))
             <<"\" y=\""<<(int)(cy-len*0.55*std::cos(az))
             <<"\" fill=\"white\" font-size=\"24\" text-anchor=\"middle\""
             <<" opacity=\"0.6\">"<<res.sectorAzimuthDeg<<"°</text>\n";
        }

        // source marker
        f<<"  <circle cx=\""<<cx<<"\" cy=\""<<cy
         <<"\" r=\"6\" fill=\"white\" stroke=\"#1abc9c\" stroke-width=\"3\"/>\n"
         // stats
         <<"  <text x=\""<<(W-220)<<"\" y=\"20\" fill=\"#2ecc71\" font-size=\"12\">"
         <<"Covered: "<<res.countCovered<<"</text>\n"
         <<"  <text x=\""<<(W-220)<<"\" y=\"38\" fill=\"#e74c3c\" font-size=\"12\">"
         <<"Terrain: "<<res.countTerrainMasked<<"</text>\n"
         <<"  <text x=\""<<(W-220)<<"\" y=\"56\" fill=\"#3498db\" font-size=\"12\">"
         <<"BelowElev: "<<res.countBelowMinElev<<"</text>\n"
         <<"  <text x=\""<<(W-220)<<"\" y=\"74\" fill=\"#e67e22\" font-size=\"12\">"
         <<"AboveElev: "<<res.countAboveMaxElev<<"</text>\n"
         <<"  <text x=\""<<(W-220)<<"\" y=\"92\" fill=\"#9b59b6\" font-size=\"12\">"
         <<"Horizon: "<<res.countBeyondHorizon<<"</text>\n"
         <<"</svg>\n";

        std::cout<<"[RADAR] SVG: "<<filename<<"  "
                 <<std::chrono::duration<double,std::milli>(
                     std::chrono::high_resolution_clock::now()-tAll).count()
                 <<" ms\n";
    }
};

/******************************************************************************
 * MAIN
 ******************************************************************************/

int main()
{
    auto t0=std::chrono::high_resolution_clock::now();
    Global g;

    RadarCoverageEngine engine(
        g,
        33.9871, 74.7742, // Srinagar airport
        50.0,    // antenna height AGL (m)
        1000.0,  // target height AGL (m) — e.g. aircraft at 1km
        10.0,     // min elevation angle (degrees)
        30.0,    // max elevation angle (degrees)
        300.0    // max range (km)
    );

    // full 360
    auto res360 = engine.compute();
    engine.writeToSVG(res360, "radar_360.svg");

    // NE sector (45°)
    auto res45 = engine.compute(45);
    engine.writeToSVG(res45, "radar_sector_045.svg");

    // East sector (90°)
    auto res90 = engine.compute(90);
    engine.writeToSVG(res90, "radar_sector_090.svg");

    // fast mode
    engine.computeFast();        // 360
    engine.computeFast(135);     // SE sector

    auto t1=std::chrono::high_resolution_clock::now();
    std::cout<<"\n==============================================\n"
             <<">> TOTAL: "
             <<std::chrono::duration<double,std::milli>(t1-t0).count()/1000.0
             <<" s\n==============================================\n";
}
