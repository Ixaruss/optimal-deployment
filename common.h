#ifndef SHARED_H_
#define SHARED_H_


#include "preprocess/Config.h"
#include "preprocess/bin.h"
#include "preprocess/lib.h"
#include <sys/stat.h>

struct LOSResult
{
    int    x;
    int    y;
    double dist;
    double angle;
    double maxAngle;
    bool   visible;
    int elev;
};
struct CoverageResult
{
    std::vector<uint8_t>              visibility;  // flat grid
    std::vector<std::vector<LOSResult>> rays;      // per-ray detail
    int srcX, srcY;
    int radiusCells;
    int gridCols, gridRows;
};
struct RadarQuery
{
    double lat, lon;
    int    heightAbove;
    double radiusMeters;
    bool   earthCurvature;
};

struct LOSQuery
{
    double lat0, lon0; int h0;
    double lat1, lon1; int h1;
};

//
//   std::vector<LOSResult> lineOfVisibilityFast(
//       double lat0, double lon0, int h0,
//       double lat1, double lon1, int h1);
//

class Global {
public:
    Bin bin;
    Config& conf = bin.conf;
    Global() {
        bin.init({ ELEVATION });
    }

    public: std::vector<LOSResult>  lineOfVisibilityopt(double lat0, double lon0, int h0, double lat1, double lon1, int h1);

    public: std::vector<LOSResult>  lineOfVisibilitypara(double lat0, double lon0, int h0, double lat1, double lon1, int h1);

    public: std::vector<LOSResult> lineOfVisibilityOptimized(double lat0, double lon0, int h0, double lat1, double lon1, int h1);

    public:  bool lineOfSight(double lat0, double long0,int h0, double lat1, double long1,int h1);

    public: static std::vector<std::pair<int,int>> bresenham(int x0, int y0, int x1, int y1);

    public:  std::vector<LOSResult> lineOfVisibility(double lat0, double long0,int h0, double lat1, double long1,int h1);

    public: static std::vector<std::vector<LOSResult>> lineOfVisibilityBatch(const std::vector<LOSQuery>& queries, int numThreads = 0);  // 0 = auto detect

    public: static inline double deg2rad(double deg) { return deg * M_PI / 180.0; };

    public: static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; };

    public: static double computeDistance(double lonA, double latA, double lonB, double latB, OGRCoordinateTransformation* poCT);

    public: static double computeBearing(double lonA, double latA, double lonB, double latB, OGRCoordinateTransformation* poCT);

    public: static bool dropPointAtDistanceAndBearing(double startLon, double startLat,
                                                      double distanceMeters, double bearingDegrees,
                                                      OGRCoordinateTransformation* poForwardCT,
                                                      OGRCoordinateTransformation* poInverseCT,
                                                      double& outLon, double& outLat);
    CoverageResult radarCoverage(
        double lat,    double lon,
        int    heightAbove,
        double radiusMeters,
        bool   earthCurvature,
        int    numThreads);
    CoverageResult radarCoverageFast(
        double lat,    double lon,
        int    heightAbove,
        double radiusMeters,
        bool   earthCurvature,
        int    numThreads);
    std::vector<CoverageResult> radarCoverageBatch(
        const std::vector<RadarQuery>& queries,
        int numThreads);

};

#endif
