#ifndef SHARED_H_
#define SHARED_H_


#include "preprocess/Config.h"
#include "preprocess/bin.h"
#include "preprocess/lib.h"

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
    public: Config conf;
    public: Bin bin;

    public: Global(){
        Bin b;
        bin = b;
        if(Config::is_available()){
            conf = Config::load();
        } else {
            std::cout<< "[Terrain] config not found. using default" <<std::endl;
        }
    }
    public: static bool lineOfSight(double lat0, double long0,int h0, double lat1, double long1,int h1);
    public: static std::vector<std::pair<int,int>> bresenham(int x0, int y0, int x1, int y1);
    public: static std::vector<LOSResult> lineOfVisibility(double lat0, double long0,int h0, double lat1, double long1,int h1);
        std::vector<std::vector<LOSResult>> lineOfVisibilityBatch(
            const std::vector<LOSQuery>& queries,
            int numThreads = 0);  // 0 = auto detect
};

#endif
