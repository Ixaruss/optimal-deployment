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
};

#endif
