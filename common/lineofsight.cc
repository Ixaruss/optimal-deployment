#include "../common.h"
#include <cmath>
#include <cstddef>
#include <chrono>
#include <gdal.h>
#include <ogr_geometry.h>
#include <ogr_spatialref.h>
#include <ogr_srs_api.h>
using namespace std;

// int get_dist(double lat0, double long0, double lat1,double long1){
//     GDALAllRegister();
//     OGRSpatialReference src;
//     src.importFromEPSG(7755);
//     src.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
//     OGRSpatialReference target;
//     src.importFromEPSG(7755);
//     src.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
//     OGRCoordinateTransformation *poct = OGRCreateCoordinateTransformation(&src,&target);
//     if(poct ==nullptr) {
//         std::cerr << "transformation failed";
//         return 1;
//     }
//     OGRPoint p1(lat0,long0);
//     OGRPoint p2(lat1,long1);
//     if(p1.transform((poct)))
// }
bool Global::lineOfSight(double lat0, double long0, int h0,
                          double lat1, double long1, int h1) {
    // no local Global — use this->bin which is already loaded
    int x0, y0, x1, y1;
    auto start = chrono::steady_clock::now();

    if (!bin.getGridCoords(lat0, long0, x0, y0) ||
        !bin.getGridCoords(lat1, long1, x1, y1)) {
        cout << "no\n";
        return false;
    }

    double dx       = x1 - x0;
    double dy       = y1 - y0;
    double distance = sqrt(dx*dx + dy*dy) / RESOLUTION;

    int src_elev  = bin.getElevation(lat0, long0) + h0;
    int targ_elev = bin.getElevation(lat1, long1) + h1;

    cout << "src: "    << src_elev  << "\n";
    cout << "target: " << targ_elev << "\n";

    auto cells = Global::bresenham(x0, y0, x1, y1);
    double delta = (double)(targ_elev - src_elev) / cells.size();

    cout << "delta: " << delta
         << "  cells: " << cells.size()
         << "  dist: "  << distance << "\n";

    for (size_t i = 0; i < cells.size(); i++) {
        int lineh = src_elev + (int)(delta * i);
        int cellh = bin.elev_matrix[
            (size_t)cells[i].first * conf.grid.cols + cells[i].second];
        if (cellh > lineh) return false;
    }

    auto end   = chrono::steady_clock::now();
    int  total = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    cout << "total time: " << total << "ms\n";
    return true;
}
