#include "../common.h"
#include <cstddef>
#include <chrono>
using namespace std;
bool Global::lineOfSight(double lat0, double long0,int h0, double lat1, double long1,int h1) {
    Global g;
    int x0, y0;
    int x1, y1;
     auto start = chrono::steady_clock::now();
    if(!g.bin.getGridCoords(lat0, long0, x0, y0) || !g.bin.getGridCoords(lat1, long1, x1, y1)){
        cout << "no" << endl;
        return false;
    }

    int src_elev = g.bin.getElevation(lat0, long0) + h0;
    int targ_elev = g.bin.getElevation(lat1, long1) + h1;

    cout << "src: " << src_elev << endl;
    cout << "target: " << targ_elev << endl;

    auto cells = Global::bresenham(x0, y0, x1, y1);
    auto end = chrono::steady_clock::now();
    int total = chrono::duration_cast<chrono::milliseconds>(end - start).count();

    cout << "total time: " <<total << endl;
    double delta = (double)(targ_elev - src_elev) /cells.size();
    cout << "delta:" << delta << " " <<cells.size() << endl;

    for (size_t i = 0; i < cells.size(); i++)
    {
        int lineh = src_elev + (delta * i);
        int cellh = g.bin.elev_matrix[(size_t)cells[i].first * g.conf.grid.cols + cells[i].second];
        // cout << lineh << " , "<< cellh << endl;
        if(cellh > lineh){
            // auto end = chrono::steady_clock::now();
            // int total = chrono::duration_cast<chrono::milliseconds>(end - start).count();
            // cout << "total time: " <<total << endl;
            return false;
        }
    }
    // auto end = chrono::steady_clock::now();
    // int total = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    cout << "total time: " <<total << endl;
    return true;
}
