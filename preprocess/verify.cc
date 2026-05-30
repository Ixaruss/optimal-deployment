#include "./bin.h"

void Bin::get_elevation_color(int16_t alt, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (alt == -9999) {
            r = 10;  g = 25;  b = 60;    // nodata → deep blue
        } else if (alt == 0) {
            r = 20;  g = 24;  b = 38;    // exact zero → dark (outside DEM)
        } else if (alt < 200) {
            r = 34;  g = 120; b = 45;
        }else if (alt < 800) {
        r = 140; g = 175; b = 65;
    } else if (alt < 2000) {
        r = 165; g = 115; b = 60;
    } else if (alt < 4000) {
        r = 95;  g = 65;  b = 40;
    } else {
        r = 240; g = 240; b = 245;
    }
}
