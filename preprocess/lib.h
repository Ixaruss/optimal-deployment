#ifndef LIB_H_
#define LIB_H_

#include "Config.h"
#include <gdal.h>
#include <ogrsf_frmts.h>
#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <fstream>

//default conf
const int    RESOLUTION      = 100;
const int    ELEV_RESOLUTION = 100;

const double VEC_ORIGIN_X    = 2.88589e+06;
const double VEC_ORIGIN_Y    = 5.26028e+06;
const double VEC_MIN_X = 2.28137e+06;
const double VEC_MAX_Y = 5.67272e+06;

const int   DISTANCE_FROM_IB = 10000;
const int   DISTANCE_FROM_WATER = 1000;
const int   DISTANCE_FROM_RAIL = 1000;
const int   DISTANCE_FROM_ROAD = 1000;

const double ELEV_ORIGIN_X = 2653415.0;
const double ELEV_ORIGIN_Y = 5368236.0;

const int ELEV_COLS = 29308;
const int ELEV_ROWS = 32165;

const int BIT_ROAD       = 0;
const int BIT_RAIL       = 1;
const int BIT_WATER_AREA = 2;
const int BIT_WATER_LINE = 3;
const int BIT_IB         = 4;

//input files
const std::string DEFAULT_ROAD_SHP       = "roads.shp";
const std::string DEFAULT_RAIL_SHP       = "rails.shp";
const std::string DEFAULT_WATER_AREA_SHP = "water_areas.shp";
const std::string DEFAULT_WATER_LINE_SHP = "water_lines.shp";
const std::string DEFAULT_ELEVATION_FILE = "alt.tif";
const std::string DEFAULT_IB_SHP         = "ib.shp";

//output files
const std::string DEFAULT_PARAM_MATRIX     = "map.bin";
const std::string DEFAULT_ELEVATION_MATRIX = "elev.bin";
const std::string DEFAULT_FEASIBLE_MATRIX  = "feasible.bin";

namespace lib {

    int calc_rows (int resolution);

    int calc_cols (int resolution);

    std::vector<double> get_min_max_coordiantes(std::string path, int type =0);

     int refresh_layer_by_index(int bit_index, std::optional<std::string> path, const std::string& default_path);

     int layerName(Layer_vals val);

    template <typename T>
     bool writeBin(const std::string& path, const std::vector<T>& matrix){
        if (matrix.empty()) {
            std::cerr << "[bin] ERROR: matrix is empty, nothing to write\n";
            return false;
        }

        FILE* f = fopen(path.c_str(), "wb");
        if (!f) {
            std::cerr << "[bin] ERROR: could not open " << path << " for writing\n";
            return false;
        }

        size_t total_cells    = matrix.size();
        size_t total_bytes    = total_cells * sizeof(T);
        size_t written        = fwrite(matrix.data(), sizeof(T), total_cells, f);
        fclose(f);

        if (written != total_cells) {
            std::cerr << "[bin] ERROR: wrote " << written << " of " << total_cells << " cells\n";
            return false;
        }

        std::cout << "[bin] Written → " << path << " ("
                  << total_bytes / 1e6 << " MB, "
                  << total_cells << " cells, "
                  << sizeof(T) << " bytes/cell)\n";
        return true;
    }

     template <typename T>
     bool readBin(const std::string& path, std::vector<T>& matrix, size_t expected_cells){
        std::ifstream f(path, std::ios::binary);
        if (!f.is_open()) {
            std::cerr << "[bin] ERROR: could not open " << path << " for reading\n";
            return false;
        }

        // check file size matches expectation
        f.seekg(0, std::ios::end);
        size_t file_bytes    = f.tellg();
        size_t expected_bytes = expected_cells * sizeof(T);
        f.seekg(0, std::ios::beg);

        if (file_bytes != expected_bytes) {
            std::cerr << "[bin] ERROR: file size mismatch — expected "
                      << expected_bytes << " bytes, got " << file_bytes << " bytes\n";
            return false;
        }

        matrix.resize(expected_cells);
        f.read(reinterpret_cast<char*>(matrix.data()), expected_bytes);

        if ((size_t)f.gcount() != expected_bytes) {
            std::cerr << "[bin] ERROR: read " << f.gcount() << " of " << expected_bytes << " bytes\n";
            matrix.clear();
            return false;
        }

        std::cout << "[bin] Read ← " << path << " ("
                  << file_bytes / 1e6 << " MB, "
                  << expected_cells << " cells, "
                  << sizeof(T) << " bytes/cell)\n";
        return true;
    }

};

#endif
