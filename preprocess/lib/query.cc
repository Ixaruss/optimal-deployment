#include "../bin.h"
#include <vector>

using namespace std;
using namespace lib;

bool Bin::getGridCoords(double lat, double lng, int& out_r, int& out_c) {
    init({ CONSTRAINTS });
    if (!vector_loaded || !poCT) return false;

    double x = lng, y = lat;
    if (!poCT->Transform(1, &x, &y)) return false;

    out_c = (int)std::floor((x - conf.grid.origin_x) / RESOLUTION);
    out_r = (int)std::floor((conf.grid.origin_y - y)  / RESOLUTION);

    return (out_r >= 0 && out_r < VEC_ROWS &&
            out_c >= 0 && out_c < VEC_COLS);
}

int16_t Bin::getElevation(double lat, double lng) {
    init({ ELEVATION });
    if (!elevation_loaded || !poCT) return -9999;

    double x = lng, y = lat;
    if (!poCT->Transform(1, &x, &y)) return -9999;

    double ELEV_OX, ELEV_OY;
    int    E_ROWS, E_COLS;
    if (Config::is_available()) {
        Config cfg = Config::load();
        ELEV_OX = cfg.grid.origin_x;
        ELEV_OY = cfg.grid.origin_y;
        E_ROWS  = cfg.grid.rows;
        E_COLS  = cfg.grid.cols;
    } else {
        ELEV_OX = ELEV_ORIGIN_X;
        ELEV_OY = ELEV_ORIGIN_Y;
        E_ROWS  = ELEV_ROWS;
        E_COLS  = ELEV_COLS;
    }

    int elev_c = (int)std::floor((x - ELEV_OX) / RESOLUTION);
    int elev_r = (int)std::floor((ELEV_OY - y)  / RESOLUTION);

    if (elev_r >= 0 && elev_r < E_ROWS &&
        elev_c >= 0 && elev_c < E_COLS) {
        return elev_matrix[(size_t)elev_r * E_COLS + elev_c];
    }
    return -9999;
}

bool Bin::getSlope(double lat, double lng) {
    int r, c;
    if (!getGridCoords(lat, lng, r, c)) return false;
    return (vec_matrix[(size_t)r * VEC_COLS + c] & (1 << BIT_SLOPE)) != 0;
}

bool Bin::getStatusRoad(double lat, double lng) {
    int r, c;
    if (!getGridCoords(lat, lng, r, c)) return false;
    return (vec_matrix[(size_t)r * VEC_COLS + c] & (1 << BIT_ROAD)) != 0;
}

bool Bin::getStatusRail(double lat, double lng) {
    int r, c;
    if (!getGridCoords(lat, lng, r, c)) return false;
    return (vec_matrix[(size_t)r * VEC_COLS + c] & (1 << BIT_RAIL)) != 0;
}

bool Bin::getStatusWaterArea(double lat, double lng) {
    int r, c;
    if (!getGridCoords(lat, lng, r, c)) return false;
    return (vec_matrix[(size_t)r * VEC_COLS + c] & (1 << BIT_WATER_AREA)) != 0;
}

bool Bin::getStatusWaterLine(double lat, double lng) {
    int r, c;
    if (!getGridCoords(lat, lng, r, c)) return false;
    return (vec_matrix[(size_t)r * VEC_COLS + c] & (1 << BIT_WATER_LINE)) != 0;
}

bool Bin::getStatusIB(double lat, double lng) {
    int r, c;
    if (!getGridCoords(lat, lng, r, c)) return false;
    return (vec_matrix[(size_t)r * VEC_COLS + c] & (1 << BIT_IB)) != 0;
}

vector<int> Bin::getStatusAll(double lat, double lng) {
    init({});   // vec + elev
    vector<int> res;
    int r, c;
    if (!getGridCoords(lat, lng, r, c)) return res;

    uint8_t cell = vec_matrix[(size_t)r * VEC_COLS + c];
    res.push_back((cell >> BIT_ROAD)       & 1);
    res.push_back((cell >> BIT_RAIL)       & 1);
    res.push_back((cell >> BIT_WATER_AREA) & 1);
    res.push_back((cell >> BIT_WATER_LINE) & 1);
    res.push_back((cell >> BIT_IB)         & 1);
    res.push_back((cell >> BIT_SLOPE)      & 1);
    return res;
}

bool Bin::latlon_to_projected(double lat, double lon, double& px, double& py) {
    px = lon; py = lat;
    if (!poCT->Transform(1, &px, &py)) {
        cerr << "[proj] ERROR: could not transform ("
             << lat << ", " << lon << ")\n";
        return false;
    }
    return true;
}

bool Bin::point_in_polygon(double px, double py,
                            const vector<pair<double,double>>& proj_poly) {
    int  n      = (int)proj_poly.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = proj_poly[i].first,  yi = proj_poly[i].second;
        double xj = proj_poly[j].first,  yj = proj_poly[j].second;
        if ((yi > py) != (yj > py)) {
            double x_intersect = (xj - xi) * (py - yi) / (yj - yi) + xi;
            if (px < x_intersect) inside = !inside;
        }
    }
    return inside;
}

BoundingBox Bin::getMinimumBoundingBox() {
    if (conf.area_op.size() < 3) {
        cerr << "[mbr] ERROR: area_op must have at least 3 points\n";
        return { 0, 0, 0, 0 };
    }
    double min_lat =  1e9, max_lat = -1e9;
    double min_lon =  1e9, max_lon = -1e9;
    for (auto& [lat, lon] : conf.area_op) {
        min_lat = min(min_lat, lat);
        max_lat = max(max_lat, lat);
        min_lon = min(min_lon, lon);
        max_lon = max(max_lon, lon);
    }
    cout << "[mbr] BoundingBox → lat: [" << min_lat << " → " << max_lat
         << "] lon: [" << min_lon << " → " << max_lon << "]\n";
    return { min_lat, max_lat, min_lon, max_lon };
}

#ifndef _FILE_OFFSET_BITS
  #define _FILE_OFFSET_BITS 64
#endif

FeasibleGrid Bin::getFeasibleGrid(short op, short opt, short limit_ram) {
    FeasibleGrid result;

    // load vec + elev + slope
    init({ ALL });
    if (!vector_loaded) {
        cerr << "[feasible] ERROR: vec_matrix not loaded\n";
        return result;
    }

    // ── load slope.bin locally if not already cached ───────────────────────
    string slope_path = Config::is_available()
        ? Config::load().output.slope_file
        : DEFAULT_SLOPE_MATRIX;

    if (!slope_loaded) {
        size_t slope_n        = (size_t)VEC_ROWS * VEC_COLS;
        size_t expected_bytes = slope_n * sizeof(float);
        ifstream sf(slope_path, ios::binary);
        if (!sf.is_open()) {
            cerr << "[feasible] WARN: cannot open " << slope_path
                 << " — slope filter disabled\n";
        } else {
            sf.seekg(0, ios::end);
            size_t file_bytes = (size_t)sf.tellg();
            sf.seekg(0, ios::beg);
            if (file_bytes != expected_bytes) {
                cerr << "[feasible] WARN: slope.bin size mismatch — slope disabled\n";
            } else {
                slope_matrix.resize(slope_n);
                sf.read(reinterpret_cast<char*>(slope_matrix.data()), expected_bytes);
                if ((size_t)sf.gcount() == expected_bytes) {
                    slope_loaded = true;
                    cout << "[feasible] Slope loaded: " << file_bytes / 1e6 << " MB\n";
                } else {
                    cerr << "[feasible] WARN: short read on slope.bin — slope disabled\n";
                    slope_matrix.clear();
                }
            }
        }
    }

    // ── grid bounds ────────────────────────────────────────────────────────
    int r_min, c_min, r_max, c_max;
    bool use_polygon = false;
    vector<pair<double,double>> proj_poly;

    if (opt == 1) {
        r_min = 0; c_min = 0;
        r_max = VEC_ROWS; c_max = VEC_COLS;
        cout << "[feasible] opt=1 — scanning entire grid: "
             << VEC_ROWS << " x " << VEC_COLS << "\n";
    } else {
        BoundingBox mbr = getMinimumBoundingBox();
        if (!getGridCoords(mbr.max_lat, mbr.min_lon, r_min, c_min)) {
            cerr << "[feasible] ERROR: top-left corner out of grid\n";
            return result;
        }
        if (!getGridCoords(mbr.min_lat, mbr.max_lon, r_max, c_max)) {
            cerr << "[feasible] ERROR: bottom-right corner out of grid\n";
            return result;
        }
        cout << "[feasible] AABB → rows: [" << r_min << " → " << r_max
             << "] cols: [" << c_min << " → " << c_max << "]\n";

        proj_poly.reserve(conf.area_op.size());
        for (auto& [lat, lon] : conf.area_op) {
            double px, py;
            if (!latlon_to_projected(lat, lon, px, py)) {
                cerr << "[feasible] ERROR: failed to project polygon vertex\n";
                return result;
            }
            proj_poly.push_back({ px, py });
        }
        use_polygon = true;
    }

    bool need_infra = flag_road || flag_rail;

    result.origin_r = r_min;
    result.origin_c = c_min;
    result.height   = r_max - r_min;
    result.width    = c_max - c_min;

    int outside_count  = 0;
    int water_count    = 0;
    int ib_count       = 0;
    int slope_count    = 0;
    int no_infra_count = 0;

    cout << "[feasible] Scanning "
         << (long)(r_max - r_min) * (c_max - c_min) << " cells...\n";

    for (int r = r_min; r < r_max; r++) {
        for (int c = c_min; c < c_max; c++) {

            if (use_polygon) {
                double cx = conf.grid.origin_x + (c + 0.5) * RESOLUTION;
                double cy = conf.grid.origin_y - (r + 0.5) * RESOLUTION;
                if (!Bin::point_in_polygon(cx, cy, proj_poly)) {
                    outside_count++; continue;
                }
            }

            uint8_t cell = vec_matrix[(size_t)r * VEC_COLS + c];

            if (flag_water_line && (cell & (1 << BIT_WATER_LINE))) { water_count++; continue; }
            if (flag_water_area && (cell & (1 << BIT_WATER_AREA))) { water_count++; continue; }
            if (flag_ib         && (cell & (1 << BIT_IB)))         { ib_count++;    continue; }

            if (slope_loaded) {
                float s = slope_matrix[(size_t)r * VEC_COLS + c];
                if (s != -9999.0f && s > 10.0f) {
                    slope_count++; continue;
                }
            }

            if (need_infra) {
                bool road_ok = flag_road && (cell & (1 << BIT_ROAD));
                bool rail_ok = flag_rail && (cell & (1 << BIT_RAIL));
                if (!road_ok && !rail_ok) { no_infra_count++; continue; }
            }

            result.cells.push_back({ r, c, cell });
        }
    }

    if (op == WRITE)
        writeBin("./feasable.bin", result.cells);

    cout << "[feasible] Outside polygon : " << outside_count  << "\n";
    cout << "[feasible] Water cells     : " << water_count    << "\n";
    cout << "[feasible] IB cells        : " << ib_count       << "\n";
    cout << "[feasible] Slope rejected  : " << slope_count    << "\n";
    cout << "[feasible] No infra        : " << no_infra_count << "\n";
    cout << "[feasible] Feasible        : " << result.cells.size()
         << " / " << (long)(r_max - r_min) * (c_max - c_min) << "\n";

    return result;
}

int Bin::getTerrianResolution() {
    if (!Config::is_available()) return RESOLUTION;
    return conf.grid.resolution;
}

pair<double,double> Bin::getMapOrigins() {
    if (!Config::is_available()) return { VEC_ORIGIN_X, VEC_ORIGIN_Y };
    return { conf.grid.origin_x, conf.grid.origin_y };
}
