#include "../bin.h"
#include <vector>

using namespace std;
using namespace lib;

bool Bin::ensure_matrices_loaded() {
    string param_path,elev_path;
    if(Config::is_available()) {
        Config conf = Config::load();

        param_path = conf.output.matrix_file;
        elev_path = conf.output.elevation_file;
    }else {
        param_path = DEFAULT_PARAM_MATRIX;
        elev_path = DEFAULT_ELEVATION_MATRIX;
    }
     if (VEC_ROWS == 0 || VEC_COLS == 0) {
         VEC_ROWS = calc_rows(RESOLUTION);
         VEC_COLS = calc_cols(RESOLUTION);
     }
     if (!vector_loaded) {
         size_t total_vec_cells = (size_t)VEC_ROWS * VEC_COLS;
         std::cout << "[lazy-load] Loading "<<param_path<<" into memory (" << total_vec_cells / 1e6 << " MB)...\n";

         std::ifstream file(param_path, std::ios::binary);
         if (!file.is_open()) {
             std::cerr << "[lazy-load] ERROR: Could not open"<< param_path <<"from disk!\n";
             return false;
         }

         vec_matrix.resize(total_vec_cells);
         file.read(reinterpret_cast<char*>(vec_matrix.data()), total_vec_cells);
         if (file.gcount() != (std::streamsize)total_vec_cells) {
             std::cerr << "[lazy-load] ERROR: Size mismatch while reading "<< param_path <<"\n";
             vec_matrix.clear();
             return false;
         }
         file.close();
         vector_loaded = true;
     }

     if (!elevation_loaded) {
         size_t total_elev_cells = (size_t)ELEV_ROWS * ELEV_COLS;
         std::cout << "[lazy-load] Loading "<< elev_path <<"into memory (" << (total_elev_cells * sizeof(int16_t)) / 1e6 << " MB)...\n";

         std::ifstream file(elev_path , std::ios::binary);
         if (!file.is_open()) {
             std::cerr << "[lazy-load] ERROR: Could not open "<< elev_path <<"from disk!\n";
             return false;
         }

         elev_matrix.resize(total_elev_cells);
         file.read(reinterpret_cast<char*>(elev_matrix.data()), total_elev_cells * sizeof(int16_t));
         if (file.gcount() != (std::streamsize)(total_elev_cells * sizeof(int16_t))) {
             std::cerr << "[lazy-load] ERROR: Size mismatch while reading "<< elev_path <<"\n";
             elev_matrix.clear();
             return false;
         }
         file.close();
         elevation_loaded = true;
     }

     return vector_loaded && elevation_loaded;
 }


 bool Bin::getGridCoords(double lat, double lng, int& out_r, int& out_c) {
     if (!poCT || !ensure_matrices_loaded()) return false;

     double x = lng;
     double y = lat;

     if (!poCT->Transform(1, &x, &y)) return false;

     out_c = std::floor((x - VEC_ORIGIN_X) / RESOLUTION);
     out_r = std::floor((VEC_ORIGIN_Y - y) / RESOLUTION);

     return (out_r >= 0 && out_r < VEC_ROWS && out_c >= 0 && out_c < VEC_COLS);
 }

 // Extracts elevation data value in meters
 int16_t Bin::getElevation(double lat, double lng) {
     if (!poCT || !ensure_matrices_loaded()) return -9999;

     double x = lng;
     double y = lat;
     if (!poCT->Transform(1, &x, &y)) return -9999;

     int elev_c = std::floor((x - ELEV_ORIGIN_X) / RESOLUTION);
     int elev_r = std::floor((ELEV_ORIGIN_Y - y) / RESOLUTION);

     if (elev_r >= 0 && elev_r < ELEV_ROWS && elev_c >= 0 && elev_c < ELEV_COLS) {
         size_t idx = (size_t)elev_r * ELEV_COLS + elev_c;
         return elev_matrix[idx];
     }
     return -9999;
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
 // Generates a comprehensive descriptive overview string of all layer interactions
 vector<int> Bin::getStatusAll(double lat, double lng) {
     int r, c;
     vector<int> res;
     if (!getGridCoords(lat, lng, r, c)) return res;

     uint8_t cell_val = vec_matrix[(size_t)r * VEC_COLS + c];
     int16_t alt = getElevation(lat, lng);

    res.push_back(cell_val & (1 << BIT_ROAD));
    res.push_back(cell_val & (1 << BIT_RAIL));
    res.push_back(cell_val & (1 << BIT_WATER_AREA));
    res.push_back(cell_val & (1 << BIT_WATER_LINE));

     return res;
 }

 bool Bin::latlon_to_projected(double lat, double lon, double& px, double& py) {
     px = lon; py = lat;   // poCT expects X=lon, Y=lat
     if (!poCT->Transform(1, &px, &py)) {
         std::cerr << "[proj] ERROR: could not transform (" << lat << ", " << lon << ")\n";
         return false;
     }
     return true;
 }

 // free function — no class state needed
bool Bin::point_in_polygon(double px, double py,
                               const std::vector<std::pair<double,double>>& proj_poly) {
     int  n      = proj_poly.size();
     bool inside = false;
     for (int i = 0, j = n - 1; i < n; j = i++) {
         double xi = proj_poly[i].first,  yi = proj_poly[i].second;
         double xj = proj_poly[j].first,  yj = proj_poly[j].second;
         bool crosses_y = ((yi > py) != (yj > py));
         if (crosses_y) {
             double x_intersect = (xj - xi) * (py - yi) / (yj - yi) + xi;
             if (px < x_intersect) inside = !inside;
         }
     }
     return inside;
 }

 BoundingBox Bin::getMinimumBoundingBox() {
     if (conf.area_op.size() < 3) {
         std::cerr << "[mbr] ERROR: area_op must have at least 3 points\n";
         return { 0, 0, 0, 0 };
     }
     double min_lat =  1e9, max_lat = -1e9;
     double min_lon =  1e9, max_lon = -1e9;
     for (auto& [lat, lon] : conf.area_op) {
         min_lat = std::min(min_lat, lat);
         max_lat = std::max(max_lat, lat);
         min_lon = std::min(min_lon, lon);
         max_lon = std::max(max_lon, lon);
     }
     std::cout << "[mbr] BoundingBox → lat: [" << min_lat << " → " << max_lat
               << "] lon: [" << min_lon << " → " << max_lon << "]\n";
     return { min_lat, max_lat, min_lon, max_lon };
 }

 FeasibleGrid Bin::getFeasibleGrid(short op) {
     FeasibleGrid result;

     if (!ensure_matrices_loaded()) {
         std::cerr << "[feasible] ERROR: matrices not loaded\n";
         return result;
     }

     // --- 1. AABB for loop bounds ---
     BoundingBox mbr = getMinimumBoundingBox();
     int r_min, c_min, r_max, c_max;
     if (!getGridCoords(mbr.max_lat, mbr.min_lon, r_min, c_min)) {
         std::cerr << "[feasible] ERROR: top-left corner out of grid\n";
         return result;
     }
     if (!getGridCoords(mbr.min_lat, mbr.max_lon, r_max, c_max)) {
         std::cerr << "[feasible] ERROR: bottom-right corner out of grid\n";
         return result;
     }
     std::cout << "[feasible] AABB → rows: [" << r_min << " → " << r_max
               << "] cols: [" << c_min << " → " << c_max << "]\n";

     // --- 2. project polygon vertices once before loop ---
     std::vector<std::pair<double,double>> proj_poly;
     proj_poly.reserve(conf.area_op.size());
     for (auto& [lat, lon] : conf.area_op) {
         double px, py;
         if (!latlon_to_projected(lat, lon, px, py)) {
             std::cerr << "[feasible] ERROR: failed to project polygon vertex\n";
             return result;
         }
         proj_poly.push_back({ px, py });
     }

     // --- 3. flag logic ---
     bool need_infra = flag_road || flag_rail;

     result.origin_r = r_min;
     result.origin_c = c_min;
     result.height   = r_max - r_min;
     result.width    = c_max - c_min;

     int outside_count  = 0;
     int water_count    = 0;
     int ib_count       = 0;
     int no_infra_count = 0;

     std::cout << "[feasible] Scanning "
               << (long)(r_max - r_min) * (c_max - c_min) << " cells...\n";

     for (int r = r_min; r < r_max; r++) {
         for (int c = c_min; c < c_max; c++) {

             // cell center in projected coords — same space as proj_poly
             double cx = VEC_ORIGIN_X + (c + 0.5) * RESOLUTION;
             double cy = VEC_ORIGIN_Y - (r + 0.5) * RESOLUTION;

             if (!Bin::point_in_polygon(cx, cy, proj_poly)) {
                 outside_count++;
                 continue;
             }

             uint8_t cell = vec_matrix[(size_t)r * VEC_COLS + c];

             // exclusion bits — only block if the corresponding flag is true
             if (flag_water_line && (cell & (1 << BIT_WATER_LINE))) { water_count++; continue; }
             if (flag_water_area && (cell & (1 << BIT_WATER_AREA))) { water_count++; continue; }
             if (flag_ib         && (cell & (1 << BIT_IB)))         { ib_count++;    continue; }

             // infra requirement — only if at least one infra flag is on
             if (need_infra) {
                 bool road_ok = flag_road && (cell & (1 << BIT_ROAD));
                 bool rail_ok = flag_rail && (cell & (1 << BIT_RAIL));
                 if (!road_ok && !rail_ok) { no_infra_count++; continue; }
             }

             result.cells.push_back({ r, c, cell });
         }
     }

     std::cout << "[feasible] Outside polygon : " << outside_count  << "\n";
     std::cout << "[feasible] Water cells     : " << water_count    << "\n";
     std::cout << "[feasible] IB cells        : " << ib_count       << "\n";
     std::cout << "[feasible] No infra        : " << no_infra_count << "\n";
     std::cout << "[feasible] Feasible        : " << result.cells.size()
               << " / " << (long)(r_max - r_min) * (c_max - c_min) << "\n";

     if (op == WRITE)
         writeBin("./feasable.bin", result.cells);

     return result;
 }

 int Bin::getTerrianResolution() {
     if(!Config::is_available())
         return RESOLUTION;
    return conf.grid.resolution;
 }

 pair<double,double> Bin::getMapOrigins(){
     if(!Config::is_available())
         return { VEC_ORIGIN_X, VEC_ORIGIN_Y };
     return { conf.grid.origin_x, conf.grid.origin_y };
 }
