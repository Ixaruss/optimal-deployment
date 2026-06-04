#include "../bin.h"
#include <cstdint>
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
             // get elev grid dims from config if available, else hardcoded constants
             size_t elev_rows, elev_cols;
             if (Config::is_available()) {
                 Config cfg = Config::load();
                 elev_rows  = (size_t)cfg.grid.rows;
                 elev_cols  = (size_t)cfg.grid.cols;
             } else {
                 elev_rows  = (size_t)ELEV_ROWS;
                 elev_cols  = (size_t)ELEV_COLS;
             }

             size_t total_elev_cells = elev_rows * elev_cols;
             size_t expected_bytes   = total_elev_cells * sizeof(int16_t);

             std::cout << "[lazy-load] Loading " << elev_path << " into memory ("
                       << expected_bytes / 1e6 << " MB, "
                       << total_elev_cells << " cells, "
                       << sizeof(int16_t) << " bytes/cell)...\n";

             std::ifstream file(elev_path, std::ios::binary);
             if (!file.is_open()) {
                 std::cerr << "[lazy-load] ERROR: could not open " << elev_path << "\n";
                 return false;
             }

             // verify file size before reading
             file.seekg(0, std::ios::end);
             size_t file_bytes = (size_t)file.tellg();
             file.seekg(0, std::ios::beg);

             if (file_bytes != expected_bytes) {
                 std::cerr << "[lazy-load] ERROR: size mismatch reading " << elev_path
                           << " — expected " << expected_bytes / 1e6
                           << " MB, got "    << file_bytes   / 1e6 << " MB\n"
                           << "  Hint: elev.bin may be old int16 format — rebuild with build_elevation_matrix()\n";
                 return false;
             }

             elev_matrix.resize(total_elev_cells);
             file.read(reinterpret_cast<char*>(elev_matrix.data()), expected_bytes);

             if ((size_t)file.gcount() != expected_bytes) {
                 std::cerr << "[lazy-load] ERROR: read " << file.gcount()
                           << " of " << expected_bytes << " bytes from " << elev_path << "\n";
                 elev_matrix.clear();
                 return false;
             }

             file.close();
             elevation_loaded = true;
             std::cout << "[lazy-load] Elevation loaded — "
                       << elev_rows << " x " << elev_cols << "\n";
         }

     return vector_loaded && elevation_loaded;
 }


 bool Bin::getGridCoords(double lat, double lng, int& out_r, int& out_c) {
     if (!poCT || !ensure_matrices_loaded()) return false;

     double x = lng;
     double y = lat;

     if (!poCT->Transform(1, &x, &y)) return false;

     out_c = std::floor((x - conf.grid.origin_x) / RESOLUTION);
     out_r = std::floor((conf.grid.origin_y - y) / RESOLUTION);

     return (out_r >= 0 && out_r < conf.grid.rows && out_c >= 0 && out_c < conf.grid.cols);
 }

 int16_t Bin::getElevation(double lat, double lng) {
     if (!poCT || !ensure_matrices_loaded()) return -9999;

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

 // ensure fseeko uses 64-bit offset on Linux (must be before any system headers,
 // but since bin.h is already included we set it here as a guard — works on most distros)
 #ifndef _FILE_OFFSET_BITS
   #define _FILE_OFFSET_BITS 64
 #endif

 FeasibleGrid Bin::getFeasibleGrid(short op, short opt, short limit_ram) {
     FeasibleGrid result;

     if (!ensure_matrices_loaded()) {
         std::cerr << "[feasible] ERROR: matrices not loaded\n";
         return result;
     }

     int r_min, c_min, r_max, c_max;
     bool use_polygon = false;
     std::vector<std::pair<double,double>> proj_poly;

     if (opt == 1) {
         r_min = 0; c_min = 0;
         r_max = VEC_ROWS; c_max = VEC_COLS;
         std::cout << "[feasible] opt=1 — scanning entire grid: "
                   << VEC_ROWS << " x " << VEC_COLS << "\n";
     } else {
         BoundingBox mbr = getMinimumBoundingBox();
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

         proj_poly.reserve(conf.area_op.size());
         for (auto& [lat, lon] : conf.area_op) {
             double px, py;
             if (!latlon_to_projected(lat, lon, px, py)) {
                 std::cerr << "[feasible] ERROR: failed to project polygon vertex\n";
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
     int no_infra_count = 0;
     int slope_count    = 0;

     std::cout << "[feasible] Scanning "
               << (long)(r_max - r_min) * (c_max - c_min) << " cells...\n";
     std::cout << "[feasible] RAM mode: " << (limit_ram ? "chunked" : "full-load") << "\n";

     if (limit_ram) {
         // ── chunked mode ───────────────────────────────────────────────────
         // elev_matrix is NOT used — read elev.bin row-chunk by row-chunk
         // result.cells is NOT accumulated — write directly to feasible.bin
         // RAM: vec_matrix (1GB) + one chunk of elev rows + small write buffer

         string elev_path;
         int E_ROWS, E_COLS;
         double ELEV_OX, ELEV_OY;
         if (Config::is_available()) {
             Config cfg = Config::load();
             elev_path  = cfg.output.elevation_file;
             E_ROWS     = cfg.grid.rows;
             E_COLS     = cfg.grid.cols;
             ELEV_OX    = cfg.grid.origin_x;
             ELEV_OY    = cfg.grid.origin_y;
         } else {
             elev_path  = DEFAULT_ELEVATION_MATRIX;
             E_ROWS     = ELEV_ROWS;
             E_COLS     = ELEV_COLS;
             ELEV_OX    = ELEV_ORIGIN_X;
             ELEV_OY    = ELEV_ORIGIN_Y;
         }

         FILE* elev_f = fopen(elev_path.c_str(), "rb");
         if (!elev_f) {
             std::cerr << "[feasible] ERROR: cannot open " << elev_path
                       << " for chunked read\n";
             return result;
         }

         // open feasible bin for streaming write if WRITE mode
         FILE* out_f = nullptr;
         string out_path = (Config::is_available())
             ? Config::load().output.feasible
             : DEFAULT_FEASIBLE_MATRIX;

         if (op == WRITE) {
             out_f = fopen(out_path.c_str(), "wb");
             if (!out_f) {
                 std::cerr << "[feasible] ERROR: cannot open " << out_path << " for write\n";
                 fclose(elev_f);
                 return result;
             }
         }

         // chunk size — 500 rows of elev at a time
         // at 100m India grid: 500 × 33242 × 6 bytes ≈ 100MB per chunk
         const int CHUNK_ROWS = 500;
         std::vector<ElevCell> elev_chunk;
         size_t total_feasible = 0;

         // write buffer — flush to disk every N cells to avoid large in-memory accumulation
         const size_t WRITE_BUF_SIZE = 100000;
         std::vector<FeasibleCell> write_buf;
         if (op == WRITE) write_buf.reserve(WRITE_BUF_SIZE);

         auto flush_write_buf = [&]() {
             if (out_f && !write_buf.empty()) {
                 fwrite(write_buf.data(), sizeof(FeasibleCell),
                        write_buf.size(), out_f);
                 write_buf.clear();
             }
         };

         for (int r = r_min; r < r_max; r++) {

             // load elev chunk when needed — map r to elev row
             double proj_y   = VEC_ORIGIN_Y - (r * RESOLUTION);
             int    elev_r   = (int)std::floor((ELEV_OY - proj_y) / RESOLUTION);

             // which chunk does elev_r belong to?
             int chunk_start = (elev_r / CHUNK_ROWS) * CHUNK_ROWS;
             int chunk_end   = std::min(chunk_start + CHUNK_ROWS, E_ROWS);
             int chunk_size  = chunk_end - chunk_start;

             // reload chunk if needed
             if (elev_chunk.empty() ||
                 elev_r < chunk_start ||
                 elev_r >= (int)(chunk_start + elev_chunk.size() / E_COLS)) {

                 elev_chunk.resize((size_t)chunk_size * E_COLS);
                 size_t byte_offset = (size_t)chunk_start * E_COLS * sizeof(ElevCell);
                 // 64-bit safe seek — fseek(long) overflows on files >2GB on Windows
                 #ifdef _WIN32
                     _fseeki64(elev_f, (int64_t)byte_offset, SEEK_SET);
                 #else
                     fseeko(elev_f, (off_t)byte_offset, SEEK_SET);
                 #endif
                 size_t read = fread(elev_chunk.data(), sizeof(ElevCell),
                                     (size_t)chunk_size * E_COLS, elev_f);
                 if (read != (size_t)chunk_size * E_COLS) {
                     std::cerr << "[feasible] WARN: short read on elev chunk at row "
                               << chunk_start << "\n";
                 }

                 if (r % 5000 == 0 && r > r_min)
                     std::cout << "[feasible] chunk loaded: elev rows ["
                               << chunk_start << " → " << chunk_end << "]\n";
             }

             for (int c = c_min; c < c_max; c++) {

                 if (use_polygon) {
                     double cx = VEC_ORIGIN_X + (c + 0.5) * RESOLUTION;
                     double cy = VEC_ORIGIN_Y - (r + 0.5) * RESOLUTION;
                     if (!Bin::point_in_polygon(cx, cy, proj_poly)) {
                         outside_count++; continue;
                     }
                 }

                 uint8_t cell = vec_matrix[(size_t)r * VEC_COLS + c];

                 if (flag_water_line && (cell & (1 << BIT_WATER_LINE))) { water_count++; continue; }
                 if (flag_water_area && (cell & (1 << BIT_WATER_AREA))) { water_count++; continue; }
                 if (flag_ib         && (cell & (1 << BIT_IB)))         { ib_count++;    continue; }

                 // slope from chunk
                 double proj_x   = VEC_ORIGIN_X + (c * RESOLUTION);
                 int    elev_c   = (int)std::floor((proj_x - ELEV_OX) / RESOLUTION);
                 int    local_r  = elev_r - chunk_start;

                 if (elev_r  >= 0 && elev_r  < E_ROWS &&
                     elev_c  >= 0 && elev_c  < E_COLS &&
                     local_r >= 0 && local_r < chunk_size) {
                     float slope = elev_chunk[(size_t)local_r * E_COLS + elev_c].slope;
                     if (slope != -9999.0f && slope > 10.0f) {
                         slope_count++; continue;
                     }
                 }

                 if (need_infra) {
                     bool road_ok = flag_road && (cell & (1 << BIT_ROAD));
                     bool rail_ok = flag_rail && (cell & (1 << BIT_RAIL));
                     if (!road_ok && !rail_ok) { no_infra_count++; continue; }
                 }

                 total_feasible++;

                 if (op == WRITE) {
                     write_buf.push_back({ r, c, cell });
                     if (write_buf.size() >= WRITE_BUF_SIZE) flush_write_buf();
                 } else {
                     result.cells.push_back({ r, c, cell });
                 }
             }
         }

         flush_write_buf();   // flush remainder
         if (elev_f) fclose(elev_f);
         if (out_f)  fclose(out_f);

         result.cells.shrink_to_fit();
         std::cout << "[feasible] Feasible (chunked): " << total_feasible
                   << " / " << (long)(r_max - r_min) * (c_max - c_min) << "\n";
         if (op == WRITE)
             std::cout << "[feasible] Written → " << out_path << "\n";

     } else {
         for (int r = r_min; r < r_max; r++) {
             for (int c = c_min; c < c_max; c++) {

                 if (use_polygon) {
                     double cx = VEC_ORIGIN_X + (c + 0.5) * RESOLUTION;
                     double cy = VEC_ORIGIN_Y - (r + 0.5) * RESOLUTION;
                     if (!Bin::point_in_polygon(cx, cy, proj_poly)) {
                         outside_count++; continue;
                     }
                 }

                 uint8_t cell = vec_matrix[(size_t)r * VEC_COLS + c];

                 if (flag_water_line && (cell & (1 << BIT_WATER_LINE))) { water_count++; continue; }
                 if (flag_water_area && (cell & (1 << BIT_WATER_AREA))) { water_count++; continue; }
                 if (flag_ib         && (cell & (1 << BIT_IB)))         { ib_count++;    continue; }

                 if (elevation_loaded) {
                     double proj_x = VEC_ORIGIN_X + (c * RESOLUTION);
                     double proj_y = VEC_ORIGIN_Y - (r * RESOLUTION);
                     double ELEV_OX, ELEV_OY;
                     int E_ROWS, E_COLS;
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
                     int elev_c = (int)std::floor((proj_x - ELEV_OX) / RESOLUTION);
                     int elev_r = (int)std::floor((ELEV_OY - proj_y)  / RESOLUTION);
                     if (elev_r >= 0 && elev_r < E_ROWS &&
                         elev_c >= 0 && elev_c < E_COLS) {
                         float slope = elev_matrix[(size_t)elev_r * E_COLS + elev_c]; // to-do: give proper slope location if in future func is used
                         if (slope != -9999.0f && slope > 10.0f) {
                             slope_count++; continue;
                         }
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
     }

     std::cout << "[feasible] Outside polygon : " << outside_count  << "\n";
     std::cout << "[feasible] Water cells     : " << water_count    << "\n";
     std::cout << "[feasible] IB cells        : " << ib_count       << "\n";
     std::cout << "[feasible] Slope rejected  : " << slope_count    << "\n";
     std::cout << "[feasible] No infra        : " << no_infra_count << "\n";
     std::cout << "[feasible] Feasible        : " << result.cells.size()
               << " / " << (long)(r_max - r_min) * (c_max - c_min) << "\n";

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
