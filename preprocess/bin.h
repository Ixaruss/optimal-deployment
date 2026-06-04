#ifndef BIN_H_
#define BIN_H_


#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <iostream>
#include <fstream>
#include <optional>
#include <vector>
#include <cstdint>
#include <cmath>
#include <string>
#include "./lib.h"

// Global constants
const bool WRITE = 1;
const bool IN_MEM = 0;
const bool USE_MULTI_THREADS = 1;
const bool FULL_MATRIX = 1;
const bool LIMIT_MEMORY = 1;
const bool COMBINE = 0;

struct BoundingBox {
    double min_lat;
    double max_lat;
    double min_lon;
    double max_lon;
};

#pragma pack(1)
struct ElevCell {
    int16_t elevation;
    float   slope;
};
#pragma pack()

struct FeasibleCell {
    int r;
    int c;
    uint8_t value;
};

struct FeasibleGrid {
    std::vector<FeasibleCell> cells;
    int origin_r;    // top-left of AABB in original matrix
    int origin_c;
    int height;      // jAABB dimensions in cells
    int width;
};
class Bin {
    // In-memory data caches
    public: std::vector<uint8_t> vec_matrix;
    public: std::vector<int16_t> elev_matrix;
    public: Config conf;


    // Loading tracking flags
    public: bool vector_loaded = false;
    public: bool elevation_loaded = false;

    public: int VEC_ROWS = 0;
    public: int VEC_COLS = 0;

    public: bool flag_road = false;
    public: bool flag_rail = false;
    public: bool flag_water_line = true;
    public: bool flag_water_area = true;
    public: bool flag_ib = true;

    public: BoundingBox mbr;


    OGRCoordinateTransformation* poCT;

   public: bool ensure_matrices_loaded();
public:
    Bin() : poCT(nullptr) {
        GDALAllRegister();
        if(Config::is_available())
            conf = Config::load();
        // Instantiate spatial converter from standard Lat/Lng (WGS84) to your local project grid (EPSG:7755)
        OGRSpatialReference oSourceSRS, oTargetSRS;
        oSourceSRS.importFromEPSG(4326);
        oTargetSRS.importFromEPSG(7755);

        // Enforce traditional spatial mapping (X = Longitude, Y = Latitude) regardless of system GDAL version
        oSourceSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        oTargetSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        poCT = OGRCreateCoordinateTransformation(&oSourceSRS, &oTargetSRS);
        if (!poCT) {
            std::cerr << "[init] CRITICAL ERROR: Could not initialize OGRCoordinateTransformation!\n";
        }
    }

    ~Bin() {
        if (poCT) OGRCoordinateTransformation::DestroyCT(poCT);
    }

    public: static int build_matrix();

    public: static int build_elevation_matrix(bool split = 1);

    public: static int refresh_road_layer(std::optional<std::string> path);

    public: static int refresh_rail_layer(std::optional<std::string> path);

    public: static int refresh_water_area_layer(std::optional<std::string> path);

    public: static int refresh_water_line_layer(std::optional<std::string> path);

    public: static int refresh_ib_layer(std::optional<std::string> path = std::nullopt);

    public: static bool point_in_polygon(double px, double py, const std::vector<std::pair<double,double>>& proj_poly);

    bool getGridCoords(double lat, double lng, int& out_r, int& out_c);

    int16_t getElevation(double lat, double lng);

    bool getSlope(double lat, double lng);

    bool getStatusRoad(double lat, double lng);

    bool getStatusRail(double lat, double lng);

    bool getStatusIB(double lat, double lng);

    bool getStatusWaterArea(double lat, double lng);

    bool getStatusWaterLine(double lat, double lng);

    bool latlon_to_projected(double lat, double lon, double& px, double& py);

    std::vector<int> getStatusAll(double lat, double lng);

    BoundingBox getMinimumBoundingBox();

    FeasibleGrid getFeasibleGrid(short op = IN_MEM, short opt = 0, short limit_ram = false);

    std::pair<double,double> getMapOrigins();

    int getTerrianResolution();

    bool lineOfSight(int x1, int y1, int x2, int y2);

    void visualize(const std::string& output_path);

    static void get_elevation_color(int16_t alt, uint8_t& r, uint8_t& g, uint8_t& b);

    public: template <typename T>
    static bool verifyMatrixBin(const std::string& path, int rows, int cols) {
        std::cout << "[verify] Checking: " << path << "\n";
        std::cout << "[verify] Expected: " << rows << " x " << cols
                  << " (" << (size_t)rows * cols << " cells, "
                  << ((size_t)rows * cols * sizeof(T)) / 1e6 << " MB, "
                  << sizeof(T) << " bytes/cell)\n";

        // --- 1. check file exists ---
        std::ifstream f(path, std::ios::binary);
        if (!f.is_open()) {
            std::cerr << "[verify] FAIL: could not open " << path << "\n";
            return false;
        }

        // --- 2. check file size ---
        f.seekg(0, std::ios::end);
        size_t file_bytes     = f.tellg();
        size_t expected_bytes = (size_t)rows * cols * sizeof(T);
        f.seekg(0, std::ios::beg);

        if (file_bytes != expected_bytes) {
            std::cerr << "[verify] FAIL: size mismatch — expected "
                      << expected_bytes / 1e6 << " MB, got "
                      << file_bytes / 1e6 << " MB\n";
            return false;
        }
        std::cout << "[verify] Size OK: " << file_bytes / 1e6 << " MB\n";

        // --- 3. read ---
        std::vector<T> matrix((size_t)rows * cols);
        f.read(reinterpret_cast<char*>(matrix.data()), expected_bytes);
        if ((size_t)f.gcount() != expected_bytes) {
            std::cerr << "[verify] FAIL: read " << f.gcount()
                      << " of " << expected_bytes << " bytes\n";
            return false;
        }
        f.close();

        // derive ppm path from bin path
        std::string ppm_path = path.substr(0, path.find_last_of('.')) + "_verify.ppm";

        // --- 4. stats + ppm ---
        if constexpr (std::is_same<T, uint8_t>::value) {
            // ---- uint8 (vector/bitmask matrix) ----
            T      min_val = matrix[0];
            T      max_val = matrix[0];
            size_t nonzero = 0;

            for (size_t i = 0; i < matrix.size(); i++) {
                if (matrix[i] < min_val) min_val = matrix[i];
                if (matrix[i] > max_val) max_val = matrix[i];
                if (matrix[i] != 0)      nonzero++;
            }
            double coverage = (nonzero / (double)matrix.size()) * 100.0;
            std::cout << "[verify] Min value : " << (int)min_val  << "\n";
            std::cout << "[verify] Max value : " << (int)max_val  << "\n";
            std::cout << "[verify] Non-zero  : " << nonzero << " cells (" << coverage << "%)\n";
            if (nonzero == 0)
                std::cerr << "[verify] WARN: matrix is entirely zero — likely a failed build\n";

            // ppm — same color scheme as proof.ppm
            std::ofstream img(ppm_path, std::ios::binary);
            img << "P6\n" << cols << " " << rows << "\n255\n";
            for (int r = 0; r < rows; r++) {
                if (r % 2000 == 0 && r > 0)
                    std::cout << "[verify] Writing row " << r << "/" << rows << "\n";
                for (int c = 0; c < cols; c++) {
                    uint8_t val = matrix[(size_t)r * cols + c];
                    uint8_t red = 20, green = 24, blue = 38;   // dark background
                    if (val & (1 << 0)) { red = 255; green = 50;  blue = 0;   }  // roads   → orange
                    if (val & (1 << 1)) { red = 255; green = 255; blue = 0;   }  // railway → yellow
                    if (val & (1 << 2)) { red = 0;   green = 100; blue = 255; }  // waterline → blue
                    if (val & (1 << 3)) { red = 0;   green = 180; blue = 255; }  // waterbody → light blue
                    if ((val & (1 << 0)) && (val & (1 << 1))) {
                        red = 0; green = 255; blue = 255;   // road+rail crossing → cyan
                    }
                    img.put(red); img.put(green); img.put(blue);
                }
            }
            img.close();

        } else if constexpr (std::is_same<T, int16_t>::value) {
            // ---- int16 (elevation matrix) ----
            T      min_val = matrix[0];
            T      max_val = matrix[0];
            size_t nonzero = 0;

            for (size_t i = 0; i < matrix.size(); i++) {
                if (matrix[i] < min_val) min_val = matrix[i];
                if (matrix[i] > max_val) max_val = matrix[i];
                if (matrix[i] != 0)      nonzero++;
            }
            double coverage = (nonzero / (double)matrix.size()) * 100.0;
            std::cout << "[verify] Min value : " << min_val << "m\n";
            std::cout << "[verify] Max value : " << max_val << "m\n";
            std::cout << "[verify] Non-zero  : " << nonzero << " cells (" << coverage << "%)\n";
            if (nonzero == 0)
                std::cerr << "[verify] WARN: matrix is entirely zero — likely a failed build\n";

            // ppm — terrain color scheme
            std::ofstream img(ppm_path, std::ios::binary);
                img << "P6\n" << cols << " " << rows << "\n255\n";
                for (int r = 0; r < rows; r++) {
                    if (r % 2000 == 0 && r > 0)
                        std::cout << "[verify] Writing row " << r << "/" << rows << "\n";
                    for (int c = 0; c < cols; c++) {
                        uint8_t val = matrix[(size_t)r * cols + c];
                        uint8_t red = 20, green = 24, blue = 38;

                        if (val & (1 << BIT_ROAD))       { red = 255; green = 50;  blue = 0;   }
                        if (val & (1 << BIT_RAIL))       { red = 255; green = 255; blue = 0;   }
                        if (val & (1 << BIT_WATER_LINE)) { red = 0;   green = 100; blue = 255; }
                        if (val & (1 << BIT_WATER_AREA)) { red = 0;   green = 180; blue = 255; }
                        if (val & (1 << BIT_IB))         { red = 80;  green = 0;   blue = 80;  }

                        // intersections
                        if ((val & (1 << BIT_ROAD)) && (val & (1 << BIT_RAIL)))
                            { red = 0; green = 255; blue = 255; }

                        img.put(red); img.put(green); img.put(blue);
                    }
                }
                img.close();

        } else {
            // ---- struct (FeasibleCell etc.) ----
            // treat as a sparse point cloud — plot each cell's (r,c) on a blank grid
            std::cout << "[verify] Cell count: " << matrix.size() << "\n";
            std::cout << "[verify] Cell size : " << sizeof(T) << " bytes (struct)\n";

            int min_r = INT_MAX, min_c = INT_MAX;
            int max_r = 0,       max_c = 0;
            for (auto& cell : matrix) {
                if (cell.r < min_r) min_r = cell.r;
                if (cell.c < min_c) min_c = cell.c;
                if (cell.r > max_r) max_r = cell.r;
                if (cell.c > max_c) max_c = cell.c;
            }

            int img_rows = max_r - min_r + 1;
            int img_cols = max_c - min_c + 1;
            std::cout << "[verify] Reconstructing " << img_rows << " x " << img_cols << " grid from cell coords\n";

            // blank dark canvas
            std::vector<uint8_t> canvas((size_t)img_rows * img_cols * 3, 0);
            for (auto& cell : matrix) {
                size_t idx = ((size_t)(cell.r - min_r) * img_cols + (cell.c - min_c)) * 3;
                canvas[idx + 0] = 255;  // white dot per feasible cell
                canvas[idx + 1] = 255;
                canvas[idx + 2] = 255;
            }

            std::ofstream img(ppm_path, std::ios::binary);
            img << "P6\n" << img_cols << " " << img_rows << "\n255\n";
            img.write(reinterpret_cast<char*>(canvas.data()), canvas.size());
            img.close();
        }

        std::cout << "[verify] PPM written → " << ppm_path << "\n";
        std::cout << "[verify] PASS: " << path << "\n";
        return true;
    }

};

#endif
