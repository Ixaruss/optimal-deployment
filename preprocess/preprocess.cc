#include <cstdint>
#include <cstdio>
#include <png.h>
#include "bin.h"
#include "lib.h"
#include "../common.h"
#include <chrono>
using namespace std;

static bool make_dir(const string& path) {
#ifdef _WIN32
    int ret = system(("mkdir \"" + path + "\" 2>nul").c_str());
#else
    int ret = system(("mkdir -p \"" + path + "\"").c_str());
#endif
    (void)ret;
    return true;
}

static bool write_bin_u8(const string& path, const uint8_t* data, size_t n) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { cerr << "[preprocess] ERROR: cannot open " << path << "\n"; return false; }
    fwrite(data, 1, n, f);
    fclose(f);
    cout << "[preprocess] bin → " << path << " (" << n / 1e6 << " MB)\n";
    return true;
}

static bool write_bin_elevcell(const string& path, const ElevCell* data, size_t n) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { cerr << "[preprocess] ERROR: cannot open " << path << "\n"; return false; }
    fwrite(data, sizeof(ElevCell), n, f);
    fclose(f);
    cout << "[preprocess] bin → " << path
         << " (" << (n * sizeof(ElevCell)) / 1e6 << " MB, "
         << sizeof(ElevCell) << " bytes/cell)\n";
    return true;
}

static bool write_png(const string& path, int rows, int cols, const uint8_t* rgb) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { cerr << "[preprocess] ERROR: cannot open " << path << " for write\n"; return false; }

    png_structp png  = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) { cerr << "[preprocess] ERROR: png_create_write_struct failed\n"; fclose(f); return false; }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        cerr << "[preprocess] ERROR: png_create_info_struct failed\n";
        png_destroy_write_struct(&png, nullptr); fclose(f); return false;
    }
    if (setjmp(png_jmpbuf(png))) {
        cerr << "[preprocess] ERROR: libpng error writing " << path << "\n";
        png_destroy_write_struct(&png, &info); fclose(f); return false;
    }

    png_init_io(png, f);
    png_set_IHDR(png, info,
                 (png_uint_32)cols, (png_uint_32)rows,
                 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    for (int r = 0; r < rows; r++) {
        if (r % 5000 == 0 && r > 0)
            cout << "[preprocess] png row " << r << "/" << rows << "\n";
        png_write_row(png, const_cast<png_bytep>(rgb + (size_t)r * cols * 3));
    }

    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    fclose(f);
    cout << "[preprocess] png → " << path << "\n";
    return true;
}

static uint8_t* mask_to_rgb(const uint8_t* mask, size_t N) {
    uint8_t* rgb = (uint8_t*)malloc(N * 3);
    if (!rgb) return nullptr;
    for (size_t i = 0; i < N; i++) {
        uint8_t v    = mask[i] ? 255 : 0;
        rgb[i*3]     = v;
        rgb[i*3 + 1] = v;
        rgb[i*3 + 2] = v;
    }
    return rgb;
}

static uint8_t* elev_to_rgb(const int16_t* cells, size_t N) {
    uint8_t* rgb = (uint8_t*)malloc(N * 3);
    if (!rgb) return nullptr;
    for (size_t i = 0; i < N; i++)
        Bin::get_elevation_color(cells[i],
            rgb[i*3], rgb[i*3 + 1], rgb[i*3 + 2]);
    return rgb;
}

static uint8_t* slope_to_rgb(const float* slope, size_t N) {
    uint8_t* rgb = (uint8_t*)malloc(N * 3);
    if (!rgb) return nullptr;
    for (size_t i = 0; i < N; i++) {
        float s = slope[i];
        if (s < 0.0f)  s = 0.0f;
        if (s > 90.0f) s = 90.0f;
        uint8_t v    = (uint8_t)(s / 90.0f * 255.0f);
        rgb[i*3]     = v;
        rgb[i*3 + 1] = v;
        rgb[i*3 + 2] = v;
    }
    return rgb;
}

// ── rasterize ──────────────────────────────────────────────────────────────
static uint8_t* rasterize_layer(const string& shp_path,
                                  int ROWS, int COLS,
                                  double ox, double oy,
                                  bool boundary_only = false) {
    GDALDataset* shp_ds = (GDALDataset*)GDALOpenEx(
        shp_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!shp_ds) { cerr << "[preprocess] ERROR: cannot open " << shp_path << "\n"; return nullptr; }

    OGRLayer* layer = shp_ds->GetLayer(0);
    if (!layer) {
        cerr << "[preprocess] ERROR: no layer in " << shp_path << "\n";
        GDALClose(shp_ds); return nullptr;
    }

    // print extent for diagnostic
    OGREnvelope env;
    if (layer->GetExtent(&env) == OGRERR_NONE) {
        cout << "[preprocess] shp extent: ("
             << env.MinX << ", " << env.MinY << ") - ("
             << env.MaxX << ", " << env.MaxY << ")\n";
    }

    vector<OGRGeometryH> geoms;
    OGRFeature* feat;
    layer->ResetReading();
    while ((feat = layer->GetNextFeature()) != nullptr) {
        OGRGeometry* geom = feat->GetGeometryRef();
        if (geom) {
            if (boundary_only) {
                OGRGeometry* bnd = geom->Boundary();
                if (bnd) geoms.push_back((OGRGeometryH)bnd);
            } else {
                geoms.push_back((OGRGeometryH)geom->clone());
            }
        }
        OGRFeature::DestroyFeature(feat);
    }
    GDALClose(shp_ds);

    if (geoms.empty()) { cerr << "[preprocess] WARN: no geometries in " << shp_path << "\n"; return nullptr; }
    cout << "[preprocess] loaded " << geoms.size() << " geoms\n";
    cout << "[preprocess] MEM raster: " << COLS << " x " << ROWS
         << " origin=(" << ox << ", " << oy << ")\n";

    GDALDriver*  drv = GetGDALDriverManager()->GetDriverByName("MEM");
    GDALDataset* ds  = drv->Create("", COLS, ROWS, 1, GDT_Byte, nullptr);
    double gt[6] = { ox, RESOLUTION, 0, oy, 0, -RESOLUTION };
    ds->SetGeoTransform(gt);
    GDALRasterBand* band = ds->GetRasterBand(1);
    band->Fill(0);

    int band_list[1] = { 1 };
    vector<double> burn_vals(geoms.size(), 1.0);
    CPLErr err = (CPLErr)GDALRasterizeGeometries(
        ds, 1, band_list,
        (int)geoms.size(), geoms.data(),
        nullptr, nullptr, burn_vals.data(),
        nullptr, nullptr, nullptr);

    for (auto g : geoms) OGR_G_DestroyGeometry(g);

    if (err != CE_None) {
        cerr << "[preprocess] ERROR: GDALRasterizeGeometries failed (CPLErr=" << err << ")\n";
        GDALClose(ds); return nullptr;
    }

    size_t   n   = (size_t)ROWS * COLS;
    uint8_t* buf = (uint8_t*)malloc(n);
    if (!buf) { GDALClose(ds); return nullptr; }

    CPLErr rio = band->RasterIO(GF_Read, 0, 0, COLS, ROWS, buf, COLS, ROWS, GDT_Byte, 0, 0);
    if (rio != CE_None)
        cerr << "[preprocess] WARN: RasterIO returned error " << rio << "\n";

    // quick scan — how many cells burned?
    size_t burned = 0;
    for (size_t i = 0; i < n; i++) if (buf[i]) burned++;
    cout << "[preprocess] burned cells after rasterize: " << burned << "\n";

    GDALClose(ds);
    return buf;
}

// ── distance transform ─────────────────────────────────────────────────────
static void expand_buffer(uint8_t* buf, size_t N, int ROWS, int COLS, int radius) {
    if (radius <= 0) {
        cout << "[preprocess] radius=0 — skipping expansion\n";
        return;
    }

    vector<int> dist(N, INT_MAX);
    for (size_t i = 0; i < N; i++)
        if (buf[i] == 1) dist[i] = 0;

    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            size_t idx = (size_t)r * COLS + c;
            if (c > 0 && dist[idx-1] != INT_MAX)
                dist[idx] = min(dist[idx], dist[idx-1] + 1);
            if (r > 0 && dist[idx-COLS] != INT_MAX)
                dist[idx] = min(dist[idx], dist[idx-COLS] + 1);
        }
    }
    for (int r = ROWS-1; r >= 0; r--) {
        for (int c = COLS-1; c >= 0; c--) {
            size_t idx = (size_t)r * COLS + c;
            if (c < COLS-1 && dist[idx+1] != INT_MAX)
                dist[idx] = min(dist[idx], dist[idx+1] + 1);
            if (r < ROWS-1 && dist[idx+COLS] != INT_MAX)
                dist[idx] = min(dist[idx], dist[idx+COLS] + 1);
        }
    }

    int expanded = 0;
    for (size_t i = 0; i < N; i++) {
        buf[i] = (dist[i] <= radius) ? 1 : 0;
        if (buf[i]) expanded++;
    }
    cout << "[preprocess] expanded → " << expanded << " cells\n";
}

static void print_elev_stats(const int16_t* elevation,const float* slope, size_t N) {
    int16_t emin =  32767, emax = -32768;
    float   smin =  1e9f,  smax = -1e9f;
    size_t  nodata = 0;
    for (size_t i = 0; i < N; i++) {
        if (elevation[i] == -9999) { nodata++; continue; }
        if (elevation[i] < emin) emin = elevation[i];
        if (elevation[i] > emax) emax = elevation[i];
        if (slope[i]     < smin) smin = slope[i];
        if (slope[i]     > smax) smax = slope[i];
    }
    cout << "[preprocess] elevation : " << emin << "m → " << emax << "m\n";
    cout << "[preprocess] slope     : " << smin << "° → " << smax << "°\n";
    cout << "[preprocess] nodata    : " << nodata << " (" << (nodata * 100.0 / N) << "%)\n";
}

void Global::preprocess(bool images, bool timer, bool seperate) {
    GDALAllRegister();
     chrono::time_point<chrono::steady_clock, chrono::duration<long, ratio<1, 1000000000>>> start,end;
    if (timer){ cout<< ">>> Starting timer count" << endl;
         start = chrono::steady_clock::now();
    }
    make_dir("bin");
    if (images) make_dir("image");
    Bin::build_elevation_matrix();
    string road_shp, rail_shp, warea_shp, wline_shp, ib_shp, elev_bin_path, opath,slope_path;
    int road_bit, rail_bit, waterarea_bit, waterlines_bit, ib_bit, slope_bit;
    int dist_road, dist_rail, dist_water, dist_ib;
    int ELEV_R, ELEV_C;
    double ox, oy;
    long long total = 0;
    long long feasible = 0;

    if (Config::is_available()) {
        Config cfg    = Config::load();
        road_shp      = cfg.input.road;
        rail_shp      = cfg.input.rail;
        warea_shp     = cfg.input.water_area;
        wline_shp     = cfg.input.water_lines;
        ib_shp        = cfg.input.ib;
        elev_bin_path = cfg.output.elevation_file;
        opath         = cfg.output.matrix_file;

        dist_road  = cfg.distance.from_road  > 0 ? cfg.distance.from_road  : 1000;
        dist_rail  = cfg.distance.from_rail  > 0 ? cfg.distance.from_rail  : 1000;
        dist_water = cfg.distance.from_water > 0 ? cfg.distance.from_water : 1000;
        dist_ib    = cfg.distance.from_ib    > 0 ? cfg.distance.from_ib    : 10000;

        road_bit       = lib::layerName(cfg.layers[BIT_ROAD]);
        rail_bit       = lib::layerName(cfg.layers[BIT_RAIL]);
        waterarea_bit  = lib::layerName(cfg.layers[BIT_WATER_AREA]);
        waterlines_bit = lib::layerName(cfg.layers[BIT_WATER_LINE]);
        ib_bit         = lib::layerName(cfg.layers[BIT_IB]);

        // Dynamic assignment or hardcoded to 5 based on request
        slope_bit      = 5;
        slope_path     = cfg.output.slope_file;

        ox = cfg.grid.min_x;
        oy = cfg.grid.max_y;
        ELEV_R = cfg.grid.rows;
        ELEV_C = cfg.grid.cols;
    } else {
        road_shp      = DEFAULT_ROAD_SHP;
        rail_shp      = DEFAULT_RAIL_SHP;
        warea_shp     = DEFAULT_WATER_AREA_SHP;
        wline_shp     = DEFAULT_WATER_LINE_SHP;
        ib_shp        = DEFAULT_IB_SHP;
        elev_bin_path = DEFAULT_ELEVATION_MATRIX;
        dist_road  = DISTANCE_FROM_ROAD;
        dist_rail  = DISTANCE_FROM_RAIL;
        dist_water = DISTANCE_FROM_WATER;
        dist_ib    = DISTANCE_FROM_IB;
        ox = VEC_ORIGIN_X;
        oy = VEC_ORIGIN_Y;
        ELEV_R = ELEV_ROWS;
        ELEV_C = ELEV_COLS;
        road_bit       = BIT_ROAD;
        rail_bit       = BIT_RAIL;
        waterarea_bit  = BIT_WATER_AREA;
        waterlines_bit = BIT_WATER_LINE;
        ib_bit         = BIT_IB;
        slope_bit      = 5; // Hardcoded requirement for index 5
        opath          = DEFAULT_PARAM_MATRIX;
        slope_path     = DEFAULT_SLOPE_MATRIX;
    }



    int    ROWS = lib::calc_rows(RESOLUTION);
    int    COLS = lib::calc_cols(RESOLUTION);
    size_t N    = (size_t)ROWS * COLS;
    uint8_t* matrix = (uint8_t*)calloc(N, 1);

    cout << "[preprocess] Grid       : " << ROWS << " x " << COLS << " (" << N / 1e6 << "M cells)\n";
    cout << "[preprocess] origin     : (" << ox << ", " << oy << ")\n";
    cout << "[preprocess] dist_road  : " << dist_road  << "m (" << dist_road  / RESOLUTION << " cells)\n";
    cout << "[preprocess] dist_rail  : " << dist_rail  << "m (" << dist_rail  / RESOLUTION << " cells)\n";
    cout << "[preprocess] dist_water : " << dist_water << "m (" << dist_water / RESOLUTION << " cells)\n";
    cout << "[preprocess] dist_ib    : " << dist_ib    << "m (" << dist_ib    / RESOLUTION << " cells)\n";
    cout << "[preprocess] elev grid  : " << ELEV_C << " x " << ELEV_R << "\n\n";

    // Fixed typo: mapped road to road_bit (was rail_bit)
    struct LayerJob {
        string name;
        string shp;
        int    distance;
        bool   boundary_only;
        int    index;
    };

    vector<LayerJob> jobs = {
        { "road",      road_shp,  dist_road,  false, road_bit },
        { "rail",      rail_shp,  dist_rail,  false, rail_bit },
        { "waterarea", warea_shp, dist_water, false, waterarea_bit },
        { "waterline", wline_shp, dist_water, false, waterlines_bit },
        { "ib",        ib_shp,    dist_ib,    true, ib_bit  },
    };

    // ── elevation + slope ──────────────────────────────────────────────────

    cout << "\n=== elevation + slope ===\n";

    cout << "[preprocess] reading " << ELEV_C << " x " << ELEV_R << " ElevCell grid from " << elev_bin_path << "\n";

    size_t elev_n = (size_t)ELEV_R * ELEV_C;
    int16_t* elev = (int16_t*)malloc(elev_n * sizeof(int16_t));
    float* slope  = (float*)malloc(elev_n * sizeof(float));

    if (!elev) { cerr << "[preprocess] ERROR: cannot allocate elevation buffer\n"; return; }
    if (!slope) { cerr << "[preprocess] ERROR: cannot allocate slope buffer\n"; return; }

    FILE* ef = fopen(elev_bin_path.c_str(), "rb");
    FILE* sf = fopen(slope_path.c_str(), "rb");

    if (!ef || !sf) {
        cerr << "[preprocess] WARN: Elevation or Slope binaries missing — skipping allocation tracking\n";
        if(ef) fclose(ef);
        if(sf) fclose(sf);
        free(elev);
        free(slope);
    } else {
        // verify file size first
        fseek(ef, 0, SEEK_END);
        size_t file_bytes    = ftell(ef);
        size_t expected_bytes = elev_n * sizeof(int16_t);
        fseek(ef, 0, SEEK_SET);

        cout << "[preprocess] elev.bin size  : " << file_bytes / 1e6 << " MB\n";
        cout << "[preprocess] expected size  : " << expected_bytes / 1e6 << " MB\n";

        if (file_bytes != expected_bytes) {
            cerr << "[preprocess] ERROR: size mismatch — expected " << expected_bytes / 1e6 << " MB, got " << file_bytes / 1e6 << " MB\n";
            fclose(ef);
            fclose(sf);
            free(elev);
            free(slope);
            return ;
        }

        size_t read = fread(elev, sizeof(int16_t), elev_n, ef);
        fclose(ef);

        size_t read1 = fread(slope, sizeof(float), elev_n, sf);
        fclose(sf);

        print_elev_stats(elev, slope, read);
        if (images){
            uint8_t* rgb_e = elev_to_rgb(elev, read);
            if (rgb_e) {
                write_png("image/elev.png", ELEV_R, ELEV_C, rgb_e);
                free(rgb_e);
            }

            uint8_t* rgb_s = slope_to_rgb(slope, read1);
            if (rgb_s) {
                write_png("image/slope.png", ELEV_R, ELEV_C, rgb_s);
                free(rgb_s);
            }
        }


        // ── CRITICAL STEP: Burn slope bit to combined matrix ────────────────
        // Condition: slope < 10. Bit position: 5 (1 << 5)
        // Note: Assumes N equals elev_n (Grid match dimension)
        size_t loops = (N < elev_n) ? N : elev_n;
        for (size_t i = 0; i < loops; i++) {
            if (slope[i] > 10.0f) {
                matrix[i] |= (1 << slope_bit); // Burns bit 5 (value 32)
            }
        }
        cout << "[preprocess] writen slope bit";
        free(elev);
        free(slope); // Freed safely after matrix calculations
    }


    int elevTime ;
    if(timer){
        end = chrono::steady_clock::now();
       elevTime = chrono::duration_cast<chrono::seconds>(end - start).count();
       start = chrono::steady_clock::now();
    }
    for (auto& job : jobs) {
        cout << "\n=== " << job.name << " (buffer=" << job.distance << "m) ===\n";
        if (!lib::is_7755(job.shp)) {
            cout << "[preprocess] reprojecting " << job.shp << "...\n";
            if (lib::reproject(job.shp) != 0) {
                cerr << "[preprocess] ERROR: reprojection failed\n";
                return;
            }
        }
        uint8_t* buf = rasterize_layer(job.shp, ROWS, COLS, ox, oy, job.boundary_only);
        if (!buf) { cerr << "[preprocess] SKIP: " << job.name << "\n"; continue; }

        size_t raw = 0;
        for (size_t i = 0; i < N; i++) if (buf[i]) raw++;
        cout << "[preprocess] raw hits   : " << raw << " (" << (raw * 100.0 / N) << "%)\n";

        if (raw == 0)
            cerr << "[preprocess] WARN: no cells burned — geotransform or SHP issue\n";

        expand_buffer(buf, N, ROWS, COLS, job.distance / RESOLUTION);

        size_t final_hits = 0;
        for (size_t i = 0; i < N; i++){
            total++;
            matrix[i] |= (buf[i] << job.index);
            if (buf[i]) final_hits++;
        }

        feasible += final_hits;
        cout << "[preprocess] final hits : " << final_hits << " (" << (final_hits * 100.0 / N) << "%)\n";

        if (seperate) {
            write_bin_u8("bin/" + job.name + ".bin", buf, N);
        }

        if (images) {
            uint8_t* rgb = mask_to_rgb(buf, N);
            if (rgb) {
                write_png("image/" + job.name + ".png", ROWS, COLS, rgb);
                free(rgb);
            }
        }
        free(buf);
    }

    auto f = fopen(opath.c_str(), "wb");
    fwrite(matrix, 1, N, f);
    fclose(f);

    int constraintTime;
    if(timer){
        end = chrono::steady_clock::now();
         constraintTime = chrono::duration_cast<chrono::seconds>(end - start).count();
    }

    if (images) {
        uint8_t* rgb = mask_to_rgb(matrix, N);
        if (rgb) {
            write_png("image/combined.png", ROWS, COLS, rgb);
            free(rgb);
        }
    }
    if(timer) {
    cout << "\n>>> Elevation Build took: " << elevTime << " seconds"<< (images ? " (with png writing)" : "") << endl;
    cout << "\n>>> Combined bin took combined: " << constraintTime << " seconds" << (images ? " (with png writing)" : "") << endl;
    }
    cout<< "total cells: " << total <<endl;
    cout<< "feasible cells: " << feasible << "i.e. " << ((double)feasible / total) * 100 << "%" <<endl;
    cout << "\n>>> Completed All Tasks Succesfully..\n";
    }
