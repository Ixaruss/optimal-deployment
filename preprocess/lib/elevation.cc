#include "../bin.h"
#include <gdal_utils.h>
using namespace std;

// Horn's formula, nodata-aware.
// Nodata neighbors are skipped — falls back to one-sided diff.
// If both sides of an axis are nodata → slope = -9999.0f
static float compute_slope(const int16_t* elev,
                             int r, int c,
                             int ROWS, int COLS,
                             int resolution) {
    // center must be valid
    int16_t center = elev[(size_t)r * COLS + c];
    if (center == -9999) return -9999.0f;

    // helper — returns value or -9999 sentinel if out of bounds
    auto get = [&](int rr, int cc) -> int16_t {
        if (rr < 0 || rr >= ROWS || cc < 0 || cc >= COLS) return -9999;
        return elev[(size_t)rr * COLS + cc];
    };

    int16_t N  = get(r-1, c);
    int16_t S  = get(r+1, c);
    int16_t W  = get(r,   c-1);
    int16_t E  = get(r,   c+1);

    // ── dz/dx (west→east) ─────────────────────────────────────────────────
    float dzdx;
    bool e_ok = (E != -9999);
    bool w_ok = (W != -9999);

    if      (e_ok && w_ok)  dzdx = (E - W) / (2.0f * resolution);
    else if (e_ok)          dzdx = (E - center) / (float)resolution;
    else if (w_ok)          dzdx = (center - W) / (float)resolution;
    else                    return -9999.0f;   // both sides nodata

    // ── dz/dy (north→south) ───────────────────────────────────────────────
    float dzdy;
    bool n_ok = (N != -9999);
    bool s_ok = (S != -9999);

    if      (n_ok && s_ok)  dzdy = (N - S) / (2.0f * resolution);
    else if (n_ok)          dzdy = (center - N) / (float)resolution;
    else if (s_ok)          dzdy = (S - center) / (float)resolution;
    else                    return -9999.0f;   // both sides nodata

    // ── Horn's slope in degrees ────────────────────────────────────────────
    return atan(sqrt(dzdx * dzdx + dzdy * dzdy)) * (180.0f / (float)M_PI);
}

int Bin::build_elevation_matrix() {
    string opath, ipath;
    Config::is_available() ? opath = Config::load().output.elevation_file : opath = DEFAULT_ELEVATION_MATRIX;
    Config::is_available() ? ipath = Config::load().input.elevation        : ipath = DEFAULT_ELEVATION_FILE;
    GDALAllRegister();

    // ── 1. open source TIF ─────────────────────────────────────────────────
    cout << "[elevation] Opening: " << ipath << "\n";
    GDALDataset* src_ds = (GDALDataset*)GDALOpenEx(
        ipath.c_str(), GDAL_OF_RASTER, nullptr, nullptr, nullptr);
    if (!src_ds) {
        cerr << "[elevation] ERROR: could not open " << ipath << "\n";
        return -1;
    }
    cout << "[elevation] Source size: "
         << src_ds->GetRasterXSize() << " x " << src_ds->GetRasterYSize()
         << "  bands: " << src_ds->GetRasterCount() << "\n";

    // ── 2. derive EPSG:7755 extent from TIF corners ────────────────────────
    double src_gt[6];
    if (src_ds->GetGeoTransform(src_gt) != CE_None) {
        cerr << "[elevation] ERROR: could not get source geotransform\n";
        GDALClose(src_ds);
        return -1;
    }

    int src_cols = src_ds->GetRasterXSize();
    int src_rows = src_ds->GetRasterYSize();

    double corner_x[4] = {
        src_gt[0],
        src_gt[0] + src_cols * src_gt[1],
        src_gt[0],
        src_gt[0] + src_cols * src_gt[1]
    };
    double corner_y[4] = {
        src_gt[3],
        src_gt[3],
        src_gt[3] + src_rows * src_gt[5],
        src_gt[3] + src_rows * src_gt[5]
    };

    const char* src_wkt = src_ds->GetProjectionRef();
    OGRSpatialReference src_srs, dst_srs;
    src_srs.importFromWkt(src_wkt);
    dst_srs.importFromEPSG(7755);
    src_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    dst_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRCoordinateTransformation* ct =
        OGRCreateCoordinateTransformation(&src_srs, &dst_srs);
    if (!ct) {
        cerr << "[elevation] ERROR: could not create coordinate transform\n";
        GDALClose(src_ds);
        return -1;
    }
    ct->Transform(4, corner_x, corner_y);
    OGRCoordinateTransformation::DestroyCT(ct);

    double proj_min_x = *min_element(corner_x, corner_x + 4);
    double proj_max_x = *max_element(corner_x, corner_x + 4);
    double proj_min_y = *min_element(corner_y, corner_y + 4);
    double proj_max_y = *max_element(corner_y, corner_y + 4);

    cout << "[elevation] Projected extent (EPSG:7755):\n"
         << "  X: " << proj_min_x << " → " << proj_max_x << "\n"
         << "  Y: " << proj_min_y << " → " << proj_max_y << "\n";

    // ── 3. compute output grid at same RESOLUTION as vector matrix ─────────
    int    OUT_COLS = (int)ceil((proj_max_x - proj_min_x) / RESOLUTION);
    int    OUT_ROWS = (int)ceil((proj_max_y - proj_min_y) / RESOLUTION);
    double ORIGIN_X = proj_min_x;
    double ORIGIN_Y = proj_max_y;

    cout << "[elevation] Output grid: " << OUT_COLS << " x " << OUT_ROWS
         << " @ " << RESOLUTION << "m/cell\n";

    // ── 4. create MEM destination raster ──────────────────────────────────
    GDALDriver*  mem_driver = GetGDALDriverManager()->GetDriverByName("MEM");
    GDALDataset* dst_ds     = mem_driver->Create(
        "", OUT_COLS, OUT_ROWS, 1, GDT_Int16, nullptr);
    if (!dst_ds) {
        cerr << "[elevation] ERROR: failed to create MEM raster\n";
        GDALClose(src_ds);
        return -1;
    }

    double gt[6] = { ORIGIN_X, RESOLUTION, 0, ORIGIN_Y, 0, -RESOLUTION };
    dst_ds->SetGeoTransform(gt);

    char* dst_wkt_str = nullptr;
    dst_srs.exportToWkt(&dst_wkt_str);
    dst_ds->SetProjection(dst_wkt_str);
    CPLFree(dst_wkt_str);

    // ── 5. warp with multithreading ────────────────────────────────────────
    cout << "[elevation] Warping to EPSG:7755 @ " << RESOLUTION << "m...\n";

    GDALWarpOptions* warp_opts      = GDALCreateWarpOptions();
    warp_opts->hSrcDS               = src_ds;
    warp_opts->hDstDS               = dst_ds;
    warp_opts->nBandCount           = 1;
    warp_opts->panSrcBands          = (int*)CPLMalloc(sizeof(int));
    warp_opts->panDstBands          = (int*)CPLMalloc(sizeof(int));
    warp_opts->panSrcBands[0]       = 1;
    warp_opts->panDstBands[0]       = 1;
    warp_opts->eResampleAlg         = GRA_Bilinear;
    warp_opts->padfSrcNoDataReal    = (double*)CPLMalloc(sizeof(double));
    warp_opts->padfDstNoDataReal    = (double*)CPLMalloc(sizeof(double));
    warp_opts->padfSrcNoDataReal[0] = -9999.0;
    warp_opts->padfDstNoDataReal[0] = -9999.0;
    warp_opts->pTransformerArg      = GDALCreateGenImgProjTransformer(
        src_ds, nullptr, dst_ds, nullptr, FALSE, 0, 1);
    warp_opts->pfnTransformer       = GDALGenImgProjTransform;
    // use all available CPU cores for warp
    warp_opts->papszWarpOptions     = CSLSetNameValue(
        warp_opts->papszWarpOptions, "NUM_THREADS", "ALL_CPUS");

    GDALWarpOperation warp_op;
    if (warp_op.Initialize(warp_opts) != CE_None) {
        cerr << "[elevation] ERROR: warp init failed\n";
        GDALDestroyWarpOptions(warp_opts);
        GDALClose(dst_ds); GDALClose(src_ds);
        return -1;
    }
    if ((CPLErr)warp_op.ChunkAndWarpImage(0, 0, OUT_COLS, OUT_ROWS) != CE_None) {
        cerr << "[elevation] ERROR: warp failed\n";
        GDALDestroyWarpOptions(warp_opts);
        GDALClose(dst_ds); GDALClose(src_ds);
        return -1;
    }
    GDALDestroyGenImgProjTransformer(warp_opts->pTransformerArg);
    GDALDestroyWarpOptions(warp_opts);
    cout << "[elevation] Warp done\n";

    // ── 6. read elevation band into int16 buffer ───────────────────────────
    size_t   total_cells = (size_t)OUT_ROWS * OUT_COLS;
    int16_t* elev_buf    = (int16_t*)malloc(total_cells * sizeof(int16_t));
    if (!elev_buf) {
        cerr << "[elevation] ERROR: failed to allocate elevation buffer\n";
        GDALClose(dst_ds); GDALClose(src_ds);
        return -1;
    }

    if (dst_ds->GetRasterBand(1)->RasterIO(GF_Read,
            0, 0, OUT_COLS, OUT_ROWS,
            elev_buf, OUT_COLS, OUT_ROWS,
            GDT_Int16, 0, 0) != CE_None) {
        cerr << "[elevation] ERROR: elevation RasterIO failed\n";
        free(elev_buf);
        GDALClose(dst_ds); GDALClose(src_ds);
        return -1;
    }
    GDALClose(dst_ds);
    GDALClose(src_ds);
    cout << "[elevation] Elevation read complete\n";

    // ── 7. allocate output ElevCell array ──────────────────────────────────
    // slope is computed directly into out[i].slope — no intermediate
    // float buffer needed. RAM = elev_buf (int16) + out (ElevCell) only.
    ElevCell* out = (ElevCell*)malloc(total_cells * sizeof(ElevCell));
    if (!out) {
        cerr << "[elevation] ERROR: failed to allocate ElevCell buffer\n";
        free(elev_buf);
        return -1;
    }

    // ── 8. compute slope in-place using Horn's formula ─────────────────────
    // Nodata-aware: skips -9999 neighbors, falls back to one-sided diff.
    // If both sides of an axis are nodata → slope = -9999.0f
    cout << "[elevation] Computing slope (Horn's formula, nodata-aware)...\n";

    size_t nodata_slope = 0;
    for (int r = 0; r < OUT_ROWS; r++) {
        if (r % 5000 == 0 && r > 0)
            cout << "[elevation] slope row " << r << "/" << OUT_ROWS << "\n";
        for (int c = 0; c < OUT_COLS; c++) {
            size_t idx        = (size_t)r * OUT_COLS + c;
            out[idx].elevation = elev_buf[idx];
            out[idx].slope     = compute_slope(
                elev_buf, r, c, OUT_ROWS, OUT_COLS, RESOLUTION);
            if (out[idx].slope == -9999.0f) nodata_slope++;
        }
    }

    free(elev_buf);   // done with raw elevation — ElevCell has it now

    cout << "[elevation] Slope done. Nodata slope cells: " << nodata_slope
         << " (" << (nodata_slope * 100.0 / total_cells) << "%)\n";

    // ── 9. write bin ───────────────────────────────────────────────────────
    FILE* f = fopen(opath.c_str(), "wb");
    if (!f) {
        cerr << "[elevation] ERROR: could not open " << opath << " for writing\n";
        free(out);
        return -1;
    }
    fwrite(out, sizeof(ElevCell), total_cells, f);
    fclose(f);
    free(out);

    cout << "[elevation] Written → " << opath << " ("
         << (total_cells * sizeof(ElevCell)) / 1e6 << " MB, "
         << sizeof(ElevCell) << " bytes/cell)\n";

    // ── 10. save derived grid params to config ─────────────────────────────
    if (Config::is_available()) {
        Config cfg               = Config::load();
        cfg.grid.rows            = OUT_ROWS;
        cfg.grid.cols            = OUT_COLS;
        cfg.grid.origin_x        = (float)ORIGIN_X;
        cfg.grid.origin_y        = (float)ORIGIN_Y;
        cfg.grid.min_x           = proj_min_x;
        cfg.grid.max_x           = proj_max_x;
        cfg.grid.min_y           = proj_min_y;
        cfg.grid.max_y           = proj_max_y;
        cfg.save();
        cout << "[elevation] Config updated:\n"
             << "  elev grid : " << OUT_COLS << " x " << OUT_ROWS << "\n"
             << "  min_x     : " << proj_min_x << "\n"
             << "  max_x     : " << proj_max_x << "\n"
             << "  min_y     : " << proj_min_y << "\n"
             << "  max_y     : " << proj_max_y << "\n";
    } else {
        cerr << "[elevation] WARN: no config found — grid extents not saved.\n"
             << "  Run with a valid config.json so calc_rows/calc_cols\n"
             << "  pick up correct extents for build_matrix.\n";
    }

    return 0;
}
