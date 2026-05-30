#include "../bin.h"

#include <gdal.h>
#include <ogr_api.h>
#include <ogrsf_frmts.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <string>
#include <gdal_alg.h>

using namespace std;
using namespace lib;

int Bin::build_matrix() {
    GDALAllRegister();
    int unfeasible_distance, from_road, from_rail, from_water;
    int road_bit, rail_bit, waterarea_bit, waterlines_bit, ib_bit;
    string road_ipath, rail_ipath, waterarea_ipath, waterlines_ipath, ib_ipath,opath;
    FILE* f = nullptr;

    if (Config::is_available()) {
        Config cfg = Config::load();

        road_bit       = layerName(cfg.layers[BIT_ROAD]);
        rail_bit       = layerName(cfg.layers[BIT_RAIL]);
        waterarea_bit  = layerName(cfg.layers[BIT_WATER_AREA]);
        waterlines_bit = layerName(cfg.layers[BIT_WATER_LINE]);
        ib_bit         = layerName(cfg.layers[BIT_IB]);

        road_ipath       = cfg.input.road;
        rail_ipath       = cfg.input.rail;
        waterarea_ipath  = cfg.input.water_area;
        waterlines_ipath = cfg.input.water_lines;
        ib_ipath         = cfg.input.ib;
        opath            = cfg.output.matrix_file;

        unfeasible_distance = cfg.distance.from_ib;
        from_rail           = cfg.distance.from_rail;
        from_road           = cfg.distance.from_road;
        from_water          = cfg.distance.from_water;

    } else {
        road_bit       = BIT_ROAD;
        rail_bit       = BIT_RAIL;
        waterarea_bit  = BIT_WATER_AREA;
        waterlines_bit = BIT_WATER_LINE;
        ib_bit         = BIT_IB;

        road_ipath       = DEFAULT_ROAD_SHP;
        rail_ipath       = DEFAULT_RAIL_SHP;
        waterarea_ipath  = DEFAULT_WATER_AREA_SHP;
        waterlines_ipath = DEFAULT_WATER_LINE_SHP;
        ib_ipath         = DEFAULT_IB_SHP;

        opath               = DEFAULT_PARAM_MATRIX;
        unfeasible_distance = DISTANCE_FROM_IB;
        from_rail           = DISTANCE_FROM_RAIL;
        from_road           = DISTANCE_FROM_ROAD;
        from_water          = DISTANCE_FROM_WATER;
    }


    int ROWS = calc_rows(RESOLUTION);
    int COLS = calc_cols(RESOLUTION);
    size_t total_cells = (size_t)ROWS * COLS;

    uint8_t* matrix = (uint8_t*)calloc(total_cells, 1);
    if (!matrix) {
        cerr << "[matrix] ERROR: failed to allocate master matrix\n";
        return -1;
    }
    cout << "[matrix] Allocated master matrix: " << total_cells / 1e6 << " MB\n";

    GDALDriver* mem_driver = GetGDALDriverManager()->GetDriverByName("MEM");
    if (!mem_driver) {
            cerr << "[matrix] ERROR: MEM driver not found. Did you call GDALAllRegister()?\n";
            free(matrix);
            return -1;
        }
    GDALDataset* temp_ds = mem_driver->Create("", COLS, ROWS, 1, GDT_Byte, nullptr);
    if (!temp_ds) {
        cerr << "[matrix] ERROR: failed to create MEM raster\n";
        free(matrix);
        return -1;
    }

    double geotransform[6] = { VEC_ORIGIN_X, RESOLUTION, 0, VEC_ORIGIN_Y, 0, -RESOLUTION };
    temp_ds->SetGeoTransform(geotransform);
    cout << "[matrix] Created temp MEM raster: " << COLS << " x " << ROWS << "\n";

    GDALRasterBand* band = temp_ds->GetRasterBand(1);
    if (!band) {
        cerr << "[matrix] ERROR: could not get raster band\n";
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    uint8_t* temp_buffer = (uint8_t*)malloc(total_cells);
    if (!temp_buffer) {
        cerr << "[matrix] ERROR: failed to allocate temp buffer\n";
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    // --- process each layer individually ---
    auto process_layer = [&](const std::string& shp_path, int bit_index,int distance) -> bool {
        cout << "[matrix] Processing: " << shp_path << " → bit " << bit_index << "\n";

        GDALDataset* shp_ds = (GDALDataset*)GDALOpenEx(shp_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
        if (!shp_ds) {
            cerr << "[matrix] ERROR: could not open " << shp_path << "\n";
            return false;
        }

        OGRLayer* layer = shp_ds->GetLayer(0);
        if (!layer) {
            cerr << "[matrix] ERROR: no layer found in " << shp_path << "\n";
            GDALClose(shp_ds);
            return false;
        }

        vector<OGRGeometryH> geometries;
        OGRFeature* feature;
        layer->ResetReading();
        while ((feature = layer->GetNextFeature()) != nullptr) {
            OGRGeometry* geom = feature->GetGeometryRef();
            if (geom) geometries.push_back((OGRGeometryH)geom->clone());
            OGRFeature::DestroyFeature(feature);
        }
        cout << "[matrix] Loaded " << geometries.size() << " geometries\n";

        CPLErr fill_err = band->Fill(0);
        if (fill_err != CE_None) {
            cerr << "[matrix] ERROR: failed to zero band\n";
            for (auto g : geometries) OGR_G_DestroyGeometry(g);
            GDALClose(shp_ds);
            return false;
        }

        int band_list[1] = { 1 };
        vector<double> burn_values(geometries.size(), 1.0);

        CPLErr rasterize_err = (CPLErr)GDALRasterizeGeometries(
            temp_ds, 1, band_list,
            (int)geometries.size(), geometries.data(),
            nullptr, nullptr,
            burn_values.data(),
            nullptr, nullptr, nullptr
        );
        if (rasterize_err != CE_None) {
            cerr << "[matrix] ERROR: rasterization failed for " << shp_path << "\n";
            for (auto g : geometries) OGR_G_DestroyGeometry(g);
            GDALClose(shp_ds);
            return false;
        }
        cout << "[matrix] Rasterize done\n";

        CPLErr read_err = band->RasterIO(GF_Read, 0, 0, COLS, ROWS, temp_buffer, COLS, ROWS, GDT_Byte, 0, 0);
        if (read_err != CE_None) {
            cerr << "[matrix] ERROR: RasterIO failed\n";
            for (auto g : geometries) OGR_G_DestroyGeometry(g);
            GDALClose(shp_ds);
            return false;
        }

        for (size_t i = 0; i < total_cells; i++) {
            matrix[i] |= (temp_buffer[i] << bit_index);
        }
        cout << "[matrix] ORed into master matrix at bit " << bit_index << "\n";

        int radius = distance / RESOLUTION;
        cout << "[matrix] Expanding border by " << radius << " pixels (" << distance << "m)\n";

        // pass 1 — for each cell compute distance to nearest border pixel
        // using 1D scan per row then per col (separable distance transform approximation)
        vector<int> dist(total_cells, INT_MAX);

        // mark border pixels as distance 0
        for (size_t i = 0; i < total_cells; i++)
            if (temp_buffer[i] == 1) dist[i] = 0;

        // forward scan — left to right, top to bottom
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                size_t idx = (size_t)r * COLS + c;
                if (c > 0) dist[idx] = min(dist[idx], dist[idx - 1] == INT_MAX ? INT_MAX : dist[idx - 1] + 1);
                if (r > 0) dist[idx] = min(dist[idx], dist[idx - COLS] == INT_MAX ? INT_MAX : dist[idx - COLS] + 1);
            }
        }

        // backward scan — right to left, bottom to top
        for (int r = ROWS - 1; r >= 0; r--) {
            for (int c = COLS - 1; c >= 0; c--) {
                size_t idx = (size_t)r * COLS + c;
                if (c < COLS - 1) dist[idx] = min(dist[idx], dist[idx + 1] == INT_MAX ? INT_MAX : dist[idx + 1] + 1);
                if (r < ROWS - 1) dist[idx] = min(dist[idx], dist[idx + COLS] == INT_MAX ? INT_MAX : dist[idx + COLS] + 1);
            }
        }

        // OR into master matrix wherever distance <= radius
        int burned = 0;
        for (size_t i = 0; i < total_cells; i++) {
            if (dist[i] <= radius) {
                matrix[i] |= (1 << bit_index);
                burned++;
            }
        }
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        return true;
    };

    auto process_ib_layer = [&](const std::string& shp_path, int bit_index) -> bool {
        cout << "[matrix] Processing IB layer: " << shp_path << " → bit " << bit_index << "\n";

        GDALDataset* shp_ds = (GDALDataset*)GDALOpenEx(shp_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
        if (!shp_ds) { cerr << "[matrix] ERROR: could not open " << shp_path << "\n"; return false; }

        OGRLayer* layer = shp_ds->GetLayer(0);
        if (!layer) { cerr << "[matrix] ERROR: no layer found\n"; GDALClose(shp_ds); return false; }

        // burn polygon boundaries as thin lines — no Buffer() call, no RAM spike
        vector<OGRGeometryH> geometries;
        OGRFeature* feature;
        layer->ResetReading();
        while ((feature = layer->GetNextFeature()) != nullptr) {
            OGRGeometry* geom = feature->GetGeometryRef();
            if (geom) {
                OGRGeometry* boundary = geom->Boundary();
                if (boundary)
                    geometries.push_back((OGRGeometryH)boundary);
            }
            OGRFeature::DestroyFeature(feature);
        }
        cout << "[matrix] Loaded " << geometries.size() << " boundary geometries\n";

        if (geometries.empty()) {
            cerr << "[matrix] WARN: no geometries\n";
            GDALClose(shp_ds);
            return false;
        }

        CPLErr fill_err = band->Fill(0);
        if (fill_err != CE_None) {
            for (auto g : geometries) OGR_G_DestroyGeometry(g);
            GDALClose(shp_ds);
            return false;
        }

        int band_list[1] = { 1 };
        vector<double> burn_values(geometries.size(), 1.0);
        CPLErr rasterize_err = (CPLErr)GDALRasterizeGeometries(
            temp_ds, 1, band_list,
            (int)geometries.size(), geometries.data(),
            nullptr, nullptr, burn_values.data(),
            nullptr, nullptr, nullptr
        );
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);

        if (rasterize_err != CE_None) { cerr << "[matrix] ERROR: rasterization failed\n"; return false; }

        // read burned boundary into temp_buffer
        CPLErr read_err = band->RasterIO(GF_Read, 0, 0, COLS, ROWS, temp_buffer, COLS, ROWS, GDT_Byte, 0, 0);
        if (read_err != CE_None) { cerr << "[matrix] ERROR: RasterIO failed\n"; return false; }

        // --- pixel expansion using distance transform ---
        // instead of expanding each border pixel (O(n*r²)),
        // use two-pass row/col scan — O(n) memory, O(n) time
        int radius = unfeasible_distance / RESOLUTION;
        cout << "[matrix] Expanding border by " << radius << " pixels (" << unfeasible_distance << "m)\n";

        // pass 1 — for each cell compute distance to nearest border pixel
        // using 1D scan per row then per col (separable distance transform approximation)
        vector<int> dist(total_cells, INT_MAX);

        // mark border pixels as distance 0
        for (size_t i = 0; i < total_cells; i++)
            if (temp_buffer[i] == 1) dist[i] = 0;

        // forward scan — left to right, top to bottom
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                size_t idx = (size_t)r * COLS + c;
                if (c > 0) dist[idx] = min(dist[idx], dist[idx - 1] == INT_MAX ? INT_MAX : dist[idx - 1] + 1);
                if (r > 0) dist[idx] = min(dist[idx], dist[idx - COLS] == INT_MAX ? INT_MAX : dist[idx - COLS] + 1);
            }
        }

        // backward scan — right to left, bottom to top
        for (int r = ROWS - 1; r >= 0; r--) {
            for (int c = COLS - 1; c >= 0; c--) {
                size_t idx = (size_t)r * COLS + c;
                if (c < COLS - 1) dist[idx] = min(dist[idx], dist[idx + 1] == INT_MAX ? INT_MAX : dist[idx + 1] + 1);
                if (r < ROWS - 1) dist[idx] = min(dist[idx], dist[idx + COLS] == INT_MAX ? INT_MAX : dist[idx + COLS] + 1);
            }
        }

        // OR into master matrix wherever distance <= radius
        int burned = 0;
        for (size_t i = 0; i < total_cells; i++) {
            if (dist[i] <= radius) {
                matrix[i] |= (1 << bit_index);
                burned++;
            }
        }

        cout << "[matrix] IB burn complete — " << burned << " cells marked\n";
        return true;
    };

    if (!process_layer(road_ipath,         road_bit, from_road))       goto cleanup;
       if (!process_layer(rail_ipath,         rail_bit,from_rail))       goto cleanup;
       if (!process_layer(waterarea_ipath,    waterarea_bit,from_water))  goto cleanup;
       if (!process_layer(waterlines_ipath,   waterlines_bit,from_water)) goto cleanup;
       if (!process_ib_layer(ib_ipath,        ib_bit))         goto cleanup;

    // --- write to disk ---
    f = fopen(opath.c_str(), "wb");
    if (!f) {
        cerr << "[matrix] ERROR: could not open india_map.bin for writing\n";
        free(temp_buffer);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }
    fwrite(matrix, 1, total_cells, f);
    fclose(f);
    cout << "[matrix] Written to india_map.bin\n";

    free(temp_buffer);
    GDALClose(temp_ds);
    free(matrix);
    return 0;

    cleanup:
        free(temp_buffer);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
}

int lib::refresh_layer_by_index(int bit_index, std::optional<std::string> path, const std::string& default_path) {
    string opath;
    Config::is_available()? opath = Config::load().output.matrix_file: opath = DEFAULT_PARAM_MATRIX;

    std::string shp_path = path.value_or(default_path);
    int ROWS = calc_rows(RESOLUTION);
    int COLS = calc_cols(RESOLUTION);
    size_t total_cells = (size_t)ROWS * COLS;

    cout << "[refresh] Starting refresh for bit " << bit_index << " using: " << shp_path << "\n";

    uint8_t* matrix = (uint8_t*)malloc(total_cells);
    if (!matrix) {
        cerr << "[refresh] ERROR: failed to allocate matrix memory\n";
        return -1;
    }

    FILE* f_in = fopen(opath.c_str(), "rb");
    if (!f_in) {
        cerr << "[refresh] WARNING: india_map.bin not found. Initialising clear matrix.\n";
        std::memset(matrix, 0, total_cells);
    } else {
        size_t read_bytes = fread(matrix, 1, total_cells, f_in);
        fclose(f_in);
        if (read_bytes != total_cells) {
            cerr << "[refresh] ERROR: size mismatch reading india_map.bin\n";
            free(matrix);
            return -1;
        }
        cout << "[refresh] Loaded existing matrix from india_map.bin\n";
    }

    uint8_t clear_mask = ~(1 << bit_index);
    for (size_t i = 0; i < total_cells; i++) {
        matrix[i] &= clear_mask;
    }
    cout << "[refresh] Cleared bit " << bit_index << " from all cells\n";

    GDALDriver* mem_driver = GetGDALDriverManager()->GetDriverByName("MEM");
    GDALDataset* temp_ds = mem_driver->Create("", COLS, ROWS, 1, GDT_Byte, nullptr);
    if (!temp_ds) {
        cerr << "[refresh] ERROR: failed to create MEM raster\n";
        free(matrix);
        return -1;
    }

    double geotransform[6] = { VEC_ORIGIN_X, RESOLUTION, 0, VEC_ORIGIN_Y, 0, -RESOLUTION };
    temp_ds->SetGeoTransform(geotransform);
    GDALRasterBand* band = temp_ds->GetRasterBand(1);

    GDALDataset* shp_ds = (GDALDataset*)GDALOpenEx(shp_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!shp_ds) {
        cerr << "[refresh] ERROR: could not open " << shp_path << " — aborting, matrix NOT written\n";
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    OGRLayer* layer = shp_ds->GetLayer(0);
    if (!layer) {
        cerr << "[refresh] ERROR: no layer in " << shp_path << " — aborting, matrix NOT written\n";
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    vector<OGRGeometryH> geometries;
    OGRFeature* feature;
    layer->ResetReading();
    while ((feature = layer->GetNextFeature()) != nullptr) {
        OGRGeometry* geom = feature->GetGeometryRef();
        if (geom) geometries.push_back((OGRGeometryH)geom->clone());
        OGRFeature::DestroyFeature(feature);
    }
    cout << "[refresh] Loaded " << geometries.size() << " geometries\n";

    CPLErr fill_err = band->Fill(0);
    if (fill_err != CE_None) {
        cerr << "[refresh] ERROR: failed to zero band\n";
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    int band_list[1] = { 1 };
    vector<double> burn_values(geometries.size(), 1.0);

    CPLErr rasterize_err = (CPLErr)GDALRasterizeGeometries(
        temp_ds, 1, band_list,
        (int)geometries.size(), geometries.data(),
        nullptr, nullptr,
        burn_values.data(),
        nullptr, nullptr, nullptr
    );
    if (rasterize_err != CE_None) {
        cerr << "[refresh] ERROR: rasterization failed\n";
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }
    cout << "[refresh] Rasterization complete\n";

    uint8_t* temp_buffer = (uint8_t*)calloc(total_cells, 1);
    if (!temp_buffer) {
        cerr << "[refresh] ERROR: failed to allocate temp buffer\n";
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    CPLErr read_err = band->RasterIO(GF_Read, 0, 0, COLS, ROWS, temp_buffer, COLS, ROWS, GDT_Byte, 0, 0);
    if (read_err != CE_None) {
        cerr << "[refresh] ERROR: RasterIO failed\n";
        free(temp_buffer);
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    for (size_t i = 0; i < total_cells; i++) {
        matrix[i] |= (temp_buffer[i] << bit_index);
    }
    cout << "[refresh] Merged new layer at bit " << bit_index << "\n";

    FILE* f_out = fopen(opath.c_str(), "wb");
    if (!f_out) {
        cerr << "[refresh] ERROR: could not open india_map.bin for writing\n";
        free(temp_buffer);
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }
    fwrite(matrix, 1, total_cells, f_out);
    fclose(f_out);
    cout << "[refresh] Successfully written india_map.bin for bit " << bit_index << "\n";

    free(temp_buffer);
    free(matrix);
    for (auto g : geometries) OGR_G_DestroyGeometry(g);
    GDALClose(shp_ds);
    GDALClose(temp_ds);

    return 0;
}

int Bin::refresh_road_layer(std::optional<std::string> path) {
    return refresh_layer_by_index(BIT_ROAD, path, DEFAULT_ROAD_SHP);
}

int Bin::refresh_rail_layer(std::optional<std::string> path) {
    return refresh_layer_by_index(BIT_RAIL, path, DEFAULT_RAIL_SHP);
}

int Bin::refresh_water_area_layer(std::optional<std::string> path) {
    return refresh_layer_by_index(BIT_WATER_AREA, path, DEFAULT_WATER_AREA_SHP);
}

int Bin::refresh_water_line_layer(std::optional<std::string> path) {
    return refresh_layer_by_index(BIT_WATER_LINE, path, DEFAULT_WATER_LINE_SHP);
}
int Bin::refresh_ib_layer(std::optional<std::string> path) {
    std::string shp_path = path.value_or(
        Config::is_available() ? Config::load().input.ib : DEFAULT_IB_SHP
    );
    int unfeasible_distance = Config::is_available() ? Config::load().distance.from_ib : DISTANCE_FROM_IB;
    int ROWS = calc_rows(RESOLUTION);
    int COLS = calc_cols(RESOLUTION);
    size_t total_cells = (size_t)ROWS * COLS;

    cout << "[refresh] Starting IB refresh, buffer=" << unfeasible_distance << "m\n";

    // --- 1. load existing matrix ---
    uint8_t* matrix = (uint8_t*)malloc(total_cells);
    if (!matrix) {
        cerr << "[refresh] ERROR: failed to allocate matrix\n";
        return -1;
    }

    string opath = Config::is_available() ? Config::load().output.matrix_file : DEFAULT_PARAM_MATRIX;

    FILE* f_in = fopen(opath.c_str(), "rb");
    if (!f_in) {
        cerr << "[refresh] WARNING: matrix file not found, initialising clear\n";
        memset(matrix, 0, total_cells);
    } else {
        size_t read = fread(matrix, 1, total_cells, f_in);
        fclose(f_in);
        if (read != total_cells) {
            cerr << "[refresh] ERROR: size mismatch reading matrix\n";
            free(matrix);
            return -1;
        }
    }

    // --- 2. clear IB bit ---
    uint8_t clear_mask = ~(1 << BIT_IB);
    for (size_t i = 0; i < total_cells; i++)
        matrix[i] &= clear_mask;
    cout << "[refresh] Cleared IB bit\n";

    // --- 3. setup GDAL ---
    GDALDriver* mem_driver = GetGDALDriverManager()->GetDriverByName("MEM");
    GDALDataset* temp_ds = mem_driver->Create("", COLS, ROWS, 1, GDT_Byte, nullptr);
    if (!temp_ds) {
        cerr << "[refresh] ERROR: failed to create MEM raster\n";
        free(matrix);
        return -1;
    }
    double geotransform[6] = { VEC_ORIGIN_X, RESOLUTION, 0, VEC_ORIGIN_Y, 0, -RESOLUTION };
    temp_ds->SetGeoTransform(geotransform);
    GDALRasterBand* band = temp_ds->GetRasterBand(1);

    // --- 4. open shapefile ---
    GDALDataset* shp_ds = (GDALDataset*)GDALOpenEx(shp_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!shp_ds) {
        cerr << "[refresh] ERROR: could not open " << shp_path << " — aborting\n";
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    OGRLayer* layer = shp_ds->GetLayer(0);
    if (!layer) {
        cerr << "[refresh] ERROR: no layer found — aborting\n";
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    // --- 5. collect buffered geometries ---
    vector<OGRGeometryH> geometries;
    OGRFeature* feature;
    layer->ResetReading();
    OGRGeometry* union_boundary = nullptr;
    layer->ResetReading();
    while ((feature = layer->GetNextFeature()) != nullptr) {
        OGRGeometry* geom = feature->GetGeometryRef();
        if (geom) {
            OGRGeometry* boundary = geom->Boundary();
            if (boundary) {
                if (!union_boundary) {
                    union_boundary = boundary;
                } else {
                    OGRGeometry* merged = union_boundary->Union(boundary);
                    delete union_boundary;
                    delete boundary;
                    union_boundary = merged;
                }
            }
        }
        OGRFeature::DestroyFeature(feature);
    }

    // buffer the single merged geometry once
    if (union_boundary) {
        OGRGeometry* buffered = union_boundary->Buffer(unfeasible_distance, 2);
        if (buffered) geometries.push_back((OGRGeometryH)buffered);
        delete union_boundary;
    }
    cout << "[refresh] Buffered " << geometries.size() << " IB geometries\n";

    if (geometries.empty()) {
        cerr << "[refresh] ERROR: no geometries after buffering — aborting\n";
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    // --- 6. rasterize ---
    CPLErr fill_err = band->Fill(0);
    if (fill_err != CE_None) {
        cerr << "[refresh] ERROR: band fill failed\n";
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    int band_list[1] = { 1 };
    vector<double> burn_values(geometries.size(), 1.0);
    CPLErr rasterize_err = (CPLErr)GDALRasterizeGeometries(
        temp_ds, 1, band_list,
        (int)geometries.size(), geometries.data(),
        nullptr, nullptr, burn_values.data(),
        nullptr, nullptr, nullptr
    );
    if (rasterize_err != CE_None) {
        cerr << "[refresh] ERROR: rasterization failed\n";
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    // --- 7. read + OR ---
    uint8_t* temp_buffer = (uint8_t*)calloc(total_cells, 1);
    if (!temp_buffer) {
        cerr << "[refresh] ERROR: failed to allocate temp buffer\n";
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    CPLErr read_err = band->RasterIO(GF_Read, 0, 0, COLS, ROWS, temp_buffer, COLS, ROWS, GDT_Byte, 0, 0);
    if (read_err != CE_None) {
        cerr << "[refresh] ERROR: RasterIO failed\n";
        free(temp_buffer);
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }

    for (size_t i = 0; i < total_cells; i++)
        matrix[i] |= (temp_buffer[i] << BIT_IB);
    cout << "[refresh] ORed IB buffer into matrix\n";

    // --- 8. write back ---
    FILE* f_out = fopen(opath.c_str(), "wb");
    if (!f_out) {
        cerr << "[refresh] ERROR: could not open matrix for writing\n";
        free(temp_buffer);
        for (auto g : geometries) OGR_G_DestroyGeometry(g);
        GDALClose(shp_ds);
        GDALClose(temp_ds);
        free(matrix);
        return -1;
    }
    fwrite(matrix, 1, total_cells, f_out);
    fclose(f_out);
    cout << "[refresh] IB layer refreshed successfully\n";

    // --- 9. cleanup ---
    free(temp_buffer);
    free(matrix);
    for (auto g : geometries) OGR_G_DestroyGeometry(g);
    GDALClose(shp_ds);
    GDALClose(temp_ds);
    return 0;
}

int lib::layerName(Layer_vals val) {
    switch (val) {
        case ROADS:       return BIT_ROAD;
        case RAILS:       return BIT_RAIL;
        case WATER_LINES: return BIT_WATER_LINE;
        case WATER_AREAS: return BIT_WATER_AREA;
        case IB:          return BIT_IB;
        case ALT:         return ALT;
        default:          return -1;
    }
}
