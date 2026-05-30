#include <png.h>
#include "../bin.h"

using namespace std;
using namespace lib;

void Bin::visualize(const std::string& output_path) {

    if (VEC_ROWS == 0 || VEC_COLS == 0) {
        VEC_ROWS = calc_rows(RESOLUTION);
        VEC_COLS = calc_cols(RESOLUTION);
    }

    if (!ensure_matrices_loaded()) {
        cerr << "[viz] ERROR: failed to load matrices\n";
        return;
    }

    int ROWS = VEC_ROWS;
    int COLS = VEC_COLS;

    if (vec_matrix.size() != (size_t)ROWS * COLS) {
        cerr << "[viz] ERROR: vec_matrix size mismatch — "
             << vec_matrix.size() << " vs expected " << (size_t)ROWS * COLS << "\n";
        return;
    }

    // ── elev grid params — from config if available, else hardcoded ────────
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

    // ── sanity check: verify top-left vec cell maps into elev grid ─────────
    {
        double px = VEC_ORIGIN_X + (0 * RESOLUTION);
        double py = VEC_ORIGIN_Y - (0 * RESOLUTION);
        int ec = (int)floor((px - ELEV_OX) / RESOLUTION);
        int er = (int)floor((ELEV_OY - py) / RESOLUTION);
        cout << "[viz] top-left vec cell → elev cell: r=" << er << " c=" << ec << "\n";
        if (ec < 0 || er < 0)
            cerr << "[viz] WARN: elev origin mismatch — some cells will have no elevation\n";
    }

    // ── open PNG ───────────────────────────────────────────────────────────
    FILE* f = fopen(output_path.c_str(), "wb");
    if (!f) {
        cerr << "[viz] ERROR: could not open " << output_path << "\n";
        return;
    }

    png_structp png  = png_create_write_struct(
        PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        cerr << "[viz] ERROR: png_create_write_struct failed\n";
        fclose(f); return;
    }
    png_infop info = png_create_info_struct(png);
    if (!info) {
        cerr << "[viz] ERROR: png_create_info_struct failed\n";
        png_destroy_write_struct(&png, nullptr);
        fclose(f); return;
    }
    if (setjmp(png_jmpbuf(png))) {
        cerr << "[viz] ERROR: PNG write error\n";
        png_destroy_write_struct(&png, &info);
        fclose(f); return;
    }

    png_init_io(png, f);
    png_set_IHDR(png, info,
                 (png_uint_32)COLS, (png_uint_32)ROWS,
                 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    cout << "[viz] Grid  : " << ROWS << " x " << COLS << "\n";
    cout << "[viz] Output: " << output_path << "\n";

    // allocate one row buffer — never holds full image in RAM
    vector<uint8_t> row_buf((size_t)COLS * 3);

    for (int r = 0; r < ROWS; r++) {
        if (r % 2000 == 0 && r > 0)
            cout << "[viz] Row " << r << "/" << ROWS
                 << " (" << (int)(r * 100.0 / ROWS) << "%)\n";

        for (int c = 0; c < COLS; c++) {
            uint8_t cell  = vec_matrix[(size_t)r * COLS + c];
            uint8_t red   = 20, green = 24, blue = 38;   // dark background

            // 1. elevation base — read ElevCell.elevation
            if (elevation_loaded) {
                double proj_x = VEC_ORIGIN_X + (c * RESOLUTION);
                double proj_y = VEC_ORIGIN_Y - (r * RESOLUTION);
                int elev_c    = (int)floor((proj_x - ELEV_OX) / RESOLUTION);
                int elev_r    = (int)floor((ELEV_OY - proj_y)  / RESOLUTION);
                if (elev_r >= 0 && elev_r < E_ROWS &&
                    elev_c >= 0 && elev_c < E_COLS) {
                    int16_t alt = elev_matrix[
                        (size_t)elev_r * E_COLS + elev_c].elevation;
                    get_elevation_color(alt, red, green, blue);
                }
            }

            // 2. water
            if (cell & (1 << BIT_WATER_AREA)) { red = 0;   green = 80;  blue = 180; }
            if (cell & (1 << BIT_WATER_LINE)) { red = 0;   green = 140; blue = 255; }

            // 3. infrastructure
            if (cell & (1 << BIT_RAIL))       { red = 255; green = 220; blue = 0;   }
            if (cell & (1 << BIT_ROAD))       { red = 255; green = 80;  blue = 0;   }

            // 4. road+rail intersection
            if ((cell & (1 << BIT_ROAD)) && (cell & (1 << BIT_RAIL)))
                                              { red = 0;   green = 255; blue = 255; }

            // 5. IB — always on top
            if (cell & (1 << BIT_IB))         { red = 80;  green = 0;   blue = 80;  }

            size_t buf_idx       = (size_t)c * 3;
            row_buf[buf_idx]     = red;
            row_buf[buf_idx + 1] = green;
            row_buf[buf_idx + 2] = blue;
        }

        png_write_row(png, row_buf.data());
    }

    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    fclose(f);

    cout << "[viz] Done → " << output_path << "\n";
}

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
