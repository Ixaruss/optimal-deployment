#include <png.h>
#include "../bin.h"


using namespace std;
using namespace lib;
void Bin::visualize(const std::string& output_path) {
    // add temporarily at start of visualize() for cell r=0, c=0
    double px = VEC_ORIGIN_X + (0 * RESOLUTION);
    double py = VEC_ORIGIN_Y - (0 * RESOLUTION);
    int ec = floor((px - ELEV_ORIGIN_X) / RESOLUTION);
    int er = floor((ELEV_ORIGIN_Y - py) / RESOLUTION);
    cout << "[viz] top-left vec cell maps to elev cell: r=" << er << " c=" << ec << "\n";
    // ec should be >= 0, er should be >= 0
    // if ec is negative, ELEV_ORIGIN_X is too large

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
        cerr << "[viz] ERROR: matrix size mismatch\n";
        return;
    }

    // --- open PNG file ---
    FILE* f = fopen(output_path.c_str(), "wb");
    if (!f) {
        cerr << "[viz] ERROR: could not open " << output_path << "\n";
        return;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        cerr << "[viz] ERROR: png_create_write_struct failed\n";
        fclose(f);
        return;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        cerr << "[viz] ERROR: png_create_info_struct failed\n";
        png_destroy_write_struct(&png, nullptr);
        fclose(f);
        return;
    }

    if (setjmp(png_jmpbuf(png))) {
        cerr << "[viz] ERROR: PNG write error\n";
        png_destroy_write_struct(&png, &info);
        fclose(f);
        return;
    }

    png_init_io(png, f);
    png_set_IHDR(
        png, info,
        COLS, ROWS,
        8,
        PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT
    );
    png_write_info(png, info);

    cout << "[viz] Grid  : " << ROWS << " x " << COLS << "\n";
    cout << "[viz] Output: " << output_path << "\n";

    // write one row at a time — never holds full image in memory
    std::vector<uint8_t> row_buf(COLS * 3);

    for (int r = 0; r < ROWS; r++) {
        if (r % 2000 == 0 && r > 0)
            cout << "[viz] Row " << r << "/" << ROWS
                 << " (" << (int)((r / (double)ROWS) * 100) << "%)\n";

        for (int c = 0; c < COLS; c++) {
            size_t idx   = (size_t)r * COLS + c;
            uint8_t cell = vec_matrix[idx];

            uint8_t red = 20, green = 24, blue = 38;

            // elevation base
            if (elevation_loaded) {
                double proj_x = VEC_ORIGIN_X + (c * RESOLUTION);
                double proj_y = VEC_ORIGIN_Y - (r * RESOLUTION);
                int elev_c    = std::floor((proj_x - ELEV_ORIGIN_X) / RESOLUTION);
                int elev_r    = std::floor((ELEV_ORIGIN_Y - proj_y)  / RESOLUTION);
                if (elev_r >= 0 && elev_r < ELEV_ROWS &&
                    elev_c >= 0 && elev_c < ELEV_COLS) {
                    int16_t alt = elev_matrix[(size_t)elev_r * ELEV_COLS + elev_c];
                    get_elevation_color(alt, red, green, blue);
                }
            }

            // 2. water
            if (cell & (1 << BIT_WATER_AREA))  { red = 0;   green = 80;  blue = 180; }
            if (cell & (1 << BIT_WATER_LINE))  { red = 0;   green = 140; blue = 255; }

            // 3. infrastructure
            if (cell & (1 << BIT_RAIL))        { red = 255; green = 220; blue = 0;   }
            if (cell & (1 << BIT_ROAD))        { red = 255; green = 80;  blue = 0;   }

            // 4. intersections
            if ((cell & (1 << BIT_ROAD)) && (cell & (1 << BIT_RAIL)))
                                               { red = 0;   green = 255; blue = 255; }

            // 5. IB last — overrides everything so border zone is always visible
            if (cell & (1 << BIT_IB))          { red = 80;  green = 0;   blue = 80;  }

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
