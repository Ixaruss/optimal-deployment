#include "../common.h"
#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <png.h>
#include <algorithm>

namespace png {

    struct RGBA { uint8_t r, g, b, a; };

    /******************************************************************************
     * DRAW HELPERS
     ******************************************************************************/

    static void drawRect(
        std::vector<uint8_t>& img,
        int W, int H,
        int x, int y, int w, int h,
        RGBA col)
    {
        for (int ry = y; ry < y + h; ry++)
        for (int rx = x; rx < x + w; rx++)
        {
            if (rx < 0 || ry < 0 || rx >= W || ry >= H) continue;
            size_t idx = (ry * W + rx) * 4;
            img[idx+0] = col.r;
            img[idx+1] = col.g;
            img[idx+2] = col.b;
            img[idx+3] = col.a;
        }
    }

    static void drawHLine(
        std::vector<uint8_t>& img,
        int W, int H,
        int x0, int x1, int y,
        RGBA col, bool dashed = false)
    {
        for (int x = x0; x <= x1; x++)
        {
            if (dashed && ((x / 6) % 2 == 1)) continue;
            if (x < 0 || x >= W || y < 0 || y >= H) continue;
            size_t idx = (y * W + x) * 4;
            img[idx+0] = col.r;
            img[idx+1] = col.g;
            img[idx+2] = col.b;
            img[idx+3] = col.a;
        }
    }

    // Tiny bitmap font: digits 0-9 and letters, 5x7
    // Encodes each char as 5 columns of 7 bits (bottom = bit0)
    static const uint8_t FONT5x7[][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, // 0
        {0x00,0x42,0x7F,0x40,0x00}, // 1
        {0x42,0x61,0x51,0x49,0x46}, // 2
        {0x21,0x41,0x45,0x4B,0x31}, // 3
        {0x18,0x14,0x12,0x7F,0x10}, // 4
        {0x27,0x45,0x45,0x45,0x39}, // 5
        {0x3C,0x4A,0x49,0x49,0x30}, // 6
        {0x01,0x71,0x09,0x05,0x03}, // 7
        {0x36,0x49,0x49,0x49,0x36}, // 8
        {0x06,0x49,0x49,0x29,0x1E}, // 9
        {0x00,0x00,0x00,0x00,0x00}, // ' ' (10)
        {0x7C,0x12,0x11,0x12,0x7C}, // A (11)
        {0x00,0x36,0x36,0x00,0x00}, // : (12)
        {0x08,0x1C,0x2A,0x08,0x08}, // arrow up (13)
        {0x7F,0x49,0x49,0x49,0x36}, // B (14) - used for labels
    };

    static int charIndex(char c)
    {
        if (c >= '0' && c <= '9') return c - '0';
        if (c == ' ') return 10;
        if (c == 'A') return 11;
        if (c == ':') return 12;
        return 10;
    }

    static void drawChar(
        std::vector<uint8_t>& img,
        int W, int H,
        int px, int py,
        char c, RGBA col, int scale = 1)
    {
        int idx = charIndex(c);
        for (int col_ = 0; col_ < 5; col_++)
        {
            uint8_t bits = FONT5x7[idx][col_];
            for (int row = 0; row < 7; row++)
            {
                if (bits & (1 << row))
                {
                    drawRect(img, W, H,
                        px + col_*scale,
                        py + row*scale,
                        scale, scale, col);
                }
            }
        }
    }

    static void drawText(
        std::vector<uint8_t>& img,
        int W, int H,
        int px, int py,
        const std::string& s,
        RGBA col, int scale = 1)
    {
        int x = px;
        for (char c : s)
        {
            drawChar(img, W, H, x, py, c, col, scale);
            x += (5 + 1) * scale;
        }
    }

    /******************************************************************************
     * GENERATE LOS PNG
     *
     * Parameters:
     *      results    -> LOSResult vector from lineOfVisibility()
     *      filename   -> output PNG path
     *
     ******************************************************************************/

    template<typename LOSResult>
    void generateLOSPng(
        const std::vector<LOSResult>& results,
        const std::string& filename)
    {
        if (results.empty()) return;

        // -------------------------------------------------------------------------
        // LAYOUT
        // -------------------------------------------------------------------------

        const int PAD_L   = 60;   // left padding (for y-axis labels)
        const int PAD_R   = 20;
        const int PAD_T   = 30;
        const int PAD_B   = 50;   // bottom padding (for x labels)

        const int BAR_W   = 36;   // bar width px
        const int BAR_GAP = 6;    // gap between bars
        const int CHART_H = 280;  // drawable chart height

        int N = (int)results.size();
        int W = PAD_L + N * (BAR_W + BAR_GAP) - BAR_GAP + PAD_R;
        int H = PAD_T + CHART_H + PAD_B + 60; // +60 for legend

        std::vector<uint8_t> img(W * H * 4, 0);

        // Background: dark #1a1a1a
        for (int i = 0; i < W * H; i++)
        {
            img[i*4+0] = 26;
            img[i*4+1] = 26;
            img[i*4+2] = 26;
            img[i*4+3] = 255;
        }

        // -------------------------------------------------------------------------
        // FIND ELEVATION RANGE
        // -------------------------------------------------------------------------

        float minElev =  1e9f;
        float maxElev = -1e9f;

        for (auto& r : results)
        {
            if (r.elev < -9000) continue;
            minElev = std::min(minElev, (float)r.elev);
            maxElev = std::max(maxElev, (float)r.elev);
        }

        if (maxElev <= minElev) maxElev = minElev + 1;

        // -------------------------------------------------------------------------
        // COLORS
        // -------------------------------------------------------------------------

        RGBA C_BG      = {26,  26,  26,  255};
        RGBA C_SOURCE  = {83,  74,  183, 255}; // purple
        RGBA C_VISIBLE = {15,  110, 86,  255}; // green
        RGBA C_HIDDEN  = {153, 60,  29,  255}; // red
        RGBA C_BLOCKER = {186, 117, 23,  255}; // amber
        RGBA C_GRID    = {50,  50,  50,  255}; // grid lines
        RGBA C_TEXT    = {180, 180, 180, 255};
        RGBA C_RAY     = {56,  122, 173, 100};

        // -------------------------------------------------------------------------
        // DETERMINE BAR COLOR
        // -------------------------------------------------------------------------

        auto barColor = [&](int i) -> RGBA {
            auto& r = results[i];
            if (i == 0)        return C_SOURCE;
            if (!r.visible)    return C_HIDDEN;
            if (r.angle > r.maxAngle) return C_BLOCKER;
            return C_VISIBLE;
        };

        // -------------------------------------------------------------------------
        // HELPER: elevation -> y pixel
        // -------------------------------------------------------------------------

        auto elevToY = [&](float elev) -> int {
            float t = (elev - minElev) / (maxElev - minElev);
            return PAD_T + CHART_H - (int)(t * (CHART_H - 20)) - 10;
        };

        // -------------------------------------------------------------------------
        // DRAW GRID LINES (horizontal)
        // -------------------------------------------------------------------------

        for (int gi = 0; gi <= 4; gi++)
        {
            float elev = minElev + gi * (maxElev - minElev) / 4.0f;
            int   gy   = elevToY(elev);
            drawHLine(img, W, H, PAD_L, W - PAD_R, gy, C_GRID, true);

            // label
            std::string lbl = std::to_string((int)elev);
            drawText(img, W, H, 2, gy - 3, lbl, C_TEXT, 1);
        }

        // -------------------------------------------------------------------------
        // DRAW RAY LINE (dashed, source top to last bar top)
        // -------------------------------------------------------------------------

        int srcBarX = PAD_L + (BAR_W / 2);
        int srcBarY = elevToY(results[0].elev);
        int dstBarX = PAD_L + (N-1) * (BAR_W + BAR_GAP) + BAR_W / 2;
        int dstBarY = elevToY(results[N-1].elev);

        // Simple Bresenham line for the ray
        {
            int x0 = srcBarX, y0 = srcBarY;
            int x1 = dstBarX, y1 = dstBarY;
            int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
            int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
            int err = dx-dy, step=0;
            while(true) {
                if (step % 10 < 6) // dashed
                {
                    if (x0>=0&&y0>=0&&x0<W&&y0<H) {
                        size_t idx=(y0*W+x0)*4;
                        img[idx+0]=56; img[idx+1]=122; img[idx+2]=173; img[idx+3]=120;
                    }
                }
                if(x0==x1&&y0==y1) break;
                int e2=2*err;
                if(e2>-dy){err-=dy;x0+=sx;}
                if(e2< dx){err+=dx;y0+=sy;}
                step++;
            }
        }

        // -------------------------------------------------------------------------
        // DRAW BARS
        // -------------------------------------------------------------------------

        for (int i = 0; i < N; i++)
        {
            auto& r   = results[i];
            int   bx  = PAD_L + i * (BAR_W + BAR_GAP);
            int   top = elevToY(r.elev < -9000 ? minElev : r.elev);
            int   bot = PAD_T + CHART_H;
            int   bh  = bot - top;
            if (bh < 2) bh = 2;

            RGBA col = barColor(i);

            // Bar with slight rounded top feel (draw main bar)
            drawRect(img, W, H, bx, top, BAR_W, bh, col);

            // Slightly lighter top edge
            RGBA topEdge = {
                (uint8_t)std::min(255, col.r + 40),
                (uint8_t)std::min(255, col.g + 40),
                (uint8_t)std::min(255, col.b + 40),
                255
            };
            drawRect(img, W, H, bx, top, BAR_W, 2, topEdge);

            // Elevation label on top of bar
            std::string elevStr = std::to_string((int)(r.elev < -9000 ? 0 : r.elev));
            int labelX = bx + BAR_W/2 - (int)(elevStr.size() * 3);
            drawText(img, W, H, labelX, top - 10, elevStr, C_TEXT, 1);

            // Cell index below bar
            std::string idxStr = std::to_string(i);
            int idxX = bx + BAR_W/2 - (int)(idxStr.size() * 3);
            drawText(img, W, H, idxX, PAD_T + CHART_H + 8, idxStr, C_TEXT, 1);
        }

        // SRC label above source bar
        drawText(img, W, H, PAD_L + 2, PAD_T + CHART_H - elevToY(results[0].elev) - 20,
                 "0", {150,140,230,255}, 1);

        // -------------------------------------------------------------------------
        // LEGEND
        // -------------------------------------------------------------------------

        int legendY = PAD_T + CHART_H + 35;
        int legendX = PAD_L;

        struct LegItem { RGBA col; const char* label; };
        LegItem items[] = {
            {C_SOURCE,  "SOURCE"},
            {C_VISIBLE, "VISIBLE"},
            {C_HIDDEN,  "HIDDEN"},
            {C_BLOCKER, "BLOCKER"},
        };

        for (auto& item : items)
        {
            drawRect(img, W, H, legendX, legendY, 10, 10, item.col);
            drawText(img, W, H, legendX + 14, legendY + 1, item.label, C_TEXT, 1);
            legendX += 80;
        }

        // -------------------------------------------------------------------------
        // ENCODE PNG
        // -------------------------------------------------------------------------

        FILE* fp = fopen(filename.c_str(), "wb");
            if (!fp) { printf("Failed to open %s\n", filename.c_str()); return; }

            png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
            png_infop   info = png_create_info_struct(png);

            if (setjmp(png_jmpbuf(png))) {
                fclose(fp);
                return;
            }

            png_init_io(png, fp);
            png_set_IHDR(png, info, W, H, 8,
                PNG_COLOR_TYPE_RGBA,
                PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_DEFAULT,
                PNG_FILTER_TYPE_DEFAULT);
            png_write_info(png, info);

            // write row by row
            for (int y = 0; y < H; y++)
                png_write_row(png, img.data() + y * W * 4);

            png_write_end(png, NULL);
            png_destroy_write_struct(&png, &info);
            fclose(fp);
            printf("elev range: %.1f to %.1f\n", minElev, maxElev);
            printf("LOS chart saved: %s (%dx%d)\n", filename.c_str(), W, H);
    }

}
