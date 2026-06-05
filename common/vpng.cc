#pragma once

/******************************************************************************
 * COVERAGE PNG GENERATOR
 *
 * Generates a top-down PNG of radar coverage overlaid on terrain heightmap.
 *
 * Visual layers (bottom to top):
 *   1. Shaded terrain heightmap (hillshade)
 *   2. Visibility overlay     (green=visible, dark red=hidden, within radius)
 *   3. Radar source marker
 *   4. Range circle
 *   5. Legend + stats
 *
 ******************************************************************************/

#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <png.h>
#include "../common.h"

namespace png {

/******************************************************************************
 * RGBA
 ******************************************************************************/

struct RGBA { uint8_t r, g, b, a; };

/******************************************************************************
 * BLEND
 * Alpha blend src over dst
 ******************************************************************************/

static inline RGBA blend(RGBA dst, RGBA src)
{
    float a = src.a / 255.0f;
    return {
        (uint8_t)(src.r * a + dst.r * (1.0f - a)),
        (uint8_t)(src.g * a + dst.g * (1.0f - a)),
        (uint8_t)(src.b * a + dst.b * (1.0f - a)),
        255
    };
}

/******************************************************************************
 * SET PIXEL
 ******************************************************************************/

static inline void setPixel(
    std::vector<uint8_t>& img,
    int W, int H,
    int x, int y, RGBA c)
{
    if (x < 0 || y < 0 || x >= W || y >= H) return;
    size_t idx = ((size_t)y * W + x) * 4;
    img[idx+0] = c.r;
    img[idx+1] = c.g;
    img[idx+2] = c.b;
    img[idx+3] = c.a;
}

static inline void blendPixel(
    std::vector<uint8_t>& img,
    int W, int H,
    int x, int y, RGBA src)
{
    if (x < 0 || y < 0 || x >= W || y >= H) return;
    size_t idx = ((size_t)y * W + x) * 4;
    RGBA dst = { img[idx+0], img[idx+1], img[idx+2], img[idx+3] };
    RGBA out = blend(dst, src);
    img[idx+0] = out.r;
    img[idx+1] = out.g;
    img[idx+2] = out.b;
    img[idx+3] = out.a;
}

/******************************************************************************
 * DRAW FILLED CIRCLE
 ******************************************************************************/

static void drawFilledCircle(
    std::vector<uint8_t>& img,
    int W, int H,
    int cx, int cy, int r, RGBA c)
{
    for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++)
        if (dx*dx + dy*dy <= r*r)
            blendPixel(img, W, H, cx+dx, cy+dy, c);
}

/******************************************************************************
 * DRAW CIRCLE OUTLINE (Bresenham)
 ******************************************************************************/

static void drawCircleOutline(
    std::vector<uint8_t>& img,
    int W, int H,
    int cx, int cy, int r, RGBA c, int thickness = 1)
{
    for (int t = 0; t < thickness; t++)
    {
        int rr = r - t;
        if (rr <= 0) break;
        int x = 0, y = rr, d = 3 - 2 * rr;
        while (y >= x)
        {
            blendPixel(img, W, H, cx+x, cy+y, c);
            blendPixel(img, W, H, cx-x, cy+y, c);
            blendPixel(img, W, H, cx+x, cy-y, c);
            blendPixel(img, W, H, cx-x, cy-y, c);
            blendPixel(img, W, H, cx+y, cy+x, c);
            blendPixel(img, W, H, cx-y, cy+x, c);
            blendPixel(img, W, H, cx+y, cy-x, c);
            blendPixel(img, W, H, cx-y, cy-x, c);
            if (d < 0) d += 4*x+6;
            else { d += 4*(x-y)+10; y--; }
            x++;
        }
    }
}

/******************************************************************************
 * DRAW LINE (Bresenham)
 ******************************************************************************/

static void drawLine(
    std::vector<uint8_t>& img,
    int W, int H,
    int x0, int y0, int x1, int y1,
    RGBA c, bool dashed = false)
{
    int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
    int err = dx-dy, step = 0;
    while (true)
    {
        if (!dashed || (step/4)%2==0)
            blendPixel(img, W, H, x0, y0, c);
        if (x0==x1 && y0==y1) break;
        int e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
        step++;
    }
}

/******************************************************************************
 * TINY FONT (5x7 bitmap)
 ******************************************************************************/

static const uint8_t FONT[][5] = {
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
    {0x00,0x00,0x00,0x00,0x00}, // ' ' 10
    {0x7E,0x11,0x11,0x11,0x7E}, // D  11
    {0x7F,0x49,0x49,0x49,0x41}, // E  12
    {0x7F,0x09,0x09,0x09,0x06}, // F  13
    {0x7F,0x40,0x40,0x40,0x40}, // L  14
    {0x7F,0x09,0x19,0x29,0x46}, // R  15
    {0x32,0x49,0x49,0x49,0x26}, // S  16
    {0x7F,0x02,0x0C,0x02,0x7F}, // M  17
    {0x3E,0x41,0x41,0x41,0x3E}, // O  18
    {0x7F,0x08,0x08,0x08,0x7F}, // H  19
    {0x00,0x41,0x7F,0x41,0x00}, // I  20
    {0x7F,0x40,0x40,0x40,0x7F}, // U  21
    {0x7F,0x49,0x49,0x49,0x36}, // B  22
    {0x3E,0x41,0x41,0x41,0x22}, // C  23
    {0x41,0x63,0x55,0x49,0x41}, // X  24
    {0x63,0x14,0x08,0x14,0x63}, // * -> use for radar icon 25
    {0x00,0x00,0x5F,0x00,0x00}, // !  26
    {0x06,0x09,0x09,0x06,0x00}, // °  27
    {0x08,0x08,0x2A,0x1C,0x08}, // arrow right 28
    {0x36,0x49,0x55,0x22,0x50}, // %  29
    {0x7F,0x41,0x41,0x41,0x3E}, // D again -> use for 'D' in "RADAR" 30
    {0x01,0x01,0x7F,0x01,0x01}, // T  31
    {0x7C,0x12,0x11,0x12,0x7C}, // A  32
    {0x7F,0x09,0x09,0x09,0x01}, // P  33
    {0x3E,0x45,0x49,0x51,0x3E}, // G  34 (close enough)
    {0x7F,0x04,0x08,0x10,0x7F}, // N  35
    {0x00,0x36,0x36,0x00,0x00}, // :  36
    {0x08,0x08,0x08,0x08,0x08}, // -  37
    {0x14,0x14,0x14,0x14,0x14}, // =  38
    {0x00,0x00,0x00,0x00,0x00}, // placeholder 39
    {0x20,0x10,0x08,0x04,0x02}, // /  40
    {0x36,0x49,0x49,0x49,0x36}, // 0-dup for K placeholder 41
    {0x7F,0x08,0x14,0x22,0x41}, // K  42
    {0x40,0x3C,0x04,0x3C,0x40}, // V  43 (close)
    {0x7C,0x04,0x18,0x04,0x7C}, // W  44
    {0x41,0x22,0x14,0x08,0x00}, // Y  45 (simplified)
};

static int fontIdx(char c)
{
    if (c>='0'&&c<='9') return c-'0';
    switch(c) {
        case ' ': return 10;
        case 'D': return 11;
        case 'E': return 12;
        case 'F': return 13;
        case 'L': return 14;
        case 'R': return 15;
        case 'S': return 16;
        case 'M': return 17;
        case 'O': return 18;
        case 'H': return 19;
        case 'I': return 20;
        case 'U': return 21;
        case 'B': return 22;
        case 'C': return 23;
        case 'X': return 24;
        case '!': return 26;
        case ':': return 36;
        case '-': return 37;
        case 'T': return 31;
        case 'A': return 32;
        case 'P': return 33;
        case 'G': return 34;
        case 'N': return 35;
        case 'K': return 42;
        case 'V': return 43;
        case 'W': return 44;
        case 'Y': return 45;
        case '/': return 40;
        case '.': return 10;
    }
    return 10;
}

static void drawChar(
    std::vector<uint8_t>& img,
    int W, int H,
    int px, int py, char c,
    RGBA col, int scale=1)
{
    int fi = fontIdx(c);
    for (int col_=0; col_<5; col_++)
    {
        uint8_t bits = FONT[fi][col_];
        for (int row=0; row<7; row++)
            if (bits & (1<<row))
                for (int sy=0; sy<scale; sy++)
                for (int sx=0; sx<scale; sx++)
                    blendPixel(img,W,H,
                        px+col_*scale+sx,
                        py+row*scale+sy, col);
    }
}

static void drawText(
    std::vector<uint8_t>& img,
    int W, int H,
    int px, int py,
    const std::string& s,
    RGBA col, int scale=1)
{
    int x=px;
    for (char c:s)
    {
        drawChar(img,W,H,x,py,c,col,scale);
        x += (5+1)*scale;
    }
}

static void drawRect(
    std::vector<uint8_t>& img,
    int W, int H,
    int x, int y, int w, int h,
    RGBA c)
{
    for (int ry=y; ry<y+h; ry++)
    for (int rx=x; rx<x+w; rx++)
        blendPixel(img,W,H,rx,ry,c);
}

/******************************************************************************
 * HILLSHADE
 *
 * Computes a shading value [0,1] for cell (x,y) based on simulated
 * light direction. Makes terrain 3D-looking.
 *
 * Uses Sobel filter on elevation to compute surface normal,
 * then dots with light direction.
 *
 ******************************************************************************/

static float hillshade(
    const std::vector<short>& elev,
    int COLS, int ROWS,
    int x, int y)
{
    auto e = [&](int cx, int cy) -> float {
        cx = std::clamp(cx, 0, COLS-1);
        cy = std::clamp(cy, 0, ROWS-1);
        float v = (float)elev[(size_t)cx * COLS + cy];
        return v < -9000 ? 0.0f : v;
    };

    // Sobel gradient
    float gx = (e(x+1,y-1) + 2*e(x+1,y) + e(x+1,y+1))
              -(e(x-1,y-1) + 2*e(x-1,y) + e(x-1,y+1));
    float gy = (e(x-1,y+1) + 2*e(x,y+1) + e(x+1,y+1))
              -(e(x-1,y-1) + 2*e(x,y-1) + e(x+1,y-1));

    // surface normal (unnormalized, z=8*cellsize)
    float z  = 8.0f * (float)RESOLUTION;
    float len = std::sqrt(gx*gx + gy*gy + z*z);
    float nx = -gx/len, ny = -gy/len, nz = z/len;

    // light from NW, elevated 45°
    const float lx =  0.5774f;
    const float ly = -0.5774f;
    const float lz =  0.5774f;

    float shade = nx*lx + ny*ly + nz*lz;
    return std::clamp(shade * 0.6f + 0.5f, 0.15f, 1.0f);
}

/******************************************************************************
 * HEIGHT TO COLOR
 *
 * Maps elevation to a terrain color (hypsometric tinting).
 * Low = green, mid = brown, high = white (snow)
 *
 ******************************************************************************/

static RGBA elevColor(float elev, float minE, float maxE)
{
    float t = (elev - minE) / (maxE - minE + 1.0f);
    t = std::clamp(t, 0.0f, 1.0f);

    // color stops: water -> lowland -> highland -> rock -> snow
    struct Stop { float t; uint8_t r,g,b; };
    static const Stop stops[] = {
        {0.00f,  70, 130, 180}, // deep blue (water/low)
        {0.10f,  60, 140,  60}, // green (lowland)
        {0.35f, 140, 160,  80}, // yellow-green
        {0.55f, 160, 120,  60}, // brown
        {0.75f, 130,  90,  60}, // dark brown
        {0.90f, 180, 170, 160}, // rock grey
        {1.00f, 240, 240, 250}, // snow white
    };
    const int NS = 7;

    for (int i = 0; i < NS-1; i++)
    {
        if (t >= stops[i].t && t <= stops[i+1].t)
        {
            float f = (t - stops[i].t) / (stops[i+1].t - stops[i].t);
            return {
                (uint8_t)(stops[i].r + f*(stops[i+1].r - stops[i].r)),
                (uint8_t)(stops[i].g + f*(stops[i+1].g - stops[i].g)),
                (uint8_t)(stops[i].b + f*(stops[i+1].b - stops[i].b)),
                255
            };
        }
    }
    return {240,240,250,255};
}

/******************************************************************************
 * GENERATE COVERAGE PNG
 *
 * Parameters:
 *      cov         -> CoverageResult from radarCoverage()
 *      elev_matrix -> raw elevation grid (bin.elev_matrix)
 *      filename    -> output PNG path
 *      maxPxSize   -> max output image dimension in pixels (default 1024)
 *                     grid is downsampled if larger
 *
 ******************************************************************************/

inline void generateCoveragePng(
    const CoverageResult&        cov,
    const std::vector<short>&    elev_matrix,
    const std::string&           filename  = "coverage.png",
    int                          maxPxSize = 1024)
{
    if (cov.visibility.empty()) return;

    const int COLS = cov.gridCols;
    const int ROWS = cov.gridRows;
    const int R    = cov.radiusCells;

    // -------------------------------------------------------------------------
    // CROP TO COVERAGE AREA + MARGIN
    // Only render the bounding box of the circle, not the whole DEM
    // -------------------------------------------------------------------------

    const int MARGIN = (int)(R * 0.08) + 10;

    int cropX0 = std::max(0,    cov.srcX - R - MARGIN);
    int cropY0 = std::max(0,    cov.srcY - R - MARGIN);
    int cropX1 = std::min(COLS, cov.srcX + R + MARGIN);
    int cropY1 = std::min(ROWS, cov.srcY + R + MARGIN);

    int cropW = cropX1 - cropX0;
    int cropH = cropY1 - cropY0;

    // -------------------------------------------------------------------------
    // DOWNSAMPLE IF NEEDED
    // -------------------------------------------------------------------------

    int scale = 1;
    while (cropW / scale > maxPxSize || cropH / scale > maxPxSize)
        scale++;

    int imgW = cropW / scale;
    int imgH = cropH / scale;

    // legend area at bottom
    const int LEGEND_H = 60;
    const int FULL_H   = imgH + LEGEND_H;

    std::vector<uint8_t> img((size_t)imgW * FULL_H * 4, 0);

    // -------------------------------------------------------------------------
    // FIND ELEVATION RANGE IN CROP AREA
    // -------------------------------------------------------------------------

    float minE = 1e9f, maxE = -1e9f;
    for (int gx = cropX0; gx < cropX1; gx += scale)
    for (int gy = cropY0; gy < cropY1; gy += scale)
    {
        float e = (float)elev_matrix[(size_t)gx * COLS + gy];
        if (e < -9000) continue;
        minE = std::min(minE, e);
        maxE = std::max(maxE, e);
    }

    // -------------------------------------------------------------------------
    // LAYER 1: TERRAIN HEIGHTMAP WITH HILLSHADE
    // -------------------------------------------------------------------------

    for (int py = 0; py < imgH; py++)
    for (int px = 0; px < imgW; px++)
    {
        int gx = cropX0 + px * scale;
        int gy = cropY0 + py * scale;

        float e = (float)elev_matrix[(size_t)gx * COLS + gy];
        if (e < -9000) e = minE;

        RGBA  tc    = elevColor(e, minE, maxE);
        float shade = hillshade(elev_matrix, COLS, ROWS, gx, gy);

        setPixel(img, imgW, FULL_H, px, py, {
            (uint8_t)(tc.r * shade),
            (uint8_t)(tc.g * shade),
            (uint8_t)(tc.b * shade),
            255
        });
    }

    // -------------------------------------------------------------------------
    // LAYER 2: VISIBILITY OVERLAY (within circle only)
    // -------------------------------------------------------------------------

    // colors
    // visible   : semi-transparent green
    // hidden    : semi-transparent dark red
    // outside R : no overlay (terrain shows through)

    for (int py = 0; py < imgH; py++)
    for (int px = 0; px < imgW; px++)
    {
        int gx = cropX0 + px * scale;
        int gy = cropY0 + py * scale;

        int ddx = gx - cov.srcX;
        int ddy = gy - cov.srcY;

        // only overlay inside the radius
        if (ddx*ddx + ddy*ddy > R*R) continue;

        uint8_t vis = cov.visibility[(size_t)gx * COLS + gy];

        RGBA overlay;
        if (vis)
            overlay = {50, 200, 80,  120};  // green, semi-transparent
        else
            overlay = {180, 30, 20,  100};  // dark red, semi-transparent

        blendPixel(img, imgW, FULL_H, px, py, overlay);
    }

    // -------------------------------------------------------------------------
    // LAYER 3: RANGE CIRCLE OUTLINE
    // -------------------------------------------------------------------------

    int srcPX = (cov.srcX - cropX0) / scale;
    int srcPY = (cov.srcY - cropY0) / scale;
    int radiusPX = R / scale;

    drawCircleOutline(img, imgW, FULL_H, srcPX, srcPY, radiusPX,
                      {255, 220, 50, 200}, 2);

    // -------------------------------------------------------------------------
    // LAYER 4: CROSSHAIR + SOURCE MARKER
    // -------------------------------------------------------------------------

    // crosshair lines
    drawLine(img, imgW, FULL_H,
             srcPX - 12, srcPY, srcPX - 4, srcPY,
             {255,255,255,220});
    drawLine(img, imgW, FULL_H,
             srcPX + 4,  srcPY, srcPX + 12, srcPY,
             {255,255,255,220});
    drawLine(img, imgW, FULL_H,
             srcPX, srcPY - 12, srcPX, srcPY - 4,
             {255,255,255,220});
    drawLine(img, imgW, FULL_H,
             srcPX, srcPY + 4,  srcPX, srcPY + 12,
             {255,255,255,220});

    // source dot
    drawFilledCircle(img, imgW, FULL_H, srcPX, srcPY, 5,
                     {255, 80, 80, 255});
    drawCircleOutline(img, imgW, FULL_H, srcPX, srcPY, 5,
                      {255, 255, 255, 200}, 1);

    // -------------------------------------------------------------------------
    // LAYER 5: CARDINAL DIRECTION LABELS
    // -------------------------------------------------------------------------

    RGBA labelCol = {255, 255, 255, 200};
    int  labelR   = radiusPX + 12;

    drawText(img, imgW, FULL_H,
             srcPX - 3,          srcPY - labelR - 10, "N", labelCol, 2);
    drawText(img, imgW, FULL_H,
             srcPX - 3,          srcPY + labelR + 4,  "S", labelCol, 2);
    drawText(img, imgW, FULL_H,
             srcPX + labelR + 4, srcPY - 7,           "E", labelCol, 2);
    drawText(img, imgW, FULL_H,
             srcPX - labelR - 16,srcPY - 7,           "W", labelCol, 2);

    // -------------------------------------------------------------------------
    // LAYER 6: LEGEND BAR
    // -------------------------------------------------------------------------

    // dark background for legend
    drawRect(img, imgW, FULL_H, 0, imgH, imgW, LEGEND_H, {20,20,20,255});

    // visible swatch
    drawRect(img, imgW, FULL_H, 10, imgH+8, 16, 16, {50,200,80,255});
    drawText(img, imgW, FULL_H, 30, imgH+10, "VISIBLE", {200,200,200,255}, 1);

    // hidden swatch
    drawRect(img, imgW, FULL_H, 110, imgH+8, 16, 16, {180,30,20,255});
    drawText(img, imgW, FULL_H, 130, imgH+10, "HIDDEN", {200,200,200,255}, 1);

    // source swatch
    drawFilledCircle(img, imgW, FULL_H, 220, imgH+16, 6, {255,80,80,255});
    drawText(img, imgW, FULL_H, 230, imgH+10, "SOURCE", {200,200,200,255}, 1);

    // stats
    int visCount = 0;
    int totCount = 0;
    for (int gx=cropX0; gx<cropX1; gx++)
    for (int gy=cropY0; gy<cropY1; gy++)
    {
        int ddx = gx - cov.srcX;
        int ddy = gy - cov.srcY;
        if (ddx*ddx + ddy*ddy > R*R) continue;
        totCount++;
        visCount += cov.visibility[(size_t)gx * COLS + gy];
    }

    int pct = totCount > 0 ? (visCount * 100 / totCount) : 0;

    std::string statsStr =
        "CVG " + std::to_string(pct) + "  " +
        "VIS " + std::to_string(visCount) +
        " TOT " + std::to_string(totCount);

    drawText(img, imgW, FULL_H, 10, imgH + 34, statsStr, {160,160,160,255}, 1);

    // scale bar (rough)
    int scaleBarCells = R / 4;
    int scaleBarPx    = scaleBarCells / scale;
    int scaleBarM     = (int)(scaleBarCells * RESOLUTION);
    int sbX = imgW - scaleBarPx - 20;
    int sbY = imgH + 20;

    drawLine(img, imgW, FULL_H, sbX, sbY, sbX + scaleBarPx, sbY,
             {200,200,200,255});
    drawLine(img, imgW, FULL_H, sbX, sbY-3, sbX, sbY+3,
             {200,200,200,255});
    drawLine(img, imgW, FULL_H, sbX+scaleBarPx, sbY-3, sbX+scaleBarPx, sbY+3,
             {200,200,200,255});

    std::string scaleStr;
    if (scaleBarM >= 1000)
        scaleStr = std::to_string(scaleBarM/1000) + "KM";
    else
        scaleStr = std::to_string(scaleBarM) + "M";
    drawText(img, imgW, FULL_H,
             sbX + scaleBarPx/2 - (int)(scaleStr.size()*3),
             sbY + 6, scaleStr, {200,200,200,255}, 1);

    // -------------------------------------------------------------------------
    // WRITE PNG
    // -------------------------------------------------------------------------

    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) { printf("Cannot open %s\n", filename.c_str()); return; }

    png_structp png  = png_create_write_struct(PNG_LIBPNG_VER_STRING,
                                                NULL, NULL, NULL);
    png_infop   info = png_create_info_struct(png);

    if (setjmp(png_jmpbuf(png))) { fclose(fp); return; }

    png_init_io(png, fp);
    png_set_IHDR(png, info, imgW, FULL_H, 8,
                 PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    for (int y = 0; y < FULL_H; y++)
        png_write_row(png, img.data() + (size_t)y * imgW * 4);

    png_write_end(png, NULL);
    png_destroy_write_struct(&png, &info);
    fclose(fp);

    printf("Coverage PNG saved: %s  (%dx%d)  coverage=%d%%\n",
           filename.c_str(), imgW, FULL_H, pct);
}

} // namespace png

//auto cov = g.radarCoverage(34.017, 74.505, 30, 50000.0, true, 0);
//png::generateCoveragePng(cov, g.bin.elev_matrix, "coverage.png", 1024);
//
/**
 *
 *
 *
 * Terrain base — hypsometric color (blue→green→brown→grey→white by elevation) with hillshading via Sobel filter so ridges and valleys look 3D.
 Visibility overlay — semi-transparent green for visible cells, dark red for hidden, only drawn inside the radius. Terrain shows through the transparency so you can see exactly which hills are causing dead zones.
 Range circle — yellow outline marking the exact coverage boundary.
 Source marker — red dot with white crosshair so the radar position is obvious.
 Cardinal labels — N/S/E/W outside the circle.
 Legend bar — color swatches, coverage percentage, visible cell count, total cell count, and a scale bar in km/m at bottom right.
 Crop + downsample — only renders the bounding box of the circle, not the full 33k×33k grid. Downsamples automatically if the crop area exceeds maxPxSize pixels, so a 50km radius at 90m/cell (~1111 cells wide) fits cleanly in 1024px.
 */
