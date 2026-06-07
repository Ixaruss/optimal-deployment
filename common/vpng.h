#pragma once

/******************************************************************************
 * BATCH COVERAGE PNG GENERATOR — MEMORY OPTIMIZED
 *
 * Key fix: visibleCount and inRangeCount are allocated only over the
 * crop bounding box, not the full DEM grid.
 *
 * Full grid (33k x 33k) = 1.1B cells — way too large.
 * Crop box (union of all radar ranges) = manageable.
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
 * HELPERS
 ******************************************************************************/

struct RGBA2 { uint8_t r, g, b, a; };

static inline RGBA2 blend2(RGBA2 dst, RGBA2 src)
{
    float a = src.a / 255.0f;
    return {
        (uint8_t)(src.r*a + dst.r*(1-a)),
        (uint8_t)(src.g*a + dst.g*(1-a)),
        (uint8_t)(src.b*a + dst.b*(1-a)),
        255
    };
}

static inline void setPixel2(std::vector<uint8_t>& img,int W,int H,
                               int x,int y,RGBA2 c)
{
    if(x<0||y<0||x>=W||y>=H) return;
    size_t i=((size_t)y*W+x)*4;
    img[i]=c.r; img[i+1]=c.g; img[i+2]=c.b; img[i+3]=c.a;
}

static inline void blendPixel2(std::vector<uint8_t>& img,int W,int H,
                                 int x,int y,RGBA2 src)
{
    if(x<0||y<0||x>=W||y>=H) return;
    size_t i=((size_t)y*W+x)*4;
    RGBA2 dst={img[i],img[i+1],img[i+2],img[i+3]};
    RGBA2 out=blend2(dst,src);
    img[i]=out.r; img[i+1]=out.g; img[i+2]=out.b; img[i+3]=out.a;
}

static inline void drawRect2(std::vector<uint8_t>& img,int W,int H,
                               int x,int y,int w,int h,RGBA2 c)
{
    for(int ry=y;ry<y+h;ry++)
    for(int rx=x;rx<x+w;rx++)
        blendPixel2(img,W,H,rx,ry,c);
}

static inline void drawCircleOutline2(std::vector<uint8_t>& img,int W,int H,
                                       int cx,int cy,int r,RGBA2 c,int thick=1)
{
    for(int t=0;t<thick;t++){
        int rr=r-t; if(rr<=0) break;
        int x=0,y=rr,d=3-2*rr;
        while(y>=x){
            auto p=[&](int px,int py){ blendPixel2(img,W,H,px,py,c); };
            p(cx+x,cy+y);p(cx-x,cy+y);p(cx+x,cy-y);p(cx-x,cy-y);
            p(cx+y,cy+x);p(cx-y,cy+x);p(cx+y,cy-x);p(cx-y,cy-x);
            if(d<0) d+=4*x+6; else{d+=4*(x-y)+10;y--;} x++;
        }
    }
}

static inline void drawFilledCircle2(std::vector<uint8_t>& img,int W,int H,
                                      int cx,int cy,int r,RGBA2 c)
{
    for(int dy=-r;dy<=r;dy++)
    for(int dx=-r;dx<=r;dx++)
        if(dx*dx+dy*dy<=r*r)
            blendPixel2(img,W,H,cx+dx,cy+dy,c);
}

static inline void drawLine2(std::vector<uint8_t>& img,int W,int H,
                               int x0,int y0,int x1,int y1,RGBA2 c,bool dashed=false)
{
    int dx=std::abs(x1-x0),dy=std::abs(y1-y0);
    int sx=x0<x1?1:-1,sy=y0<y1?1:-1,err=dx-dy,step=0;
    while(true){
        if(!dashed||(step/5)%2==0) blendPixel2(img,W,H,x0,y0,c);
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>-dy){err-=dy;x0+=sx;}
        if(e2< dx){err+=dx;y0+=sy;}
        step++;
    }
}

static const uint8_t BFONT[][5]={
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x00,0x00,0x00,0x00},
    {0x7C,0x12,0x11,0x12,0x7C},{0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},{0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x06},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x19,0x29,0x46},{0x32,0x49,0x49,0x49,0x26},
    {0x7F,0x40,0x40,0x40,0x7F},{0x08,0x08,0x08,0x08,0x08},
    {0x7F,0x09,0x09,0x09,0x01},{0x41,0x22,0x14,0x08,0x00},
    {0x00,0x36,0x36,0x00,0x00},
};
static int bfontIdx(char c){
    if(c>='0'&&c<='9') return c-'0';
    switch(c){
        case ' ':return 10; case 'A':return 11; case 'B':return 12;
        case 'C':return 13; case 'D':return 14; case 'E':return 15;
        case 'F':return 16; case 'H':return 17; case 'I':return 18;
        case 'L':return 19; case 'M':return 20; case 'N':return 21;
        case 'O':return 22; case 'R':return 23; case 'S':return 24;
        case 'U':return 25; case '-':return 26; case 'P':return 27;
        case 'Y':return 28; case ':':return 29;
    }
    return 10;
}
static void drawText2(std::vector<uint8_t>& img,int W,int H,
                       int px,int py,const std::string& s,RGBA2 col,int scale=1)
{
    int x=px;
    for(char c:s){
        int fi=bfontIdx(c);
        for(int cx=0;cx<5;cx++){
            uint8_t bits=BFONT[fi][cx];
            for(int row=0;row<7;row++)
                if(bits&(1<<row))
                    for(int sy=0;sy<scale;sy++)
                    for(int sx=0;sx<scale;sx++)
                        blendPixel2(img,W,H,x+cx*scale+sx,py+row*scale+sy,col);
        }
        x+=(5+1)*scale;
    }
}

static float hillshade2(const std::vector<short>& elev,int COLS,int ROWS,int x,int y)
{
    auto e=[&](int cx,int cy)->float{
        cx=std::clamp(cx,0,COLS-1); cy=std::clamp(cy,0,ROWS-1);
        float v=(float)elev[(size_t)cy*COLS+cx];
        return v<-9000?0:v;
    };
    float gx=(e(x+1,y-1)+2*e(x+1,y)+e(x+1,y+1))-(e(x-1,y-1)+2*e(x-1,y)+e(x-1,y+1));
    float gy=(e(x-1,y+1)+2*e(x,y+1)+e(x+1,y+1))-(e(x-1,y-1)+2*e(x,y-1)+e(x+1,y-1));
    float z=8.0f*(float)RESOLUTION;
    float len=std::sqrt(gx*gx+gy*gy+z*z);
    float nx=-gx/len,ny=-gy/len,nz=z/len;
    float shade=nx*0.5774f+ny*(-0.5774f)+nz*0.5774f;
    return std::clamp(shade*0.6f+0.5f,0.15f,1.0f);
}

static RGBA2 elevColor2(float e,float minE,float maxE)
{
    float t=std::clamp((e-minE)/(maxE-minE+1.0f),0.0f,1.0f);
    struct S{float t;uint8_t r,g,b;};
    static const S s[]={
        {0.00f,70,130,180},{0.10f,60,140,60},
        {0.35f,140,160,80},{0.55f,160,120,60},
        {0.75f,130,90,60},{0.90f,180,170,160},
        {1.00f,240,240,250}
    };
    for(int i=0;i<6;i++){
        if(t>=s[i].t&&t<=s[i+1].t){
            float f=(t-s[i].t)/(s[i+1].t-s[i].t);
            return{(uint8_t)(s[i].r+f*(s[i+1].r-s[i].r)),
                   (uint8_t)(s[i].g+f*(s[i+1].g-s[i].g)),
                   (uint8_t)(s[i].b+f*(s[i+1].b-s[i].b)),255};
        }
    }
    return{240,240,250,255};
}

static const RGBA2 RADAR_COLORS[]={
    {255, 80, 80,255},{ 80,180,255,255},{255,200, 50,255},{ 80,255,150,255},
    {220, 80,255,255},{255,140, 50,255},{ 80,230,230,255},{255,100,180,255},
    {180,255, 80,255},{200,150,255,255},
};
static const int N_RADAR_COLORS=10;

/******************************************************************************
 * GENERATE BATCH COVERAGE PNG
 ******************************************************************************/

inline void generateBatchCoveragePng(
    const std::vector<CoverageResult>& coverages,
    const std::vector<short>&          elev_matrix,
    const std::string&                 filename  = "batch_coverage.png",
    int                                maxPxSize = 2048)
{
    if(coverages.empty()) return;

    const int COLS = coverages[0].gridCols;
    const int ROWS = coverages[0].gridRows;
    const int NR   = (int)coverages.size();

    // -------------------------------------------------------------------------
    // STEP 1: CROP BOUNDING BOX (union of all radar ranges + margin)
    // Allocate count arrays ONLY over this box — not the full grid
    // -------------------------------------------------------------------------

    int cropX0=COLS, cropY0=ROWS, cropX1=0, cropY1=0;
    for(const auto& cov:coverages){
        int margin = (int)(cov.radiusCells*0.05)+10;
        int R = cov.radiusCells + margin;
        cropX0=std::max(0,    std::min(cropX0, cov.srcX-R));
        cropY0=std::max(0,    std::min(cropY0, cov.srcY-R));
        cropX1=std::min(COLS, std::max(cropX1, cov.srcX+R));
        cropY1=std::min(ROWS, std::max(cropY1, cov.srcY+R));
    }

    int cropW = cropX1 - cropX0;
    int cropH = cropY1 - cropY0;
    if(cropW<=0||cropH<=0) return;

    printf("Crop box: %dx%d cells  (%.1f MB for count arrays)\n",
           cropW, cropH,
           2.0 * cropW * cropH / 1e6);

    // -------------------------------------------------------------------------
    // STEP 2: ALLOCATE COUNT ARRAYS OVER CROP BOX ONLY
    //
    // Index: (gy - cropY0) * cropW + (gx - cropX0)
    //
    // -------------------------------------------------------------------------

    std::vector<uint8_t> visibleCount((size_t)cropW * cropH, 0);
    std::vector<uint8_t> inRangeCount((size_t)cropW * cropH, 0);

    // helper: crop-local index
    auto cropIdx = [&](int gx, int gy) -> size_t {
        return (size_t)(gy - cropY0) * cropW + (gx - cropX0);
    };

    // -------------------------------------------------------------------------
    // STEP 3: FILL COUNT ARRAYS FROM EACH RADAR'S VISIBILITY GRID
    // Only iterate within each radar's bounding box, clipped to crop
    // -------------------------------------------------------------------------

    for(int ri=0; ri<NR; ri++)
    {
        const auto& cov = coverages[ri];
        int R  = cov.radiusCells;
        int sx = cov.srcX, sy = cov.srcY;

        // radar's bbox clipped to crop
        int bx0 = std::max(cropX0, sx-R);
        int by0 = std::max(cropY0, sy-R);
        int bx1 = std::min(cropX1, sx+R+1);
        int by1 = std::min(cropY1, sy+R+1);

        for(int gx=bx0; gx<bx1; gx++)
        for(int gy=by0; gy<by1; gy++)
        {
            int ddx=gx-sx, ddy=gy-sy;
            if(ddx*ddx+ddy*ddy > R*R) continue;

            // visibility grid is indexed [gy*COLS+gx] in coverage result
            size_t visIdx = (size_t)gy * COLS + gx;
            size_t cIdx   = cropIdx(gx, gy);

            if(inRangeCount[cIdx] < 255) inRangeCount[cIdx]++;
            if(cov.visibility[visIdx] && visibleCount[cIdx] < 255)
                visibleCount[cIdx]++;
        }
    }

    // -------------------------------------------------------------------------
    // STEP 4: DOWNSAMPLE
    // -------------------------------------------------------------------------

    int scale=1;
    while(cropW/scale > maxPxSize || cropH/scale > maxPxSize) scale++;

    int imgW = cropW / scale;
    int imgH = cropH / scale;

    const int LEGEND_H = 80;
    const int FULL_H   = imgH + LEGEND_H;

    std::vector<uint8_t> img((size_t)imgW * FULL_H * 4, 0);

    // -------------------------------------------------------------------------
    // STEP 5: ELEVATION RANGE IN CROP
    // -------------------------------------------------------------------------

    float minE=1e9f, maxE=-1e9f;
    for(int gx=cropX0; gx<cropX1; gx+=scale)
    for(int gy=cropY0; gy<cropY1; gy+=scale)
    {
        float e=(float)elev_matrix[(size_t)gy*COLS+gx];
        if(e<-9000) continue;
        minE=std::min(minE,e); maxE=std::max(maxE,e);
    }

    // -------------------------------------------------------------------------
    // STEP 6: TERRAIN BASE
    // -------------------------------------------------------------------------

    for(int py=0; py<imgH; py++)
    for(int px=0; px<imgW; px++)
    {
        int gx=cropX0+px*scale;
        int gy=cropY0+py*scale;

        float e=(float)elev_matrix[(size_t)gy*COLS+gx];
        if(e<-9000) e=minE;

        RGBA2 tc=elevColor2(e,minE,maxE);
        float sh=hillshade2(elev_matrix,COLS,ROWS,gx,gy);

        setPixel2(img,imgW,FULL_H,px,py,{
            (uint8_t)(tc.r*sh*0.5f),
            (uint8_t)(tc.g*sh*0.5f),
            (uint8_t)(tc.b*sh*0.5f),
            255
        });
    }

    // -------------------------------------------------------------------------
    // STEP 7: COVERAGE OVERLAY
    // -------------------------------------------------------------------------

    for(int py=0; py<imgH; py++)
    for(int px=0; px<imgW; px++)
    {
        int gx=cropX0+px*scale;
        int gy=cropY0+py*scale;

        size_t ci = cropIdx(gx, gy);
        uint8_t inR  = inRangeCount[ci];
        uint8_t visC = visibleCount[ci];

        if(inR==0) continue;

        RGBA2 ov;
        if(visC==0)      ov={160, 20, 20,130}; // dead zone
        else if(visC==1) ov={ 30,200, 70,130}; // single coverage
        else             ov={255,230, 30,160}; // overlap

        blendPixel2(img,imgW,FULL_H,px,py,ov);
    }

    // -------------------------------------------------------------------------
    // STEP 8: OVERLAP BORDER
    // -------------------------------------------------------------------------

    for(int py=1; py<imgH-1; py++)
    for(int px=1; px<imgW-1; px++)
    {
        int gx=cropX0+px*scale;
        int gy=cropY0+py*scale;
        size_t ci=cropIdx(gx,gy);
        if(visibleCount[ci]<2) continue;

        const int ndx[]={1,-1,0,0};
        const int ndy[]={0,0,1,-1};
        for(int d=0;d<4;d++){
            int ngx=gx+ndx[d]*scale, ngy=gy+ndy[d]*scale;
            if(ngx<cropX0||ngy<cropY0||ngx>=cropX1||ngy>=cropY1) continue;
            if(visibleCount[cropIdx(ngx,ngy)]==1){
                setPixel2(img,imgW,FULL_H,px,py,{0,230,255,255});
                break;
            }
        }
    }

    // -------------------------------------------------------------------------
    // STEP 9: RANGE CIRCLES + MARKERS
    // -------------------------------------------------------------------------

    for(int ri=0; ri<NR; ri++)
    {
        const auto& cov=coverages[ri];
        RGBA2 rc=RADAR_COLORS[ri%N_RADAR_COLORS];

        int spx=(cov.srcX-cropX0)/scale;
        int spy=(cov.srcY-cropY0)/scale;
        int rpx=cov.radiusCells/scale;

        drawCircleOutline2(img,imgW,FULL_H,spx,spy,rpx,rc,2);
        drawLine2(img,imgW,FULL_H,spx-14,spy,spx-5,spy,rc);
        drawLine2(img,imgW,FULL_H,spx+5, spy,spx+14,spy,rc);
        drawLine2(img,imgW,FULL_H,spx,spy-14,spx,spy-5,rc);
        drawLine2(img,imgW,FULL_H,spx,spy+5, spx,spy+14,rc);
        drawFilledCircle2(img,imgW,FULL_H,spx,spy,5,rc);
        drawCircleOutline2(img,imgW,FULL_H,spx,spy,6,{255,255,255,200},1);
        drawText2(img,imgW,FULL_H,spx+9,spy-5,"R"+std::to_string(ri+1),rc,1);
    }

    // -------------------------------------------------------------------------
    // STEP 10: LEGEND
    // -------------------------------------------------------------------------

    drawRect2(img,imgW,FULL_H,0,imgH,imgW,LEGEND_H,{18,18,18,255});

    int ly=imgH+8, lx=10;
    struct LE{RGBA2 c; const char* l;};
    LE entries[]={
        {{30,200,70,255},"VISIBLE"},
        {{160,20,20,255},"DEAD ZONE"},
        {{255,230,30,255},"OVERLAP"},
        {{0,230,255,255},"OVERLAP BORDER"},
    };
    for(auto& en:entries){
        drawRect2(img,imgW,FULL_H,lx,ly,14,12,en.c);
        drawText2(img,imgW,FULL_H,lx+18,ly+2,en.l,{200,200,200,255},1);
        lx+=(int)(strlen(en.l)*7+30);
    }

    ly=imgH+28; lx=10;
    for(int ri=0;ri<std::min(NR,N_RADAR_COLORS);ri++){
        RGBA2 rc=RADAR_COLORS[ri%N_RADAR_COLORS];
        drawFilledCircle2(img,imgW,FULL_H,lx+6,ly+6,6,rc);
        drawText2(img,imgW,FULL_H,lx+16,ly+2,"R"+std::to_string(ri+1),rc,1);
        lx+=52;
        if(lx>imgW-60){lx=10;ly+=18;}
    }

    // stats
    int totalVis=0,totalOverlap=0,totalDead=0;
    for(size_t i=0;i<(size_t)cropW*cropH;i++){
        if(inRangeCount[i]==0) continue;
        if(visibleCount[i]>=2)      totalOverlap++;
        else if(visibleCount[i]==1) totalVis++;
        else                        totalDead++;
    }
    std::string stats=
        "RADARS-"+std::to_string(NR)+
        "  VIS-"+std::to_string(totalVis)+
        "  DEAD-"+std::to_string(totalDead)+
        "  OVERLAP-"+std::to_string(totalOverlap);
    drawText2(img,imgW,FULL_H,10,imgH+56,stats,{160,160,160,255},1);

    // -------------------------------------------------------------------------
    // STEP 11: WRITE PNG
    // -------------------------------------------------------------------------

    FILE* fp=fopen(filename.c_str(),"wb");
    if(!fp){printf("Cannot open %s\n",filename.c_str());return;}

    png_structp png =png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop   info=png_create_info_struct(png);
    if(setjmp(png_jmpbuf(png))){fclose(fp);return;}

    png_init_io(png,fp);
    png_set_IHDR(png,info,imgW,FULL_H,8,
                 PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png,info);

    for(int y=0;y<FULL_H;y++)
        png_write_row(png,img.data()+(size_t)y*imgW*4);

    png_write_end(png,NULL);
    png_destroy_write_struct(&png,&info);
    fclose(fp);

    printf("Batch coverage PNG: %s  (%dx%d)  radars=%d\n",
           filename.c_str(),imgW,FULL_H,NR);
}

} // namespace png
