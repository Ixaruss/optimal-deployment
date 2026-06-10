#include "../operations.h"

void ViewshedEngine::writeViewshedToSVG(const std::string& filename)
{
    auto tAll = std::chrono::high_resolution_clock::now();

    // centerX = row (north-south, increases southward)
    // centerY = col (east-west,  increases eastward)
    //
    // Image convention:
    //   px (horizontal) = east-west  = column axis
    //   py (vertical)   = north-south = row axis  (0=north at top)

    int rowMin = std::max(0,        centerX - maxRadiusCells - 2);
    int rowMax = std::min(gridRows, centerX + maxRadiusCells + 2);
    int colMin = std::max(0,        centerY - maxRadiusCells - 2);
    int colMax = std::min(gridCols, centerY + maxRadiusCells + 2);

    int W = colMax - colMin;   // image width  = number of columns (E-W)
    int H = rowMax - rowMin;   // image height = number of rows    (N-S)

    size_t total = (size_t)W * H;
    std::vector<uint8_t> rgba(total * 4, 0);

    long long rSq =
        static_cast<long long>(maxRadiusCells) * maxRadiusCells;

    auto t0 = std::chrono::high_resolution_clock::now();

    #pragma omp parallel for schedule(static, 4096)
    for (size_t i = 0; i < total; ++i)
    {
        int px  = (int)(i % W);          // horizontal → column
        int py  = (int)(i / W);          // vertical   → row
        int col = colMin + px;
        int row = rowMin + py;

        int dcol = col - centerY;        // column offset from source
        int drow = row - centerX;        // row offset from source

        if ((long long)dcol*dcol + (long long)drow*drow > rSq)
            continue;

        // [row * COLS + col] — standard row-major
        size_t gridIdx = (size_t)row * gridCols + col;
        size_t pix     = i * 4;

        if (viewshedMask[gridIdx]) {
            rgba[pix+0] = 46;  rgba[pix+1] = 204;
            rgba[pix+2] = 113; rgba[pix+3] = 255;
        } else {
            rgba[pix+0] = 231; rgba[pix+1] = 76;
            rgba[pix+2] = 60;  rgba[pix+3] = 255;
        }
    }

    // BGRA swap for BMP
    #pragma omp parallel for schedule(static, 4096)
    for (size_t i = 0; i < total; ++i)
        std::swap(rgba[i*4+0], rgba[i*4+2]);

    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "[METRIC] RGBA map + BGRA swap         : "
              << std::chrono::duration<double,std::milli>(t1-t0).count()
              << " ms\n";

    t0 = std::chrono::high_resolution_clock::now();
    uint32_t fs = 54 + (uint32_t)(total * 4);
    uint8_t hdr[54] = {
        'B','M',
        (uint8_t)fs,(uint8_t)(fs>>8),(uint8_t)(fs>>16),(uint8_t)(fs>>24),
        0,0,0,0, 54,0,0,0, 40,0,0,0,
        (uint8_t)W,(uint8_t)(W>>8),(uint8_t)(W>>16),(uint8_t)(W>>24),
        (uint8_t)(-H),(uint8_t)(-H>>8),(uint8_t)(-H>>16),(uint8_t)(-H>>24),
        1,0, 32,0, 0,0,0,0, 0,0,0,0,
        0,0,0,0, 0,0,0,0, 0,0,0,0
    };
    std::vector<uint8_t> payload;
    payload.reserve(54 + total * 4);
    payload.insert(payload.end(), hdr, hdr + 54);
    payload.insert(payload.end(), rgba.begin(), rgba.end());
    std::string b64 = base64Encode(payload.data(), payload.size());

    t1 = std::chrono::high_resolution_clock::now();
    std::cout << "[METRIC] Base64 encode                : "
              << std::chrono::duration<double,std::milli>(t1-t0).count()
              << " ms\n";

    t0 = std::chrono::high_resolution_clock::now();
    // center in image coords: col offset = horizontal, row offset = vertical
    int cx = (centerY - colMin);   // east-west  → image x
    int cy = (centerX - rowMin);   // north-south → image y
    std::ofstream f(filename);
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n"
      << "<svg xmlns=\"http://www.w3.org/2000/svg\""
      << " viewBox=\"0 0 " << W << " " << H << "\""
      << " width=\"100%\" height=\"100%\""
      << " style=\"background-color:#0a0a0a;\">\n"
      << "  <style>"
      << ".rm{fill:#FFF;stroke:#1abc9c;stroke-width:8}"
      << ".rr{fill:none;stroke:#FFF;stroke-width:3;"
      << "stroke-dasharray:20,15;opacity:.4}"
      << "</style>\n"
      << "  <image width=\""<<W<<"\" height=\""<<H
      << "\" x=\"0\" y=\"0\""
      << " href=\"data:image/bmp;base64,"<<b64<<"\"/>\n"
      << "  <circle cx=\""<<cx<<"\" cy=\""<<cy
      << "\" r=\""<<maxRadiusCells<<"\" class=\"rr\"/>\n"
      << "  <circle cx=\""<<cx<<"\" cy=\""<<cy
      << "\" r=\""<<maxRadiusCells/2<<"\" class=\"rr\"/>\n"
      << "  <circle cx=\""<<cx<<"\" cy=\""<<cy
      << "\" r=\"10\" class=\"rm\"/>\n"
      << "</svg>\n";

    t1 = std::chrono::high_resolution_clock::now();
    std::cout << "[METRIC] SVG write                    : "
              << std::chrono::duration<double,std::milli>(t1-t0).count()
              << " ms\n";

    auto tEnd = std::chrono::high_resolution_clock::now();
    std::cout << "[METRIC] Total SVG generation         : "
              << std::chrono::duration<double,std::milli>(tEnd-tAll).count()
              << " ms\n";
}

std::string ViewshedEngine::base64Encode(const uint8_t* data, size_t length)
{
    static const char lut[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((length + 2) / 3) * 4);
    size_t i = 0;
    for (; i + 2 < length; i += 3) {
        uint32_t v = ((uint32_t)data[i]<<16)|((uint32_t)data[i+1]<<8)|data[i+2];
        out.push_back(lut[(v>>18)&0x3F]);
        out.push_back(lut[(v>>12)&0x3F]);
        out.push_back(lut[(v>> 6)&0x3F]);
        out.push_back(lut[ v     &0x3F]);
    }
    if (i < length) {
        uint32_t v = (uint32_t)data[i] << 16;
        if (i+1 < length) v |= (uint32_t)data[i+1] << 8;
        out.push_back(lut[(v>>18)&0x3F]);
        out.push_back(lut[(v>>12)&0x3F]);
        out.push_back(i+1 < length ? lut[(v>>6)&0x3F] : '=');
        out.push_back('=');
    }
    return out;
}

void RadarEngine::writeToSVG(
    const RadarCoverageResult& res,
    const std::string& filename)
{
    auto tAll=std::chrono::high_resolution_clock::now();

    int W=colMax-colMin, H=rowMax-rowMin;
    size_t total=(size_t)W*H;
    std::vector<uint8_t> rgba(total*4,0);
    long long rSq=(long long)maxRadiusCells*maxRadiusCells;

    #pragma omp parallel for schedule(static,4096)
    for(size_t i=0;i<total;++i){
        int px=(int)(i%W), py=(int)(i/W);
        int col=colMin+px, row=rowMin+py;
        int dc=col-srcCol, dr=row-srcRow;
        if((long long)dc*dc+(long long)dr*dr>rSq) continue;
        size_t gi=(size_t)row*gridCols+col;
        size_t pix=i*4;
        switch((CoverageStatus)res.status[gi]){
            case CoverageStatus::COVERED:
                rgba[pix+0]=46; rgba[pix+1]=204;rgba[pix+2]=113;rgba[pix+3]=255; break;
            case CoverageStatus::TERRAIN_MASKED:
                rgba[pix+0]=231;rgba[pix+1]=76; rgba[pix+2]=60; rgba[pix+3]=255; break;
            case CoverageStatus::BELOW_MIN_ELEV:
                rgba[pix+0]=52; rgba[pix+1]=152;rgba[pix+2]=219;rgba[pix+3]=255; break;
            case CoverageStatus::ABOVE_MAX_ELEV:
                rgba[pix+0]=230;rgba[pix+1]=126;rgba[pix+2]=34; rgba[pix+3]=255; break;
            case CoverageStatus::BEYOND_HORIZON:
                rgba[pix+0]=155;rgba[pix+1]=89; rgba[pix+2]=182;rgba[pix+3]=255; break;
            case CoverageStatus::OUTSIDE_SECTOR:
                rgba[pix+0]=40; rgba[pix+1]=40; rgba[pix+2]=40; rgba[pix+3]=255; break;
            default: break;
        }
    }

    #pragma omp parallel for schedule(static,4096)
    for(size_t i=0;i<total;++i)
        std::swap(rgba[i*4+0],rgba[i*4+2]);

    uint32_t fs=54+(uint32_t)(total*4);
    uint8_t hdr[54]={'B','M',
        (uint8_t)fs,(uint8_t)(fs>>8),(uint8_t)(fs>>16),(uint8_t)(fs>>24),
        0,0,0,0,54,0,0,0,40,0,0,0,
        (uint8_t)W,(uint8_t)(W>>8),(uint8_t)(W>>16),(uint8_t)(W>>24),
        (uint8_t)(-H),(uint8_t)(-H>>8),(uint8_t)(-H>>16),(uint8_t)(-H>>24),
        1,0,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::vector<uint8_t> payload;
    payload.reserve(54+total*4);
    payload.insert(payload.end(),hdr,hdr+54);
    payload.insert(payload.end(),rgba.begin(),rgba.end());
    std::string b64=ViewshedEngine::base64Encode(payload.data(),payload.size());

    int cx=srcCol-colMin, cy=srcRow-rowMin;
    int rr=maxRadiusCells;

    std::ofstream f(filename);
    f<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
     <<"<svg xmlns=\"http://www.w3.org/2000/svg\""
     <<" viewBox=\"0 0 "<<W<<" "<<H<<"\" width=\"100%\" height=\"100%\""
     <<" style=\"background:#0a0a0a\">\n"
     // legend
     <<"  <rect x=\"10\" y=\"10\" width=\"12\" height=\"12\" fill=\"#2ecc71\"/>"
     <<"<text x=\"26\" y=\"21\" fill=\"#eee\" font-size=\"12\">COVERED</text>\n"
     <<"  <rect x=\"10\" y=\"28\" width=\"12\" height=\"12\" fill=\"#e74c3c\"/>"
     <<"<text x=\"26\" y=\"39\" fill=\"#eee\" font-size=\"12\">TERRAIN MASKED</text>\n"
     <<"  <rect x=\"10\" y=\"46\" width=\"12\" height=\"12\" fill=\"#3498db\"/>"
     <<"<text x=\"26\" y=\"57\" fill=\"#eee\" font-size=\"12\">BELOW MIN ELEV</text>\n"
     <<"  <rect x=\"10\" y=\"64\" width=\"12\" height=\"12\" fill=\"#e67e22\"/>"
     <<"<text x=\"26\" y=\"75\" fill=\"#eee\" font-size=\"12\">ABOVE MAX ELEV</text>\n"
     <<"  <rect x=\"10\" y=\"82\" width=\"12\" height=\"12\" fill=\"#9b59b6\"/>"
     <<"<text x=\"26\" y=\"93\" fill=\"#eee\" font-size=\"12\">BEYOND HORIZON</text>\n"
     <<"  <rect x=\"10\" y=\"100\" width=\"12\" height=\"12\" fill=\"#282828\""
     <<" stroke=\"#555\" stroke-width=\"1\"/>"
     <<"<text x=\"26\" y=\"111\" fill=\"#eee\" font-size=\"12\">OUTSIDE SECTOR</text>\n"
     // image
     <<"  <image width=\""<<W<<"\" height=\""<<H
     <<"\" x=\"0\" y=\"0\" href=\"data:image/bmp;base64,"<<b64<<"\"/>\n"
     // range circle
     <<"  <circle cx=\""<<cx<<"\" cy=\""<<cy<<"\" r=\""<<rr
     <<"\" fill=\"none\" stroke=\"white\" stroke-width=\"2\""
     <<" stroke-dasharray=\"20,15\" opacity=\"0.4\"/>\n"
     // horizon circle
     <<"  <circle cx=\""<<cx<<"\" cy=\""<<cy
     <<"\" r=\""<<(int)(horizonM/RESOLUTION)
     <<"\" fill=\"none\" stroke=\"#9b59b6\" stroke-width=\"2\""
     <<" stroke-dasharray=\"10,8\" opacity=\"0.5\"/>\n";

    // sector boundary lines
    if(res.sectorAzimuthDeg >= 0){
        double az  = res.sectorAzimuthDeg * DEG2RAD;
        double hw  = 22.5 * DEG2RAD;
        double len = (double)rr;

        // left edge: az - halfWidth (SVG: x=east=col, y=south=row)
        double az1 = az - hw;
        double x1  = cx + len * std::sin(az1);
        double y1  = cy - len * std::cos(az1);
        // right edge
        double az2 = az + hw;
        double x2  = cx + len * std::sin(az2);
        double y2  = cy - len * std::cos(az2);

        f<<"  <line x1=\""<<cx<<"\" y1=\""<<cy
         <<"\" x2=\""<<(int)x1<<"\" y2=\""<<(int)y1
         <<"\" stroke=\"white\" stroke-width=\"2\" opacity=\"0.7\""
         <<" stroke-dasharray=\"12,8\"/>\n"
         <<"  <line x1=\""<<cx<<"\" y1=\""<<cy
         <<"\" x2=\""<<(int)x2<<"\" y2=\""<<(int)y2
         <<"\" stroke=\"white\" stroke-width=\"2\" opacity=\"0.7\""
         <<" stroke-dasharray=\"12,8\"/>\n"
         // sector label
         <<"  <text x=\""<<(int)(cx+len*0.55*std::sin(az))
         <<"\" y=\""<<(int)(cy-len*0.55*std::cos(az))
         <<"\" fill=\"white\" font-size=\"24\" text-anchor=\"middle\""
         <<" opacity=\"0.6\">"<<res.sectorAzimuthDeg<<"°</text>\n";
    }

    // source marker
    f<<"  <circle cx=\""<<cx<<"\" cy=\""<<cy
     <<"\" r=\"6\" fill=\"white\" stroke=\"#1abc9c\" stroke-width=\"3\"/>\n"
     // stats
     <<"  <text x=\""<<(W-220)<<"\" y=\"20\" fill=\"#2ecc71\" font-size=\"12\">"
     <<"Covered: "<<res.countCovered<<"</text>\n"
     <<"  <text x=\""<<(W-220)<<"\" y=\"38\" fill=\"#e74c3c\" font-size=\"12\">"
     <<"Terrain: "<<res.countTerrainMasked<<"</text>\n"
     <<"  <text x=\""<<(W-220)<<"\" y=\"56\" fill=\"#3498db\" font-size=\"12\">"
     <<"BelowElev: "<<res.countBelowMinElev<<"</text>\n"
     <<"  <text x=\""<<(W-220)<<"\" y=\"74\" fill=\"#e67e22\" font-size=\"12\">"
     <<"AboveElev: "<<res.countAboveMaxElev<<"</text>\n"
     <<"  <text x=\""<<(W-220)<<"\" y=\"92\" fill=\"#9b59b6\" font-size=\"12\">"
     <<"Horizon: "<<res.countBeyondHorizon<<"</text>\n"
     <<"</svg>\n";

    std::cout<<"[RADAR] SVG: "<<filename<<"  "
             <<std::chrono::duration<double,std::milli>(
                 std::chrono::high_resolution_clock::now()-tAll).count()
             <<" ms\n";
}
