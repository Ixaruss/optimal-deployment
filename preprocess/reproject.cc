#include <json/value.h>
#include <filesystem>
#include "lib.h"

using namespace std;

int lib::reproject(const string& input_file_path) {
    cout << "[reproject] Starting reprojection: EPSG:4326 → EPSG:7755\n";
    cout << "[reproject] Input: " << input_file_path << "\n";

    // check input exists
    if (!filesystem::exists(input_file_path)) {
        cerr << "[reproject] ERROR: input file not found: " << input_file_path << "\n";
        return -1;
    }

    // temp output alongside the input file
    filesystem::path in_path(input_file_path);
    string stem      = in_path.stem().string();
    string ext       = in_path.extension().string();
    string parent    = in_path.parent_path().string();
    if (!parent.empty()) parent += "/";

    string tmp_path  = parent + stem + "_7755_tmp" + ext;
    string final_path = input_file_path;   // overwrite in place

    cout << "[reproject] Temp output : " << tmp_path << "\n";

    // run ogr2ogr to temp file
    string cmd = "ogr2ogr -t_srs EPSG:7755 \"" + tmp_path + "\" \"" + input_file_path + "\"";
    cout << "[reproject] Running: " << cmd << "\n";

    int ret = system(cmd.c_str());
    if (ret != 0) {
        cerr << "[reproject] ERROR: ogr2ogr failed with exit code " << ret << "\n";
        cerr << "[reproject] Check that the input file exists and ogr2ogr is installed.\n";
        // clean up temp if it was partially created
        filesystem::remove(tmp_path);
        return -1;
    }

    // for shapefiles ogr2ogr creates multiple sidecar files (.dbf, .prj, .shx etc.)
    // rename each sidecar from _7755_tmp back to original name
    string tmp_stem   = stem + "_7755_tmp";
    string final_stem = stem;

    for (auto& entry : filesystem::directory_iterator(
            parent.empty() ? "." : parent)) {
        string fname = entry.path().stem().string();
        string fext  = entry.path().extension().string();
        if (fname == tmp_stem) {
            string new_name = (parent.empty() ? "" : parent)
                              + final_stem + fext;
            filesystem::rename(entry.path(), new_name);
            cout << "[reproject] Renamed: " << entry.path().filename()
                 << " → " << final_stem + fext << "\n";
        }
    }

    cout << "[reproject] Done. Reprojected in place: " << input_file_path << "\n";
    return 0;
}

string _extract_part(const string& file_path, unsigned short with_extension) {
    filesystem::path p(file_path);
    if (!with_extension) {
        return p.filename().string();
    }
    return p.stem().string();
}

// helper — returns true if layer is already EPSG:7755
bool lib::is_7755(const string& shp_path) {
    GDALDataset* ds = (GDALDataset*)GDALOpenEx(
        shp_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!ds) return false;
    OGRLayer* layer = ds->GetLayer(0);
    if (!layer) { GDALClose(ds); return false; }
    const OGRSpatialReference* srs = layer->GetSpatialRef();
    if (!srs) { GDALClose(ds); return false; }
    OGRSpatialReference target;
    target.importFromEPSG(7755);
    bool same = srs->IsSame(&target);
    GDALClose(ds);
    return same;
}
