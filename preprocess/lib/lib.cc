#include "../lib.h"

using namespace std;

int lib::calc_rows(int resolution) {

    if (Config::is_available()) {
        Config cfg = Config::load();
        return (int)ceil((cfg.grid.max_y - cfg.grid.min_y) / resolution);


    }
    return (int)ceil((VEC_MAX_Y - VEC_ORIGIN_X) / resolution);
}

int lib::calc_cols(int resolution) {
    if (Config::is_available()) {
        Config cfg = Config::load();
        return (int)ceil((cfg.grid.max_x - cfg.grid.min_x) / resolution);
    }
    return (int)ceil((VEC_ORIGIN_Y - VEC_MIN_X) / resolution);
}

vector<double> lib::get_min_max_coordiantes(string path, int type) {
    GDALAllRegister();

    GDALDataset *poDS = (GDALDataset*) GDALOpenEx(path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if (poDS == NULL) {
        std::cerr << "Open failed.\n";
    }

    OGRLayer *poLayer = poDS->GetLayer(0);
    if (poLayer == NULL) {
        std::cerr << "Layer retrieval failed.\n";
        GDALClose(poDS);
    }

    const OGRSpatialReference *poSourceSRS = poLayer->GetSpatialRef();
    if (poSourceSRS == NULL) {
        std::cerr << "Layer has no defined coordinate system.\n";
        GDALClose(poDS);
    }

    OGRSpatialReference oTargetSRS;
    oTargetSRS.importFromEPSG(4326);

    #if GDAL_VERSION_NUM >= GDAL_COMPUTE_VERSION(3,0,0)
    oTargetSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    #endif

    OGRCoordinateTransformation *poCT = OGRCreateCoordinateTransformation(poSourceSRS, &oTargetSRS);
    if (poCT == NULL) {
        std::cerr << "Transformation creation failed.\n";
        GDALClose(poDS);
    }
    vector<double> coordinates(4);
    OGREnvelope oEnvelope;
    if (poLayer->GetExtent(&oEnvelope, 1) == OGRERR_NONE) {


        if(type == 0 ){
            double left_lon = oEnvelope.MinX;
            double bottom_lat = oEnvelope.MinY;
            double right_lon = oEnvelope.MaxX;
            double top_lat = oEnvelope.MaxY;
            if (poCT->Transform(1, &left_lon, &bottom_lat) && poCT->Transform(1, &right_lon, &top_lat)) {
                coordinates[0] = left_lon;
                coordinates[1] = right_lon;
                coordinates[2] = bottom_lat;
                coordinates[3] = top_lat;
            } else {
                std::cerr << "Coordinate transform failed.\n";
            }
        }else {
            coordinates[0] = oEnvelope.MinX;
            coordinates[1] = oEnvelope.MaxX;
            coordinates[2]= oEnvelope.MinY;
            coordinates[3] = oEnvelope.MaxY;
        }
    }

    OGRCoordinateTransformation::DestroyCT(poCT);
    GDALClose(poDS);
    return coordinates;
}
