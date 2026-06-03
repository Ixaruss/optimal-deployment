#include "Config.h"
#include <fstream>
#include <filesystem>
#include <stdexcept>

using namespace std;


static Layer_vals str_to_layer(const string& val) {
    if (val == "roads")       return ROADS;
    if (val == "railways")    return RAILS;
    if (val == "waterlines")  return WATER_LINES;
    if (val == "waterbodies") return WATER_AREAS;
    if (val == "slope")       return SLOPE;
    if (val == "ib")          return IB;
    if (val == "alt")         return ALT;
    throw runtime_error("Invalid layer value: " + val);
}

static string layer_to_str(Layer_vals v) {
    switch (v) {
        case ROADS:       return "roads";
        case RAILS:       return "railways";
        case WATER_LINES: return "waterlines";
        case WATER_AREAS: return "waterbodies";
        case SLOPE:       return "slope";
        case IB:          return "ib";
        case ALT:         return "alt";

        default:          throw runtime_error("Unknown Layer_vals");
    }
}


Config Config::load(optional<const string> path) {
    string p = path.value_or("../config.json");

    ifstream f(p);
    if (!f.is_open())
        throw runtime_error("Cannot open config file: " + p);

    Json::Value root;
    Json::CharReaderBuilder reader;
    string errors;
    if (!Json::parseFromStream(reader, f, &root, &errors))
        throw runtime_error("Failed to parse config JSON: " + errors);

    return from_json(root);
}

void Config::save(string path) const {
    ofstream f(path);
    if (!f.is_open())
        throw runtime_error("Cannot write config file: " + path);

    Json::StreamWriterBuilder writer;
    writer["indentation"] = "    ";
    f << Json::writeString(writer, to_json());
}

bool Config::is_available(optional<const string> path) {
    return filesystem::exists(path.value_or("../config.json"));
}


Config Config::from_json(const Json::Value& j) {
    Config conf;


    string t = j["target"].asString();
    conf.target = (t == "build") ? Target::BUILD : Target::SERVER;

    conf.input.road = j["input"]["road"].asString();
    conf.input.rail = j["input"]["rail"].asString();
    conf.input.water_area = j["input"]["water_area"].asString();
    conf.input.water_lines = j["input"]["water_lines"].asString();
    conf.input.ib = j["input"]["ib"].asString();
    conf.input.elevation = j["input"]["elevation"].asString();
    conf.output.matrix_file = j["output"]["matrix_file"].asString();
    conf.output.elevation_file = j["output"]["elevation_file"].asString();
    conf.output.feasible = j["output"]["feasible"].asString();

    conf.grid.resolution = j["grid"]["resolution"].asInt();
    conf.grid.origin_x   = j["grid"]["origin_x"].asDouble();
    conf.grid.origin_y   = j["grid"]["origin_y"].asDouble();
    conf.grid.min_x      = j["grid"]["min_x"].asDouble();
    conf.grid.max_x      = j["grid"]["max_x"].asDouble();
    conf.grid.min_y      = j["grid"]["min_y"].asDouble();
    conf.grid.max_y      = j["grid"]["max_y"].asDouble();
    conf.grid.rows       = j["grid"]["rows"].asInt();
    conf.grid.cols       = j["grid"]["cols"].asInt();

    conf.distance.from_road = j["distance"]["from_road"].asInt();
    conf.distance.from_rail = j["distance"]["from_rail"].asInt();
    conf.distance.from_water = j["distance"]["from_water"].asInt();
    conf.distance.from_ib = j["distance"]["from_ib"].asInt();

    const Json::Value array = j["operation_area"];
    if (array.isArray()) {
        for (Json::Value::ArrayIndex i = 0; i + 1 < array.size(); i += 2) {
            double lat = array[i].asDouble();
            double lon = array[i + 1].asDouble();
            conf.area_op.push_back(std::make_pair(lat, lon));
        }
    }

    for (const string& key : j["layers"].getMemberNames())
        conf.layers[stoi(key)] = str_to_layer(j["layers"][key].asString());


        return conf;
}

Json::Value Config::to_json() const {
    Json::Value j;

    // target
    j["target"] = (target == Target::BUILD) ? "build" : "server";

    // vector grid
    j["grid"]["resolution"]      = grid.resolution;
    j["grid"]["origin_x"]        = grid.origin_x;
    j["grid"]["origin_y"]        = grid.origin_y;

    j["grid"]["min_x"]               = grid.min_x;
    j["grid"]["max_x"]               = grid.max_x;
    j["grid"]["min_y"]               = grid.min_y;
    j["grid"]["max_y"]               = grid.max_y;

    // elevation grid — populated by build_elevation_matrix
    j["grid"]["rows"]       = grid.rows;
    j["grid"]["cols"]       = grid.cols;
    Json::Value array_flat(Json::arrayValue);

    // Loop through your pairs in reverse order
    for (auto it = area_op.rbegin(); it != area_op.rend(); ++it) {
        array_flat.append(it->first);  // Append lat
        printf("coordx: %f", it->first);
        array_flat.append(it->second); // Append lon
        printf("coordy: %f", it->second);
    }

    // Assign the flat array to the key
    j["operation_area"] = array_flat;
    // layers bit → name
    for (const auto& [bit, layer] : layers)
        j["layers"][std::to_string(bit)] = layer_to_str(layer);

    // input paths
    j["input"]["road"]       = input.road;
    j["input"]["rail"]       = input.rail;
    j["input"]["water_area"] = input.water_area;
    j["input"]["water_lines"]= input.water_lines;
    j["input"]["ib"]         = input.ib;
    j["input"]["elevation"]  = input.elevation;

    j["distance"]["from_ib"]       = distance.from_ib;
    j["distance"]["from_water"]    = distance.from_water;
    j["distance"]["from_road"]     = distance.from_road;
    j["distance"]["from_rail"]     = distance.from_rail;

    // output paths
    j["output"]["matrix_file"]    = output.matrix_file;
    j["output"]["elevation_file"] = output.elevation_file;
    j["output"]["feasible"]       = output.feasible;

    return j;
}
