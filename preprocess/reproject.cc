#include <format>
#include <json/value.h>
#include <optional>
#include <filesystem>
#include "Config.h"

using namespace std;

int EPSG4326_TO_EPSG7755(string input_file_path, optional<string> output_file_path = nullopt, optional<string> parent_folder_path= nullopt) {
    cout << "[reproject] Starting reprojection: EPSG:4326 → EPSG:7755\n";
    cout << "[reproject] Input:  " << input_file_path << "\n";
    string output_file = output_file_path.value_or(_extract_part(input_file_path));
    if(filesystem::exists("config.json")) {
        Json::Value root;
    }
    const string folder_path = parent_folder_path.value_or("reprojected-maps/");
    if (!folder_path.ends_with('/')){
        cerr << "[reproject] \033[31mERROR:\033[0m Folder path must end with forward oblique i.e. '/'" <<endl;
        return -1;
    }
    cout << "[reproject] Creating output directory: " << folder_path << "\n";
    filesystem::create_directories(folder_path);
    cout<< folder_path << " " <<  output_file << endl;
    output_file = folder_path + output_file;

    string cmd = format("ogr2ogr -t_srs EPSG:7755 {1} {0}", input_file_path, output_file);
    cout << "[reproject] Running: " << cmd << "\n";

    int _flag = system(cmd.c_str());
    if (_flag != 0){
        cerr << "[reproject] \033[31mERROR:\033[0m ogr2ogr failed with exit code " << _flag << "\n";
        cerr << "[reproject] Check that the input file exists and libgdal is installed.\n";
        return -1;
    }
    cout << "[reproject] Done. Output written to: " << output_file << "\n";
    return 0;
}

string _extract_part(const string& file_path, unsigned short with_extension) {
    filesystem::path p(file_path);
    if (!with_extension) {
        return p.filename().string();
    }
    return p.stem().string();
}
