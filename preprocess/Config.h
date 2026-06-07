#ifndef BUILD_H_
#define BUILD_H_

#include <gdal.h>
#include <iostream>
#include <string>
#include <optional>
#include <iostream>
#include <json/json.h>
#include <map>


//json parser enums and class
enum Target {
    BUILD,
    SERVER
};
struct Input {
    std::string road;
    std::string rail;
    std::string water_area;
    std::string water_lines;
    std::string ib;
    std::string elevation;
};

struct Output {
    std::string matrix_file;
    std::string elevation_file;
    std::string feasible;
    std::string slope_file;
};

enum Layer_vals {
    ROADS,
    RAILS,
    WATER_LINES,
    WATER_AREAS,
    IB,
    SLOPE,
    ALT
};

struct Grid_Conf {
  int resolution;
  float origin_x;
  float origin_y;
  double min_x, max_x;
  double min_y, max_y;
  int   rows;
  int   cols;
};

struct Distance {
    int from_road;
    int from_rail;
    int from_water;
    int from_ib;
};

class Config {
    public: Target target;
    public: Input input;
    public: Output output;
    public: Grid_Conf grid;
    public: std::map<int,Layer_vals> layers;
    public: Distance distance;
    public: std::vector<std::pair<double,double>> area_op; // can be a polygon or retangle, if not rectangle find highest breath and hightest height and assign the values to above variable to form a rectangle;

    static Config load(std::optional <const std::string> path = std::nullopt);
    void save(std::string path = "../config.json") const;
    static Config from_json(const Json::Value& j);
    Json::Value to_json() const;
    static bool is_available(std::optional <const std::string> path = std::nullopt);
};

//helper functions
std::string _extract_part(const std::string& file_path, unsigned short with_extension = 0);
#endif
