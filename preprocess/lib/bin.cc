#include "../bin.h"
#include <cstdint>
using namespace std;
using namespace lib;

static void load_vec(Bin* b, const string& param_path) {
    if (b->vector_loaded) return;

    if (b->VEC_ROWS == 0 || b->VEC_COLS == 0) {
        b->VEC_ROWS = calc_rows(RESOLUTION);
        b->VEC_COLS = calc_cols(RESOLUTION);
    }

    size_t total = (size_t)b->VEC_ROWS * b->VEC_COLS;
    cout << "[bin] Loading " << param_path
         << " (" << total / 1e6 << " MB)...\n";

    ifstream f(param_path, ios::binary);
    if (!f.is_open()) {
        cerr << "[bin] ERROR: cannot open " << param_path << "\n";
        return;
    }
    b->vec_matrix.resize(total);
    f.read(reinterpret_cast<char*>(b->vec_matrix.data()), total);
    if ((size_t)f.gcount() != total) {
        cerr << "[bin] ERROR: size mismatch reading " << param_path << "\n";
        b->vec_matrix.clear();
        return;
    }
    b->vector_loaded = true;
    cout << "[bin] Vec loaded — "
         << b->VEC_ROWS << " x " << b->VEC_COLS << "\n";
}

static void load_elev(Bin* b, const string& elev_path) {
    if (b->elevation_loaded) return;

    size_t elev_rows, elev_cols;
    if (Config::is_available()) {
        Config cfg = Config::load();
        elev_rows  = (size_t)cfg.grid.rows;
        elev_cols  = (size_t)cfg.grid.cols;
    } else {
        elev_rows  = (size_t)ELEV_ROWS;
        elev_cols  = (size_t)ELEV_COLS;
    }

    size_t total          = elev_rows * elev_cols;
    size_t expected_bytes = total * sizeof(int16_t);   // 6 bytes/cell

    cout << "[bin] Loading " << elev_path
         << " (" << expected_bytes / 1e6 << " MB)...\n";

    ifstream f(elev_path, ios::binary);
    if (!f.is_open()) {
        cerr << "[bin] ERROR: cannot open " << elev_path << "\n";
        return;
    }

    f.seekg(0, ios::end);
    size_t file_bytes = (size_t)f.tellg();
    f.seekg(0, ios::beg);

    if (file_bytes != expected_bytes) {
        cerr << "[bin] ERROR: size mismatch — expected "
             << expected_bytes / 1e6 << " MB, got "
             << file_bytes     / 1e6 << " MB\n"
             << "  Hint: rebuild with build_elevation_matrix()\n";
        return;
    }

    b->elev_matrix.resize(total);
    f.read(reinterpret_cast<char*>(b->elev_matrix.data()), expected_bytes);
    if ((size_t)f.gcount() != expected_bytes) {
        cerr << "[bin] ERROR: short read on " << elev_path << "\n";
        b->elev_matrix.clear();
        return;
    }
    b->elevation_loaded = true;
    cout << "[bin] Elev loaded — " << elev_rows << " x " << elev_cols << "\n";
}

static void load_slope(Bin* b, const string& slope_path) {
    if (b->slope_loaded) return;
    if (b->VEC_ROWS == 0 || b->VEC_COLS == 0) {
        b->VEC_ROWS = calc_rows(RESOLUTION);
        b->VEC_COLS = calc_cols(RESOLUTION);
    }

    size_t total          = (size_t)b->VEC_ROWS * b->VEC_COLS;
    size_t expected_bytes = total * sizeof(float);

    cout << "[bin] Loading " << slope_path
         << " (" << expected_bytes / 1e6 << " MB)...\n";

    ifstream f(slope_path, ios::binary);
    if (!f.is_open()) {
        cerr << "[bin] WARN: cannot open " << slope_path
             << " — slope queries disabled\n";
        return;
    }

    f.seekg(0, ios::end);
    size_t file_bytes = (size_t)f.tellg();
    f.seekg(0, ios::beg);

    if (file_bytes != expected_bytes) {
        cerr << "[bin] WARN: slope.bin size mismatch — expected "
             << expected_bytes / 1e6 << " MB, got "
             << file_bytes     / 1e6 << " MB — slope disabled\n";
        return;
    }

    b->slope_matrix.resize(total);
    f.read(reinterpret_cast<char*>(b->slope_matrix.data()), expected_bytes);
    if ((size_t)f.gcount() != expected_bytes) {
        cerr << "[bin] WARN: short read on slope.bin — slope disabled\n";
        b->slope_matrix.clear();
        return;
    }
    b->slope_loaded = true;
    cout << "[bin] Slope loaded — " << total / 1e6 << "M cells\n";
}

void Bin::init(set<LoadOptions> options) {
    // resolve paths once
    string param_path, elev_path, slope_path;
    if (Config::is_available()) {
        Config cfg  = Config::load();
        param_path  = cfg.output.matrix_file;
        elev_path   = cfg.output.elevation_file;
        slope_path  = cfg.output.slope_file;
    } else {
        param_path  = DEFAULT_PARAM_MATRIX;
        elev_path   = DEFAULT_ELEVATION_MATRIX;
        slope_path  = DEFAULT_SLOPE_MATRIX;
    }

    if (options.empty()) {
        // default — load vec + elev
        load_vec (this, param_path);
        load_elev(this, elev_path);

    } else if (options.count(ALL)) {
        load_vec  (this, param_path);
        load_elev (this, elev_path);
        load_slope(this, slope_path);

    } else {
        if (options.count(CONSTRAINTS)) load_vec  (this, param_path);
        if (options.count(ELEVATION))   load_elev (this, elev_path);
        if (options.count(SLOPE_))       load_slope(this, slope_path);
    }
}
