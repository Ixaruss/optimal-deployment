#define ODLIB_EXPORTS
#include "export.h"
#include "bin.h"
#include <optional>
#include <string>

// ── internal wrapper ───────────────────────────────────────────────────────
// Holds a Bin instance + the last computed FeasibleGrid together.
// .NET gets an opaque void* pointing at this.
struct BinHandle {
    Bin          bin;
    FeasibleGrid last_feasible;
    bool         feasible_ready = false;
};

static inline BinHandle* cast(void* h) {
    return reinterpret_cast<BinHandle*>(h);
}

// ── lifecycle ──────────────────────────────────────────────────────────────
void* Bin_Create() {
    return new BinHandle();
}

void Bin_Destroy(void* handle) {
    delete cast(handle);
}

// ── flags ──────────────────────────────────────────────────────────────────
void Bin_SetFlagRoad      (void* h, int v) { cast(h)->bin.flag_road       = (v != 0); }
void Bin_SetFlagRail      (void* h, int v) { cast(h)->bin.flag_rail       = (v != 0); }
void Bin_SetFlagWaterLine (void* h, int v) { cast(h)->bin.flag_water_line = (v != 0); }
void Bin_SetFlagWaterArea (void* h, int v) { cast(h)->bin.flag_water_area = (v != 0); }
void Bin_SetFlagIB        (void* h, int v) { cast(h)->bin.flag_ib         = (v != 0); }

// ── polygon input ──────────────────────────────────────────────────────────
void Bin_ClearAreaOp(void* h) {
    cast(h)->bin.conf.area_op.clear();
}

void Bin_AddPoint(void* h, double lat, double lon) {
    cast(h)->bin.conf.area_op.push_back({ lat, lon });
}

// ── build ──────────────────────────────────────────────────────────────────
// Static methods — no handle needed, they read config from disk themselves.
int Bin_BuildMatrix() {
    return Bin::build_matrix();
}

int Bin_BuildElevationMatrix() {
    return Bin::build_elevation_matrix();
}

// ── per-layer refresh ──────────────────────────────────────────────────────
// path == NULL → use config/default
int Bin_RefreshRoadLayer(const char* path) {
    return Bin::refresh_road_layer(path ? std::optional<std::string>(path) : std::nullopt);
}

int Bin_RefreshRailLayer(const char* path) {
    return Bin::refresh_rail_layer(path ? std::optional<std::string>(path) : std::nullopt);
}

int Bin_RefreshWaterAreaLayer(const char* path) {
    return Bin::refresh_water_area_layer(path ? std::optional<std::string>(path) : std::nullopt);
}

int Bin_RefreshWaterLineLayer(const char* path) {
    return Bin::refresh_water_line_layer(path ? std::optional<std::string>(path) : std::nullopt);
}

int Bin_RefreshIBLayer(const char* path) {
    return Bin::refresh_ib_layer(path ? std::optional<std::string>(path) : std::nullopt);
}

// ── single-cell queries ────────────────────────────────────────────────────
int16_t Bin_GetElevation(void* h, double lat, double lon) {
    return cast(h)->bin.getElevation(lat, lon);
}

int Bin_GetStatusRoad     (void* h, double lat, double lon) { return cast(h)->bin.getStatusRoad     (lat, lon) ? 1 : 0; }
int Bin_GetStatusRail     (void* h, double lat, double lon) { return cast(h)->bin.getStatusRail     (lat, lon) ? 1 : 0; }
int Bin_GetStatusWaterArea(void* h, double lat, double lon) { return cast(h)->bin.getStatusWaterArea(lat, lon) ? 1 : 0; }
int Bin_GetStatusWaterLine(void* h, double lat, double lon) { return cast(h)->bin.getStatusWaterLine(lat, lon) ? 1 : 0; }
int Bin_GetStatusIB       (void* h, double lat, double lon) { return cast(h)->bin.getStatusIB       (lat, lon) ? 1 : 0; }

uint8_t Bin_GetCellBitmask(void* h, double lat, double lon) {
    // getStatusAll returns vector<int> of individual bit values.
    // Reconstruct as raw bitmask so .NET can do its own bit checks.
    auto bits = cast(h)->bin.getStatusAll(lat, lon);
    uint8_t mask = 0;
    for (int i = 0; i < (int)bits.size() && i < 8; i++)
        if (bits[i]) mask |= (1 << i);
    return mask;
}

// ── MBR ───────────────────────────────────────────────────────────────────
void Bin_GetMBR(void* h,
                double* min_lat, double* max_lat,
                double* min_lon, double* max_lon) {
    BoundingBox bb = cast(h)->bin.getMinimumBoundingBox();
    if (min_lat) *min_lat = bb.min_lat;
    if (max_lat) *max_lat = bb.max_lat;
    if (min_lon) *min_lon = bb.min_lon;
    if (max_lon) *max_lon = bb.max_lon;
}

// ── feasible grid ──────────────────────────────────────────────────────────
int Bin_ComputeFeasibleGrid(void* h, int write) {
    BinHandle* bh = cast(h);
    bh->last_feasible  = bh->bin.getFeasibleGrid(write ? WRITE : IN_MEM);
    bh->feasible_ready = true;
    return (int)bh->last_feasible.cells.size();
}

int Bin_GetFeasibleCount(void* h) {
    BinHandle* bh = cast(h);
    if (!bh->feasible_ready) return 0;
    return (int)bh->last_feasible.cells.size();
}

void Bin_GetFeasibleCell(void* h, int index,
                          int* out_r, int* out_c, uint8_t* out_value) {
    BinHandle* bh = cast(h);
    if (!bh->feasible_ready) return;
    if (index < 0 || index >= (int)bh->last_feasible.cells.size()) return;

    const FeasibleCell& cell = bh->last_feasible.cells[index];
    if (out_r)     *out_r     = cell.r;
    if (out_c)     *out_c     = cell.c;
    if (out_value) *out_value = cell.value;
}

void Bin_GetFeasibleMeta(void* h,
                          int* origin_r, int* origin_c,
                          int* height,   int* width) {
    BinHandle* bh = cast(h);
    if (!bh->feasible_ready) return;
    if (origin_r) *origin_r = bh->last_feasible.origin_r;
    if (origin_c) *origin_c = bh->last_feasible.origin_c;
    if (height)   *height   = bh->last_feasible.height;
    if (width)    *width    = bh->last_feasible.width;
}

// ── visualize ─────────────────────────────────────────────────────────────
void Bin_Visualize(void* h, const char* output_path) {
    if (!output_path) return;
    cast(h)->bin.visualize(std::string(output_path));
}

// ── verify ────────────────────────────────────────────────────────────────
int Bin_VerifyVectorMatrix(const char* path, int rows, int cols) {
    return Bin::verifyMatrixBin<uint8_t>(std::string(path), rows, cols) ? 1 : 0;
}

int Bin_VerifyElevationMatrix(const char* path, int rows, int cols) {
    return Bin::verifyMatrixBin<int16_t>(std::string(path), rows, cols) ? 1 : 0;
}

int Bin_VerifyFeasibleMatrix(const char* path, int rows, int cols) {
    return Bin::verifyMatrixBin<FeasibleCell>(std::string(path), rows, cols) ? 1 : 0;
}
