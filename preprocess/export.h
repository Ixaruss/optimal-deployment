#ifndef EXPORT_H_
#define EXPORT_H_

#include <cstdint>

// ── symbol visibility ──────────────────────────────────────────────────────
#ifdef _WIN32
  #ifdef ODLIB_EXPORTS
    #define ODLIB_API extern "C" __declspec(dllexport)
  #else
    #define ODLIB_API extern "C" __declspec(dllimport)
  #endif
#else
  #define ODLIB_API extern "C" __attribute__((visibility("default")))
#endif

// ── lifecycle ──────────────────────────────────────────────────────────────
// Returns opaque handle. Pass to every instance-level call.
ODLIB_API void*   Bin_Create();
ODLIB_API void    Bin_Destroy(void* handle);

// ── flags (1 = consider, 0 = ignore) ──────────────────────────────────────
ODLIB_API void    Bin_SetFlagRoad      (void* handle, int val);
ODLIB_API void    Bin_SetFlagRail      (void* handle, int val);
ODLIB_API void    Bin_SetFlagWaterLine (void* handle, int val);
ODLIB_API void    Bin_SetFlagWaterArea (void* handle, int val);
ODLIB_API void    Bin_SetFlagIB        (void* handle, int val);

// ── polygon input ──────────────────────────────────────────────────────────
ODLIB_API void    Bin_ClearAreaOp (void* handle);
ODLIB_API void    Bin_AddPoint    (void* handle, double lat, double lon);

// ── build (no handle — static ops, config-driven) ─────────────────────────
ODLIB_API int     Bin_BuildMatrix          ();
ODLIB_API int     Bin_BuildElevationMatrix ();

// ── per-layer refresh (pass NULL to use config/default path) ──────────────
ODLIB_API int     Bin_RefreshRoadLayer      (const char* path);
ODLIB_API int     Bin_RefreshRailLayer      (const char* path);
ODLIB_API int     Bin_RefreshWaterAreaLayer (const char* path);
ODLIB_API int     Bin_RefreshWaterLineLayer (const char* path);
ODLIB_API int     Bin_RefreshIBLayer        (const char* path);

// ── single-cell queries ────────────────────────────────────────────────────
ODLIB_API int16_t Bin_GetElevation      (void* handle, double lat, double lon);
ODLIB_API int     Bin_GetStatusRoad     (void* handle, double lat, double lon);
ODLIB_API int     Bin_GetStatusRail     (void* handle, double lat, double lon);
ODLIB_API int     Bin_GetStatusWaterArea(void* handle, double lat, double lon);
ODLIB_API int     Bin_GetStatusWaterLine(void* handle, double lat, double lon);
ODLIB_API int     Bin_GetStatusIB       (void* handle, double lat, double lon);

// Returns raw uint8 bitmask for the cell — .NET checks individual bits.
// Bit layout: 0=road 1=rail 2=water_area 3=water_line 4=ib
ODLIB_API uint8_t Bin_GetCellBitmask   (void* handle, double lat, double lon);

// ── MBR ───────────────────────────────────────────────────────────────────
// Caller allocates 4 doubles. Filled: min_lat, max_lat, min_lon, max_lon
ODLIB_API void    Bin_GetMBR(void* handle,
                               double* min_lat, double* max_lat,
                               double* min_lon, double* max_lon);

// ── feasible grid ──────────────────────────────────────────────────────────
// Compute + cache result inside the handle. write=1 → also writes feasible.bin
ODLIB_API int     Bin_ComputeFeasibleGrid(void* handle, int write);

// Number of feasible cells from last Bin_ComputeFeasibleGrid call
ODLIB_API int     Bin_GetFeasibleCount(void* handle);

// Fetch one cell by index. All out-params are caller-allocated.
ODLIB_API void    Bin_GetFeasibleCell(void* handle, int index,
                                       int* out_r, int* out_c,
                                       uint8_t* out_value);

// AABB metadata from last Bin_ComputeFeasibleGrid call
ODLIB_API void    Bin_GetFeasibleMeta(void* handle,
                                       int* origin_r, int* origin_c,
                                       int* height,   int* width);

// ── visualize ─────────────────────────────────────────────────────────────
ODLIB_API void    Bin_Visualize(void* handle, const char* output_path);

// ── verify (standalone — no handle) ───────────────────────────────────────
ODLIB_API int     Bin_VerifyVectorMatrix   (const char* path, int rows, int cols);
ODLIB_API int     Bin_VerifyElevationMatrix(const char* path, int rows, int cols);
ODLIB_API int     Bin_VerifyFeasibleMatrix (const char* path, int rows, int cols);

#endif // EXPORTS_H_
