/******************************************************************************
 * LOS UNIT TESTS
 *
 * Tests lineOfVisibility logic directly using a synthetic flat grid,
 * without needing the real DEM binary files.
 *
 * Tests:
 *   1. Flat terrain    -> all cells visible
 *   2. Single blocker  -> cells behind it hidden
 *   3. Tall src        -> observer height clears blocker
 *   4. Data integrity  -> dist increases, elev valid, angles finite
 *   5. Single cell     -> src == dst
 *   6. Rising terrain  -> all visible (each cell taller than last)
 *   7. Valley          -> dip then rise, far peak visible
 *
 ******************************************************************************/

#include <vector>
#include <cmath>
#include <cstdio>
#include <string>
#include <cassert>
#include <limits>

/******************************************************************************
 * MINIMAL INLINE LOS
 *
 * Self-contained version of the algorithm for testing.
 * Same logic as Global::lineOfVisibility, no DEM dependency.
 *
 ******************************************************************************/

struct TestLOSResult
{
    int    x;
    double dist;
    double angle;
    double maxAngle;
    float  elev;
    bool   visible;
};

// heights: flat array, 1D grid (y=0 always in tests)
static std::vector<TestLOSResult>
runLOS(const std::vector<float>& heights,
       double cellSize,
       int srcIdx, double srcExtraHeight,
       int dstIdx)
{
    std::vector<TestLOSResult> results;

    float srcTerrain = heights[srcIdx];
    double srcElev   = srcTerrain + srcExtraHeight;

    double maxAngle = -1e18;

    int step = (dstIdx >= srcIdx) ? 1 : -1;

    for (int i = srcIdx; i != dstIdx + step; i += step)
    {
        if (i < 0 || i >= (int)heights.size()) break;

        float  terrainElev = heights[i];
        double dist        = std::abs(i - srcIdx) * cellSize;

        TestLOSResult r;
        r.x    = i;
        r.dist = dist;
        r.elev = terrainElev;

        if (i == srcIdx)
        {
            r.angle    = 0.0;
            r.maxAngle = -1e18;
            r.visible  = true;
            results.push_back(r);
            continue;
        }

        r.angle    = std::atan2(terrainElev - srcElev, dist);
        r.visible  = (r.angle >= maxAngle);
        r.maxAngle = maxAngle;

        if (r.angle > maxAngle)
            maxAngle = r.angle;

        results.push_back(r);
    }

    return results;
}

/******************************************************************************
 * TEST HELPERS
 ******************************************************************************/

static int passed = 0;
static int failed = 0;

static void check(bool condition, const std::string& name)
{
    if (condition)
    {
        printf("  [PASS] %s\n", name.c_str());
        passed++;
    }
    else
    {
        printf("  [FAIL] %s\n", name.c_str());
        failed++;
    }
}

/******************************************************************************
 * TESTS
 ******************************************************************************/

// 1. Flat terrain: every cell should be visible
static void test_flat()
{
    printf("\nTest 1: Flat terrain\n");

    std::vector<float> h(10, 100.0f); // all height 100
    auto res = runLOS(h, 10.0, 0, 0.0, 9);

    check(res.size() == 10, "result count == 10");

    bool allVisible = true;
    for (auto& r : res) if (!r.visible) { allVisible = false; break; }
    check(allVisible, "all cells visible on flat terrain");

    // angles should be non-decreasing... actually all equal (flat)
    bool anglesFlat = true;
    for (size_t i = 1; i < res.size(); i++)
        if (res[i].angle > res[i-1].angle + 1e-6) { anglesFlat = false; break; }
    check(anglesFlat, "angles non-increasing on flat terrain (all equal)");
}

// 2. Single blocker: tall spike at index 3, cells behind it hidden
static void test_single_blocker()
{
    printf("\nTest 2: Single blocker\n");

    // flat at 100, spike at index 3 = 200, rest back to 100
    std::vector<float> h = {100,100,100,200,100,100,100,100,100,100};
    auto res = runLOS(h, 10.0, 0, 0.0, 9);

    check(res[0].visible, "source visible");
    check(res[1].visible, "cell 1 visible (before blocker)");
    check(res[2].visible, "cell 2 visible (before blocker)");
    check(res[3].visible, "cell 3 visible (IS the blocker)");
    check(!res[4].visible, "cell 4 hidden (behind blocker)");
    check(!res[5].visible, "cell 5 hidden (behind blocker)");
    check(!res[6].visible, "cell 6 hidden (behind blocker)");
}

// 3. Observer height clears blocker
static void test_observer_height()
{
    printf("\nTest 3: Observer height clears blocker\n");

    std::vector<float> h = {100,100,100,130,100,100,100,100,100,100};
    // without extra height: cell 3 blocks
    auto res_low  = runLOS(h, 10.0, 0,   0.0, 9);
    // with extra height: observer is at 200, clears the 130 spike
    auto res_high = runLOS(h, 10.0, 0, 100.0, 9);

    check(!res_low[4].visible,  "cell 4 hidden without observer height");
    check(res_high[4].visible,  "cell 4 visible with observer height");
    check(res_high[9].visible,  "target visible with observer height");
}

// 4. Data integrity
static void test_data_integrity()
{
    printf("\nTest 4: Data integrity\n");

    std::vector<float> h = {100,110,90,150,80,120,100,130,95,105};
    auto res = runLOS(h, 30.0, 0, 0.0, 9);

    check(!res.empty(), "results not empty");
    check(res[0].dist == 0.0, "source dist == 0");

    bool distIncreasing = true;
    for (size_t i = 1; i < res.size(); i++)
        if (res[i].dist <= res[i-1].dist)
            { distIncreasing = false; break; }
    check(distIncreasing, "dist strictly increasing");

    bool elevValid = true;
    for (auto& r : res)
        if (r.elev < -9000)
            { elevValid = false; break; }
    check(elevValid, "no invalid elevations (-9999)");

    bool anglesFinite = true;
    for (auto& r : res)
        if (!std::isfinite(r.angle))
            { anglesFinite = false; break; }
    check(anglesFinite, "all angles finite");

    bool maxAngleNonDecreasing = true;
    double prev = -1e18;
    for (size_t i = 1; i < res.size(); i++)
    {
        // maxAngle stored is BEFORE this cell, so running max
        // should never decrease
        double runMax = std::max(res[i].maxAngle, res[i].angle);
        if (runMax < prev - 1e-9)
            { maxAngleNonDecreasing = false; break; }
        prev = runMax;
    }
    check(maxAngleNonDecreasing, "running max angle never decreases");
}

// 5. src == dst (single cell)
static void test_single_cell()
{
    printf("\nTest 5: src == dst\n");

    std::vector<float> h(5, 100.0f);
    auto res = runLOS(h, 10.0, 2, 0.0, 2);

    check(res.size() == 1,    "exactly 1 result");
    check(res[0].visible,     "single cell is visible");
    check(res[0].dist == 0.0, "dist == 0");
}

// 6. Rising terrain: each cell taller than previous, all visible
static void test_rising_terrain()
{
    printf("\nTest 6: Rising terrain\n");

    std::vector<float> h = {100,110,120,130,140,150,160,170,180,190};
    auto res = runLOS(h, 10.0, 0, 0.0, 9);

    bool allVisible = true;
    for (auto& r : res) if (!r.visible) { allVisible = false; break; }
    check(allVisible, "all cells visible on rising terrain");
}

// 7. Valley: dip then rise, far peak should be visible
static void test_valley()
{
    printf("\nTest 7: Valley (dip then tall peak)\n");

    // src=200, valley dips to 50, then rises to 250 at end
    std::vector<float> h = {200,150,100,50,50,100,150,200,230,250};
    auto res = runLOS(h, 10.0, 0, 0.0, 9);

    check(res[0].visible, "source visible");

    // valley floor should be hidden (below the initial downward angle? no —
    // from a high src looking down, valley IS visible until terrain rises)
    // cells 1-7 descend then rise: all visible from high src
    // cell 9 (250) is higher than src (200) so definitely visible
    check(res[9].visible, "tall far peak visible over valley");

    // the dip cells (3,4) - visible because looking downward from high src
    check(res[3].visible, "valley floor visible from high source");
}

// 8. Blocker color logic sanity
//    A cell that raises maxAngle should have angle > its own maxAngle
static void test_blocker_detection()
{
    printf("\nTest 8: Blocker detection\n");

    std::vector<float> h = {100,100,100,200,100,100,100,100,100,100};
    auto res = runLOS(h, 10.0, 0, 0.0, 9);

    // cell 3 is the blocker — its angle > maxAngle at that point
    check(res[3].angle > res[3].maxAngle, "blocker cell: angle > maxAngle");

    // hidden cells: angle < maxAngle
    check(res[4].angle < res[4].maxAngle + 1e-9, "hidden cell: angle <= maxAngle");
}

/******************************************************************************
 * MAIN
 ******************************************************************************/

int main()
{
    printf("========================================\n");
    printf("  LOS UNIT TESTS\n");
    printf("========================================\n");

    test_flat();
    test_single_blocker();
    test_observer_height();
    test_data_integrity();
    test_single_cell();
    test_rising_terrain();
    test_valley();
    test_blocker_detection();

    printf("\n========================================\n");
    printf("  RESULTS: %d passed, %d failed\n", passed, failed);
    printf("========================================\n");

    return failed > 0 ? 1 : 0;
}
