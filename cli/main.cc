#include "cli.h"
#include "../common.h"
#include "../operations.h"

int main(int argc, char** argv) {

    CLI::App app{"Geospatial Analysis and Radar CLI Tool"};
    app.require_subcommand(1);

    auto pre_sub = app.add_subcommand("build", "Build bin files from corresponding maps. config REQUIRED");
    bool images = false, timer = false;
    pre_sub->add_flag("-v,--with-images",images,"Generates images for bin visualization");
    pre_sub->add_flag("-c,--count-time",timer,"Counts the time taken for preprocessing");

    auto viewshed_sub = app.add_subcommand("viewshed", "Calculate viewshed for a given location");

    double vs_lat = 33.9871;
    double vs_lon = 74.7742;
    float vs_range = 300.0f;
    float vs_ant_height = 30.0f;
    bool vs_curvature = true;
    std::string vs_output = "viewshed_timed.svg";

    viewshed_sub->add_option("--lat", vs_lat, "Latitude of the origin")->required();
    viewshed_sub->add_option("--lon", vs_lon, "Longitude of the origin")->required();
    viewshed_sub->add_option("-r,--range", vs_range, "Range limit in kilometers");
    viewshed_sub->add_option("-a,--antenna", vs_ant_height, "Antenna height in meters");
    viewshed_sub->add_option("-o,--output", vs_output, "Output SVG filename");

    bool no_curvature = false;
    viewshed_sub->add_flag("--no-curvature", no_curvature, "Disable earth curvature corrections");

    auto radar_sub = app.add_subcommand("radar", "Calculate radar profile coverage.");

    double r_lat = 33.9871;
    double r_lon = 74.7742;
    double r_ant_agl = 50.0;
    double r_tgt_agl = 1000.0;
    double r_min_elev = 10.0;
    double r_max_elev = 30.0;
    double r_max_range = 300.0;
    std::string r_output = "radar_output.svg";


    double r_sector = -1.0; // Left at -1 to detect if user passes a specific sector angle
    bool r_fast = false;


    radar_sub->add_option("--lat", r_lat, "Latitude of radar base")->required();
    radar_sub->add_option("--lon", r_lon, "Longitude of radar base")->required();
    radar_sub->add_option("-a,--antenna", r_ant_agl, "Antenna height AGL (meters)");
    radar_sub->add_option("-t,--target", r_tgt_agl, "Target aircraft height AGL (meters)");
    radar_sub->add_option("--min-elev", r_min_elev, "Minimum elevation angle (degrees)");
    radar_sub->add_option("--max-elev", r_max_elev, "Maximum elevation angle (degrees)");
    radar_sub->add_option("-r,--range", r_max_range, "Maximum radar range (kilometers)");
    radar_sub->add_option("-o,--output", r_output, "Output SVG filename");

    radar_sub->add_option("-s,--sector", r_sector, "Specific sector slice in degrees (e.g. 45, 90)");
    radar_sub->add_flag("-f,--fast", r_fast, "Execute calculation in fast mode (without image)");

    auto rlos_sub = app.add_subcommand("rlos", "Calculate radar Line of Sight (LOS) between a source and a target.");

    double srcLat = 34.017346;
    double srcLon = 74.50587999;
    double tgtLat = 34.16910062;
    double tgtLon = 74.71267172;
    double los_ant_height = 30.0;
    double los_tgt_height = 1000.0;
    double los_min_elev = 0.5;
    double los_max_elev = 30.0;
    double los_max_range = 300.0;
    bool detailed = false;

    rlos_sub->add_option("--src-lat", srcLat, "Source (radar) latitude")->required();
    rlos_sub->add_option("--src-lon", srcLon, "Source (radar) longitude")->required();
    rlos_sub->add_option("--tgt-lat", tgtLat, "Target latitude")->required();
    rlos_sub->add_option("--tgt-lon", tgtLon, "Target longitude")->required();
    rlos_sub->add_option("-a,--antenna", los_ant_height, "Radar antenna height AGL (meters)");
    rlos_sub->add_option("-t,--target", los_tgt_height, "Target aircraft height AGL (meters)");
    rlos_sub->add_option("--min-elev", los_min_elev, "Minimum elevation angle (degrees)");
    rlos_sub->add_option("--max-elev", los_max_elev, "Maximum elevation angle (degrees)");
    rlos_sub->add_option("-r,--range", los_max_range, "Maximum calculation range (kilometers)");

    rlos_sub->add_flag("-d,--detailed", detailed, "Execute detailed profile and print cell telemetry");

    auto los_sub = app.add_subcommand("los", "Calculate Line of Sight (LOS) between a source and a target.");
    los_sub->add_option("--src-lat", srcLat, "Source (radar) latitude")->required();
    los_sub->add_option("--src-lon", srcLon, "Source (radar) longitude")->required();
    los_sub->add_option("--tgt-lat", tgtLat, "Target latitude")->required();
    los_sub->add_option("--tgt-lon", tgtLon, "Target longitude")->required();
    los_sub->add_option("-a,--antenna", los_ant_height, "Radar antenna height AGL (meters)");
    los_sub->add_option("-t,--target", los_tgt_height, "Target aircraft height AGL (meters)");
    los_sub->add_flag("-d,--detailed", detailed, "Execute detailed profile and print cell telemetry");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }

    Global g;
    if (*viewshed_sub) {
        vs_curvature = !no_curvature; // Flip state if flag passed

        auto engine = std::make_unique<ViewshedEngine>(
            g, vs_lat, vs_lon, vs_range, vs_ant_height, vs_curvature
        );

        engine->computeViewshed();
        engine->writeViewshedToSVG(vs_output);
    }

    // Route 2: Execution for Radar
    else if (*radar_sub) {
        RadarEngine engine(
            g, r_lat, r_lon, r_ant_agl, r_tgt_agl, r_min_elev, r_max_elev, r_max_range
        );

        bool has_sector = radar_sub->count("--sector") > 0;

        if (r_fast) {
            if (has_sector) {
                engine.computeFast(r_sector);
            } else {
                engine.computeFast();
            }
        } else {
            if (radar_sub->count("--output") == 0 && has_sector) {
                r_output = "radar_sector_" + std::to_string(static_cast<int>(r_sector)) + ".svg";
            } else if (radar_sub->count("--output") == 0) {
                r_output = "radar_360.svg";
            }

            if (has_sector) {
                auto res = engine.compute(r_sector);
                engine.writeToSVG(res, r_output);
            } else {
                auto res = engine.compute();
                engine.writeToSVG(res, r_output);
            }
        }
    }
    else if (*pre_sub) {
        Global::preprocess(images,timer);
    }
    else if (*rlos_sub) {
        if (!detailed) {
            // 1. Fast LOS Workflow
            auto res = RadarEngine::radarLOS(
                g,
                srcLat, srcLon, los_ant_height,
                tgtLat, tgtLon, los_tgt_height,
                los_min_elev, los_max_elev, los_max_range
            );

            RadarEngine::printLOSResult(res, srcLat, srcLon, tgtLat, tgtLon);
        }
        else {
            // 2. Detailed LOS Workflow (for visualization / debugging)
            auto det = RadarEngine::radarLOSDetailed(
                g,
                srcLat, srcLon, los_ant_height,
                tgtLat, tgtLon, los_tgt_height,
                los_min_elev, los_max_elev, los_max_range
            );

            std::cout << "\nDETAILED — " << det.cells.size() << " cells along ray\n";
            std::cout << "First blocking cell: ";
            if (det.summary.status == LOSStatus::TERRAIN_MASKED) {
                std::cout << "row=" << det.summary.blockRow
                          << " col=" << det.summary.blockCol
                          << " dist=" << det.summary.blockDistM / 1000.0 << "km\n";
            } else {
                std::cout << "none — " << losStatusStr(det.summary.status) << "\n";
            }

            // Print first 10 cells telemetry table
            std::cout << "\nRow    Col    Dist(km)  TerrElev  CorrElev  TerrSlope   TargSlope   Blocks\n";
            int n = std::min((int)det.cells.size(), 10);
            for (int i = 0; i < n; i++) {
                const auto& c = det.cells[i];
                printf("%-6d %-6d %-9.2f %-9.1f %-9.1f %-11.6f %-11.6f %s\n",
                       c.row, c.col, c.distM / 1000.0,
                       (double)c.terrainElev, (double)c.correctedElev,
                       c.terrainSlope, c.targetSlope,
                       c.blocks ? "BLOCK" : "ok");
            }
        }
    }
    else if (*los_sub) {
        auto res = g.lineOfVisibilityopt(srcLat, srcLon, los_ant_height, tgtLat, tgtLon, los_tgt_height);
        auto target = res.back();
        float srcTerrain = g.bin.getElevation(srcLat, srcLon);
        double srcElev = srcTerrain + los_ant_height;
           if (detailed) {
               std::cout << "=========================================\n";
                  std::cout << "Target Status : " << (target.visible ? "VISIBLE" : "BLOCKED") << "\n";
                  std::cout << "Total Distance: " << std::fixed << std::setprecision(2) << target.dist << " meters\n";
                  std::cout << "Final Angle   : " << target.angle << "°\n";
                  std::cout << "Max Obstacle  : " << target.maxAngle << "°\n";
                  std::cout << "Src elev      : " << srcElev << "m\n";
                  std::cout << "Target Elev   : " << target.elev << "m\n";
                  std::cout << "=========================================\n\n";

               size_t totalCells = res.size();
                 size_t printCount = (totalCells < 10) ? totalCells : 10;
                 size_t startIndex = totalCells - printCount;

                 std::cout << "--- Printing Last " << printCount << " Path Cells ---\n";

                 // Table Header
                 std::cout << std::setw(6)  << "Idx"
                           << std::setw(12) << "Coord (X,Y)"
                           << std::setw(10) << "Dist(m)"
                           << std::setw(9)  << "Elev(m)"
                           << std::setw(9)  << "Angle"
                           << std::setw(10) << "MaxAng"
                           << std::setw(9)  << "Visible" << "\n";
                 std::cout << std::string(65, '-') << "\n";

                 // Print rows
                 for (size_t i = startIndex; i < totalCells; ++i) {
                     const auto& cell = res[i];
                     std::string coordStr = "(" + std::to_string(cell.x) + "," + std::to_string(cell.y) + ")";

                     std::cout << std::setw(6)  << i
                               << std::setw(12) << coordStr
                               << std::setw(10) << std::fixed << std::setprecision(1) << cell.dist
                               << std::setw(9)  << cell.elev
                               << std::setw(9)  << std::setprecision(2) << cell.angle
                               << std::setw(10) << cell.maxAngle
                               << std::setw(9)  << (cell.visible ? "Yes" : "No") << "\n";
                 }
                 std::cout << std::string(65, '-') << "\n";
           }else {
               std::cout << "Target Status : " << (target.visible ? "VISIBLE" : "BLOCKED") << "\n";
               std::cout << "Total Distance: " << std::fixed << std::setprecision(2) << target.dist << " meters\n";
               std::cout << "Total cells : " << res.size() << "\n";
           }
    }
    return 0;
}
