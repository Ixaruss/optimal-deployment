
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono> // Standard high-resolution profiling clocks library
#include <iomanip> // Mechanics handling console print formatting rules
#include <fstream> // File system stream management for disk writing

// Mathematical Pi constant used for conversions between degrees and radians
const double PI = 3.14159265358979323846;

/**
 * @brief Spatial bounding rectangle defining the geographical limits of the optimization scenario.
 * Units are represented in decimal degrees for latitude and longitude.
 */
struct BoundingBox {
    double minLat, maxLat, minLon, maxLon;
};

/**
 * @brief Represents a single node location and its directional coverage parameters.
 */
struct GunDeploymentPoint {
    int gridX, gridY;        // Discrete matrix coordinates matching the underlying spatial resolution
    double lat, lon;         // Geodetic coordinates derived from the grid spacing scale
    double leftArcDeg;       // Left boundary sweep angle of the coverage zone [0, 360)
    double rightArcDeg;      // Right boundary sweep angle of the coverage zone [0, 360)
    double rangeKm;          // Absolute operational coverage distance limit in kilometers

    /**
     * @brief Computes planar approximation distance to target using flat-earth localized latitude scaling.
     * @return Distance metric in kilometers.
     */
    double distanceTo(double targetLat, double targetLon) const {
        // Standardized meridional radius scaling factor: ~111.0 km per degree latitude
        double dLat = (targetLat - lat) * 111.0;
        // Longitudinal degree length diminishes proportionally with the cosine of local latitude
        double dLon = (targetLon - lon) * 111.0 * std::cos(lat * PI / 180.0);
        return std::sqrt(dLat * dLat + dLon * dLon);
    }
};

/**
 * @brief Genetic Individual encapsulating a potential deployment arrangement.
 */
struct GAIndividual {
    std::vector<GunDeploymentPoint> chromosome; // Sequence of localized coordinate nodes (Size = 3)
    double fitness;                            // Calculated survival metric score
};

// ============================================================================
// STANDALONE NATIVE VECTOR VISUALIZATION ENGINE
// ============================================================================
/**
 * @brief Translates geographical solution layouts into raw vector files (SVG) exported to disk.
 */
void generateDeploymentMap(const GAIndividual& bestConfig, double assetLat, double assetLon, BoundingBox bbox) {
    // FIXED: Correctly added the missing high-resolution starting anchor stamp
    auto mapStart = std::chrono::high_resolution_clock::now();

    std::ofstream outFile("deployment_layout.svg");
    if (!outFile) {
        std::cerr << "Error creating vector file on disk.\n";
        return;
    }

    // Fixed internal display canvas pixel dimensions
    int width = 1000;
    int height = 1000;

    // Linear translation lambda mappings interpolating coordinate positions inside pixel limits
    auto mapX = [&](double lon) { return ((lon - bbox.minLon) / (bbox.maxLon - bbox.minLon)) * width; };
    auto mapY = [&](double lat) { return height - (((lat - bbox.minLat) / (bbox.maxLat - bbox.minLat)) * height); };

    // Emit SVG container declaration header
    outFile << "<svg width=\"" << width << "\" height=\"" << height << "\" xmlns=\"http://www.w3.org/2000/svg\" style=\"background:#141923;\">\n";

    // Draw background alignment grid lines spaced at 10% structural increments
    for (int i = 1; i < 10; ++i) {
        outFile << "<line x1=\"" << (width / 10) * i << "\" y1=\"0\" x2=\"" << (width / 10) * i << "\" y2=\"" << height << "\" stroke=\"#232d3f\" stroke-width=\"1\" />\n";
        outFile << "<line x1=\"0\" y1=\"" << (height / 10) * i << "\" x2=\"" << width << "\" y2=\"" << (height / 10) * i << "\" stroke=\"#232d3f\" stroke-width=\"1\" />\n";
    }

    // Draw reference radius ring denoting the spatial optimization limit around target center
    double referenceRadiusPixels = (2.0 / 111.0) / (bbox.maxLat - bbox.minLat) * height;
    double aX = mapX(assetLon);
    double aY = mapY(assetLat);
    outFile << "\n";
    outFile << "<circle cx=\"" << aX << "\" cy=\"" << aY << "\" r=\"" << referenceRadiusPixels << "\" fill=\"none\" stroke=\"#3a4b63\" stroke-width=\"2\" stroke-dasharray=\"6\" />\n";

    // Loop through individual chromosomes to draw structural path data
    for (size_t i = 0; i < bestConfig.chromosome.size(); ++i) {
        const auto& gun = bestConfig.chromosome[i];
        double gX = mapX(gun.lon);
        double gY = mapY(gun.lat);

        // Convert range vector lengths directly from world coordinates into pixel distances
        double rPixels = (gun.rangeKm / 111.0) / (bbox.maxLat - bbox.minLat) * height;

        // Transform clockwise polar degrees into standard Cartesian trigonometry space radians
        double radLeft = (90.0 - gun.leftArcDeg) * PI / 180.0;
        double radRight = (90.0 - gun.rightArcDeg) * PI / 180.0;

        // Determine destination edge nodes projecting outwards along coverage limits
        double xLeft = gX + rPixels * std::cos(radLeft);
        double yLeft = gY - rPixels * std::sin(radLeft);
        double xRight = gX + rPixels * std::cos(radRight);
        double yRight = gY - rPixels * std::sin(radRight);

        // Evaluate orientation flip conditions to correctly render arc vectors past 180 degrees
        int largeArcFlag = (std::fmod(gun.rightArcDeg - gun.leftArcDeg + 360.0, 360.0) > 180.0) ? 1 : 0;

        // Render solid filled polygon representing operational field-of-view sweeps
        outFile << "\n\n";
        outFile << "<path d=\"M " << gX << " " << gY << " L " << xLeft << " " << yLeft
                << " A " << rPixels << " " << rPixels << " 0 " << largeArcFlag << " 1 " << xRight << " " << yRight
                << " Z\" fill=\"rgba(0, 162, 255, 0.12)\" stroke=\"#00a2ff\" stroke-width=\"2\" />\n";
    }

    // Secondary vector pass drawing foreground overlay labels and node centers
    for (size_t i = 0; i < bestConfig.chromosome.size(); ++i) {
        const auto& gun = bestConfig.chromosome[i];
        double gX = mapX(gun.lon);
        double gY = mapY(gun.lat);
        outFile << "<circle cx=\"" << gX << "\" cy=\"" << gY << "\" r=\"7\" fill=\"#00ffff\" stroke=\"#ffffff\" stroke-width=\"2\" />\n";
        outFile << "<text x=\"" << gX + 12 << "\" y=\"" << gY + 5 << "\" fill=\"#ffffff\" font-family=\"sans-serif\" font-size=\"14\" font-weight=\"bold\">Gun " << (i+1) << "</text>\n";
    }

    // Render center focus markings indicating the position of the protected point asset
    outFile << "\n\n";
    outFile << "<circle cx=\"" << aX << "\" cy=\"" << aY << "\" r=\"14\" fill=\"none\" stroke=\"#ff3b30\" stroke-width=\"3\" />\n";
    outFile << "<line x1=\"" << aX - 22 << "\" y1=\"" << aY << "\" x2=\"" << aX + 22 << "\" y2=\"" << aY << "\" stroke=\"#ff3b30\" stroke-width=\"3\" />\n";
    outFile << "<line x1=\"" << aX << "\" y1=\"" << aY - 22 << "\" x2=\"" << aX << "\" y2=\"" << aY + 22 << "\" stroke=\"#ff3b30\" stroke-width=\"3\" />\n";
    outFile << "<text x=\"" << aX + 18 << "\" y=\"" << aY - 15 << "\" fill=\"#ff3b30\" font-family=\"sans-serif\" font-size=\"14\" font-weight=\"bold\">Protected Asset</text>\n";

    // Overlay descriptive metadata headers onto the drawing layout
    outFile << "<text x=\"30\" y=\"50\" fill=\"#ffffff\" font-family=\"sans-serif\" font-size=\"22\" font-weight=\"bold\">Tactical Sector Layout Optimizer</text>\n";
    outFile << "<text x=\"30\" y=\"80\" fill=\"#8e9aa8\" font-family=\"sans-serif\" font-size=\"14\">Optimized Full Frame View (Fixed 140-degree Arcs - Expanded Boundary)</text>\n";

    outFile << "</svg>\n";
    outFile.close();

    // Capture completion timestamp and process differences
    auto mapEnd = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> mapElapsed = mapEnd - mapStart;

    std::cout << "\n[Success] Strategy generated. View visual output in your browser by opening: 'deployment_layout.svg'\n";

    // FIXED: Explicit namespaces bound safely to avoid error generation
    std::cout << "[Profile] SVG Rendering Phase completed in: " << std::fixed << std::setprecision(3) << mapElapsed.count() << " ms\n";
}

// ============================================================================
// GENETIC ALGORITHM ENGINE
// ============================================================================
class AdGunDeploymentGA {
private:
    BoundingBox bbox;
    double latStep, lonStep;
    int maxGridX, maxGridY;
    int popSize, generations;
    double mutationRate, crossoverRate;

    double assetLat, assetLon;
    double maxAllowedRadiusKm;
    double gunRangeKm;
    double desiredOverlapFactor;
    double fixedSweepWidth;

    // Profiling tracking variable accumulated over generation steps
    double totalEvaluationTimeMs = 0.0;

    /**
     * @brief Translates bounding box space constraints into a discrete computational mesh grid mapping.
     */
    void calculateGridSteps() {
        double centerLat = (bbox.minLat + bbox.maxLat) / 2.0;
        // Fixed steps scale coordinate grids to ~100m spatial cells
        latStep = 100.0 / 111000.0;
        lonStep = 100.0 / (111000.0 * std::cos(centerLat * PI / 180.0));
        maxGridX = static_cast<int>((bbox.maxLon - bbox.minLon) / lonStep);
        maxGridY = static_cast<int>((bbox.maxLat - bbox.minLat) / latStep);
    }

    /**
     * @brief Instantiates a deployment point container safely tracking valid bounding grid parameters.
     */
    GunDeploymentPoint createDeploymentPoint(int x, int y, double leftArc, double rightArc) {
        x = std::clamp(x, 0, maxGridX);
        y = std::clamp(y, 0, maxGridY);
        return {
            x, y,
            bbox.minLat + (static_cast<double>(y) * latStep) + (latStep / 2.0),
            bbox.minLon + (static_cast<double>(x) * lonStep) + (lonStep / 2.0),
            leftArc, rightArc, gunRangeKm
        };
    }

    /**
     * @brief Reverse translates real coordinate points directly back down to indexing steps.
     */
    void getGridCoords(double lat, double lon, int& x, int& y) {
        x = static_cast<int>((lon - bbox.minLon) / lonStep);
        y = static_cast<int>((lat - bbox.minLat) / latStep);
    }

    /**
     * @brief Calculates sector arcs to ensure coverage sweeps point directly outward from center asset.
     */
    void calculateOutwardFacingSector(double gunLat, double gunLon, double sweepWidthDeg, double& leftArc, double& rightArc) {
        double dLat = (assetLat - gunLat) * 111.0;
        double dLon = (assetLon - gunLon) * 111.0 * std::cos(gunLat * PI / 180.0);

        // Standard trigonometric atan2 calculation converted into clockwise directional angles
        double bearingToAsset = std::atan2(dLon, dLat) * 180.0 / PI;
        if (bearingToAsset < 0.0) bearingToAsset += 360.0;

        // Invert vector orientation to point outward into incoming threat vectors
        double outwardBearing = std::fmod(bearingToAsset + 180.0, 360.0);

        // Anchor symmetric boundaries evenly left and right around target heading vector
        leftArc = std::fmod(outwardBearing - (sweepWidthDeg / 2.0) + 360.0, 360.0);
        rightArc = std::fmod(outwardBearing + (sweepWidthDeg / 2.0) + 360.0, 360.0);
    }

public:
    AdGunDeploymentGA(BoundingBox box, double aLat, double aLon, double maxRadius, double range, double overlapFactor, int pop, double mRate, double cRate, int gens)
        : bbox(box), assetLat(aLat), assetLon(aLon), maxAllowedRadiusKm(maxRadius), gunRangeKm(range), desiredOverlapFactor(overlapFactor), popSize(pop), mutationRate(mRate), crossoverRate(cRate), generations(gens) {
        fixedSweepWidth = 140.0;
        calculateGridSteps();
    }

    /**
     * @brief Computes fitness score based on geometric alignment constraints and distance limitations.
     */
    void evaluateIndividual(GAIndividual& ind) {
        // High-Resolution Profiling Checkpoint: Tracking Individual Evaluation passes
        auto evalStart = std::chrono::high_resolution_clock::now();

        double assetDistancePenalty = 0.0;
        double structuralAlignmentPenalty = 0.0;

        // Constraint Loop 1: Verify elements sit squarely within minimum/maximum perimeter rings
        for (const auto& gun : ind.chromosome) {
            double dist = gun.distanceTo(assetLat, assetLon);
            if (dist > maxAllowedRadiusKm) {
                assetDistancePenalty += (dist - maxAllowedRadiusKm) * 150000.0;
            }
            if (dist < 0.08) {
                assetDistancePenalty += (0.08 - dist) * 50000.0;
            }
        }

        // Constraint Loop 2: Evaluate interconnected angular spacing gaps among deployed units
        for (size_t i = 0; i < ind.chromosome.size(); ++i) {
            for (size_t j = i + 1; j < ind.chromosome.size(); ++j) {
                double lat1 = ind.chromosome[i].lat - assetLat;
                double lon1 = (ind.chromosome[i].lon - assetLon) * std::cos(assetLat * PI / 180.0);
                double lat2 = ind.chromosome[j].lat - assetLat;
                double lon2 = (ind.chromosome[j].lon - assetLon) * std::cos(assetLat * PI / 180.0);

                double b1 = std::atan2(lon1, lat1) * 180.0 / PI; if (b1 < 0) b1 += 360.0;
                double b2 = std::atan2(lon2, lat2) * 180.0 / PI; if (b2 < 0) b2 += 360.0;

                double angleDiff = std::abs(b1 - b2);
                if (angleDiff > 180.0) angleDiff = 360.0 - angleDiff;

                // Penalize deviations from target angular spacing limits
                if (angleDiff < 105.0) {
                    structuralAlignmentPenalty += (105.0 - angleDiff) * 100000.0;
                }
                if (angleDiff > 135.0) {
                    structuralAlignmentPenalty += (angleDiff - 135.0) * 100000.0;
                }
            }
        }

        // Deduct collected penalty weights directly from the ideal baseline score
        double score = 180000.0 - (assetDistancePenalty + structuralAlignmentPenalty);
        ind.fitness = (score <= 0.0) ? 0.01 : score;

        // Accumulate timing data metrics over all iterations
        auto evalEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> evalElapsed = evalEnd - evalStart;
        totalEvaluationTimeMs += evalElapsed.count();
    }

    /**
     * @brief Executes the genetic optimization sequence over the configured population size.
     */
    GAIndividual run() {
        // High-Resolution Profiling Checkpoint: Start tracking main GA pipeline execution
        auto gaStart = std::chrono::high_resolution_clock::now();

        std::mt19937 runEngine(99); // Deterministic engine seed configuration
        std::vector<GAIndividual> population(popSize);
        std::uniform_real_distribution<double> distAngle(0.0, 360.0);
        std::uniform_real_distribution<double> distRadius(0.08, maxAllowedRadiusKm);

        // Profiling Checkpoint: Measure Population Initialization Phase
        auto initStart = std::chrono::high_resolution_clock::now();

        // Step 1: Initial creation loop allocating random spatial chromosomes
        for (int i = 0; i < popSize; ++i) {
            for (int j = 0; j < 3; ++j) {
                double bearing = (j * 120.0) + distAngle(runEngine) * 0.03;
                double rad = distRadius(runEngine);

                double spawnLat = assetLat + (rad / 111.0) * std::cos(bearing * PI / 180.0);
                double spawnLon = assetLon + (rad / (111.0 * std::cos(assetLat * PI / 180.0))) * std::sin(bearing * PI / 180.0);

                int gX, gY;
                getGridCoords(spawnLat, spawnLon, gX, gY);

                double leftArc, rightArc;
                calculateOutwardFacingSector(spawnLat, spawnLon, fixedSweepWidth, leftArc, rightArc);

                population[i].chromosome.push_back(createDeploymentPoint(gX, gY, leftArc, rightArc));
            }
            evaluateIndividual(population[i]);
        }

        auto initEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> initElapsed = initEnd - initStart;

        // Step 2: Main evolution iterations processing selection, crossover, and mutation steps
        auto loopsStart = std::chrono::high_resolution_clock::now();

        for (int gen = 0; gen < generations; ++gen) {
            std::vector<GAIndividual> nextGen;

            // Sort population descending to isolate high-performing chromosomes
            std::sort(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness > b.fitness; });

            // Direct elitism transfer preserving top candidate configurations unaltered
            nextGen.push_back(population[0]);
            nextGen.push_back(population[1]);

            // Fill remainder of subsequent population matrix
            while (nextGen.size() < (size_t)popSize) {
                auto tournament = [&]() {
                    int idx1 = runEngine() % popSize, idx2 = runEngine() % popSize;
                    return population[idx1].fitness > population[idx2].fitness ? population[idx1] : population[idx2];
                };
                GAIndividual p1 = tournament(), p2 = tournament();

                GAIndividual child = p1;
                // Single-point chromosome crossover execution pass
                if ((runEngine() % 100) < (crossoverRate * 100)) {
                    child.chromosome[1] = p2.chromosome[1];
                }

                // Node mutation via random adjacent spatial shifts
                if ((runEngine() % 100) < (mutationRate * 100)) {
                    int targetGene = runEngine() % 3;
                    std::uniform_int_distribution<int> nudge(-1, 1);

                    int newX = child.chromosome[targetGene].gridX + nudge(runEngine);
                    int newY = child.chromosome[targetGene].gridY + nudge(runEngine);

                    double checkLat = bbox.minLat + (static_cast<double>(newY) * latStep) + (latStep / 2.0);
                    double checkLon = bbox.minLon + (static_cast<double>(newX) * lonStep) + (lonStep / 2.0);

                    double leftArc, rightArc;
                    calculateOutwardFacingSector(checkLat, checkLon, fixedSweepWidth, leftArc, rightArc);

                    child.chromosome[targetGene] = createDeploymentPoint(newX, newY, leftArc, rightArc);
                }

                evaluateIndividual(child);
                nextGen.push_back(child);
            }
            population = std::move(nextGen);
        }

        // Final sort calculation pass to isolate the absolute optimal solution candidate
        std::sort(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness > b.fitness; });

        auto loopsEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> loopsElapsed = loopsEnd - loopsStart;

        auto gaEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> gaTotalElapsed = gaEnd - gaStart;

        // Print core optimization scorecard summary
        std::cout << "\n========================================================================\n";
        std::cout << "                 GA COMPUTATIONAL RUNTIME PROFILE REPORT                  \n";
        std::cout << "========================================================================\n";
        std::cout << " * Total Optimization Engine Execution Time : " << gaTotalElapsed.count() << " ms\n";
        std::cout << " * Population Initialization Step Duration  : " << initElapsed.count() << " ms\n";
        std::cout << " * Generational Evolution Loop Block Span   : " << loopsElapsed.count() << " ms\n";
        std::cout << " * Aggregated Fitness Evaluation Pass Time   : " << totalEvaluationTimeMs << " ms\n";
        std::cout << "========================================================================\n";

        return population[0];
    }
};

// ============================================================================
// MAIN RUN ENTRY
// ============================================================================
int main() {
    // Spatial search boundary canvas dimensions
    BoundingBox areaOfOps = { 34.0372, 34.1072, -118.2587, -118.1687 };
    double assetLat = 34.0772;
    double assetLon = -118.2187;

    // Instantiation setup:
    // Box, Asset coordinates, Max radius (0.18km), Range (3.0km), Overlap (0.30), Population (250), Mutation (0.08), Crossover (0.8), Generations (250)
    AdGunDeploymentGA engine(areaOfOps, assetLat, assetLon, 0.18, 3.0, 0.30, 250, 0.08, 0.8, 250);
    GAIndividual structuralSolution = engine.run();

    // Export structural solution parameters directly into vector representation drawings
    generateDeploymentMap(structuralSolution, assetLat, assetLon, areaOfOps);

    return 0;
}
