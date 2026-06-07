#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono> // Standard high-resolution profiling clocks library
#include <iomanip> // Mechanics handling console print formatting rules
#include <fstream> // File system stream management for disk writing
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <future>
#include <functional>
#include <string>

// Mathematical Pi constant used for conversions between degrees and radians
const double PI = 3.14159265358979323846;

// ============================================================================
// TECHNIQUE 1: THE C++17 THREAD POOL PATTERN
// ============================================================================
class ThreadPool {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable cv;
    bool stop;

public:
    explicit ThreadPool(size_t threads) : stop(false) {
        for (size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queueMutex);
                        this->cv.wait(lock, [this] {
                            return this->stop || !this->tasks.empty();
                        });
                        if (this->stop && this->tasks.empty()) {
                            return;
                        }
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>> {
        using return_type = std::invoke_result_t<F, Args...>;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (stop) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            tasks.emplace([task]() { (*task)(); });
        }
        cv.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        cv.notify_all();
        for (std::thread &worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }
};

// ============================================================================
// CONFIGURABLE GA METHOD ENUMS
// ============================================================================
enum class SelectionMethod { TOURNAMENT, ROULETTE_WHEEL, RANK_BASED, SUS };
enum class CrossoverMethod { SINGLE_POINT, MULTI_POINT, UNIFORM };
enum class MutationMethod  { SINGLE_POINT, SPATIAL_NUDGE, GAUSSIAN_CREEP };

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
void generateDeploymentMap(const GAIndividual& bestConfig, double assetLat, double assetLon, BoundingBox bbox, const std::string& filename = "deployment_layout.svg") {
    // FIXED: Correctly added the missing high-resolution starting anchor stamp
    auto mapStart = std::chrono::high_resolution_clock::now();

    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "Error creating vector file on disk: " << filename << "\n";
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

    std::cout << "[Success] Strategy generated. View visual output in your browser by opening: '" << filename << "'\n";
    std::cout << "[Profile] SVG Rendering Phase completed in: " << std::fixed << std::setprecision(3) << mapElapsed.count() << " ms\n\n";
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

    // Thread pool instance allocated for concurrent evaluation of individuals
    ThreadPool pool;

    // Mutex used to synchronize updates to the shared 'totalEvaluationTimeMs' profiling variable
    std::mutex evalTimeMutex;

    // Configured Selection, Crossover, and Mutation methods
    SelectionMethod selectionMethod;
    CrossoverMethod crossoverMethod;
    MutationMethod mutationMethod;

    // Control individual output prints
    bool verbose;

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

    /**
     * @brief Selection Implementation helper selecting parent indices for mating.
     */
    std::vector<int> selectParentIndices(const std::vector<GAIndividual>& pop, int numNeeded, std::mt19937& rng) {
        std::vector<int> indices;
        indices.reserve(numNeeded);

        if (selectionMethod == SelectionMethod::TOURNAMENT) {
            // TOURNAMENT SELECTION: Sample two random individuals, keep the best.
            for (int k = 0; k < numNeeded; ++k) {
                int idx1 = rng() % popSize;
                int idx2 = rng() % popSize;
                indices.push_back(pop[idx1].fitness > pop[idx2].fitness ? idx1 : idx2);
            }
        }
        else if (selectionMethod == SelectionMethod::ROULETTE_WHEEL) {
            // ROULETTE WHEEL SELECTION: Probability proportional to fitness.
            double totalFitness = 0.0;
            for (const auto& ind : pop) {
                totalFitness += ind.fitness;
            }

            std::uniform_real_distribution<double> dist(0.0, totalFitness);
            for (int k = 0; k < numNeeded; ++k) {
                double target = dist(rng);
                double currentSum = 0.0;
                int selectedIdx = popSize - 1; // Fallback
                for (int i = 0; i < popSize; ++i) {
                    currentSum += pop[i].fitness;
                    if (currentSum >= target) {
                        selectedIdx = i;
                        break;
                    }
                }
                indices.push_back(selectedIdx);
            }
        }
        else if (selectionMethod == SelectionMethod::RANK_BASED) {
            // RANK-BASED SELECTION: Probability proportional to linear rank (best has rank N, worst has rank 1).
            // Sum of ranks = N * (N + 1) / 2
            double totalRankSum = static_cast<double>(popSize * (popSize + 1)) / 2.0;
            std::uniform_real_distribution<double> dist(0.0, totalRankSum);
            for (int k = 0; k < numNeeded; ++k) {
                double target = dist(rng);
                double currentSum = 0.0;
                int selectedIdx = popSize - 1; // Fallback
                for (int i = 0; i < popSize; ++i) {
                    // Since pop is sorted descending: index 0 is rank = popSize; index i is rank = popSize - i
                    double rank = static_cast<double>(popSize - i);
                    currentSum += rank;
                    if (currentSum >= target) {
                        selectedIdx = i;
                        break;
                    }
                }
                indices.push_back(selectedIdx);
            }
        }
        else if (selectionMethod == SelectionMethod::SUS) {
            // STOCHASTIC UNIVERSAL SAMPLING (SUS): Single-phase selection using N pointers spaced equally.
            double totalFitness = 0.0;
            for (const auto& ind : pop) {
                totalFitness += ind.fitness;
            }

            double pointerDistance = totalFitness / static_cast<double>(numNeeded);
            std::uniform_real_distribution<double> dist(0.0, pointerDistance);
            double startPoint = dist(rng);

            int popIdx = 0;
            double currentSum = pop[0].fitness;

            for (int k = 0; k < numNeeded; ++k) {
                double pointer = startPoint + static_cast<double>(k) * pointerDistance;
                while (currentSum < pointer && popIdx < popSize - 1) {
                    popIdx++;
                    currentSum += pop[popIdx].fitness;
                }
                indices.push_back(popIdx);
            }

            // Shuffle chosen parent indices so that partners are paired randomly
            std::shuffle(indices.begin(), indices.end(), rng);
        }
        return indices;
    }

public:
    /**
     * @brief Constructor initializing optimization parameters and launching worker threads.
     */
    AdGunDeploymentGA(BoundingBox box, double aLat, double aLon, double maxRadius, double range, double overlapFactor, int pop, double mRate, double cRate, int gens,
                      SelectionMethod selMethod = SelectionMethod::TOURNAMENT,
                      CrossoverMethod crossMethod = CrossoverMethod::MULTI_POINT,
                      MutationMethod mutMethod = MutationMethod::SPATIAL_NUDGE,
                      bool isVerbose = true)
        : bbox(box), assetLat(aLat), assetLon(aLon), maxAllowedRadiusKm(maxRadius), gunRangeKm(range), desiredOverlapFactor(overlapFactor), popSize(pop), mutationRate(mRate), crossoverRate(cRate), generations(gens),
          pool(std::max(1u, std::thread::hardware_concurrency())),
          selectionMethod(selMethod), crossoverMethod(crossMethod), mutationMethod(mutMethod),
          verbose(isVerbose) {
        fixedSweepWidth = 140.0;
        calculateGridSteps();
    }

    std::string getSelectionMethodName() const {
        switch (selectionMethod) {
            case SelectionMethod::TOURNAMENT: return "Tournament";
            case SelectionMethod::ROULETTE_WHEEL: return "Roulette Wheel";
            case SelectionMethod::RANK_BASED: return "Rank-Based";
            case SelectionMethod::SUS: return "SUS";
        }
        return "Unknown";
    }

    std::string getCrossoverMethodName() const {
        switch (crossoverMethod) {
            case CrossoverMethod::SINGLE_POINT: return "Single-Point";
            case CrossoverMethod::MULTI_POINT: return "Multi-Point";
            case CrossoverMethod::UNIFORM: return "Uniform";
        }
        return "Unknown";
    }

    std::string getMutationMethodName() const {
        switch (mutationMethod) {
            case MutationMethod::SINGLE_POINT: return "Single-Point Reset";
            case MutationMethod::SPATIAL_NUDGE: return "Spatial Nudge";
            case MutationMethod::GAUSSIAN_CREEP: return "Gaussian Creep";
        }
        return "Unknown";
    }

    /**
     * @brief Computes fitness score based on geometric alignment constraints and distance limitations.
     */
    void evaluateIndividual(GAIndividual& ind) {
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

        // Accumulate timing data metrics over all iterations safely using a mutex
        auto evalEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> evalElapsed = evalEnd - evalStart;
        {
            std::lock_guard<std::mutex> lock(evalTimeMutex);
            totalEvaluationTimeMs += evalElapsed.count();
        }
    }

    /**
     * @brief Executes the genetic optimization sequence over the configured population size.
     */
    GAIndividual run() {
        auto gaStart = std::chrono::high_resolution_clock::now();

        std::mt19937 runEngine(99); // Deterministic engine seed configuration
        std::vector<GAIndividual> population(popSize);
        std::uniform_real_distribution<double> distAngle(0.0, 360.0);
        std::uniform_real_distribution<double> distRadius(0.08, maxAllowedRadiusKm);

        // Profiling Checkpoint: Measure Population Initialization Phase
        auto initStart = std::chrono::high_resolution_clock::now();

        // Step 1: Initial creation loop allocating random spatial chromosomes (Sequential Generation)
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
        }

        // Parallel Evaluation Phase for the Initial Population
        {
            std::vector<std::future<void>> futures;
            futures.reserve(popSize);
            for (int i = 0; i < popSize; ++i) {
                futures.push_back(pool.enqueue([this, &population, i]() {
                    evaluateIndividual(population[i]);
                }));
            }
            for (auto& f : futures) {
                f.get();
            }
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

            // Pre-select all parent indices for the generation in one go
            int numParentsNeeded = 2 * (popSize - 2);
            std::vector<int> parentIndices = selectParentIndices(population, numParentsNeeded, runEngine);
            int parentIdxCursor = 0;

            // Fill remainder of subsequent population matrix
            while (nextGen.size() < (size_t)popSize) {
                int p1Idx = parentIndices[parentIdxCursor++];
                int p2Idx = parentIndices[parentIdxCursor++];
                const GAIndividual& p1 = population[p1Idx];
                const GAIndividual& p2 = population[p2Idx];

                GAIndividual child = p1;

                // Perform Crossover based on CrossoverMethod
                if ((runEngine() % 100) < (crossoverRate * 100)) {
                    if (crossoverMethod == CrossoverMethod::SINGLE_POINT) {
                        int crossPoint = 1 + (runEngine() % 2); // 1 or 2
                        for (int g = crossPoint; g < 3; ++g) {
                            child.chromosome[g] = p2.chromosome[g];
                        }
                    }
                    else if (crossoverMethod == CrossoverMethod::MULTI_POINT) {
                        child.chromosome[1] = p2.chromosome[1];
                    }
                    else if (crossoverMethod == CrossoverMethod::UNIFORM) {
                        for (int g = 0; g < 3; ++g) {
                            if (runEngine() % 2 == 1) {
                                child.chromosome[g] = p2.chromosome[g];
                            }
                        }
                    }
                }

                // Perform Mutation based on MutationMethod
                if ((runEngine() % 100) < (mutationRate * 100)) {
                    if (mutationMethod == MutationMethod::SINGLE_POINT) {
                        int targetGene = runEngine() % 3;
                        double bearing = (targetGene * 120.0) + distAngle(runEngine) * 0.03;
                        double rad = distRadius(runEngine);

                        double spawnLat = assetLat + (rad / 111.0) * std::cos(bearing * PI / 180.0);
                        double spawnLon = assetLon + (rad / (111.0 * std::cos(assetLat * PI / 180.0))) * std::sin(bearing * PI / 180.0);

                        int gX, gY;
                        getGridCoords(spawnLat, spawnLon, gX, gY);

                        double leftArc, rightArc;
                        calculateOutwardFacingSector(spawnLat, spawnLon, fixedSweepWidth, leftArc, rightArc);

                        child.chromosome[targetGene] = createDeploymentPoint(gX, gY, leftArc, rightArc);
                    }
                    else if (mutationMethod == MutationMethod::SPATIAL_NUDGE) {
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
                    else if (mutationMethod == MutationMethod::GAUSSIAN_CREEP) {
                        int targetGene = runEngine() % 3;
                        std::normal_distribution<double> distGaussian(0.0, 0.05); // standard deviation of 50 meters (0.05 km)

                        double lat = child.chromosome[targetGene].lat;
                        double lon = child.chromosome[targetGene].lon;

                        double creepLatKm = distGaussian(runEngine);
                        double creepLonKm = distGaussian(runEngine);

                        double newLat = lat + (creepLatKm / 111.0);
                        double newLon = lon + (creepLonKm / (111.0 * std::cos(lat * PI / 180.0)));

                        int gX, gY;
                        getGridCoords(newLat, newLon, gX, gY);

                        double leftArc, rightArc;
                        calculateOutwardFacingSector(newLat, newLon, fixedSweepWidth, leftArc, rightArc);

                        child.chromosome[targetGene] = createDeploymentPoint(gX, gY, leftArc, rightArc);
                    }
                }

                nextGen.push_back(child);
            }

            // Parallel Evaluation Phase for newly generated children (indices 2 to popSize - 1)
            {
                std::vector<std::future<void>> futures;
                futures.reserve(nextGen.size() - 2);
                for (size_t i = 2; i < nextGen.size(); ++i) {
                    futures.push_back(pool.enqueue([this, &nextGen, i]() {
                        evaluateIndividual(nextGen[i]);
                    }));
                }
                for (auto& f : futures) {
                    f.get();
                }
            }

            population = std::move(nextGen);
        }

        // Final sort calculation pass to isolate the absolute optimal solution candidate
        std::sort(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness > b.fitness; });

        auto loopsEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> loopsElapsed = loopsEnd - loopsStart;

        auto gaEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> gaTotalElapsed = gaEnd - gaStart;

        if (verbose) {
            // Print core optimization scorecard summary
            std::cout << "\n========================================================================\n";
            std::cout << "                 GA COMPUTATIONAL RUNTIME PROFILE REPORT                  \n";
            std::cout << "========================================================================\n";
            std::cout << " * Selection Method                         : " << getSelectionMethodName() << "\n";
            std::cout << " * Crossover Method                         : " << getCrossoverMethodName() << "\n";
            std::cout << " * Mutation Method                          : " << getMutationMethodName() << "\n";
            std::cout << " * Total Optimization Engine Execution Time : " << gaTotalElapsed.count() << " ms\n";
            std::cout << " * Population Initialization Step Duration  : " << initElapsed.count() << " ms\n";
            std::cout << " * Generational Evolution Loop Block Span   : " << loopsElapsed.count() << " ms\n";
            std::cout << " * Aggregated Fitness Evaluation Pass Time   : " << totalEvaluationTimeMs << " ms\n";
            std::cout << "========================================================================\n";
        }

        return population[0];
    }
};

// ============================================================================
// HYPERPARAMETER CONFIGURATION PROFILE
// ============================================================================
struct GAConfig {
    std::string name;
    int popSize;
    double mutationRate;
    double crossoverRate;
    int generations;
    SelectionMethod selectionMethod;
    CrossoverMethod crossoverMethod;
    MutationMethod mutationMethod;
};

struct TestResult {
    std::string name;
    double bestFitness;
    double executionTimeMs;
    SelectionMethod selection;
    CrossoverMethod crossover;
    MutationMethod mutation;
    int popSize;
    int generations;
};

// ============================================================================
// MAIN RUN ENTRY
// ============================================================================
int main() {
    // Spatial search boundary canvas dimensions
    BoundingBox areaOfOps = { 34.0372, 34.1072, -118.2587, -118.1687 };
    double assetLat = 34.0772;
    double assetLon = -118.2187;

    // List of 12 distinct representative hyperparameter & technique configurations
    std::vector<GAConfig> sweepConfigs = {
        {"Standard Control",     250, 0.08, 0.80, 250, SelectionMethod::TOURNAMENT,     CrossoverMethod::MULTI_POINT,  MutationMethod::SPATIAL_NUDGE},
        {"Lightweight Profile",  100, 0.05, 0.70, 100, SelectionMethod::TOURNAMENT,     CrossoverMethod::SINGLE_POINT, MutationMethod::SPATIAL_NUDGE},
        {"Intensive Search",     500, 0.10, 0.85, 500, SelectionMethod::TOURNAMENT,     CrossoverMethod::UNIFORM,      MutationMethod::GAUSSIAN_CREEP},
        {"High-Crossover Expl.", 250, 0.05, 0.95, 250, SelectionMethod::SUS,            CrossoverMethod::UNIFORM,      MutationMethod::SPATIAL_NUDGE},
        {"High-Mutation Expl.",  250, 0.20, 0.60, 250, SelectionMethod::TOURNAMENT,     CrossoverMethod::MULTI_POINT,  MutationMethod::GAUSSIAN_CREEP},
        {"Roulette & Creep",     250, 0.08, 0.80, 250, SelectionMethod::ROULETTE_WHEEL, CrossoverMethod::SINGLE_POINT, MutationMethod::GAUSSIAN_CREEP},
        {"Rank-Based & Uniform", 250, 0.08, 0.80, 250, SelectionMethod::RANK_BASED,     CrossoverMethod::UNIFORM,      MutationMethod::SINGLE_POINT},
        {"SUS & Gaussian Creep", 250, 0.08, 0.80, 250, SelectionMethod::SUS,            CrossoverMethod::UNIFORM,      MutationMethod::GAUSSIAN_CREEP},
        {"SUS & Spatial Nudge",  250, 0.05, 0.80, 250, SelectionMethod::SUS,            CrossoverMethod::SINGLE_POINT, MutationMethod::SPATIAL_NUDGE},
        {"Tournament & Uniform", 250, 0.12, 0.80, 250, SelectionMethod::TOURNAMENT,     CrossoverMethod::UNIFORM,      MutationMethod::GAUSSIAN_CREEP},
        {"Rank-Based & Nudge",   250, 0.08, 0.80, 250, SelectionMethod::RANK_BASED,     CrossoverMethod::MULTI_POINT,  MutationMethod::SPATIAL_NUDGE},
        {"Roulette & Nudge",     250, 0.08, 0.80, 250, SelectionMethod::ROULETTE_WHEEL, CrossoverMethod::UNIFORM,      MutationMethod::SPATIAL_NUDGE}
    };

    std::cout << "========================================================================\n";
    std::cout << "            STARTING HYPERPARAMETER OPTIMIZATION SWEEP                  \n";
    std::cout << "========================================================================\n";
    std::cout << "Running 12 test case configurations with parallelized fitness evaluations...\n\n";

    std::vector<TestResult> results;
    results.reserve(sweepConfigs.size());

    for (size_t idx = 0; idx < sweepConfigs.size(); ++idx) {
        const auto& config = sweepConfigs[idx];
        std::cout << " [*] Running Test Case " << (idx + 1) << " / " << sweepConfigs.size()
                  << ": " << config.name << "..." << std::flush;

        // Run engine with verbose = false to suppress individual console prints
        AdGunDeploymentGA engine(areaOfOps, assetLat, assetLon, 0.18, 3.0, 0.30,
                                 config.popSize, config.mutationRate, config.crossoverRate, config.generations,
                                 config.selectionMethod, config.crossoverMethod, config.mutationMethod, false);

        auto start = std::chrono::high_resolution_clock::now();
        GAIndividual bestIndividual = engine.run();
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> elapsed = end - start;

        results.push_back({
            config.name,
            bestIndividual.fitness,
            elapsed.count(),
            config.selectionMethod,
            config.crossoverMethod,
            config.mutationMethod,
            config.popSize,
            config.generations
        });

        std::cout << " Done. (Time: " << std::fixed << std::setprecision(1) << elapsed.count()
                  << " ms, Best Fitness: " << std::setprecision(2) << bestIndividual.fitness << ")\n";

        // Save a subset of maps to demonstrate solutions
        if (config.name == "Standard Control") {
            generateDeploymentMap(bestIndividual, assetLat, assetLon, areaOfOps, "deployment_standard_sweep.svg");
        } else if (config.name == "SUS & Gaussian Creep") {
            generateDeploymentMap(bestIndividual, assetLat, assetLon, areaOfOps, "deployment_sus_gaussian_sweep.svg");
        } else if (config.name == "Intensive Search") {
            generateDeploymentMap(bestIndividual, assetLat, assetLon, areaOfOps, "deployment_intensive_sweep.svg");
        }
    }

    // Sort leaderboard by Best Fitness achieved (descending), then by execution speed (ascending)
    std::sort(results.begin(), results.end(), [](const TestResult& a, const TestResult& b) {
        if (std::abs(a.bestFitness - b.bestFitness) > 0.001) {
            return a.bestFitness > b.bestFitness; // Higher fitness is better
        }
        return a.executionTimeMs < b.executionTimeMs; // Faster time is better
    });

    // Helper functions for printing enum names in leaderboard
    auto getSelectionStr = [](SelectionMethod method) {
        switch (method) {
            case SelectionMethod::TOURNAMENT: return "Tournament";
            case SelectionMethod::ROULETTE_WHEEL: return "Roulette";
            case SelectionMethod::RANK_BASED: return "Rank-Based";
            case SelectionMethod::SUS: return "SUS";
        }
        return "Unknown";
    };
    auto getCrossoverStr = [](CrossoverMethod method) {
        switch (method) {
            case CrossoverMethod::SINGLE_POINT: return "Single-Pt";
            case CrossoverMethod::MULTI_POINT: return "Multi-Pt";
            case CrossoverMethod::UNIFORM: return "Uniform";
        }
        return "Unknown";
    };
    auto getMutationStr = [](MutationMethod method) {
        switch (method) {
            case MutationMethod::SINGLE_POINT: return "Reset";
            case MutationMethod::SPATIAL_NUDGE: return "Nudge";
            case MutationMethod::GAUSSIAN_CREEP: return "Gaussian";
        }
        return "Unknown";
    };

    // Print comparative Leaderboard table in standard output
    std::cout << "\n\n";
    std::cout << "=========================================================================================\n";
    std::cout << "                           HYPERPARAMETER LEADERBOARD RESULTS                            \n";
    std::cout << "=========================================================================================\n";
    std::cout << " Rank | " << std::left << std::setw(22) << "Config Profile Name"
              << " | " << std::setw(11) << "Selection"
              << " | " << std::setw(9) << "Crossover"
              << " | " << std::setw(9) << "Mutation"
              << " | " << std::right << std::setw(5) << "Pop"
              << " | " << std::setw(5) << "Gen"
              << " | " << std::setw(12) << "Best Fitness"
              << " | " << std::setw(10) << "Time (ms)"
              << " |\n";
    std::cout << "------|------------------------|-------------|-----------|-----------|-------|-------|--------------|------------|\n";

    for (size_t idx = 0; idx < results.size(); ++idx) {
        const auto& r = results[idx];
        std::cout << "  " << std::right << std::setw(2) << (idx + 1) << "  | "
                  << std::left << std::setw(22) << r.name << " | "
                  << std::setw(11) << getSelectionStr(r.selection) << " | "
                  << std::setw(9) << getCrossoverStr(r.crossover) << " | "
                  << std::setw(9) << getMutationStr(r.mutation) << " | "
                  << std::right << std::setw(5) << r.popSize << " | "
                  << std::setw(5) << r.generations << " | "
                  << std::setw(12) << std::fixed << std::setprecision(2) << r.bestFitness << " | "
                  << std::setw(10) << std::fixed << std::setprecision(1) << r.executionTimeMs << " |\n";
    }
    std::cout << "=========================================================================================\n";
    std::cout << "Note: Leaderboard is sorted primarily by Best Fitness (descending), secondary by Time (ascending).\n\n";

    return 0;
}
