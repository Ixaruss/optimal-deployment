#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <iomanip>
#include <string>
#include <fstream>

// ============================================================================
// STRUCTS, ENUMS, & GLOBAL GEOMETRY
// ============================================================================

// Physical constants used for geospatial calculations
const double EARTH_RADIUS_KM = 6371.0; // Mean radius of the Earth for distance math
const double PI = 3.14159265358979323846;

// Enums to cleanly modularize genetic strategies
enum class SelectionMethod { TOURNAMENT, ROULETTE_WHEEL, RANK_BASED, SUS };
enum class CrossoverMethod { SINGLE_POINT, MULTI_POINT, UNIFORM };
enum class MutationMethod  { SINGLE_POINT, SPATIAL_NUDGE, GAUSSIAN_CREEP };

// Geographical area restriction mapping
struct BoundingBox {
    double minLat, maxLat, minLon, maxLon;
};

// Represents a single 2D location on our discrete 100-meter resolution map grid
struct GridPoint {
    int gridX, gridY; // Discrete offsets from the BoundingBox minimum constraints
    double lat, lon;  // True derived spherical coordinates at the center of this specific grid cell

    /**
     * Calculates the true great-circle distance between two coordinates on a sphere.
     * Uses the Haversine formula to maintain high precision even over very short distances.
     */
    double distanceTo(const GridPoint& other) const {
        // Convert latitudes from degrees to radians
        double lat1Rad = lat * PI / 180.0;
        double lat2Rad = other.lat * PI / 180.0;

        // Calculate coordinate deltas in radians
        double deltaLat = (other.lat - lat) * PI / 180.0;
        double deltaLon = (other.lon - lon) * PI / 180.0;

        // Compute the central angle between the points
        double a = std::sin(deltaLat / 2.0) * std::sin(deltaLat / 2.0) +
                   std::cos(lat1Rad) * std::cos(lat2Rad) * std::sin(deltaLon / 2.0) * std::sin(deltaLon / 2.0);

        // Return the final physical distance in kilometers
        return EARTH_RADIUS_KM * (2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a)));
    }
};

// Represents an individual solution within the Genetic Algorithm population
struct GAIndividual {
    std::vector<GridPoint> chromosome; // An ordered path of physical grid coordinate waypoints
    double fitness;                    // Optimization score (higher = better path quality)
};

// ============================================================================
// HARDWARE THREAD POOL ENGINE
// ============================================================================

class ThreadPool {
private:
    std::vector<std::thread> workers;       // Active system threads waiting for work
    std::queue<std::function<void()>> tasks;// Thread-safe FIFO queue containing pending tasks
    std::mutex queueMutex;                  // Mutex locks ensuring unique access to the queue
    std::condition_variable cv;             // Blocks or wakes worker threads dynamically
    bool stop;                              // Execution termination flag

public:
    /**
     * Initializes the worker pool based on native hardware concurrency capabilities.
     */
    ThreadPool() : stop(false) {
        // Detect available processor cores
        size_t threads = std::thread::hardware_concurrency();
        if (threads == 0) threads = 4; // Defensive fallback configuration if detection fails

        for (size_t i = 0; i < threads; ++i) {
            // Emplace active lambdas acting as continuous internal processing engines
            workers.emplace_back([this]() {
                while (true) {
                    std::function<void()> task;
                    {
                        // Safely lock the queue to verify state changes
                        std::unique_lock<std::mutex> lock(this->queueMutex);

                        // Sleep thread until new tasks are queued or system shuts down
                        this->cv.wait(lock, [this]() { return this->stop || !this->tasks.empty(); });

                        // Exit worker execution loop if termination is requested and queue is drained
                        if (this->stop && this->tasks.empty()) return;

                        // Extract task from front of queue via efficient move semantics
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    } // Mutex is implicitly unlocked here to allow other threads to read the queue

                    task(); // Execute the extracted task
                }
            });
        }
    }

    /**
     * Schedules a task asynchronously. Accepts any callable function and arguments.
     * Returns a std::future containing the eventual execution results.
     */
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result<F, Args...>::type> {
        using return_type = typename std::invoke_result<F, Args...>::type;

        // Package the function pointer alongside its arguments into a shared reference block
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (stop) throw std::runtime_error("Enqueue requested on stopped ThreadPool context");

            // Wrap the packaged task execution inside a generalized void wrapper
            tasks.emplace([task]() { (*task)(); });
        }
        cv.notify_one(); // Wake up exactly one sleeping thread to execute the new task
        return res;      // Return future context to caller
    }

    /**
     * Destructor: Ensures graceful shutdowns, joining all threads safely.
     */
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        cv.notify_all(); // Wake up all worker threads to finalize execution loops

        for (std::thread &worker : workers) {
            if (worker.joinable()) worker.join(); // Block main execution loop until thread exits
        }
    }
};

// ============================================================================
// PARALLELIZED SPATIAL GENETIC ALGORITHM FRAMEWORK
// ============================================================================

class ParallelComprehensiveGA {
private:
    BoundingBox bbox;            // Map boundary constraints
    double latStep, lonStep;     // Calculated coordinate step distances matching exactly 100 meters
    int maxGridX, maxGridY;      // Absolute maximum width and height metrics of our grid space
    int popSize, generations;    // Evolutionary structural rules limits
    double mutationRate, crossoverRate;
    ThreadPool& pool;            // Reference link connecting to the shared hardware thread pool
    std::mt19937 rng;            // High-entropy random number generation engine

    // Active operators configured dynamically at runtime
    SelectionMethod currentSelection = SelectionMethod::TOURNAMENT;
    CrossoverMethod currentCrossover  = CrossoverMethod::SINGLE_POINT;
    MutationMethod currentMutation    = MutationMethod::SINGLE_POINT;

    // Control output verbosity
    bool verbose = true;

    // --- High-Resolution Metric Fields ---
    double totalFitnessEvalTimeMs = 0.0;    // Tracks concurrent ThreadPool blocking latency
    double totalEvolutionaryTimeMs = 0.0;   // Tracks sequential selection, crossover, and mutation steps

    /**
     * Converts a raw coordinate bounding box down into a normalized 2D grid matrix
     * where each cell corresponds to a 100m x 100m physical area.
     */
    void calculateGridSteps() {
        double centerLat = (bbox.minLat + bbox.maxLat) / 2.0;

        // 1 degree of latitude is roughly fixed at 111,000 meters
        latStep = 100.0 / 111000.0;

        // Longitude tracking degrees shrink proportionally as latitude moves away from the equator
        lonStep = 100.0 / (111000.0 * std::cos(centerLat * PI / 180.0));

        // Establish maximum cell index boundaries across width and height dimensions
        maxGridX = static_cast<int>((bbox.maxLon - bbox.minLon) / lonStep);
        maxGridY = static_cast<int>((bbox.maxLat - bbox.minLat) / latStep);
    }

    /**
     * Factory function that converts simple grid coordinates into full, real-world coordinates.
     */
    GridPoint createGridPoint(int x, int y) {
        return {
            x,
            y,
            bbox.minLat + (static_cast<double>(y) * latStep) + (latStep / 2.0), // Center value offset adjustment
            bbox.minLon + (static_cast<double>(x) * lonStep) + (lonStep / 2.0)  // Center value offset adjustment
        };
    }

public:
    ParallelComprehensiveGA(BoundingBox box, int pop, double mRate, double cRate, int gens, ThreadPool& p)
        : bbox(box), popSize(pop), mutationRate(mRate), crossoverRate(cRate), generations(gens), pool(p) {
        // Seed random generator state using current system time
        rng.seed(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
        calculateGridSteps();
    }

    // Changes operators on the fly between generations or experimental benchmarks
    void setOperators(SelectionMethod sm, CrossoverMethod cm, MutationMethod mm) {
        currentSelection = sm;
        currentCrossover = cm;
        currentMutation = mm;
    }


    /**
     * Exports the optimized path as a high-resolution scalable vector image (.svg).
     * Open this file in any web browser to view the clean graphics layout.
     */
    void exportPathToSVG(const GAIndividual& bestInd, const std::string& filename = "best_path.svg") {
        std::ofstream outFile(filename);
        if (!outFile) {
            std::cout << " [!] Error: Could not create SVG file.\n";
            return;
        }

        // Image Dimensions
        const int width = 800;
        const int height = 600;
        const int padding = 50;

        // SVG Header
        outFile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
        outFile << "<svg width=\"" << width << "\" height=\"" << height
                << "\" xmlns=\"http://www.w3.org/2000/svg\" style=\"background-color: #f8f9fa;\">\n";

        // Draw Map Grid/Border lines
        outFile << "  \n";
        outFile << "  <rect x=\"" << padding << "\" y=\"" << padding << "\" width=\""
                << (width - 2 * padding) << "\" height=\"" << (height - 2 * padding)
                << "\" fill=\"#ffffff\" stroke=\"#dee2e6\" stroke-width=\"2\" rx=\"8\" />\n";

        // Helper lambda to map physical grid coordinates down to image pixel coordinates
        auto mapX = [&](int gx) {
            return padding + (static_cast<double>(gx) / maxGridX) * (width - 2 * padding);
        };
        auto mapY = [&](int gy) {
            // Invert Y axis because SVG (0,0) starts at the top-left corner
            return height - padding - (static_cast<double>(gy) / maxGridY) * (height - 2 * padding);
        };

        // Step 1: Draw the Path Polyline connecting all sequential waypoints
        outFile << "\n  \n";
        outFile << "  <polyline points=\"";
        for (const auto& wp : bestInd.chromosome) {
            outFile << mapX(wp.gridX) << "," << mapY(wp.gridY) << " ";
        }
        outFile << "\" fill=\"none\" stroke=\"#dc3545\" stroke-width=\"4\" stroke-linejoin=\"round\" stroke-linecap=\"round\" />\n";

        // Step 2: Render individual Waypoint nodes with custom distinct color markers
        outFile << "\n  \n";
        for (size_t i = 0; i < bestInd.chromosome.size(); ++i) {
            const auto& wp = bestInd.chromosome[i];
            double cx = mapX(wp.gridX);
            double cy = mapY(wp.gridY);

            std::string color = "#007bff"; // Default Waypoint blue color
            std::string label = "Wp " + std::to_string(i);
            double radius = 6.0;

            if (i == 0) {
                color = "#28a745"; // Green for the Start Node
                label = "START";
                radius = 8.0;
            } else if (i == bestInd.chromosome.size() - 1) {
                color = "#212529"; // Dark grey/black for the End Node
                label = "END";
                radius = 8.0;
            }

            // Draw node circle element
            outFile << "  <circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"" << radius
                    << "\" fill=\"" << color << "\" stroke=\"#ffffff\" stroke-width=\"2\" />\n";

            // Draw node overlay text sequence label strings
            outFile << "  <text x=\"" << (cx + 10) << "\" y=\"" << (cy + 4)
                    << "\" font-family=\"Arial, sans-serif\" font-size=\"12\" font-weight=\"bold\" fill=\"#343a40\">"
                    << label << "</text>\n";
        }

        // SVG Footer Title Elements
        outFile << "\n  \n";
        outFile << "  <text x=\"" << padding << "\" y=\"" << (padding - 15)
                << "\" font-family=\"Arial, sans-serif\" font-size=\"16\" font-weight=\"bold\" fill=\"#212529\">"
                << "Parallel GA - Optimized Path Matrix Map Result</text>\n";

        outFile << "</svg>\n";
        outFile.close();

        std::cout << ">> [Success] High-resolution path map layout exported to 'best_path.svg'!\n";
    }

    // Sets whether output logs should print to stdout during execution
    void setVerbose(bool isVerbose) {
        verbose = isVerbose;
    }

    // High-Resolution Performance Profiling Getters
    double getFitnessEvalTime() const { return totalFitnessEvalTimeMs; }
    double getEvolutionaryTime() const { return totalEvolutionaryTimeMs; }

    /**
     * Thread-Safe Evaluation: Evaluates individual total path distances.
     * Designed with a static context to protect against thread race conditions.
     */
    static void evaluateIndividual(GAIndividual& ind) {
        double totalDistance = 0.0;
        int duplicates = 0;

        for (size_t i = 0; i < ind.chromosome.size() - 1; ++i) {
            double dist = ind.chromosome[i].distanceTo(ind.chromosome[i + 1]);
            if (dist == 0.0) {
                duplicates++;
            }
            totalDistance += dist;
        }

        if (totalDistance == 0.0 || duplicates > 0) {
            // Punish solutions that stand still or reuse points
            // by making their fitness incredibly close to 0
            ind.fitness = 0.00001 / (duplicates + 1);
        } else {
            ind.fitness = 1.0 / totalDistance;
        }
    }

    /**
     * Parallelization Barrier: Spreads the work of evaluating the population's
     * fitness across all available threads.
     */
    void parallelEvaluatePopulation(std::vector<GAIndividual>& population) {
        auto startTime = std::chrono::high_resolution_clock::now();

        std::vector<std::future<void>> futures;
        futures.reserve(population.size());

        // Enqueue each individual's evaluation task into the ThreadPool
        for (auto& ind : population) {
            futures.push_back(pool.enqueue(ParallelComprehensiveGA::evaluateIndividual, std::ref(ind)));
        }

        // Synchronous Blocking Point: Wait until all threads finish calculations
        for (auto& f : futures) f.wait();

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = endTime - startTime;
        totalFitnessEvalTimeMs += duration.count();
    }

    // ============================================================================
    // SELECTION PHASE IMPLEMENTATIONS
    // ============================================================================

    /**
     * Tournament Selection: Randomly selects a few individuals and chooses the best one.
     */
    GAIndividual tournamentSelection(const std::vector<GAIndividual>& population) {
        std::uniform_int_distribution<int> dist(0, static_cast<int>(population.size() - 1));

        // Pick an initial competitor at random
        GAIndividual best = population[static_cast<size_t>(dist(rng))];

        // Compare against two more random opponents (Tournament Size = 3)
        for (int i = 0; i < 2; ++i) {
            GAIndividual comp = population[static_cast<size_t>(dist(rng))];
            if (comp.fitness > best.fitness) best = comp; // Keep the individual with the highest fitness
        }
        return best;
    }

    /**
     * Roulette Wheel Selection: Picks individuals with a probability proportional
     * to their fitness (like a weighted lottery wheel).
     */
    GAIndividual rouletteWheelSelection(const std::vector<GAIndividual>& population) {
        double totalFitness = 0.0;
        for (const auto& ind : population) totalFitness += ind.fitness;

        // Spin the wheel: pick a random value between 0 and totalFitness sum
        std::uniform_real_distribution<double> dist(0.0, totalFitness);
        double slice = dist(rng);
        double currentSum = 0.0;

        // Step through the population until the running sum reaches the random slice value
        for (const auto& ind : population) {
            currentSum += ind.fitness;
            if (currentSum >= slice) return ind;
        }
        return population.back(); // Fallback edge catch case
    }

    /**
     * Rank-Based Selection: Selects individuals based on their rank rather than raw fitness scores.
     */
    GAIndividual rankBasedSelection(const std::vector<GAIndividual>& population) {
        int n = static_cast<int>(population.size());
        // Calculate the sum of all ranks using Gauss's formula: (n * (n + 1)) / 2
        int totalRankSum = (n * (n + 1)) / 2;

        std::uniform_int_distribution<int> dist(1, totalRankSum);
        int target = dist(rng);
        int rankSum = 0;

        // Step through sorted items. Higher ranks (indexes) have a larger selection window.
        for (int i = 0; i < n; ++i) {
            rankSum += (i + 1);
            if (rankSum >= target) return population[static_cast<size_t>(i)];
        }
        return population.back();
    }

    /**
     * Stochastic Universal Sampling (SUS): Uses a single spin with multiple equally spaced
     * pointers to choose parents.
     */
    std::vector<GAIndividual> stochasticUniversalSampling(const std::vector<GAIndividual>& population, int numToSelect) {
        std::vector<GAIndividual> selected;
        double totalFitness = 0.0;
        for (const auto& ind : population) totalFitness += ind.fitness;

        // Determine the distance between pointers
        double pointerDistance = totalFitness / numToSelect;

        // Pick a random starting point for the first pointer
        std::uniform_real_distribution<double> dist(0.0, pointerDistance);
        double startPointer = dist(rng);

        double currentSum = 0.0;
        size_t idx = 0;

        // Loop through and collect parents matching each pointer position
        for (int i = 0; i < numToSelect; ++i) {
            double currentPointer = startPointer + (static_cast<double>(i) * pointerDistance);

            // Advance index until the running sum covers the current pointer position
            while (currentSum + population[idx].fitness < currentPointer && idx < population.size() - 1) {
                currentSum += population[idx].fitness;
                idx++;
            }
            selected.push_back(population[idx]);
        }
        return selected;
    }

    /**
     * Gateway selector function to route execution to the configured selection operator.
     */
    GAIndividual executeSingleSelection(const std::vector<GAIndividual>& population) {
        if (currentSelection == SelectionMethod::TOURNAMENT) return tournamentSelection(population);
        if (currentSelection == SelectionMethod::ROULETTE_WHEEL) return rouletteWheelSelection(population);
        return rankBasedSelection(population); // Handles rank fallback routes safely
    }

// ============================================================================
// CROSSOVER PHASE IMPLEMENTATIONS
// ============================================================================

    /**
     * Single-Point Crossover: Splits parents at one random point and swaps their segments.
     */
    std::pair<GAIndividual, GAIndividual> singlePointCrossover(const GAIndividual& p1, const GAIndividual& p2) {
        int len = static_cast<int>(p1.chromosome.size());
        std::uniform_int_distribution<int> distPoint(1, len - 2); // Avoid splitting at the absolute ends
        int cp = distPoint(rng); // Crossover Point index

        GAIndividual c1, c2;
        // Construct child 1: Parent 1 left side + Parent 2 right side
        c1.chromosome.insert(c1.chromosome.end(), p1.chromosome.begin(), p1.chromosome.begin() + cp);
        c1.chromosome.insert(c1.chromosome.end(), p2.chromosome.begin() + cp, p2.chromosome.end());

        // Construct child 2: Parent 2 left side + Parent 1 right side
        c2.chromosome.insert(c2.chromosome.end(), p2.chromosome.begin(), p2.chromosome.begin() + cp);
        c2.chromosome.insert(c2.chromosome.end(), p1.chromosome.begin() + cp, p1.chromosome.end());
        return {c1, c2};
    }

    /**
     * Multi-Point Crossover: Splits parents at two points and swaps the middle section.
     */
    std::pair<GAIndividual, GAIndividual> multiPointCrossover(const GAIndividual& p1, const GAIndividual& p2) {
        int len = static_cast<int>(p1.chromosome.size());
        std::uniform_int_distribution<int> distPoint(1, len - 2);
        int cp1 = distPoint(rng);
        int cp2 = distPoint(rng);
        if (cp1 > cp2) std::swap(cp1, cp2); // Keep indices in ascending order

        GAIndividual c1, c2;
        for (int i = 0; i < len; ++i) {
            // Swap genetic material if index is within the crossover window bounds
            if (i >= cp1 && i < cp2) {
                c1.chromosome.push_back(p2.chromosome[static_cast<size_t>(i)]);
                c2.chromosome.push_back(p1.chromosome[static_cast<size_t>(i)]);
            } else {
                c1.chromosome.push_back(p1.chromosome[static_cast<size_t>(i)]);
                c2.chromosome.push_back(p2.chromosome[static_cast<size_t>(i)]);
            }
        }
        return {c1, c2};
    }

    /**
     * Uniform Crossover: Iterates through each gene and randomly flips them between parents.
     */
    std::pair<GAIndividual, GAIndividual> uniformCrossover(const GAIndividual& p1, const GAIndividual& p2) {
        std::uniform_real_distribution<double> distZeroOne(0.0, 1.0);
        GAIndividual c1, c2;

        for (size_t i = 0; i < p1.chromosome.size(); ++i) {
            if (distZeroOne(rng) < 0.5) { // 50% chance to flip source mapping path lines
                c1.chromosome.push_back(p1.chromosome[i]);
                c2.chromosome.push_back(p2.chromosome[i]);
            } else {
                c1.chromosome.push_back(p2.chromosome[i]);
                c2.chromosome.push_back(p1.chromosome[i]);
            }
        }
        return {c1, c2};
    }

    /**
     * Gateway router function to handle crossover logic using the crossover rate.
     */
    std::pair<GAIndividual, GAIndividual> executeCrossover(const GAIndividual& p1, const GAIndividual& p2) {
        std::uniform_real_distribution<double> distZeroOne(0.0, 1.0);
        // Skip crossover if the rolled value is greater than the crossover rate
        if (distZeroOne(rng) > crossoverRate) return {p1, p2};

        if (currentCrossover == CrossoverMethod::SINGLE_POINT) return singlePointCrossover(p1, p2);
        if (currentCrossover == CrossoverMethod::MULTI_POINT) return multiPointCrossover(p1, p2);
        return uniformCrossover(p1, p2);
    }

    // ============================================================================
    // MUTATION PHASE IMPLEMENTATIONS
    // ============================================================================

    /**
     * Single-Point Mutation: Completely replaces one random gene with a new, random coordinate point.
     */
    void singlePointMutation(GAIndividual& ind) {
        std::uniform_int_distribution<int> distIndex(0, static_cast<int>(ind.chromosome.size() - 1));
        std::uniform_int_distribution<int> distX(0, maxGridX);
        std::uniform_int_distribution<int> distY(0, maxGridY);

        size_t idx = static_cast<size_t>(distIndex(rng));
        ind.chromosome[idx] = createGridPoint(distX(rng), distY(rng));
    }

    /**
     * Spatial Nudge Mutation: Shifts coordinates slightly within a small grid radius (+/- 2 cells).
     */
    void spatialNudgeMutation(GAIndividual& ind) {
        std::uniform_real_distribution<double> distZeroOne(0.0, 1.0);
        std::uniform_int_distribution<int> distNudge(-2, 2); // Maximum offset distance

        for (auto& gene : ind.chromosome) {
            if (distZeroOne(rng) < mutationRate) {
                // Adjust position while ensuring values stay within valid grid bounds
                int nx = std::clamp(gene.gridX + distNudge(rng), 0, maxGridX);
                int ny = std::clamp(gene.gridY + distNudge(rng), 0, maxGridY);
                gene = createGridPoint(nx, ny);
            }
        }
    }

    /**
     * Gaussian Creep Mutation: Shifts coordinates using a normal distribution curve.
     */
    void gaussianCreepMutation(GAIndividual& ind, std::normal_distribution<double>& gauss) {
        std::uniform_real_distribution<double> distZeroOne(0.0, 1.0);

        for (auto& gene : ind.chromosome) {
            if (distZeroOne(rng) < mutationRate) {
                // Apply a bell-curve offset value rounded to the nearest integer
                int nx = std::clamp(gene.gridX + static_cast<int>(std::round(gauss(rng))), 0, maxGridX);
                int ny = std::clamp(gene.gridY + static_cast<int>(std::round(gauss(rng))), 0, maxGridY);
                gene = createGridPoint(nx, ny);
            }
        }
    }

    /**
     * Gateway router function to execute the active mutation operator strategy.
     */
    void executeMutation(GAIndividual& ind, std::normal_distribution<double>& gauss) {
        std::uniform_real_distribution<double> distZeroOne(0.0, 1.0);

        if (currentMutation == MutationMethod::SINGLE_POINT) {
            if (distZeroOne(rng) < mutationRate) singlePointMutation(ind);
        } else if (currentMutation == MutationMethod::SPATIAL_NUDGE) {
            spatialNudgeMutation(ind);
        } else {
            gaussianCreepMutation(ind, gauss);
        }
    }

    // ============================================================================
    // RUN LOOPS PIPELINE CONTEXT ENGINE
    // ============================================================================

    /**
     * Runs the Genetic Algorithm simulation loop based on the current configuration.
     */
    double runGA(int pathLength) {
        // Reset track accumulation limits across individual optimization steps
        totalFitnessEvalTimeMs = 0.0;
        totalEvolutionaryTimeMs = 0.0;

        // [Phase 1 Instrumentation]: Track layout generation profiles
        auto initStart = std::chrono::high_resolution_clock::now();

        std::vector<GAIndividual> population(static_cast<size_t>(popSize));
        std::uniform_int_distribution<int> distX(0, maxGridX), distY(0, maxGridY);

        for (int i = 0; i < popSize; ++i) {
            for (int j = 0; j < pathLength; ++j) {
                population[static_cast<size_t>(i)].chromosome.push_back(createGridPoint(distX(rng), distY(rng)));
            }
        }

        auto initEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> initDuration = initEnd - initStart;

        parallelEvaluatePopulation(population);

        std::normal_distribution<double> gaussDistribution(0.0, 2.0);

        for (int gen = 1; gen <= generations; ++gen) {
            // [Phase 2 Instrumentation]: Isolate localized serial vector manipulation steps
            auto stepStart = std::chrono::high_resolution_clock::now();

            if (currentSelection == SelectionMethod::RANK_BASED) {
                std::sort(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) {
                    return a.fitness < b.fitness;
                });
            }

            std::vector<GAIndividual> nextGen;
            nextGen.reserve(static_cast<size_t>(popSize));

            auto bestInd = std::max_element(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness < b.fitness; });
            nextGen.push_back(*bestInd);

            bool useSUS = (currentSelection == SelectionMethod::SUS);
            std::vector<GAIndividual> matingPool;
            int poolSizeNeeded = popSize - static_cast<int>(nextGen.size());

            if (useSUS) {
                matingPool = stochasticUniversalSampling(population, poolSizeNeeded);
            }

            size_t poolIdx = 0;
            while (nextGen.size() < static_cast<size_t>(popSize)) {
                GAIndividual p1, p2;
                if (useSUS) {
                    p1 = matingPool[poolIdx % matingPool.size()];
                    poolIdx++;
                    p2 = matingPool[poolIdx % matingPool.size()];
                    poolIdx++;
                } else {
                    p1 = executeSingleSelection(population);
                    p2 = executeSingleSelection(population);
                }

                auto [c1, c2] = executeCrossover(p1, p2);

                executeMutation(c1, gaussDistribution);
                executeMutation(c2, gaussDistribution);

                nextGen.push_back(c1);
                if (nextGen.size() < static_cast<size_t>(popSize)) nextGen.push_back(c2);
            }

            auto stepEnd = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> stepDuration = stepEnd - stepStart;
            totalEvolutionaryTimeMs += stepDuration.count();

            parallelEvaluatePopulation(nextGen);
            population = std::move(nextGen);

            if (verbose && (gen % 25 == 0 || gen == 1)) {
                auto currentBest = std::max_element(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness < b.fitness; });
                std::cout << "    Gen " << gen << " | Shortest Path Matrix Result: " << (1.0 / currentBest->fitness) << " km\n";
            }
        }

        auto finalBest = std::max_element(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness < b.fitness; });
        if (verbose) {
            std::cout << ">> Optimization Strategy Yielded Best Distance: " << (1.0 / finalBest->fitness) << " km\n";

            // Print localized metric data block for individual matrix variations
            std::cout << "   -----------------------------------------------------------------\n";
            std::cout << "   | LOCAL METRIC BENCHMARK REPORT:                                |\n";
            std::cout << "   -----------------------------------------------------------------\n";
            std::cout << "   | [Phase 1] Chromosome Vector Space Init     : " << std::setw(10) << std::setprecision(2) << initDuration.count() << " ms |\n";
            std::cout << "   | [Phase 2] Cumulative Evolutionary Steps    : " << std::setw(10) << totalEvolutionaryTimeMs << " ms |\n";
            std::cout << "   | [Phase 3] Cumulative ThreadPool Fitness Eval: " << std::setw(10) << totalFitnessEvalTimeMs << " ms |\n";
            std::cout << "   =================================================================\n\n";

            auto finalBest = std::max_element(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness < b.fitness; });
                    if (verbose) {
                        std::cout << ">> Optimization Strategy Yielded Best Distance: " << (1.0 / finalBest->fitness) << " km\n";
                        std::cout << "   =================================================================\n";

                        // Generate and save the graphic visual chart file context out to storage disk
                        exportPathToSVG(*finalBest, "best_path.svg");
            }
        }

        return (1.0 / finalBest->fitness);
    }
};

// Helper utilities to convert Enum values to printable strings
std::string toString(SelectionMethod sm) {
    switch(sm) {
        case SelectionMethod::TOURNAMENT:     return "Tournament";
        case SelectionMethod::ROULETTE_WHEEL: return "Roulette";
        case SelectionMethod::RANK_BASED:    return "Rank-Based";
        case SelectionMethod::SUS:           return "SUS";
    }
    return "Unknown";
}

std::string toString(CrossoverMethod cm) {
    switch(cm) {
        case CrossoverMethod::SINGLE_POINT: return "Single-Point";
        case CrossoverMethod::MULTI_POINT:  return "Multi-Point";
        case CrossoverMethod::UNIFORM:      return "Uniform";
    }
    return "Unknown";
}

std::string toString(MutationMethod mm) {
    switch(mm) {
        case MutationMethod::SINGLE_POINT:  return "Reset";
        case MutationMethod::SPATIAL_NUDGE:  return "Nudge";
        case MutationMethod::GAUSSIAN_CREEP: return "Gaussian";
    }
    return "Unknown";
}

// Struct representing a configuration case for the hyperparameter search
struct GAConfig {
    std::string name;
    int popSize;
    int generations;
    double mutationRate;
    double crossoverRate;
    SelectionMethod selection;
    CrossoverMethod crossover;
    MutationMethod mutation;
};

// Struct to keep track of benchmark results for the final leaderboard
struct BenchmarkResult {
    std::string name;
    std::string selection;
    std::string crossover;
    std::string mutation;
    int popSize;
    int generations;
    double mutationRate;
    double crossoverRate;
    double bestDistance;
    double totalTimeMs;
    double evalTimeMs;
    double evoTimeMs;
};

// ============================================================================
// MAIN RUN SETUP PROGRAM
// ============================================================================
int main() {
    // 1. Setup global geographic boundaries (Los Angeles)
    BoundingBox areaOfOps = { 34.0522, 34.1022, -118.2437, -118.1937 };

    // 2. Initialize the shared multi-threaded hardware pool
    ThreadPool sharedPool;

    // 3. Define 16 diverse test case profiles combining different techniques and parameters
    std::vector<GAConfig> sweepConfigs = {
        // Control & Baseline
        {"Baseline Control",     120, 100, 0.08, 0.85, SelectionMethod::TOURNAMENT,     CrossoverMethod::SINGLE_POINT, MutationMethod::SPATIAL_NUDGE},

        // Lightweight / Fast Profiles
        {"Lightweight (Fast)",    50,  50, 0.05, 0.70, SelectionMethod::TOURNAMENT,     CrossoverMethod::SINGLE_POINT, MutationMethod::SPATIAL_NUDGE},
        {"Micro-Sweep (SUS)",     40,  60, 0.08, 0.80, SelectionMethod::SUS,            CrossoverMethod::MULTI_POINT,  MutationMethod::GAUSSIAN_CREEP},

        // Intensive Searches (Large populations and generations)
        {"Intensive Search",     300, 200, 0.05, 0.80, SelectionMethod::TOURNAMENT,     CrossoverMethod::MULTI_POINT,  MutationMethod::SPATIAL_NUDGE},
        {"Intensive SUS",        250, 150, 0.08, 0.85, SelectionMethod::SUS,            CrossoverMethod::UNIFORM,      MutationMethod::GAUSSIAN_CREEP},

        // Exploration-Heavy (High mutation, uniform crossover, rank-based)
        {"Uniform Exploration",  150, 120, 0.15, 0.90, SelectionMethod::RANK_BASED,     CrossoverMethod::UNIFORM,      MutationMethod::GAUSSIAN_CREEP},
        {"SUS High-Mutation",    120, 100, 0.20, 0.75, SelectionMethod::SUS,            CrossoverMethod::UNIFORM,      MutationMethod::GAUSSIAN_CREEP},

        // Exploitation-Heavy (Low mutation, high crossover)
        {"Exploitation Focus",   150, 100, 0.02, 0.95, SelectionMethod::TOURNAMENT,     CrossoverMethod::MULTI_POINT,  MutationMethod::SPATIAL_NUDGE},

        // Roulette Wheel Variations
        {"Roulette & Uniform",   120, 100, 0.08, 0.85, SelectionMethod::ROULETTE_WHEEL, CrossoverMethod::UNIFORM,      MutationMethod::GAUSSIAN_CREEP},
        {"Roulette & Nudge",     120, 100, 0.06, 0.80, SelectionMethod::ROULETTE_WHEEL, CrossoverMethod::MULTI_POINT,  MutationMethod::SPATIAL_NUDGE},

        // Rank-Based Variations
        {"Rank & Gaussian",      120, 100, 0.08, 0.85, SelectionMethod::RANK_BASED,     CrossoverMethod::MULTI_POINT,  MutationMethod::GAUSSIAN_CREEP},
        {"Rank & Nudge",         120, 100, 0.06, 0.80, SelectionMethod::RANK_BASED,     CrossoverMethod::SINGLE_POINT, MutationMethod::SPATIAL_NUDGE},

        // Single Point Reset Mutation Variations
        {"Tournament & Reset",   120, 100, 0.10, 0.80, SelectionMethod::TOURNAMENT,     CrossoverMethod::SINGLE_POINT, MutationMethod::SINGLE_POINT},
        {"SUS & Reset",          120, 100, 0.10, 0.80, SelectionMethod::SUS,            CrossoverMethod::SINGLE_POINT, MutationMethod::SINGLE_POINT},
        {"Rank & Reset",         120, 100, 0.10, 0.80, SelectionMethod::RANK_BASED,     CrossoverMethod::SINGLE_POINT, MutationMethod::SINGLE_POINT},

        // High Crossover Exploration
        {"High-Cross SUS Nudge", 120, 100, 0.05, 0.95, SelectionMethod::SUS,            CrossoverMethod::SINGLE_POINT, MutationMethod::SPATIAL_NUDGE}
    };

    std::vector<BenchmarkResult> leaderboard;
    leaderboard.reserve(sweepConfigs.size());

    std::cout << "========================================================================\n";
    std::cout << "        STARTING HYPERPARAMETER OPTIMIZATION SWEEP (16 Test Profiles)    \n";
    std::cout << "========================================================================\n";
    std::cout << "Running evaluation sweep over C++ ThreadPool...\n\n";

    for (size_t idx = 0; idx < sweepConfigs.size(); ++idx) {
        const auto& c = sweepConfigs[idx];
        std::cout << " [*] Running Test " << std::setw(2) << (idx + 1) << " / " << sweepConfigs.size()
                  << ": " << std::left << std::setw(25) << c.name << " ... " << std::flush;

        ParallelComprehensiveGA gaEngine(areaOfOps, c.popSize, c.mutationRate, c.crossoverRate, c.generations, sharedPool);
        gaEngine.setOperators(c.selection, c.crossover, c.mutation);
        gaEngine.setVerbose(false); // Run silently to maintain a clean console

        auto start = std::chrono::high_resolution_clock::now();
        double resultDistance = gaEngine.runGA(6); // Optimize a path of 6 waypoints
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> elapsed = end - start;

        leaderboard.push_back({
            c.name,
            toString(c.selection),
            toString(c.crossover),
            toString(c.mutation),
            c.popSize,
            c.generations,
            c.mutationRate,
            c.crossoverRate,
            resultDistance,
            elapsed.count(),
            gaEngine.getFitnessEvalTime(),
            gaEngine.getEvolutionaryTime()
        });

        std::cout << "Done. (Distance: " << std::fixed << std::setprecision(3) << resultDistance
                  << " km, Time: " << std::setprecision(1) << elapsed.count() << " ms)\n";
    }

    // Sort leaderboard by best (shortest) distance ascending
    std::sort(leaderboard.begin(), leaderboard.end(), [](const BenchmarkResult& a, const BenchmarkResult& b) {
        return a.bestDistance < b.bestDistance;
        });

    // Print Ranked Performance Results Leaderboard
    std::cout << "\n\n";
    std::cout << "=========================================================================================================================================\n";
    std::cout << "                                                 HYPERPARAMETER OPTIMIZATION LEADERBOARD                                                 \n";
    std::cout << "=========================================================================================================================================\n";
    std::cout << " Rank | " << std::left << std::setw(23) << "Config Profile Name"
              << " | " << std::setw(11) << "Selection"
              << " | " << std::setw(12) << "Crossover"
              << " | " << std::setw(9) << "Mutation"
              << " | " << std::right << std::setw(4) << "Pop"
              << " | " << std::setw(4) << "Gen"
              << " | " << std::setw(5) << "MutR"
              << " | " << std::setw(5) << "XOver"
              << " | " << std::setw(15) << "Best Dist (km)"
              << " | " << std::setw(10) << "Total (ms)"
              << " |\n";
    std::cout << "------|-------------------------|-------------|--------------|-----------|------|------|-------|-------|----------------|------------|\n";

    for (size_t i = 0; i < leaderboard.size(); ++i) {
        const auto& r = leaderboard[i];
        std::cout << "  " << std::right << std::setw(2) << (i + 1) << "  | "
                  << std::left << std::setw(23) << r.name << " | "
                  << std::setw(11) << r.selection << " | "
                  << std::setw(12) << r.crossover << " | "
                  << std::setw(9) << r.mutation << " | "
                  << std::right << std::setw(4) << r.popSize << " | "
                  << std::setw(4) << r.generations << " | "
                  << std::setw(5) << std::fixed << std::setprecision(2) << r.mutationRate << " | "
                  << std::setw(5) << r.crossoverRate << " | "
                  << std::setw(15) << std::setprecision(4) << r.bestDistance << " | "
                  << std::setw(10) << std::setprecision(1) << r.totalTimeMs << " |\n";
    }
    std::cout << "=========================================================================================================================================\n";
    std::cout << "Note: Leaderboard is sorted primarily by Best Distance (ascending, shortest is best).\n\n";

    return 0;
}
