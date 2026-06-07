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
        std::cout<<"execution over";
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
     * Thread-Safe Evaluation: Evaluates individual total path distances.
     * Designed with a static context to protect against thread race conditions.
     */
    static void evaluateIndividual(GAIndividual& ind) {
        double totalDistance = 0.0;
        // Step through the entire path and accumulate individual leg distances
        for (size_t i = 0; i < ind.chromosome.size() - 1; ++i) {
            totalDistance += ind.chromosome[i].distanceTo(ind.chromosome[i + 1]);
        }
        // Fitness mapping formula: Inverse of total distance. Avoid division by zero.
        ind.fitness = (totalDistance == 0) ? 99999.0 : (1.0 / totalDistance);
    }

    /**
     * Parallelization Barrier: Spreads the work of evaluating the population's
     * fitness across all available threads.
     */
    void parallelEvaluatePopulation(std::vector<GAIndividual>& population) {
        std::vector<std::future<void>> futures;
        futures.reserve(population.size());

        // Enqueue each individual's evaluation task into the ThreadPool
        for (auto& ind : population) {
            futures.push_back(pool.enqueue(ParallelComprehensiveGA::evaluateIndividual, std::ref(ind)));
        }

        // Synchronous Blocking Point: Wait until all threads finish calculations
        for (auto& f : futures) f.wait();
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
     * This prevents highly fit individuals from dominating the selection pool too early.
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
     * pointers to choose parents. This gives weak solutions a fair chance and eliminates
     * the selection bias found in standard roulette wheels.
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
     * Uniform Crossover: Iterates through each gene and randomly flipp flips them between parents.
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
     * This creates a natural blend of small, local adjustments and occasional larger jumps.
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
    // ============================================================================
    // RUN LOOPS PIPELINE CONTEXT ENGINE
    // ============================================================================
    // CHANGE THIS: change from 'void' to 'double'
    double runGA(int pathLength) {
        std::vector<GAIndividual> population(static_cast<size_t>(popSize));
        std::uniform_int_distribution<int> distX(0, maxGridX), distY(0, maxGridY);

        for (int i = 0; i < popSize; ++i) {
            for (int j = 0; j < pathLength; ++j) {
                population[static_cast<size_t>(i)].chromosome.push_back(createGridPoint(distX(rng), distY(rng)));
            }
        }
        parallelEvaluatePopulation(population);

        std::normal_distribution<double> gaussDistribution(0.0, 2.0);

        for (int gen = 1; gen <= generations; ++gen) {
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

            parallelEvaluatePopulation(nextGen);
            population = std::move(nextGen);

            // Optional: Comment these lines out if you want to keep the terminal clean
            // during your 36-combination benchmark run!
            if (gen % 25 == 0 || gen == 1) {
                auto currentBest = std::max_element(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness < b.fitness; });
                std::cout << "    Gen " << gen << " | Shortest Path Matrix Result: " << (1.0 / currentBest->fitness) << " km\n";
            }
        }

        auto finalBest = std::max_element(population.begin(), population.end(), [](const GAIndividual& a, const GAIndividual& b) { return a.fitness < b.fitness; });
        std::cout << ">> Optimization Strategy Yielded Best Distance: " << (1.0 / finalBest->fitness) << " km\n";

        // ADD THIS LINE: Return the final optimized distance back to main()
        return (1.0 / finalBest->fitness);
    }
};


// Helper utilities to convert Enum values to printable strings
std::string toString(SelectionMethod sm) {
    switch(sm) {
        case SelectionMethod::TOURNAMENT:    return "Tournament";
        case SelectionMethod::ROULETTE_WHEEL: return "Roulette Wheel";
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
        case MutationMethod::SINGLE_POINT:  return "Single-Point";
        case MutationMethod::SPATIAL_NUDGE:  return "Spatial Nudge";
        case MutationMethod::GAUSSIAN_CREEP: return "Gaussian Creep";
    }
    return "Unknown";
}

// Struct to keep track of our benchmark results for the final leaderboard
struct BenchmarkResult {
    std::string selection;
    std::string crossover;
    std::string mutation;
    double bestDistance;
};

// ============================================================================
// MAIN RUN SETUP PROGRAM
// ============================================================================
int main() {
    // 1. Setup global geographic boundaries (Los Angeles)
    BoundingBox areaOfOps = { 34.0522, 34.1022, -118.2437, -118.1937 };

    // 2. Initialize the shared multi-threaded hardware pool
    ThreadPool sharedPool;

    // 3. Define all array combinations to loop through
    std::vector<SelectionMethod> selections = {
        SelectionMethod::TOURNAMENT,
        SelectionMethod::ROULETTE_WHEEL,
        SelectionMethod::RANK_BASED,
        SelectionMethod::SUS
    };

    std::vector<CrossoverMethod> crossovers = {
        CrossoverMethod::SINGLE_POINT,
        CrossoverMethod::MULTI_POINT,
        CrossoverMethod::UNIFORM
    };

    std::vector<MutationMethod> mutations = {
        MutationMethod::SINGLE_POINT,
        MutationMethod::SPATIAL_NUDGE,
        MutationMethod::GAUSSIAN_CREEP
    };

    std::vector<BenchmarkResult> leaderboard;
    int runCounter = 1;

    std::cout << "====================================================================\n";
    std::cout << "STARTING AUTOMATED COMBINATORIAL BENCHMARK (36 Unique Strategies)\n";
    std::cout << "====================================================================\n\n";

    // 4. Nested Loop Matrix: Dynamic Permutation Brute Force
    for (auto sel : selections) {
        for (auto cross : crossovers) {
            for (auto mut : mutations) {

                std::cout << "[" << runCounter << "/36] Testing Matrix: "
                          << toString(sel) << " + " << toString(cross) << " + " << toString(mut) << "\n";

                // Re-instantiate engine to clear out old genetic memory pools completely
                ParallelComprehensiveGA gaEngine(areaOfOps, 120, 0.08, 0.85, 100, sharedPool);
                gaEngine.setOperators(sel, cross, mut);

                // Execute optimization for 6 waypoints
                double resultDistance = gaEngine.runGA(6);

                // Record analytics
                leaderboard.push_back({toString(sel), toString(cross), toString(mut), resultDistance});
                runCounter++;
            }
        }
    }

    // 5. Sort the complete leaderboard by Best Distance Ascending (Shortest path wins!)
    std::sort(leaderboard.begin(), leaderboard.end(), [](const BenchmarkResult& a, const BenchmarkResult& b) {
        return a.bestDistance < b.bestDistance;
    });

    // 6. Print the Final Ranked Performance Results
    std::cout << "\n====================================================================\n";
    std::cout << "                  FINAL BENCHMARK RANKING LEADERBOARD               \n";
    std::cout << "====================================================================\n";
    std::cout << std::left << std::setw(6)  << "Rank"
              << std::setw(16) << "Selection"
              << std::setw(15) << "Crossover"
              << std::setw(16) << "Mutation"
              << "Best Distance\n";
    std::cout << "--------------------------------------------------------------------\n";

    for (size_t i = 0; i < leaderboard.size(); ++i) {
        std::cout << std::left << std::setw(6)  << ("#" + std::to_string(i + 1))
                  << std::setw(16) << leaderboard[i].selection
                  << std::setw(15) << leaderboard[i].crossover
                  << std::setw(16) << leaderboard[i].mutation
                  << std::fixed << std::setprecision(5) << leaderboard[i].bestDistance << " km\n";
    }
    std::cout << "====================================================================\n";

    return 0;
}
