/**
 * @file ParallelRadarPSO.cpp
 * @brief Multithreaded Particle Swarm Optimization (PSO) Engine for Optimal Radar Placement.
 * * DESIGN PRINCIPLE:
 * This system treats a swarm of "Particles" as competing configuration setups. Each particle
 * holds a collection of coordinates representing 'N' distinct radar units. The fitness function
 * optimizes the geographic "spread" (average pairwise Haversine distance) of these radars
 * within a defined latitude/longitude bounding box.
 * * CONCURRENCY MODEL:
 * Particle fitness evaluations are independent tasks. A worker ThreadPool processes these
 * evaluations in parallel every iteration, maximizing multi-core CPU utilization.
 * * BENCHMARKING ENGINE:
 * Uses high-resolution instrumentation tracks to profile allocation bottlenecks, kinematic
 * overhead, and thread scheduling latencies across diverse optimization runs.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <iomanip>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <string>

// ============================================================================
// CONFIGURATION ENUMS & MATHEMATICAL CONSTANTS
// ============================================================================

enum class PSOTopology {
    GLOBAL_STAR, ///< Every particle pulls toward the absolute best particle in the entire swarm. Fast convergence.
    LOCAL_RING   ///< Particles only communicate with their immediate neighbors. Slower, but avoids premature local minima traps.
};

enum class InertiaStrategy {
    CONSTANT,      ///< Static momentum coefficient throughout the optimization lifecycle.
    LINEAR_DECAY,  ///< Momentum drops over time; prioritizes exploration early, exploitation late.
    RANDOM_JITTER  ///< Introduces dynamic stochastic variance to break out of flat optimization plateaus.
};

enum class BoundaryHandler {
    ABSORB_CLAMP,   ///< Sticks the particle directly onto the boundary edge and stops momentum.
    REFLECT_BOUNCE  ///< Bounces the particle back into valid space by flipping its directional velocity vector.
};

constexpr double EARTH_RADIUS_KM = 6371.0;
constexpr double PI = 3.14159265358979323846;

struct BoundingBox {
    double minLat, maxLat, minLon, maxLon;
};

struct GridPoint {
    int gridX, gridY;
    double lat, lon;

    double distanceTo(const GridPoint& other) const {
        double lat1Rad = lat * PI / 180.0;
        double lat2Rad = other.lat * PI / 180.0;
        double deltaLat = (other.lat - lat) * PI / 180.0;
        double deltaLon = (other.lon - lon) * PI / 180.0;

        double a = std::sin(deltaLat / 2.0) * std::sin(deltaLat / 2.0) +
                   std::cos(lat1Rad) * std::cos(lat2Rad) * std::sin(deltaLon / 2.0) * std::sin(deltaLon / 2.0);

        return EARTH_RADIUS_KM * (2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a)));
    }
};

struct ConfigurableParticle {
    std::vector<GridPoint> positions;
    std::vector<int> velX;
    std::vector<int> velY;
    std::vector<GridPoint> pBestPositions;
    double pBestFitness;
    double currentFitness;
};

// ============================================================================
// PERFORMANCE DATA MATRIX TRACKER
// ============================================================================

/**
 * @brief Structure holding performance profiling dimensions returned to the leaderboard aggregator.
 */
struct PSOBenchmarkResult {
    std::string profileName;
    std::string topology;
    std::string inertia;
    std::string boundary;
    int swarmSize;
    int iterations;
    double bestSpreadKm;
    double initTimeMs;
    double kinematicTimeMs;
    double fitnessEvalTimeMs;
    double totalTimeMs;
};

// ============================================================================
// WORKER THREAD POOL ENGINE (Task Parallelism)
// ============================================================================
class ThreadPool {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable cv;
    bool stop;

public:
    ThreadPool() : stop(false) {
        size_t threads = std::thread::hardware_concurrency();
        if (threads == 0) threads = 4;

        for (size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this]() {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queueMutex);
                        this->cv.wait(lock, [this]() { return this->stop || !this->tasks.empty(); });

                        if (this->stop && this->tasks.empty()) return;

                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) -> std::future<typename std::invoke_result<F, Args...>::type> {
        using return_type = typename std::invoke_result<F, Args...>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (stop) throw std::runtime_error("Enqueue on stopped ThreadPool Context");
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
            if (worker.joinable()) worker.join();
        }
    }
};

// ============================================================================
// PARALLEL CONFIGURABLE PSO ENGINE WITH INSTRUMENTATION
// ============================================================================
class ParallelAdvancedPSO {
private:
    BoundingBox bbox;
    double latStep, lonStep;
    int maxGridX, maxGridY;
    int swarmSize, iterations, numRadars;
    ThreadPool& pool;
    std::mt19937 rng;

    PSOTopology currentTopology     = PSOTopology::GLOBAL_STAR;
    InertiaStrategy currentInertia  = InertiaStrategy::CONSTANT;
    BoundaryHandler currentBoundary = BoundaryHandler::ABSORB_CLAMP;
    std::string profileName         = "Default Profile";

    std::vector<GridPoint> gBestPositions;
    double gBestFitness;
    std::mutex gBestMutex;

    double totalFitnessEvalTimeMs = 0.0;
    double totalKinematicUpdateTimeMs = 0.0;

    void calculateGridSteps() {
        double centerLat = (bbox.minLat + bbox.maxLat) / 2.0;
        latStep = 100.0 / 111000.0;
        lonStep = 100.0 / (111000.0 * std::cos(centerLat * PI / 180.0));

        maxGridX = static_cast<int>((bbox.maxLon - bbox.minLon) / lonStep);
        maxGridY = static_cast<int>((bbox.maxLat - bbox.minLat) / latStep);
    }

    GridPoint createGridPoint(int x, int y) {
        return {
            x, y,
            bbox.minLat + (y * latStep) + (latStep / 2.0),
            bbox.minLon + (x * lonStep) + (lonStep / 2.0)
        };
    }

    std::string getTopologyString() const {
        return (currentTopology == PSOTopology::GLOBAL_STAR) ? "Global" : "Ring";
    }

    std::string getInertiaString() const {
        if (currentInertia == InertiaStrategy::CONSTANT) return "Constant";
        if (currentInertia == InertiaStrategy::LINEAR_DECAY) return "Decay";
        return "Jitter";
    }

    std::string getBoundaryString() const {
        return (currentBoundary == BoundaryHandler::ABSORB_CLAMP) ? "Clamp" : "Bounce";
    }

public:
    ParallelAdvancedPSO(BoundingBox box, int radars, ThreadPool& p)
        : bbox(box), numRadars(radars), pool(p) {
        rng.seed(std::chrono::system_clock::now().time_since_epoch().count());
        gBestFitness = -1.0;
        calculateGridSteps();
    }

    void setRunConfig(const std::string& name, int size, int iters, PSOTopology topo, InertiaStrategy inertia, BoundaryHandler boundary) {
        profileName = name;
        swarmSize = size;
        iterations = iters;
        currentTopology = topo;
        currentInertia = inertia;
        currentBoundary = boundary;
    }

    static void evaluateParticleFitness(ConfigurableParticle& particle) {
        double totalSpreadDistance = 0.0;
        int pairs = 0;

        for (size_t i = 0; i < particle.positions.size(); ++i) {
            for (size_t j = i + 1; j < particle.positions.size(); ++j) {
                totalSpreadDistance += particle.positions[i].distanceTo(particle.positions[j]);
                pairs++;
            }
        }
        particle.currentFitness = (pairs == 0) ? 0.0 : (totalSpreadDistance / pairs);
    }

    void parallelEvaluateSwarm(std::vector<ConfigurableParticle>& swarm) {
        auto startTime = std::chrono::high_resolution_clock::now();

        std::vector<std::future<void>> futures;
        futures.reserve(swarm.size());

        for (auto& particle : swarm) {
            futures.push_back(pool.enqueue(ParallelAdvancedPSO::evaluateParticleFitness, std::ref(particle)));
        }
        for (auto& f : futures) f.wait();

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = endTime - startTime;
        totalFitnessEvalTimeMs += duration.count();
    }

    /**
     * @brief Executes the optimization sequence and wraps benchmarking indices for structural return arrays.
     */
    PSOBenchmarkResult runPSO() {
        gBestFitness = -1.0;
        totalFitnessEvalTimeMs = 0.0;
        totalKinematicUpdateTimeMs = 0.0;

        auto totalExecutionStart = std::chrono::high_resolution_clock::now();

        // --- Phase 1: Initialization ---
        auto initStart = std::chrono::high_resolution_clock::now();

        std::vector<ConfigurableParticle> swarm(swarmSize);
        std::uniform_int_distribution<int> distX(0, maxGridX), distY(0, maxGridY);
        std::uniform_int_distribution<int> vDistX(-3, 3), vDistY(-3, 3);

        for (int i = 0; i < swarmSize; ++i) {
            swarm[i].positions.resize(numRadars);
            swarm[i].velX.resize(numRadars);
            swarm[i].velY.resize(numRadars);
            for (int r = 0; r < numRadars; ++r) {
                swarm[i].positions[r] = createGridPoint(distX(rng), distY(rng));
                swarm[i].velX[r] = vDistX(rng);
                swarm[i].velY[r] = vDistY(rng);
            }
        }

        parallelEvaluateSwarm(swarm);

        for (int i = 0; i < swarmSize; ++i) {
            swarm[i].pBestPositions = swarm[i].positions;
            swarm[i].pBestFitness = swarm[i].currentFitness;
            if (swarm[i].pBestFitness > gBestFitness) {
                gBestFitness = swarm[i].pBestFitness;
                gBestPositions = swarm[i].pBestPositions;
            }
        }

        auto initEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> initDuration = initEnd - initStart;

        // --- Phase 2 & 3: Main Iterative Life Cycle ---
        std::uniform_real_distribution<double> dist01(0.0, 1.0);
        constexpr double c1 = 1.494;
        constexpr double c2 = 1.494;

        for (int iter = 1; iter <= iterations; ++iter) {
            auto kinematicStart = std::chrono::high_resolution_clock::now();

            double w = 0.729;
            if (currentInertia == InertiaStrategy::LINEAR_DECAY) {
                w = 0.9 - (0.5 * ((double)iter / iterations));
            } else if (currentInertia == InertiaStrategy::RANDOM_JITTER) {
                std::uniform_real_distribution<double> jitter(0.4, 0.9);
                w = jitter(rng);
            }

            for (int i = 0; i < swarmSize; ++i) {
                std::vector<GridPoint> targetPositions = gBestPositions;

                if (currentTopology == PSOTopology::LOCAL_RING) {
                    int leftNeighbor = (i - 1 + swarmSize) % swarmSize;
                    int rightNeighbor = (i + 1) % swarmSize;
                    double maxLocalBest = swarm[i].pBestFitness;
                    targetPositions = swarm[i].pBestPositions;

                    if (swarm[leftNeighbor].pBestFitness > maxLocalBest) {
                        maxLocalBest = swarm[leftNeighbor].pBestFitness;
                        targetPositions = swarm[leftNeighbor].pBestPositions;
                    }
                    if (swarm[rightNeighbor].pBestFitness > maxLocalBest) {
                        targetPositions = swarm[rightNeighbor].pBestPositions;
                    }
                }

                for (int r = 0; r < numRadars; ++r) {
                    double r1 = dist01(rng);
                    double r2 = dist01(rng);

                    int nvX = (int)std::round(w * swarm[i].velX[r] +
                              c1 * r1 * (swarm[i].pBestPositions[r].gridX - swarm[i].positions[r].gridX) +
                              c2 * r2 * (targetPositions[r].gridX - swarm[i].positions[r].gridX));

                    int nvY = (int)std::round(w * swarm[i].velY[r] +
                              c1 * r1 * (swarm[i].pBestPositions[r].gridY - swarm[i].positions[r].gridY) +
                              c2 * r2 * (targetPositions[r].gridY - swarm[i].positions[r].gridY));

                    swarm[i].velX[r] = std::clamp(nvX, -8, 8);
                    swarm[i].velY[r] = std::clamp(nvY, -8, 8);

                    int nextX = swarm[i].positions[r].gridX + swarm[i].velX[r];
                    int nextY = swarm[i].positions[r].gridY + swarm[i].velY[r];

                    if (currentBoundary == BoundaryHandler::ABSORB_CLAMP) {
                        nextX = std::clamp(nextX, 0, maxGridX);
                        nextY = std::clamp(nextY, 0, maxGridY);
                    } else if (currentBoundary == BoundaryHandler::REFLECT_BOUNCE) {
                        if (nextX < 0)         { nextX = -nextX; swarm[i].velX[r] *= -1; }
                        if (nextX > maxGridX)  { nextX = maxGridX - (nextX - maxGridX); swarm[i].velX[r] *= -1; }
                        if (nextY < 0)         { nextY = -nextY; swarm[i].velY[r] *= -1; }
                        if (nextY > maxGridY)  { nextY = maxGridY - (nextY - maxGridY); swarm[i].velY[r] *= -1; }

                        nextX = std::clamp(nextX, 0, maxGridX);
                        nextY = std::clamp(nextY, 0, maxGridY);
                    }

                    swarm[i].positions[r] = createGridPoint(nextX, nextY);
                }
            }
            auto kinematicEnd = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> kinematicDuration = kinematicEnd - kinematicStart;
            totalKinematicUpdateTimeMs += kinematicDuration.count();

            parallelEvaluateSwarm(swarm);

            for (int i = 0; i < swarmSize; ++i) {
                if (swarm[i].currentFitness > swarm[i].pBestFitness) {
                    swarm[i].pBestFitness = swarm[i].currentFitness;
                    swarm[i].pBestPositions = swarm[i].positions;
                }

                if (swarm[i].currentFitness > gBestFitness) {
                    std::lock_guard<std::mutex> lock(gBestMutex);
                    if (swarm[i].currentFitness > gBestFitness) {
                        gBestFitness = swarm[i].currentFitness;
                        gBestPositions = swarm[i].positions;
                    }
                }
            }
        }

        auto totalExecutionEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> totalExecutionDuration = totalExecutionEnd - totalExecutionStart;

        // Build and return data metrics structure tracking state records
        return {
            profileName, getTopologyString(), getInertiaString(), getBoundaryString(),
            swarmSize, iterations, gBestFitness, initDuration.count(),
            totalKinematicUpdateTimeMs, totalFitnessEvalTimeMs, totalExecutionDuration.count()
        };
    }
};

// ============================================================================
// HELPER SETUP DATA DEF STRUCT
// ============================================================================
struct TestConfig {
    std::string name;
    int pop;
    int gen;
    PSOTopology topo;
    InertiaStrategy inertia;
    BoundaryHandler boundary;
};

// ============================================================================
// ENTRY TESTING DRIVER
// ============================================================================
int main() {
    BoundingBox areaBounds = { 39.0000, 39.3000, -105.3000, -105.0000 };
    ThreadPool pool;

    ParallelAdvancedPSO psoEngine(areaBounds, 4, pool);
    std::vector<PSOBenchmarkResult> leaderboard;

    // Hyperparameter Sweep Setup Formats
    std::vector<TestConfig> sweepConfigs = {
        { "Standard Baseline",   120, 100, PSOTopology::GLOBAL_STAR, InertiaStrategy::CONSTANT,      BoundaryHandler::ABSORB_CLAMP },
        { "Exploratory Ring",    150, 120, PSOTopology::LOCAL_RING,   InertiaStrategy::LINEAR_DECAY,  BoundaryHandler::REFLECT_BOUNCE },
        { "High-Jitter Flight",  150, 120, PSOTopology::LOCAL_RING,   InertiaStrategy::RANDOM_JITTER, BoundaryHandler::ABSORB_CLAMP },
        { "Intensive Velocity",  250, 200, PSOTopology::GLOBAL_STAR, InertiaStrategy::LINEAR_DECAY,  BoundaryHandler::REFLECT_BOUNCE },
        { "Micro-Swarm Quick",    50,  60, PSOTopology::GLOBAL_STAR, InertiaStrategy::CONSTANT,      BoundaryHandler::ABSORB_CLAMP },
        { "Chaotic Bound Star",  120, 100, PSOTopology::GLOBAL_STAR, InertiaStrategy::RANDOM_JITTER, BoundaryHandler::REFLECT_BOUNCE },
        { "Heavy Decay Ring",    200, 150, PSOTopology::LOCAL_RING,   InertiaStrategy::LINEAR_DECAY,  BoundaryHandler::ABSORB_CLAMP }
    };

    std::cout << "===================================================================================================\n";
    std::cout << " COMMENCING PARALLEL ADVANCED RADAR PSO HYPERPARAMETER SWEEP PERFORMANCE BENCHMARKS\n";
    std::cout << "===================================================================================================\n";
    std::cout << ">> Running " << sweepConfigs.size() << " automated optimization variants against thread pool tracks...\n\n";

    for (const auto& config : sweepConfigs) {
        std::cout << " -> Processing Execution Strategy profile: [" << config.name << "]...\n";
        psoEngine.setRunConfig(config.name, config.pop, config.gen, config.topo, config.inertia, config.boundary);
        PSOBenchmarkResult result = psoEngine.runPSO();
        leaderboard.push_back(result);
    }

    // Sort leaderboard primarily by Best Spread Distance (descending order - larger spatial coverage areas win)
    std::sort(leaderboard.begin(), leaderboard.end(), [](const PSOBenchmarkResult& a, const PSOBenchmarkResult& b) {
        return a.bestSpreadKm > b.bestSpreadKm;
    });

    // ===================================================================================================
    // GLOBAL CONSOLIDATED METRICS LEADERBOARD MATRIX OUTPUT
    // ===================================================================================================
    std::cout << "\n=========================================================================================================================================\n";
    std::cout << "                                              FINAL PSO RADAR PLACEMENT PERFORMANCE LEADERBOARD\n";
    std::cout << "=========================================================================================================================================\n";
    std::cout << " Rank | Config Profile Name     | Topo   | Inertia  | Bound  | Pop  | Iters | Init (ms) | Kinemat (ms) | Eval (ms) | Best Spread | Total (ms) |\n";
    std::cout << "------|-------------------------|--------|----------|--------|------|-------|-----------|--------------|-----------|-------------|------------|\n";

    for (size_t i = 0; i < leaderboard.size(); ++i) {
        const auto& r = leaderboard[i];
        std::cout << "  " << std::right << std::setw(2) << (i + 1) << "  | "
                  << std::left << std::setw(23) << r.profileName << " | "
                  << std::setw(6) << r.topology << " | "
                  << std::setw(8) << r.inertia << " | "
                  << std::setw(6) << r.boundary << " | "
                  << std::right << std::setw(4) << r.swarmSize << " | "
                  << std::setw(5) << r.iterations << " | "
                  << std::setw(9) << std::fixed << std::setprecision(1) << r.initTimeMs << " | "
                  << std::setw(12) << r.kinematicTimeMs << " | "
                  << std::setw(9) << r.fitnessEvalTimeMs << " | "
                  << std::setw(9) << std::setprecision(4) << r.bestSpreadKm << " km | "
                  << std::setw(10) << std::setprecision(1) << r.totalTimeMs << " |\n";
    }
    std::cout << "=========================================================================================================================================\n";
    std::cout << "Note: Summary leaderboard matrix sorted by Best Spatial Distribution Spread (descending, maximal great-circle coverage is best).\n\n";

    return 0;
}
