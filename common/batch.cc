#include "../common.h"
#include <chrono>
#include <thread>
#include <future>

/******************************************************************************
 * BATCH MULTITHREADED LOS
 *
 * Runs multiple LOS queries in parallel.
 *
 * Usage:
 *
 *     std::vector<LOSQuery> queries = {
 *         {34.01, 74.50, 50,  34.16, 74.71, 500},
 *         {34.02, 74.51, 30,  34.20, 74.80, 100},
 *         ...
 *     };
 *
 *     auto results = g.lineOfVisibilityBatch(queries);
 *     // results[i] corresponds to queries[i]
 *
 * Thread count defaults to hardware concurrency.
 * Each query is independent — no shared writes.
 *
 ******************************************************************************/

std::vector<std::vector<LOSResult>> Global::lineOfVisibilityBatch(
    const std::vector<LOSQuery>& queries,
    int numThreads)
{
     auto s = std::chrono::steady_clock::now();
    if (numThreads <= 0)
        numThreads = (int)std::thread::hardware_concurrency();
    if (numThreads < 1) numThreads = 1;

    int N = (int)queries.size();
    std::vector<std::vector<LOSResult>> results(N);

    // -------------------------------------------------------------------------
    // PARTITION QUERIES INTO CHUNKS, ONE CHUNK PER THREAD
    // -------------------------------------------------------------------------

    std::vector<std::future<void>> futures;
    futures.reserve(numThreads);

    int chunkSize = (N + numThreads - 1) / numThreads;

    for (int t = 0; t < numThreads; t++)
    {
        int start = t * chunkSize;
        int end   = std::min(start + chunkSize, N);
        if (start >= end) break;

        futures.push_back(std::async(std::launch::async,
            [this, &queries, &results, start, end]()
            {
                for (int i = start; i < end; i++)
                {
                    const auto& q = queries[i];
                    results[i] = this->lineOfVisibilityopt(
                        q.lat0, q.lon0, q.h0,
                        q.lat1, q.lon1, q.h1);
                }
            }));
    }
    auto e = std::chrono::steady_clock::now();
    auto total = std::chrono::duration_cast<std::chrono::nanoseconds>(e - s).count();
    std::cout << "done. Processing time: " << total <<std::endl;
    std::cout<< "threads: " << numThreads <<std::endl;
    // wait for all threads
    for (auto& f : futures)
        f.get();

    return results;
}
