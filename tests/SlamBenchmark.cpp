#include "SlamTestUtils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

struct Timing {
    double medianMilliseconds = 0.0;
    double p95Milliseconds = 0.0;
};

template <typename Function>
Timing measure(Function&& function) {
    constexpr int kWarmups = 10;
    constexpr int kIterations = 100;
    for (int iteration = 0; iteration < kWarmups; ++iteration) {
        function();
    }
    std::vector<double> samples;
    samples.reserve(kIterations);
    for (int iteration = 0; iteration < kIterations; ++iteration) {
        const auto start = Clock::now();
        function();
        const auto end = Clock::now();
        samples.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }
    std::sort(samples.begin(), samples.end());
    return Timing{samples[samples.size() / 2], samples[94]};
}
}

int main() {
    const MapData truthMap = makeSlamTestMap();
    const Pose2D truthPose{sf::Vector2f(-120.0f, -100.0f), 0.0f};
    LidarConfig lidarConfig;
    lidarConfig.beamCount = 91;
    lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
    lidarConfig.minRange = 1.0;
    lidarConfig.maxRange = 200.0;
    lidarConfig.rangeNoiseStdDev = 0.0;
    LidarSimulator lidar(lidarConfig);
    std::mt19937 rng(77);
    const LaserScan scan = lidar.simulate(truthPose, truthMap, rng);
    const SlamFrontendConfig config = makeSlamScenarioConfig();

    OccupancyGridMapper mapper;
    SlamOccupancyGrid integrationGrid(config.grid);
    double checksum = 0.0;
    const Timing integration = measure([&] {
        const OccupancyIntegrationResult result = mapper.integrate(
            integrationGrid, scan, Pose2D{}
        );
        checksum += result.freeCells + result.occupiedCells;
    });

    SlamOccupancyGrid matchGrid(config.grid);
    mapper.integrate(matchGrid, scan, Pose2D{});
    CorrelativeScanMatcher matcher(config.matcher);
    ScanMatchResult lastMatch;
    const Timing matching = measure([&] {
        lastMatch = matcher.match(
            matchGrid, scan, Pose2D{sf::Vector2f(10.0f, -10.0f), 0.04f}
        );
        checksum += lastMatch.score + lastMatch.correctionX;
    });

    SlamFrontend frontend(config);
    frontend.process(OdometryDelta{}, scan);
    SlamUpdateResult lastUpdate;
    const Timing frontendTiming = measure([&] {
        lastUpdate = frontend.process(OdometryDelta{}, scan);
        checksum += lastUpdate.match.score + frontend.getMap().getRevision();
    });

    std::cout << std::fixed << std::setprecision(4)
              << "SLAM benchmark (91-beam 360-degree, unoptimized MinGW build)\n"
              << "Occupancy integration median/p95 ms: "
              << integration.medianMilliseconds << " / " << integration.p95Milliseconds << "\n"
              << "Correlative match median/p95 ms: "
              << matching.medianMilliseconds << " / " << matching.p95Milliseconds << "\n"
              << "Frontend update median/p95 ms: "
              << frontendTiming.medianMilliseconds << " / " << frontendTiming.p95Milliseconds << "\n"
              << "Matcher candidates coarse/fine: " << lastMatch.coarseCandidates
              << " / " << lastMatch.fineCandidates << "\n"
              << "Selected/used beams: " << lastMatch.selectedBeams
              << " / " << lastMatch.usedBeams << "\n"
              << "Checksum: " << checksum << "\n";

    const bool valid = std::isfinite(checksum)
        && lastMatch.accepted && lastUpdate.match.accepted
        && lastMatch.coarseCandidates == 729
        && lastMatch.fineCandidates == 125
        && lastMatch.selectedBeams <= config.matcher.maximumBeams
        && integration.medianMilliseconds >= 0.0
        && matching.medianMilliseconds >= 0.0
        && frontendTiming.medianMilliseconds >= 0.0;
    if (!valid) {
        std::cerr << "SLAM benchmark invariant failure.\n";
        return 1;
    }
    return 0;
}
