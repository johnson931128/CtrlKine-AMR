#include "sensors/LidarSimulator.hpp"
#include "localization/MapLikelihoodField.hpp"
#include "localization/ParticleFilter.hpp"

#include <chrono>
#include <iostream>
#include <random>

namespace {
MapData benchmarkMap() {
    MapData map(50.0f);
    map.setWorldBoundary(sf::FloatRect(
        sf::Vector2f(0.0f, 0.0f), sf::Vector2f(1000.0f, 800.0f)
    ));
    for (int row = 1; row <= 13; ++row) map.addObstacle(GridCoord{6, row});
    for (int col = 8; col <= 17; ++col) map.addObstacle(GridCoord{col, 10});
    for (const GridCoord cell : {GridCoord{2, 2}, GridCoord{4, 5}, GridCoord{12, 3}, GridCoord{17, 6}}) {
        map.addObstacle(cell);
    }
    return map;
}

template <typename Function>
double milliseconds(Function&& function) {
    const auto start = std::chrono::steady_clock::now();
    function();
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start
    ).count();
}
}

int main() {
    const MapData map = benchmarkMap();
    MapLikelihoodField field;
    const double fieldMs = milliseconds([&] {
        field.rebuild(map, 150.0);
    });
    LidarConfig lidarConfig;
    lidarConfig.beamCount = 91;
    lidarConfig.fieldOfView = 1.5 * kLocalizationPi;
    lidarConfig.minRange = 1.0;
    lidarConfig.maxRange = 900.0;
    lidarConfig.rangeNoiseStdDev = 0.5;
    LidarSimulator lidar(lidarConfig);
    std::mt19937 scanEngine(1000);
    const Pose2D truth{sf::Vector2f(175.0f, 215.0f), 0.37f};
    LaserScan scan;
    const double lidarMs = milliseconds([&] {
        scan = lidar.simulate(truth, map, scanEngine);
    });
    std::cout << "distanceFieldRebuildMs=" << fieldMs
              << " lidar91Ms=" << lidarMs << "\n";

    for (const std::size_t particles : {300u, 1000u, 2000u, 5000u}) {
        for (const std::size_t beams : {31u, 60u, 91u}) {
            if (particles == 5000 && beams != 31) continue;
            AmclConfig config;
            config.minParticles = particles;
            config.maxParticles = particles;
            config.initialParticleCount = particles;
            config.maxBeams = beams;
            config.doBeamSkip = true;
            ParticleFilter filter(config);
            std::mt19937 engine(static_cast<std::uint32_t>(particles + beams));
            filter.initializeGlobal(map, engine);
            const double motionMs = milliseconds([&] {
                filter.motionUpdate(OdometryDelta{0.0, 10.0, 0.0, true}, engine);
            });
            const double sensorMs = milliseconds([&] {
                filter.sensorUpdate(scan, field);
            });
            const double clusterMs = milliseconds([&] {
                (void)ParticleFilter::clusterParticles(filter.getParticles(), config);
            });
            const double resampleMs = milliseconds([&] {
                filter.adaptiveResample(map, 0.0, engine);
            });
            std::cout << "particles=" << particles << " beams=" << beams
                      << " motionMs=" << motionMs
                      << " sensorMs=" << sensorMs
                      << " clusterMs=" << clusterMs
                      << " kldResampleMs=" << resampleMs << "\n";
        }
    }
    return 0;
}
