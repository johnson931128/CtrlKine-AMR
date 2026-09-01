#include "slam/OccupancyGridMapper.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "TestSupport.hpp"

namespace {
SlamOccupancyGridConfig testConfig() {
    SlamOccupancyGridConfig config;
    config.resolution = 10.0;
    config.originX = -50.0;
    config.originY = -50.0;
    config.width = 10;
    config.height = 10;
    config.occupiedLogOddsIncrement = 1.0;
    config.freeLogOddsIncrement = -0.5;
    config.minimumLogOdds = -2.0;
    config.maximumLogOdds = 2.0;
    config.freeLogOddsThreshold = -0.25;
    config.occupiedLogOddsThreshold = 0.25;
    return config;
}

LaserScan singleBeam(float range, float maxRange = 40.0f) {
    LaserScan scan;
    scan.ranges = {range};
    scan.angleMin = 0.0f;
    scan.angleIncrement = 0.0f;
    scan.minRange = 1.0f;
    scan.maxRange = maxRange;
    return scan;
}

bool sameCell(const SlamGridCoord& cell, int col, int row) {
    return cell.col == col && cell.row == row;
}
}

int main() {
    TestSuite suite;

    runTest(suite, "SLAM-MAP-001", "invalid occupancy-grid configuration is rejected", [] {
        SlamOccupancyGridConfig config = testConfig();
        config.resolution = 0.0;
        try {
            SlamOccupancyGrid grid(config);
            (void)grid;
            return false;
        } catch (const std::invalid_argument&) {
            return true;
        }
    });

    runTest(suite, "SLAM-MAP-001", "all occupancy-grid configuration families are validated", [] {
        std::vector<SlamOccupancyGridConfig> invalid;
        SlamOccupancyGridConfig width = testConfig(); width.width = 0; invalid.push_back(width);
        SlamOccupancyGridConfig hit = testConfig(); hit.occupiedLogOddsIncrement = 0.0; invalid.push_back(hit);
        SlamOccupancyGridConfig miss = testConfig(); miss.freeLogOddsIncrement = 0.1; invalid.push_back(miss);
        SlamOccupancyGridConfig clamps = testConfig(); clamps.minimumLogOdds = 3.0; invalid.push_back(clamps);
        SlamOccupancyGridConfig thresholds = testConfig(); thresholds.freeLogOddsThreshold = 1.0; invalid.push_back(thresholds);
        for (const SlamOccupancyGridConfig& config : invalid) {
            try {
                SlamOccupancyGrid grid(config);
                (void)grid;
                return false;
            } catch (const std::invalid_argument&) {
            }
        }
        return true;
    });

    runTest(suite, "SLAM-MAP-002", "new cells are unknown and observed state is explicit", [] {
        SlamOccupancyGrid grid(testConfig());
        const SlamGridCoord cell{5, 5};
        return grid.getState(cell) == OccupancyState::Unknown
            && !grid.wasObserved(cell)
            && grid.getStatistics().unknownCells == 100;
    });

    runTest(suite, "SLAM-MAP-003", "world-to-cell uses origin-relative floor and half-open bounds", [] {
        SlamOccupancyGrid grid(testConfig());
        return sameCell(grid.worldToCell(sf::Vector2f(-50.0f, -50.0f)), 0, 0)
            && sameCell(grid.worldToCell(sf::Vector2f(-0.1f, -0.1f)), 4, 4)
            && sameCell(grid.worldToCell(sf::Vector2f(0.0f, 0.0f)), 5, 5)
            && grid.contains(sf::Vector2f(-50.0f, -50.0f))
            && !grid.contains(sf::Vector2f(50.0f, 0.0f));
    });

    runTest(suite, "SLAM-MAP-002/005", "evidence classifies, clamps, and commits one revision", [] {
        SlamOccupancyGrid grid(testConfig());
        const SlamGridCoord freeCell{4, 5};
        const SlamGridCoord occupiedCell{6, 5};
        if (!grid.applyEvidence({{freeCell, false}, {occupiedCell, true}})
            || grid.getRevision() != 1
            || grid.getState(freeCell) != OccupancyState::Free
            || grid.getState(occupiedCell) != OccupancyState::Occupied) {
            return false;
        }
        for (int repeat = 0; repeat < 10; ++repeat) {
            grid.applyEvidence({{freeCell, false}, {occupiedCell, true}});
        }
        return std::abs(grid.getLogOdds(freeCell) + 2.0) < 1e-6
            && std::abs(grid.getLogOdds(occupiedCell) - 2.0) < 1e-6;
    });

    runTest(suite, "SLAM-MAP-002", "observed neutral evidence remains explicitly Unknown", [] {
        SlamOccupancyGrid grid(testConfig());
        const SlamGridCoord cell{5, 5};
        grid.applyEvidence({{cell, true}});
        grid.applyEvidence({{cell, false}});
        grid.applyEvidence({{cell, false}});
        return grid.wasObserved(cell) && std::abs(grid.getLogOdds(cell)) < 1e-6
            && grid.getState(cell) == OccupancyState::Unknown;
    });

    runTest(suite, "SLAM-MAP-005", "ray traversal is deterministic for cardinal, diagonal, reverse, and clipped rays", [] {
        SlamOccupancyGrid grid(testConfig());
        const auto cardinal = OccupancyGridMapper::traceRay(
            grid, sf::Vector2f(0.0f, 0.0f), sf::Vector2f(29.0f, 0.0f)
        );
        const auto diagonal = OccupancyGridMapper::traceRay(
            grid, sf::Vector2f(0.0f, 0.0f), sf::Vector2f(29.0f, 29.0f)
        );
        const auto reverse = OccupancyGridMapper::traceRay(
            grid, sf::Vector2f(29.0f, 0.0f), sf::Vector2f(0.0f, 0.0f)
        );
        const auto clipped = OccupancyGridMapper::traceRay(
            grid, sf::Vector2f(-100.0f, 0.0f), sf::Vector2f(100.0f, 0.0f)
        );
        return cardinal.size() == 3 && sameCell(cardinal.front(), 5, 5)
            && sameCell(cardinal.back(), 7, 5)
            && diagonal.size() == 3 && sameCell(diagonal.back(), 7, 7)
            && reverse.size() == 3 && sameCell(reverse.front(), 7, 5)
            && sameCell(reverse.back(), 5, 5)
            && clipped.size() == 10 && sameCell(clipped.front(), 0, 5)
            && sameCell(clipped.back(), 9, 5);
    });

    runTest(suite, "SLAM-MAP-004", "physical hit frees traversal and occupies the surface-side cell", [] {
        SlamOccupancyGrid grid(testConfig());
        OccupancyGridMapper mapper;
        const auto result = mapper.integrate(
            grid, singleBeam(20.0f), Pose2D{sf::Vector2f(0.0f, 0.0f), 0.0f}
        );
        return result.integrated && result.hitBeams == 1
            && result.occupiedCells == 1
            && grid.getState(SlamGridCoord{5, 5}) == OccupancyState::Free
            && grid.getState(SlamGridCoord{6, 5}) == OccupancyState::Free
            && grid.getState(SlamGridCoord{7, 5}) == OccupancyState::Occupied
            && grid.getRevision() == 1;
    });

    runTest(suite, "SLAM-MAP-004", "max-range return adds free evidence without an occupied endpoint", [] {
        SlamOccupancyGrid grid(testConfig());
        OccupancyGridMapper mapper;
        const auto result = mapper.integrate(
            grid, singleBeam(40.0f), Pose2D{sf::Vector2f(0.0f, 0.0f), 0.0f}
        );
        const SlamMapStatistics statistics = grid.getStatistics();
        return result.integrated && result.maxRangeBeams == 1
            && result.occupiedCells == 0 && statistics.occupiedCells == 0
            && statistics.freeCells > 0;
    });

    runTest(suite, "SLAM-MAP-004", "invalid scan and invalid beams do not mutate the grid", [] {
        SlamOccupancyGrid grid(testConfig());
        OccupancyGridMapper mapper;
        LaserScan malformed = singleBeam(20.0f);
        malformed.maxRange = malformed.minRange;
        const auto malformedResult = mapper.integrate(grid, malformed, Pose2D{});
        LaserScan invalidBeam = singleBeam(std::numeric_limits<float>::quiet_NaN());
        const auto invalidResult = mapper.integrate(grid, invalidBeam, Pose2D{});
        return !malformedResult.integrated && !invalidResult.integrated
            && invalidResult.invalidBeams == 1 && grid.getRevision() == 0;
    });

    runTest(suite, "SLAM-MAP-004", "invalid base pose never mutates the grid", [] {
        SlamOccupancyGrid grid(testConfig());
        OccupancyGridMapper mapper;
        Pose2D invalid;
        invalid.heading = std::numeric_limits<float>::quiet_NaN();
        const auto result = mapper.integrate(grid, singleBeam(20.0f), invalid);
        return !result.integrated && grid.getRevision() == 0;
    });

    runTest(suite, "SLAM-MAP-004", "translation and yaw extrinsics are applied in the base frame", [] {
        SlamOccupancyGrid grid(testConfig());
        OccupancyGridMapper mapper;
        LaserScan scan = singleBeam(20.0f);
        scan.sensorOffsetX = 10.0;
        scan.sensorYawOffset = kLocalizationPi * 0.5;
        const auto result = mapper.integrate(grid, scan, Pose2D{});
        return result.integrated
            && grid.getState(SlamGridCoord{6, 7}) == OccupancyState::Occupied
            && grid.getState(SlamGridCoord{6, 5}) == OccupancyState::Free;
    });

    runTest(suite, "SLAM-MAP-005", "occupied evidence wins when one scan also traverses the cell", [] {
        SlamOccupancyGrid grid(testConfig());
        OccupancyGridMapper mapper;
        LaserScan scan;
        scan.ranges = {20.0f, 40.0f};
        scan.angleMin = 0.0f;
        scan.angleIncrement = 0.0f;
        scan.minRange = 1.0f;
        scan.maxRange = 40.0f;
        const auto result = mapper.integrate(grid, scan, Pose2D{});
        return result.integrated
            && grid.getState(SlamGridCoord{7, 5}) == OccupancyState::Occupied;
    });

    return suite.exitCode();
}
