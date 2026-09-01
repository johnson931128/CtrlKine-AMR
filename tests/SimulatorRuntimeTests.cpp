#include "app/Simulator.hpp"

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "TestSupport.hpp"

struct SimulatorRuntimeTestAccess {
    static bool applyConfiguredStartPose(
        const MapData& mapData,
        AMR& amr,
        PathExecution& pathExecution
    ) {
        return Simulator::applyConfiguredStartPose(mapData, amr, pathExecution);
    }

    static void applyRobotReset(
        const MapData& mapData,
        const sf::Vector2f& defaultPosition,
        AMR& amr,
        PathExecution& pathExecution
    ) {
        Simulator::applyRobotReset(mapData, defaultPosition, amr, pathExecution);
    }

    static bool isRobotAtInitialWaypoint(
        const MapData& mapData,
        const AMR& amr,
        const PathExecution& pathExecution
    ) {
        return Simulator::isRobotAtInitialWaypoint(mapData, amr, pathExecution);
    }

    static bool isObservedPoseAtInitialWaypoint(
        const MapData& mapData,
        const Pose2D& observedPose,
        const PathExecution& pathExecution
    ) {
        return Simulator::isObservedPoseAtInitialWaypoint(
            mapData, observedPose, pathExecution
        );
    }

    static bool localizationPassesNavigationGate(
        const LocalizationEstimate& estimate,
        const LocalizationStatistics& statistics,
        const AmclConfig& config,
        std::string& reason
    ) {
        return Simulator::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
    }

    static bool applyLocalizationDrivenCommand(
        const Pose2D& observedPose,
        const sf::Vector2f& target,
        float dt,
        float maxSpeed,
        float maxAngularSpeed,
        float trackWidth,
        AMR& amr
    ) {
        return Simulator::applyLocalizationDrivenCommand(
            observedPose, target, dt, maxSpeed, maxAngularSpeed, trackWidth, amr
        );
    }

    static OdometryDelta observeAcceptedMotion(
        const Pose2D& acceptedPose,
        OdometrySimulator& odometrySimulator,
        std::mt19937& randomEngine
    ) {
        return Simulator::observeAcceptedMotion(
            acceptedPose, odometrySimulator, randomEngine
        );
    }

    static SimulatorSensorFrame acquireSensorFrame(
        const Pose2D& acceptedPose,
        const MapData& mapData,
        OdometrySimulator& odometrySimulator,
        const LidarSimulator& lidarSimulator,
        std::mt19937& odometryRandomEngine,
        std::mt19937& lidarRandomEngine
    ) {
        return Simulator::acquireSensorFrame(
            acceptedPose, mapData, odometrySimulator, lidarSimulator,
            odometryRandomEngine, lidarRandomEngine
        );
    }

    static SensorDispatchResult dispatchSensorFrame(
        const SimulatorSensorFrame& frame,
        const MapLikelihoodField& field,
        const MapData& mapData,
        AmclLocalizer& localizer,
        SlamFrontend& slamFrontend,
        std::mt19937& amclRandomEngine
    ) {
        return Simulator::dispatchSensorFrame(
            frame, field, mapData, localizer, slamFrontend, amclRandomEngine
        );
    }

    static bool resetLocalizationState(
        const MapData& mapData,
        const Pose2D& acceptedPose,
        bool initializeFromStart,
        OdometrySimulator& odometrySimulator,
        AmclLocalizer& localizer,
        std::mt19937& randomEngine
    ) {
        return Simulator::resetLocalizationState(
            mapData,
            acceptedPose,
            initializeFromStart,
            odometrySimulator,
            localizer,
            randomEngine
        );
    }
};

namespace {
constexpr float kHalfPi = 1.57079633f;

AMRConfig testConfig() {
    return AMRConfig{
        10.0f,
        20.0f,
        4.0f,
        2.0f,
        8.0f,
        12.0f,
        sf::Color::White,
        sf::Color::Black
    };
}

PathResult successfulResult(const std::vector<GridCoord>& path) {
    PathResult result;
    result.success = true;
    result.path = path;
    result.message = "success";
    return result;
}

bool near(float first, float second, float tolerance = 0.001f) {
    return std::abs(first - second) <= tolerance;
}

bool samePoint(const sf::Vector2f& first, const sf::Vector2f& second) {
    return near(first.x, second.x) && near(first.y, second.y);
}

bool executionIsCleared(const PathExecution& execution) {
    return execution.getState() == PathExecutionState::NotFollowing
        && execution.getCurrentWaypointIndex() == 0
        && !execution.hasExecutablePath()
        && !execution.getCurrentWaypoint().has_value()
        && execution.getResult().path.empty()
        && execution.getExecutionWaypoints().empty();
}

AmclConfig localizationTestConfig() {
    AmclConfig config;
    config.minParticles = 50;
    config.maxParticles = 50;
    config.initialParticleCount = 50;
    config.initialStdDevX = 10.0;
    config.initialStdDevY = 10.0;
    config.initialStdDevYaw = 0.1;
    return config;
}
}

int main() {
    TestSuite suite;

    runTest(suite, "START-001", "configured Start synchronizes pose, geometry, and execution state", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(125.0f, 140.0f), kHalfPi};
        map.setRobotStartPose(startPose);

        AMR robot(testConfig(), sf::Vector2f(10.0f, 10.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));
        execution.advanceWaypoint();

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return applied
            && samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && robot.containsPoint(startPose.position + sf::Vector2f(0.0f, 9.0f))
            && !robot.containsPoint(startPose.position + sf::Vector2f(6.0f, 0.0f))
            && executionIsCleared(execution);
    });

    runTest(suite, "START-002", "changing Start replaces the runtime pose and resets progress", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        PathExecution execution;

        map.setRobotStartPose(Pose2D{sf::Vector2f(25.0f, 25.0f), 0.3f});
        SimulatorRuntimeTestAccess::applyConfiguredStartPose(map, robot, execution);

        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));
        execution.advanceWaypoint();
        const Pose2D changedStart{sf::Vector2f(175.0f, 85.0f), -0.7f};
        map.setRobotStartPose(changedStart);

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return applied
            && samePoint(robot.getPosition(), changedStart.position)
            && near(robot.getHeading(), changedStart.heading)
            && executionIsCleared(execution);
    });

    runTest(suite, "START-003", "missing Start does not alter runtime pose or execution", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(30.0f, 40.0f));
        robot.setPose(sf::Vector2f(30.0f, 40.0f), 0.5f);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return !applied
            && samePoint(robot.getPosition(), sf::Vector2f(30.0f, 40.0f))
            && near(robot.getHeading(), 0.5f)
            && execution.getState() == PathExecutionState::Following
            && execution.getCurrentWaypointIndex() == 0;
    });

    runTest(suite, "START-004", "planning synchronization restores Start after manual runtime movement", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(75.0f, 125.0f), -0.4f};
        map.setRobotStartPose(startPose);
        AMR robot(testConfig(), sf::Vector2f(75.0f, 125.0f));
        robot.setPose(sf::Vector2f(300.0f, 250.0f), 1.0f);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{6, 5}, GridCoord{5, 5}}));

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return applied
            && samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && executionIsCleared(execution);
    });

    runTest(suite, "RESET-001", "reset with Start restores its position and heading", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(-125.0f, 75.0f), -1.1f};
        map.setRobotStartPose(startPose);
        AMR robot(testConfig(), sf::Vector2f(300.0f, 300.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        SimulatorRuntimeTestAccess::applyRobotReset(
            map, sf::Vector2f(400.0f, 400.0f), robot, execution
        );
        return samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && executionIsCleared(execution);
    });

    runTest(suite, "RESET-002", "reset without Start restores the existing default pose", [] {
        MapData map(50.0f);
        const sf::Vector2f defaultPosition(400.0f, 400.0f);
        AMR robot(testConfig(), sf::Vector2f(20.0f, 30.0f));
        robot.setPose(sf::Vector2f(20.0f, 30.0f), 1.2f);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}}));

        SimulatorRuntimeTestAccess::applyRobotReset(
            map, defaultPosition, robot, execution
        );
        return samePoint(robot.getPosition(), defaultPosition)
            && near(robot.getHeading(), 0.0f)
            && executionIsCleared(execution);
    });

    runTest(suite, "START-005", "off-center Start is accepted as initial waypoint without approach motion", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(10.0f, 20.0f), 0.8f};
        map.setRobotStartPose(startPose);
        AMR robot(testConfig(), sf::Vector2f(300.0f, 300.0f));
        robot.setPose(startPose.position, startPose.heading);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        const bool reached = SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(
            map, robot, execution
        );
        if (reached) {
            execution.advanceWaypoint();
        }
        return reached
            && samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && execution.getCurrentWaypointIndex() == 1
            && execution.getState() == PathExecutionState::Following;
    });

    runTest(suite, "START-006", "same-cell shortcut applies only to waypoint zero", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(10.0f, 20.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        if (!SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(map, robot, execution)) {
            return false;
        }
        execution.advanceWaypoint();
        robot.setPose(sf::Vector2f(60.0f, 20.0f), 0.0f);
        return !SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(map, robot, execution);
    });

    runTest(suite, "START-007", "different-cell runtime pose is not the initial waypoint", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(60.0f, 20.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));
        return !SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(map, robot, execution)
            && execution.getCurrentWaypointIndex() == 0;
    });

    runTest(suite, "LOCALIZATION-001", "only accepted post-rollback pose produces odometry motion", [] {
        OdometryConfig odometryConfig;
        odometryConfig.translationStdDevPerDistance = 0.0;
        odometryConfig.translationStdDevPerRotation = 0.0;
        odometryConfig.rotationStdDevPerRotation = 0.0;
        odometryConfig.rotationStdDevPerDistance = 0.0;
        OdometrySimulator odometry(odometryConfig);
        const Pose2D previousAccepted{sf::Vector2f(100.0f, 100.0f), 0.0f};
        odometry.reset(previousAccepted);
        std::mt19937 randomEngine(100);
        const OdometryDelta rejected = SimulatorRuntimeTestAccess::observeAcceptedMotion(
            previousAccepted, odometry, randomEngine
        );
        const Pose2D nextAccepted{sf::Vector2f(150.0f, 100.0f), 0.0f};
        const OdometryDelta accepted = SimulatorRuntimeTestAccess::observeAcceptedMotion(
            nextAccepted, odometry, randomEngine
        );
        return near(static_cast<float>(rejected.translation), 0.0f)
            && near(static_cast<float>(rejected.rotation1), 0.0f)
            && near(static_cast<float>(rejected.rotation2), 0.0f)
            && near(static_cast<float>(accepted.translation), 50.0f)
            && samePoint(odometry.getOdometryPose().position, nextAccepted.position);
    });

    runTest(suite, "LOCALIZATION-002", "Start reset initializes uncertain belief and odometry separately", [] {
        MapData map(50.0f);
        const Pose2D start{sf::Vector2f(125.0f, 140.0f), 0.4f};
        map.setRobotStartPose(start);
        const AmclConfig config = localizationTestConfig();
        AmclLocalizer localizer(config);
        OdometrySimulator odometry;
        std::mt19937 randomEngine(101);
        const bool initialized = SimulatorRuntimeTestAccess::resetLocalizationState(
            map, start, true, odometry, localizer, randomEngine
        );
        return initialized
            && localizer.getEstimate().valid
            && localizer.getEstimate().particleCount == config.initialParticleCount
            && localizer.getEstimate().covariance.xx() > 0.0
            && localizer.getEstimate().covariance.yy() > 0.0
            && samePoint(odometry.getOdometryPose().position, start.position)
            && near(odometry.getOdometryPose().heading, start.heading);
    });

    runTest(suite, "LOCALIZATION-003", "reset without Start leaves localization explicitly uninitialized", [] {
        MapData map(50.0f);
        const Pose2D acceptedPose{sf::Vector2f(40.0f, 50.0f), -0.2f};
        AmclLocalizer localizer(localizationTestConfig());
        OdometrySimulator odometry;
        std::mt19937 randomEngine(102);
        const bool initialized = SimulatorRuntimeTestAccess::resetLocalizationState(
            map, acceptedPose, false, odometry, localizer, randomEngine
        );
        return !initialized
            && !localizer.getEstimate().valid
            && localizer.getStatistics().state == LocalizationState::Uninitialized
            && samePoint(odometry.getOdometryPose().position, acceptedPose.position)
            && near(odometry.getOdometryPose().heading, acceptedPose.heading);
    });

    runTest(suite, "SLAM-SIM-001", "headless sensor frame fans one exact scan to AMCL and SLAM", [] {
        MapData map(50.0f);
        map.addObstacle(GridCoord{4, 2});
        map.addObstacle(GridCoord{1, 5});
        const Pose2D pose{sf::Vector2f(100.0f, 100.0f), 0.0f};
        OdometryConfig odometryConfig;
        odometryConfig.translationStdDevPerDistance = 0.0;
        odometryConfig.translationStdDevPerRotation = 0.0;
        odometryConfig.rotationStdDevPerRotation = 0.0;
        odometryConfig.rotationStdDevPerDistance = 0.0;
        OdometrySimulator odometry(odometryConfig);
        odometry.reset(pose);
        LidarConfig lidarConfig;
        lidarConfig.beamCount = 31;
        lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
        lidarConfig.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(lidarConfig);
        std::mt19937 odometryRng(31);
        std::mt19937 lidarRng(32);
        const SimulatorSensorFrame frame = SimulatorRuntimeTestAccess::acquireSensorFrame(
            pose, map, odometry, lidar, odometryRng, lidarRng
        );
        const LaserScan scanBefore = frame.scan;

        AmclConfig amclConfig = localizationTestConfig();
        AmclLocalizer localizer(amclConfig);
        std::mt19937 initializationRng(33);
        if (!localizer.initializeLocal(pose, map, initializationRng)) return false;
        MapLikelihoodField field;
        field.rebuild(map, amclConfig.likelihoodMaxDistance);
        SlamFrontend slam;
        std::mt19937 amclRng(34);
        const SensorDispatchResult result = SimulatorRuntimeTestAccess::dispatchSensorFrame(
            frame, field, map, localizer, slam, amclRng
        );
        return result.amclUpdated && result.slam.state == SlamState::Tracking
            && result.slam.integration.totalBeams == frame.scan.ranges.size()
            && localizer.getStatistics().sensor.totalBeams == frame.scan.ranges.size()
            && frame.scan.ranges == scanBefore.ranges
            && frame.scan.angleIncrement == scanBefore.angleIncrement;
    });

    runTest(suite, "SLAM-SIM-001", "actual acquisition seam isolates odometry from extra LiDAR cadence", [] {
        MapData map(50.0f);
        map.addObstacle(GridCoord{4, 2});
        LidarConfig lidarConfig;
        lidarConfig.beamCount = 31;
        lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
        LidarSimulator lidar(lidarConfig);
        OdometrySimulator firstOdometry;
        OdometrySimulator secondOdometry;
        const Pose2D start{sf::Vector2f(100.0f, 100.0f), 0.0f};
        firstOdometry.reset(start);
        secondOdometry.reset(start);
        std::mt19937 firstOdometryRng(41);
        std::mt19937 secondOdometryRng(41);
        std::mt19937 firstLidarRng(42);
        std::mt19937 secondLidarRng(42);
        for (int step = 0; step < 5; ++step) {
            const Pose2D pose{sf::Vector2f(100.0f + step * 15.0f, 100.0f), 0.0f};
            const SimulatorSensorFrame first = SimulatorRuntimeTestAccess::acquireSensorFrame(
                pose, map, firstOdometry, lidar, firstOdometryRng, firstLidarRng
            );
            (void)lidar.simulate(pose, map, secondLidarRng);
            const SimulatorSensorFrame second = SimulatorRuntimeTestAccess::acquireSensorFrame(
                pose, map, secondOdometry, lidar, secondOdometryRng, secondLidarRng
            );
            if (first.odometry.rotation1 != second.odometry.rotation1
                || first.odometry.translation != second.odometry.translation
                || first.odometry.rotation2 != second.odometry.rotation2) {
                return false;
            }
        }
        return true;
    });

    runTest(suite, "SLAM-ARCH-004", "SLAM reset history cannot perturb AMCL inference for identical frames", [] {
        MapData map(50.0f);
        map.addObstacle(GridCoord{4, 2});
        map.addObstacle(GridCoord{1, 5});
        const Pose2D pose{sf::Vector2f(100.0f, 100.0f), 0.0f};
        LidarConfig lidarConfig;
        lidarConfig.beamCount = 31;
        lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
        lidarConfig.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(lidarConfig);
        OdometrySimulator odometry;
        odometry.reset(pose);
        std::mt19937 odometryRng(51);
        std::mt19937 lidarRng(52);
        const SimulatorSensorFrame frame = SimulatorRuntimeTestAccess::acquireSensorFrame(
            pose, map, odometry, lidar, odometryRng, lidarRng
        );
        AmclConfig config = localizationTestConfig();
        AmclLocalizer first(config);
        AmclLocalizer second(config);
        std::mt19937 firstInitialization(53);
        std::mt19937 secondInitialization(53);
        if (!first.initializeLocal(pose, map, firstInitialization)
            || !second.initializeLocal(pose, map, secondInitialization)) return false;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        SlamFrontend firstSlam;
        SlamFrontend resetSlam;
        resetSlam.reset();
        std::mt19937 firstAmclRng(54);
        std::mt19937 secondAmclRng(54);
        SimulatorRuntimeTestAccess::dispatchSensorFrame(
            frame, field, map, first, firstSlam, firstAmclRng
        );
        SimulatorRuntimeTestAccess::dispatchSensorFrame(
            frame, field, map, second, resetSlam, secondAmclRng
        );
        const LocalizationEstimate& a = first.getEstimate();
        const LocalizationEstimate& b = second.getEstimate();
        return a.valid == b.valid && a.pose.position == b.pose.position
            && a.pose.heading == b.pose.heading
            && first.getStatistics().sensor.observationQuality
                == second.getStatistics().sensor.observationQuality;
    });

    runTest(suite, "LOCALIZATION-NAV-001", "confidence gate uses belief diagnostics without truth error", [] {
        AmclConfig config = localizationTestConfig();
        LocalizationEstimate estimate;
        estimate.valid = true;
        estimate.converged = true;
        estimate.covariance.values[0] = 25.0;
        estimate.covariance.values[4] = 25.0;
        estimate.covariance.values[8] = 0.01;
        LocalizationStatistics statistics;
        statistics.state = LocalizationState::Converged;
        statistics.support = LocalizationSupport::Good;
        statistics.dominantClusterWeight = 0.9;
        std::string reason;
        const bool accepted = SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
        statistics.state = LocalizationState::Ambiguous;
        const bool rejectsState = !SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
        statistics.state = LocalizationState::Converged;
        statistics.support = LocalizationSupport::Insufficient;
        const bool rejectsSupport = !SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
        statistics.support = LocalizationSupport::Good;
        statistics.dominantClusterWeight = config.navigationDominantWeight - 0.01;
        const bool rejectsDominance = !SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
        statistics.dominantClusterWeight = 0.9;
        estimate.covariance.values[0] = config.navigationPositionStdDev
            * config.navigationPositionStdDev * 2.0;
        const bool rejectsCovariance = !SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
        estimate.covariance.values[0] = 25.0;
        estimate.converged = false;
        const bool rejectsEstimateFlag = !SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
        return accepted && rejectsState && rejectsSupport && rejectsDominance
            && rejectsCovariance && rejectsEstimateFlag && !reason.empty();
    });

    runTest(suite, "LOCALIZATION-NAV-002", "initial waypoint can be checked from estimated pose", [] {
        MapData map(50.0f);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{2, 3}, GridCoord{3, 3}}));
        const Pose2D estimate{map.getMapper().gridToWorldCenter(GridCoord{2, 3}), 0.5f};
        return SimulatorRuntimeTestAccess::isObservedPoseAtInitialWaypoint(
            map, estimate, execution
        );
    });

    runTest(suite, "LOCALIZATION-NAV-003", "estimated-pose plan drives truth and loss stops further command", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(500.0f, 500.0f)
        ));
        const Pose2D persistentStart{map.getMapper().gridToWorldCenter(GridCoord{0, 0}), 0.0f};
        const Pose2D estimatePose{map.getMapper().gridToWorldCenter(GridCoord{2, 2}), 0.0f};
        const Pose2D goal{map.getMapper().gridToWorldCenter(GridCoord{6, 2}), 0.0f};
        map.setRobotStartPose(persistentStart);
        map.setRobotGoalPose(goal);
        const PathResult plan = PathPlanner::plan(map, estimatePose, goal, 0.0f);
        if (!plan.success || plan.path.empty() || !(plan.path.front() == GridCoord{2, 2})
            || !map.getRobotStartPose().has_value()
            || map.getRobotStartPose()->position != persistentStart.position) return false;

        AmclConfig config = localizationTestConfig();
        LocalizationEstimate estimate;
        estimate.valid = true;
        estimate.converged = true;
        estimate.pose = estimatePose;
        estimate.covariance.values[0] = 25.0;
        estimate.covariance.values[4] = 25.0;
        estimate.covariance.values[8] = 0.01;
        LocalizationStatistics statistics;
        statistics.state = LocalizationState::Converged;
        statistics.support = LocalizationSupport::Good;
        statistics.dominantClusterWeight = 0.9;
        std::string reason;
        if (!SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
                estimate, statistics, config, reason)) return false;

        AMR truth(testConfig(), estimatePose.position);
        const sf::Vector2f target = map.getMapper().gridToWorldCenter(plan.path[1]);
        SimulatorRuntimeTestAccess::applyLocalizationDrivenCommand(
            estimate.pose, target, 0.1f, 100.0f, 4.0f, testConfig().trackWidth, truth
        );
        const sf::Vector2f moved = truth.getPosition();
        if (moved.x <= estimatePose.position.x) return false;

        statistics.support = LocalizationSupport::Insufficient;
        const bool lost = !SimulatorRuntimeTestAccess::localizationPassesNavigationGate(
            estimate, statistics, config, reason
        );
        const sf::Vector2f stopped = truth.getPosition();
        return lost && !reason.empty() && samePoint(moved, stopped);
    });

    runTest(suite, "UI-LAYOUT-001", "layout remains nonnegative and non-overlapping across window sizes", [] {
        const std::array<sf::Vector2u, 4> sizes{{
            {1440u, 900u}, {1280u, 800u}, {1024u, 720u}, {240u, 40u}
        }};
        for (const sf::Vector2u size : sizes) {
            const ApplicationLayout layout = calculateApplicationLayout(size);
            if (layout.toolbar.size.x < 0.0f || layout.toolbar.size.y < 0.0f
                || layout.simulationViewport.size.x < 0.0f
                || layout.simulationViewport.size.y < 0.0f
                || layout.inspector.size.x < 0.0f || layout.inspector.size.y < 0.0f
                || !near(
                    layout.simulationViewport.position.x + layout.simulationViewport.size.x,
                    layout.inspector.position.x
                )
                || !near(
                    layout.simulationViewport.size.x + layout.inspector.size.x,
                    static_cast<float>(size.x)
                )) {
                return false;
            }
        }
        const ApplicationLayout desktop = calculateApplicationLayout({1440u, 900u});
        return near(desktop.inspector.size.x, 360.0f)
            && near(desktop.simulationViewport.size.x, 1080.0f)
            && near(desktop.toolbar.size.y, 64.0f);
    });

    runTest(suite, "UI-TOOLBAR-001", "toolbar drawing and hit testing share one button geometry", [] {
        EditorToolbar toolbar;
        toolbar.setBounds(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(1080.0f, 64.0f)));
        const std::array<EditorMode, 7> modes{{
            EditorMode::Select,
            EditorMode::PlaceObstacle,
            EditorMode::DeleteObstacle,
            EditorMode::SetStartPose,
            EditorMode::SetGoalPose,
            EditorMode::DrawWorkZone,
            EditorMode::PanView
        }};
        for (const EditorMode mode : modes) {
            const sf::FloatRect bounds = toolbar.getButtonBounds(mode);
            const sf::Vector2i center(
                static_cast<int>(bounds.position.x + bounds.size.x * 0.5f),
                static_cast<int>(bounds.position.y + bounds.size.y * 0.5f)
            );
            if (toolbar.hitTest(center) != std::optional<EditorMode>(mode)) {
                return false;
            }
        }
        return !toolbar.hitTest(sf::Vector2i(1079, 63)).has_value();
    });

    runTest(suite, "UI-INSPECTOR-001", "Inspector defaults to Map and persists tab selection", [] {
        InspectorPanel panel;
        panel.setBounds(sf::FloatRect(
            sf::Vector2f(1080.0f, 64.0f), sf::Vector2f(360.0f, 836.0f)
        ));
        if (panel.getActiveTab() != InspectorTab::Map) {
            return false;
        }
        const sf::FloatRect navigation = panel.getTabBounds(InspectorTab::Navigation);
        const sf::Vector2i click(
            static_cast<int>(navigation.position.x + navigation.size.x * 0.5f),
            static_cast<int>(navigation.position.y + navigation.size.y * 0.5f)
        );
        return panel.handleClick(click)
            && panel.getActiveTab() == InspectorTab::Navigation
            && !panel.handleClick(sf::Vector2i(100, 100))
            && panel.getActiveTab() == InspectorTab::Navigation;
    });

    runTest(suite, "UI-INSPECTOR-002", "Inspector keeps independent deterministic tab scroll offsets", [] {
        InspectorPanel panel;
        panel.setBounds(sf::FloatRect(
            sf::Vector2f(1080.0f, 64.0f), sf::Vector2f(360.0f, 500.0f)
        ));
        panel.setContentHeight(InspectorTab::Map, 1000.0f);
        panel.setContentHeight(InspectorTab::Navigation, 800.0f);
        panel.scroll(1.0f);
        const float mapOffset = panel.getScrollOffset(InspectorTab::Map);
        const sf::FloatRect navigation = panel.getTabBounds(InspectorTab::Navigation);
        panel.handleClick(sf::Vector2i(
            static_cast<int>(navigation.position.x + 2.0f),
            static_cast<int>(navigation.position.y + 2.0f)
        ));
        panel.scroll(1.0f);
        panel.scroll(1.0f);
        const float navigationOffset = panel.getScrollOffset(InspectorTab::Navigation);
        panel.setBounds(sf::FloatRect(
            sf::Vector2f(1080.0f, 64.0f), sf::Vector2f(360.0f, 1300.0f)
        ));
        return near(mapOffset, 48.0f)
            && near(navigationOffset, 96.0f)
            && near(panel.getScrollOffset(InspectorTab::Map), 0.0f)
            && near(panel.getScrollOffset(InspectorTab::Navigation), 0.0f);
    });

    runTest(suite, "UI-INSPECTOR-003", "small-height Inspector hides its footer instead of covering tabs", [] {
        InspectorPanel panel;
        panel.setBounds(sf::FloatRect(
            sf::Vector2f(320.0f, 64.0f), sf::Vector2f(360.0f, 136.0f)
        ));
        const sf::FloatRect tab = panel.getTabBounds(InspectorTab::Localization);
        const sf::FloatRect body = panel.getBodyBounds();
        const sf::FloatRect footer = panel.getFooterBounds();
        const bool compact = footer.size.y == 0.0f
            && body.position.y >= tab.position.y + tab.size.y;
        panel.setBounds(sf::FloatRect(
            sf::Vector2f(1080.0f, 64.0f), sf::Vector2f(360.0f, 836.0f)
        ));
        const sf::FloatRect desktopBody = panel.getBodyBounds();
        const sf::FloatRect desktopFooter = panel.getFooterBounds();
        return compact && desktopFooter.size.y > 0.0f
            && desktopFooter.position.y
                >= desktopBody.position.y + desktopBody.size.y;
    });

    runTest(suite, "UI-INSPECTOR-004", "Inspector exposes a fourth independently scrollable SLAM tab", [] {
        InspectorPanel panel;
        panel.setBounds(sf::FloatRect(
            sf::Vector2f(1080.0f, 64.0f), sf::Vector2f(360.0f, 500.0f)
        ));
        panel.setContentHeight(InspectorTab::Slam, 900.0f);
        const sf::FloatRect slam = panel.getTabBounds(InspectorTab::Slam);
        const sf::FloatRect localization = panel.getTabBounds(InspectorTab::Localization);
        const sf::Vector2i click(
            static_cast<int>(slam.position.x + slam.size.x * 0.5f),
            static_cast<int>(slam.position.y + slam.size.y * 0.5f)
        );
        panel.handleClick(click);
        panel.scroll(1.0f);
        return panel.getActiveTab() == InspectorTab::Slam
            && near(slam.size.x, 90.0f)
            && near(localization.position.x + localization.size.x, slam.position.x)
            && near(panel.getScrollOffset(InspectorTab::Slam), 48.0f)
            && near(panel.getScrollOffset(InspectorTab::Localization), 0.0f);
    });

    return suite.exitCode();
}
