#pragma once

#include "CorrelativeScanMatcher.hpp"
#include "OccupancyGridMapper.hpp"

class SlamFrontend {
public:
    explicit SlamFrontend(const SlamFrontendConfig& config = SlamFrontendConfig{});

    void reset();
    SlamUpdateResult process(const OdometryDelta& odometry, const LaserScan& scan);

    SlamState getState() const;
    const Pose2D& getPose() const;
    bool hasPose() const;
    const SlamOccupancyGrid& getMap() const;
    const SlamUpdateResult& getLastUpdate() const;
    const SlamFrontendConfig& getConfig() const;

    static Pose2D predictPose(const Pose2D& pose, const OdometryDelta& odometry);

private:
    bool scanHasEnoughPhysicalHits(const LaserScan& scan) const;
    void publishResult();

    SlamFrontendConfig m_config;
    SlamOccupancyGrid m_map;
    OccupancyGridMapper m_mapper;
    CorrelativeScanMatcher m_matcher;
    SlamState m_state = SlamState::Uninitialized;
    Pose2D m_pose;
    bool m_poseValid = false;
    std::size_t m_consecutiveFailures = 0;
    std::size_t m_acceptedUpdates = 0;
    std::size_t m_rejectedUpdates = 0;
    SlamUpdateResult m_lastUpdate;
};
