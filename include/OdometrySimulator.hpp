#pragma once

#include <random>

#include "LocalizationTypes.hpp"

class OdometrySimulator {
public:
    explicit OdometrySimulator(const OdometryConfig& config = OdometryConfig{});

    void reset(const Pose2D& groundTruthPose);
    void rebaseGroundTruthReference(const Pose2D& groundTruthPose);
    OdometryDelta observe(const Pose2D& currentGroundTruthPose, std::mt19937& randomEngine);

    bool isInitialized() const;
    const Pose2D& getOdometryPose() const;

private:
    OdometryConfig m_config;
    Pose2D m_previousGroundTruthPose;
    Pose2D m_odometryPose;
    bool m_initialized = false;
};
