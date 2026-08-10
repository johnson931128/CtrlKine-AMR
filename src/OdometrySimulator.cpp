#include "OdometrySimulator.hpp"

#include <cmath>
#include <stdexcept>

namespace {
bool finitePose(const Pose2D& pose) {
    return std::isfinite(pose.position.x)
        && std::isfinite(pose.position.y)
        && std::isfinite(pose.heading);
}

double gaussian(std::mt19937& randomEngine, double standardDeviation) {
    if (standardDeviation <= 0.0) {
        return 0.0;
    }
    return std::normal_distribution<double>(0.0, standardDeviation)(randomEngine);
}
}

OdometrySimulator::OdometrySimulator(const OdometryConfig& config)
    : m_config(config) {
    const bool valid = std::isfinite(config.translationStdDevPerDistance)
        && std::isfinite(config.translationStdDevPerRotation)
        && std::isfinite(config.rotationStdDevPerRotation)
        && std::isfinite(config.rotationStdDevPerDistance)
        && config.translationStdDevPerDistance >= 0.0
        && config.translationStdDevPerRotation >= 0.0
        && config.rotationStdDevPerRotation >= 0.0
        && config.rotationStdDevPerDistance >= 0.0;
    if (!valid) {
        throw std::invalid_argument("Odometry configuration is invalid.");
    }
}

void OdometrySimulator::reset(const Pose2D& groundTruthPose) {
    if (!finitePose(groundTruthPose)) {
        throw std::invalid_argument("Odometry reset pose is invalid.");
    }
    m_previousGroundTruthPose = groundTruthPose;
    m_previousGroundTruthPose.heading = static_cast<float>(
        normalizeLocalizationAngle(m_previousGroundTruthPose.heading)
    );
    m_odometryPose = m_previousGroundTruthPose;
    m_initialized = true;
}

void OdometrySimulator::rebaseGroundTruthReference(const Pose2D& groundTruthPose) {
    if (!finitePose(groundTruthPose)) {
        throw std::invalid_argument("Odometry rebase pose is invalid.");
    }
    m_previousGroundTruthPose = groundTruthPose;
    m_previousGroundTruthPose.heading = static_cast<float>(
        normalizeLocalizationAngle(m_previousGroundTruthPose.heading)
    );
    m_initialized = true;
}

OdometryDelta OdometrySimulator::observe(
    const Pose2D& currentGroundTruthPose,
    std::mt19937& randomEngine
) {
    if (!finitePose(currentGroundTruthPose)) {
        return OdometryDelta{0.0, 0.0, 0.0, false};
    }
    if (!m_initialized) {
        reset(currentGroundTruthPose);
        return OdometryDelta{};
    }

    const double dx = currentGroundTruthPose.position.x - m_previousGroundTruthPose.position.x;
    const double dy = currentGroundTruthPose.position.y - m_previousGroundTruthPose.position.y;
    const double distance = std::hypot(dx, dy);
    const double deltaHeading = normalizeLocalizationAngle(
        currentGroundTruthPose.heading - m_previousGroundTruthPose.heading
    );

    double trueRotation1 = 0.0;
    double trueTranslation = 0.0;
    if (distance > 1e-9) {
        const double forwardProjection = dx * std::cos(m_previousGroundTruthPose.heading)
            + dy * std::sin(m_previousGroundTruthPose.heading);
        const bool reverse = forwardProjection < 0.0;
        const double directedX = reverse ? -dx : dx;
        const double directedY = reverse ? -dy : dy;
        trueRotation1 = normalizeLocalizationAngle(
            std::atan2(directedY, directedX) - m_previousGroundTruthPose.heading
        );
        trueTranslation = reverse ? -distance : distance;
    }
    const double trueRotation2 = normalizeLocalizationAngle(deltaHeading - trueRotation1);

    const double translationStdDev =
        m_config.translationStdDevPerDistance * std::abs(trueTranslation)
        + m_config.translationStdDevPerRotation
            * (std::abs(trueRotation1) + std::abs(trueRotation2));
    const double rotation1StdDev =
        m_config.rotationStdDevPerRotation * std::abs(trueRotation1)
        + m_config.rotationStdDevPerDistance * std::abs(trueTranslation);
    const double rotation2StdDev =
        m_config.rotationStdDevPerRotation * std::abs(trueRotation2)
        + m_config.rotationStdDevPerDistance * std::abs(trueTranslation);

    OdometryDelta measurement;
    measurement.rotation1 = normalizeLocalizationAngle(
        trueRotation1 + gaussian(randomEngine, rotation1StdDev)
    );
    measurement.translation = trueTranslation + gaussian(randomEngine, translationStdDev);
    measurement.rotation2 = normalizeLocalizationAngle(
        trueRotation2 + gaussian(randomEngine, rotation2StdDev)
    );

    const double movementHeading = m_odometryPose.heading + measurement.rotation1;
    m_odometryPose.position.x += static_cast<float>(measurement.translation * std::cos(movementHeading));
    m_odometryPose.position.y += static_cast<float>(measurement.translation * std::sin(movementHeading));
    m_odometryPose.heading = static_cast<float>(normalizeLocalizationAngle(
        m_odometryPose.heading + measurement.rotation1 + measurement.rotation2
    ));

    m_previousGroundTruthPose = currentGroundTruthPose;
    m_previousGroundTruthPose.heading = static_cast<float>(
        normalizeLocalizationAngle(m_previousGroundTruthPose.heading)
    );
    return measurement;
}

bool OdometrySimulator::isInitialized() const {
    return m_initialized;
}

const Pose2D& OdometrySimulator::getOdometryPose() const {
    return m_odometryPose;
}
