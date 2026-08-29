#include "SlamFrontend.hpp"

#include <cmath>
#include <stdexcept>

#include "LaserScanGeometry.hpp"

namespace {
bool validFrontendConfig(const SlamFrontendConfig& config) {
    return std::isfinite(config.maximumOdometryTranslation)
        && config.maximumOdometryTranslation > 0.0
        && std::isfinite(config.maximumOdometryRotation)
        && config.maximumOdometryRotation > 0.0
        && config.failuresBeforeLost > 0;
}

bool finiteOdometry(const OdometryDelta& odometry) {
    return odometry.valid && std::isfinite(odometry.rotation1)
        && std::isfinite(odometry.translation)
        && std::isfinite(odometry.rotation2);
}
}

SlamFrontend::SlamFrontend(const SlamFrontendConfig& config)
    : m_config(config), m_map(config.grid), m_matcher(config.matcher) {
    if (!validFrontendConfig(config)) {
        throw std::invalid_argument("SLAM frontend configuration is invalid.");
    }
    reset();
}

void SlamFrontend::reset() {
    m_map.reset();
    m_state = SlamState::Uninitialized;
    m_pose = Pose2D{};
    m_poseValid = false;
    m_consecutiveFailures = 0;
    m_acceptedUpdates = 0;
    m_rejectedUpdates = 0;
    m_lastUpdate = SlamUpdateResult{};
}

Pose2D SlamFrontend::predictPose(const Pose2D& pose, const OdometryDelta& odometry) {
    if (!finiteOdometry(odometry)) {
        return pose;
    }
    Pose2D predicted = pose;
    const double movementHeading = predicted.heading + odometry.rotation1;
    predicted.position.x += static_cast<float>(odometry.translation * std::cos(movementHeading));
    predicted.position.y += static_cast<float>(odometry.translation * std::sin(movementHeading));
    predicted.heading = static_cast<float>(normalizeLocalizationAngle(
        predicted.heading + odometry.rotation1 + odometry.rotation2
    ));
    return predicted;
}

bool SlamFrontend::scanHasEnoughPhysicalHits(const LaserScan& scan) const {
    if (!OccupancyGridMapper::isScanMetadataValid(scan)) {
        return false;
    }
    std::size_t hits = 0;
    for (const float range : scan.ranges) {
        if (isLaserReturnHit(scan, range)) {
            ++hits;
        }
    }
    return hits >= m_config.matcher.minimumUsableBeams;
}

void SlamFrontend::publishResult() {
    m_lastUpdate.state = m_state;
    m_lastUpdate.pose = m_pose;
    m_lastUpdate.poseValid = m_poseValid;
    m_lastUpdate.consecutiveFailures = m_consecutiveFailures;
    m_lastUpdate.acceptedUpdates = m_acceptedUpdates;
    m_lastUpdate.rejectedUpdates = m_rejectedUpdates;
}

SlamUpdateResult SlamFrontend::process(
    const OdometryDelta& odometry,
    const LaserScan& scan
) {
    m_lastUpdate = SlamUpdateResult{};

    if (m_state == SlamState::Uninitialized) {
        if (!scanHasEnoughPhysicalHits(scan)) {
            m_lastUpdate.match.reason = OccupancyGridMapper::isScanMetadataValid(scan)
                ? ScanMatchReason::NoPhysicalHits
                : ScanMatchReason::InvalidScan;
            publishResult();
            return m_lastUpdate;
        }
        m_pose = Pose2D{};
        m_poseValid = true;
        m_lastUpdate.predictedPose = m_pose;
        m_lastUpdate.match.predictedPose = m_pose;
        m_lastUpdate.match.correctedPose = m_pose;
        m_lastUpdate.match.accepted = true;
        m_lastUpdate.match.reason = ScanMatchReason::Bootstrap;
        m_lastUpdate.integration = m_mapper.integrate(m_map, scan, m_pose);
        m_lastUpdate.mapIntegrated = m_lastUpdate.integration.integrated;
        if (m_lastUpdate.mapIntegrated) {
            m_state = SlamState::Tracking;
            ++m_acceptedUpdates;
        }
        publishResult();
        return m_lastUpdate;
    }

    if (!finiteOdometry(odometry)
        || std::abs(odometry.translation) > m_config.maximumOdometryTranslation
        || std::abs(odometry.rotation1) + std::abs(odometry.rotation2)
            > m_config.maximumOdometryRotation) {
        m_state = SlamState::Lost;
        ++m_consecutiveFailures;
        ++m_rejectedUpdates;
        m_lastUpdate.predictedPose = m_pose;
        m_lastUpdate.match.predictedPose = m_pose;
        m_lastUpdate.match.correctedPose = m_pose;
        m_lastUpdate.match.reason = ScanMatchReason::LargeOdometry;
        publishResult();
        return m_lastUpdate;
    }

    const Pose2D predicted = predictPose(m_pose, odometry);
    m_pose = predicted;
    m_lastUpdate.predictedPose = predicted;
    m_lastUpdate.match = m_matcher.match(m_map, scan, predicted);
    if (m_lastUpdate.match.accepted) {
        m_pose = m_lastUpdate.match.correctedPose;
        m_lastUpdate.integration = m_mapper.integrate(m_map, scan, m_pose);
        m_lastUpdate.mapIntegrated = m_lastUpdate.integration.integrated;
        m_state = SlamState::Tracking;
        m_consecutiveFailures = 0;
        ++m_acceptedUpdates;
    } else {
        ++m_consecutiveFailures;
        ++m_rejectedUpdates;
        if (m_consecutiveFailures >= m_config.failuresBeforeLost) {
            m_state = SlamState::Lost;
        }
    }
    publishResult();
    return m_lastUpdate;
}

SlamState SlamFrontend::getState() const {
    return m_state;
}

const Pose2D& SlamFrontend::getPose() const {
    return m_pose;
}

bool SlamFrontend::hasPose() const {
    return m_poseValid;
}

const SlamOccupancyGrid& SlamFrontend::getMap() const {
    return m_map;
}

const SlamUpdateResult& SlamFrontend::getLastUpdate() const {
    return m_lastUpdate;
}

const SlamFrontendConfig& SlamFrontend::getConfig() const {
    return m_config;
}
