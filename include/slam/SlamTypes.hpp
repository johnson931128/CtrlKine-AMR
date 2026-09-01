#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "localization/LocalizationTypes.hpp"

enum class OccupancyState {
    Unknown,
    Free,
    Occupied
};

struct SlamGridCoord {
    int col = 0;
    int row = 0;

    bool operator==(const SlamGridCoord& other) const {
        return col == other.col && row == other.row;
    }
};

struct SlamOccupancyGridConfig {
    double resolution = 20.0;
    double originX = -2000.0;
    double originY = -2000.0;
    std::size_t width = 200;
    std::size_t height = 200;
    double occupiedLogOddsIncrement = 0.85;
    double freeLogOddsIncrement = -0.40;
    double minimumLogOdds = -4.0;
    double maximumLogOdds = 4.0;
    double freeLogOddsThreshold = -0.20;
    double occupiedLogOddsThreshold = 0.20;
};

struct SlamMapStatistics {
    std::size_t unknownCells = 0;
    std::size_t freeCells = 0;
    std::size_t occupiedCells = 0;
    std::uint64_t revision = 0;
};

struct OccupancyIntegrationResult {
    bool integrated = false;
    std::size_t totalBeams = 0;
    std::size_t hitBeams = 0;
    std::size_t maxRangeBeams = 0;
    std::size_t invalidBeams = 0;
    std::size_t freeCells = 0;
    std::size_t occupiedCells = 0;
};

enum class SlamState {
    Uninitialized,
    Tracking,
    Lost
};

enum class ScanMatchReason {
    NotAttempted,
    Bootstrap,
    Accepted,
    InvalidScan,
    NoPhysicalHits,
    InsufficientMapSupport,
    PoorScore,
    LargeOdometry
};

struct CorrelativeScanMatcherConfig {
    double coarseLinearWindow = 40.0;
    double coarseLinearStep = 20.0;
    double coarseAngularWindow = 0.12;
    double coarseAngularStep = 0.04;
    double fineLinearWindow = 10.0;
    double fineLinearStep = 5.0;
    double fineAngularWindow = 0.02;
    double fineAngularStep = 0.01;
    std::size_t maximumBeams = 45;
    std::size_t minimumUsableBeams = 3;
    double minimumScore = 0.25;
    int scoreSearchRadiusCells = 2;
};

struct ScanMatchResult {
    bool attempted = false;
    bool accepted = false;
    ScanMatchReason reason = ScanMatchReason::NotAttempted;
    Pose2D predictedPose;
    Pose2D correctedPose;
    double score = 0.0;
    std::size_t selectedBeams = 0;
    std::size_t usedBeams = 0;
    std::size_t coarseCandidates = 0;
    std::size_t fineCandidates = 0;
    double correctionX = 0.0;
    double correctionY = 0.0;
    double correctionYaw = 0.0;
};

struct SlamFrontendConfig {
    SlamOccupancyGridConfig grid;
    CorrelativeScanMatcherConfig matcher;
    double maximumOdometryTranslation = 150.0;
    double maximumOdometryRotation = 0.75;
    std::size_t failuresBeforeLost = 3;
};

struct SlamUpdateResult {
    SlamState state = SlamState::Uninitialized;
    Pose2D pose;
    Pose2D predictedPose;
    bool poseValid = false;
    bool mapIntegrated = false;
    std::size_t consecutiveFailures = 0;
    std::size_t acceptedUpdates = 0;
    std::size_t rejectedUpdates = 0;
    ScanMatchResult match;
    OccupancyIntegrationResult integration;
};

inline std::string scanMatchReasonLabel(ScanMatchReason reason) {
    switch (reason) {
    case ScanMatchReason::Bootstrap: return "Bootstrap";
    case ScanMatchReason::Accepted: return "Accepted";
    case ScanMatchReason::InvalidScan: return "Invalid scan";
    case ScanMatchReason::NoPhysicalHits: return "No physical hits";
    case ScanMatchReason::InsufficientMapSupport: return "Insufficient map support";
    case ScanMatchReason::PoorScore: return "Poor score";
    case ScanMatchReason::LargeOdometry: return "Large odometry";
    case ScanMatchReason::NotAttempted:
    default: return "Not attempted";
    }
}
