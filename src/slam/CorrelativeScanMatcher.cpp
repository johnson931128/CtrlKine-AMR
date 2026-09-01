#include "slam/CorrelativeScanMatcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "sensors/LaserScanGeometry.hpp"
#include "slam/OccupancyGridMapper.hpp"

namespace {
struct LocalHit {
    double x = 0.0;
    double y = 0.0;
};

struct Candidate {
    double dx = 0.0;
    double dy = 0.0;
    double dyaw = 0.0;
    double score = 0.0;
    std::size_t used = 0;
};

bool validConfig(const CorrelativeScanMatcherConfig& config) {
    return std::isfinite(config.coarseLinearWindow) && config.coarseLinearWindow >= 0.0
        && std::isfinite(config.coarseLinearStep) && config.coarseLinearStep > 0.0
        && std::isfinite(config.coarseAngularWindow) && config.coarseAngularWindow >= 0.0
        && std::isfinite(config.coarseAngularStep) && config.coarseAngularStep > 0.0
        && std::isfinite(config.fineLinearWindow) && config.fineLinearWindow >= 0.0
        && std::isfinite(config.fineLinearStep) && config.fineLinearStep > 0.0
        && std::isfinite(config.fineAngularWindow) && config.fineAngularWindow >= 0.0
        && std::isfinite(config.fineAngularStep) && config.fineAngularStep > 0.0
        && config.maximumBeams > 0 && config.minimumUsableBeams > 0
        && config.minimumUsableBeams <= config.maximumBeams
        && std::isfinite(config.minimumScore)
        && config.minimumScore >= 0.0 && config.minimumScore <= 1.0
        && config.scoreSearchRadiusCells >= 0;
}

bool finitePose(const Pose2D& pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y)
        && std::isfinite(pose.heading);
}

std::vector<double> offsets(double window, double step) {
    const int halfSteps = static_cast<int>(std::floor(window / step + 1e-9));
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(2 * halfSteps + 1));
    for (int index = -halfSteps; index <= halfSteps; ++index) {
        values.push_back(index * step);
    }
    return values;
}

bool betterCandidate(
    const Candidate& candidate,
    const Candidate& best,
    const CorrelativeScanMatcherConfig& config
) {
    constexpr double epsilon = 1e-12;
    if (candidate.score > best.score + epsilon) {
        return true;
    }
    if (candidate.score + epsilon < best.score) {
        return false;
    }
    const double linearScale = std::min(config.coarseLinearStep, config.fineLinearStep);
    const double angularScale = std::min(config.coarseAngularStep, config.fineAngularStep);
    const double candidateNorm = std::pow(candidate.dx / linearScale, 2.0)
        + std::pow(candidate.dy / linearScale, 2.0)
        + std::pow(candidate.dyaw / angularScale, 2.0);
    const double bestNorm = std::pow(best.dx / linearScale, 2.0)
        + std::pow(best.dy / linearScale, 2.0)
        + std::pow(best.dyaw / angularScale, 2.0);
    if (candidateNorm + epsilon < bestNorm) {
        return true;
    }
    if (bestNorm + epsilon < candidateNorm) {
        return false;
    }
    if (std::abs(candidate.dyaw) + epsilon < std::abs(best.dyaw)) {
        return true;
    }
    if (std::abs(best.dyaw) + epsilon < std::abs(candidate.dyaw)) {
        return false;
    }
    if (candidate.dx != best.dx) return candidate.dx < best.dx;
    if (candidate.dy != best.dy) return candidate.dy < best.dy;
    return candidate.dyaw < best.dyaw;
}

Candidate scoreCandidate(
    const SlamOccupancyGrid& grid,
    const std::vector<LocalHit>& hits,
    const Pose2D& predictedPose,
    double dx,
    double dy,
    double dyaw,
    int radius
) {
    Candidate candidate;
    candidate.dx = dx;
    candidate.dy = dy;
    candidate.dyaw = dyaw;
    const double heading = normalizeLocalizationAngle(predictedPose.heading + dyaw);
    const double cosine = std::cos(heading);
    const double sine = std::sin(heading);
    double scoreSum = 0.0;
    for (const LocalHit& hit : hits) {
        const sf::Vector2f endpoint(
            predictedPose.position.x + static_cast<float>(dx + cosine * hit.x - sine * hit.y),
            predictedPose.position.y + static_cast<float>(dy + sine * hit.x + cosine * hit.y)
        );
        const double endpointScore = grid.occupiedScore(endpoint, radius);
        if (endpointScore > 0.0) {
            ++candidate.used;
        }
        scoreSum += endpointScore;
    }
    candidate.score = hits.empty() ? 0.0 : scoreSum / hits.size();
    return candidate;
}
}

CorrelativeScanMatcher::CorrelativeScanMatcher(const CorrelativeScanMatcherConfig& config)
    : m_config(config) {
    if (!validConfig(config)) {
        throw std::invalid_argument("Correlative scan-matcher configuration is invalid.");
    }
}

const CorrelativeScanMatcherConfig& CorrelativeScanMatcher::getConfig() const {
    return m_config;
}

ScanMatchResult CorrelativeScanMatcher::match(
    const SlamOccupancyGrid& grid,
    const LaserScan& scan,
    const Pose2D& predictedPose
) const {
    ScanMatchResult result;
    result.predictedPose = predictedPose;
    result.correctedPose = predictedPose;
    if (!finitePose(predictedPose) || !OccupancyGridMapper::isScanMetadataValid(scan)) {
        result.reason = ScanMatchReason::InvalidScan;
        return result;
    }

    std::vector<std::size_t> physicalHits;
    physicalHits.reserve(scan.ranges.size());
    for (std::size_t beam = 0; beam < scan.ranges.size(); ++beam) {
        if (isLaserReturnHit(scan, scan.ranges[beam])) {
            physicalHits.push_back(beam);
        }
    }
    if (physicalHits.empty()) {
        result.reason = ScanMatchReason::NoPhysicalHits;
        return result;
    }
    const std::size_t selectedCount = std::min(
        m_config.maximumBeams, physicalHits.size()
    );
    std::vector<std::size_t> selected;
    selected.reserve(selectedCount);
    for (std::size_t index = 0; index < selectedCount; ++index) {
        const std::size_t physicalIndex = isFullCircleScan(scan)
            ? index * physicalHits.size() / selectedCount
            : selectedCount == 1
                ? physicalHits.size() / 2
                : static_cast<std::size_t>(std::llround(
                    static_cast<double>(index) * (physicalHits.size() - 1)
                    / static_cast<double>(selectedCount - 1)
                ));
        selected.push_back(physicalHits[physicalIndex]);
    }
    result.selectedBeams = selected.size();
    std::vector<LocalHit> hits;
    hits.reserve(selected.size());
    const double endpointEpsilon = grid.getConfig().resolution * 1e-4;
    for (const std::size_t beam : selected) {
        const double range = scan.ranges[beam];
        const double angle = scan.sensorYawOffset + laserScanBeamAngle(scan, beam);
        const double extendedRange = range + endpointEpsilon;
        hits.push_back(LocalHit{
            scan.sensorOffsetX + extendedRange * std::cos(angle),
            scan.sensorOffsetY + extendedRange * std::sin(angle)
        });
    }
    result.attempted = true;

    Candidate best;
    best.score = -std::numeric_limits<double>::infinity();
    const auto coarseLinear = offsets(m_config.coarseLinearWindow, m_config.coarseLinearStep);
    const auto coarseAngular = offsets(m_config.coarseAngularWindow, m_config.coarseAngularStep);
    for (const double dx : coarseLinear) {
        for (const double dy : coarseLinear) {
            for (const double dyaw : coarseAngular) {
                const Candidate candidate = scoreCandidate(
                    grid, hits, predictedPose, dx, dy, dyaw,
                    m_config.scoreSearchRadiusCells
                );
                ++result.coarseCandidates;
                if (betterCandidate(candidate, best, m_config)) {
                    best = candidate;
                }
            }
        }
    }

    const auto fineLinear = offsets(m_config.fineLinearWindow, m_config.fineLinearStep);
    const auto fineAngular = offsets(m_config.fineAngularWindow, m_config.fineAngularStep);
    const Candidate coarseBest = best;
    for (const double dxOffset : fineLinear) {
        for (const double dyOffset : fineLinear) {
            for (const double yawOffset : fineAngular) {
                const Candidate candidate = scoreCandidate(
                    grid, hits, predictedPose,
                    coarseBest.dx + dxOffset,
                    coarseBest.dy + dyOffset,
                    coarseBest.dyaw + yawOffset,
                    m_config.scoreSearchRadiusCells
                );
                ++result.fineCandidates;
                if (betterCandidate(candidate, best, m_config)) {
                    best = candidate;
                }
            }
        }
    }

    result.score = std::max(0.0, best.score);
    result.usedBeams = best.used;
    result.correctionX = best.dx;
    result.correctionY = best.dy;
    result.correctionYaw = best.dyaw;
    result.correctedPose.position.x += static_cast<float>(best.dx);
    result.correctedPose.position.y += static_cast<float>(best.dy);
    result.correctedPose.heading = static_cast<float>(normalizeLocalizationAngle(
        result.correctedPose.heading + best.dyaw
    ));
    if (best.used < m_config.minimumUsableBeams) {
        result.reason = ScanMatchReason::InsufficientMapSupport;
    } else if (result.score < m_config.minimumScore) {
        result.reason = ScanMatchReason::PoorScore;
    } else {
        result.accepted = true;
        result.reason = ScanMatchReason::Accepted;
    }
    return result;
}
