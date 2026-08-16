#include "ParticleFilter.hpp"

#include "LaserScanGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>

namespace {
constexpr double kWeightFloor = 1e-300;

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

void validateConfig(const AmclConfig& config) {
    const bool valid = config.minParticles > 0
        && config.minParticles <= config.maxParticles
        && config.initialParticleCount >= config.minParticles
        && config.initialParticleCount <= config.maxParticles
        && config.alpha1 >= 0.0 && std::isfinite(config.alpha1)
        && config.alpha2 >= 0.0 && std::isfinite(config.alpha2)
        && config.alpha3 >= 0.0 && std::isfinite(config.alpha3)
        && config.alpha4 >= 0.0 && std::isfinite(config.alpha4)
        && config.alpha5 >= 0.0 && std::isfinite(config.alpha5)
        && config.initialStdDevX >= 0.0 && std::isfinite(config.initialStdDevX)
        && config.initialStdDevY >= 0.0 && std::isfinite(config.initialStdDevY)
        && config.initialStdDevYaw >= 0.0 && std::isfinite(config.initialStdDevYaw)
        && config.sigmaHit > 0.0 && std::isfinite(config.sigmaHit)
        && config.zHit >= 0.0 && std::isfinite(config.zHit)
        && config.zRand >= 0.0 && std::isfinite(config.zRand)
        && std::isfinite(config.zHit + config.zRand)
        && std::abs(config.zHit + config.zRand - 1.0) <= 1e-9
        && config.likelihoodMaxDistance > 0.0
        && std::isfinite(config.likelihoodMaxDistance)
        && config.maxBeams > 0
        && std::isfinite(config.beamSkipDistance) && config.beamSkipDistance >= 0.0
        && std::isfinite(config.beamSkipThreshold) && config.beamSkipThreshold >= 0.0
        && config.beamSkipThreshold <= 1.0
        && std::isfinite(config.beamSkipErrorThreshold)
        && config.beamSkipErrorThreshold >= 0.0 && config.beamSkipErrorThreshold <= 1.0
        && config.kldBinSizeX > 0.0 && std::isfinite(config.kldBinSizeX)
        && config.kldBinSizeY > 0.0 && std::isfinite(config.kldBinSizeY)
        && config.kldBinSizeYaw > 0.0 && std::isfinite(config.kldBinSizeYaw)
        && config.pfErr > 0.0 && std::isfinite(config.pfErr)
        && config.pfZ > 0.0 && std::isfinite(config.pfZ)
        && config.clusterBinSizeX > 0.0 && std::isfinite(config.clusterBinSizeX)
        && config.clusterBinSizeY > 0.0 && std::isfinite(config.clusterBinSizeY)
        && config.clusterBinSizeYaw > 0.0 && std::isfinite(config.clusterBinSizeYaw)
        && config.clusterMinimumBinWeightRatio > 0.0
        && config.clusterMinimumBinWeightRatio <= 1.0
        && std::isfinite(config.clusterMinimumBinWeightRatio)
        && config.significantClusterWeight >= 0.0 && config.significantClusterWeight <= 1.0
        && std::isfinite(config.significantClusterWeight)
        && config.dominantClusterWeight >= 0.0 && config.dominantClusterWeight <= 1.0
        && std::isfinite(config.dominantClusterWeight)
        && config.dominantToSecondRatio >= 1.0 && std::isfinite(config.dominantToSecondRatio)
        && config.dominantSwitchMargin >= 0.0 && config.dominantSwitchMargin <= 1.0
        && std::isfinite(config.dominantSwitchMargin)
        && config.minimumHeadingResultant >= 0.0 && config.minimumHeadingResultant <= 1.0
        && std::isfinite(config.minimumHeadingResultant)
        && config.recoveringProbabilityThreshold >= 0.0
        && config.recoveringProbabilityThreshold <= 1.0
        && std::isfinite(config.recoveringProbabilityThreshold)
        && config.navigationDominantWeight >= 0.0 && config.navigationDominantWeight <= 1.0
        && std::isfinite(config.navigationDominantWeight)
        && config.navigationPositionStdDev >= 0.0
        && std::isfinite(config.navigationPositionStdDev)
        && config.navigationHeadingStdDev >= 0.0
        && std::isfinite(config.navigationHeadingStdDev)
        && config.historyCapacity > 0;
    if (!valid) {
        throw std::invalid_argument("AMCL particle-filter configuration is invalid.");
    }
}
}

ParticleFilter::ParticleFilter(const AmclConfig& config)
    : m_config(config) {
    validateConfig(m_config);
}

void ParticleFilter::clear() {
    m_particles.clear();
}

bool ParticleFilter::initializeLocal(
    const Pose2D& mean,
    const MapData& mapData,
    std::mt19937& randomEngine
) {
    if (!finitePose(mean)) {
        return false;
    }

    std::normal_distribution<double> sampleX(
        mean.position.x,
        m_config.initialStdDevX > 0.0 ? m_config.initialStdDevX : 1.0
    );
    std::normal_distribution<double> sampleY(
        mean.position.y,
        m_config.initialStdDevY > 0.0 ? m_config.initialStdDevY : 1.0
    );
    std::normal_distribution<double> sampleYaw(
        mean.heading,
        m_config.initialStdDevYaw > 0.0 ? m_config.initialStdDevYaw : 1.0
    );
    std::vector<Particle> initialized;
    initialized.reserve(m_config.initialParticleCount);
    const std::size_t maximumAttempts = m_config.initialParticleCount * 200 + 1000;

    for (std::size_t attempt = 0;
         attempt < maximumAttempts && initialized.size() < m_config.initialParticleCount;
         ++attempt) {
        Pose2D pose;
        pose.position.x = static_cast<float>(
            m_config.initialStdDevX > 0.0 ? sampleX(randomEngine) : mean.position.x
        );
        pose.position.y = static_cast<float>(
            m_config.initialStdDevY > 0.0 ? sampleY(randomEngine) : mean.position.y
        );
        pose.heading = static_cast<float>(normalizeLocalizationAngle(
            m_config.initialStdDevYaw > 0.0 ? sampleYaw(randomEngine) : mean.heading
        ));
        if (isPoseFree(pose, mapData)) {
            initialized.push_back(Particle{pose, 0.0});
        }
    }

    if (initialized.size() != m_config.initialParticleCount) {
        return false;
    }
    const double uniformWeight = 1.0 / static_cast<double>(initialized.size());
    for (Particle& particle : initialized) {
        particle.weight = uniformWeight;
    }
    m_particles = std::move(initialized);
    return true;
}

bool ParticleFilter::initializeGlobal(
    const MapData& mapData,
    std::mt19937& randomEngine
) {
    std::vector<Particle> initialized;
    initialized.reserve(m_config.initialParticleCount);
    for (std::size_t index = 0; index < m_config.initialParticleCount; ++index) {
        Pose2D pose;
        if (!sampleFreePose(mapData, pose, randomEngine)) {
            return false;
        }
        initialized.push_back(Particle{pose, 0.0});
    }
    const double uniformWeight = 1.0 / static_cast<double>(initialized.size());
    for (Particle& particle : initialized) {
        particle.weight = uniformWeight;
    }
    m_particles = std::move(initialized);
    return true;
}

void ParticleFilter::motionUpdate(
    const OdometryDelta& odometry,
    std::mt19937& randomEngine
) {
    if (!odometry.valid || !std::isfinite(odometry.rotation1)
        || !std::isfinite(odometry.translation)
        || !std::isfinite(odometry.rotation2)) {
        return;
    }
    if (std::abs(odometry.rotation1) <= 1e-12
        && std::abs(odometry.translation) <= 1e-12
        && std::abs(odometry.rotation2) <= 1e-12) {
        return;
    }

    const double rotation1Variance =
        m_config.alpha1 * odometry.rotation1 * odometry.rotation1
        + m_config.alpha2 * odometry.translation * odometry.translation;
    const double translationVariance =
        m_config.alpha3 * odometry.translation * odometry.translation
        + m_config.alpha4 * (
            odometry.rotation1 * odometry.rotation1
            + odometry.rotation2 * odometry.rotation2
        );
    const double rotation2Variance =
        m_config.alpha1 * odometry.rotation2 * odometry.rotation2
        + m_config.alpha2 * odometry.translation * odometry.translation;
    const double lateralVariance =
        m_config.alpha5 * odometry.translation * odometry.translation;

    for (Particle& particle : m_particles) {
        const double sampledRotation1 = odometry.rotation1
            + gaussian(randomEngine, std::sqrt(std::max(0.0, rotation1Variance)));
        const double sampledTranslation = odometry.translation
            + gaussian(randomEngine, std::sqrt(std::max(0.0, translationVariance)));
        const double sampledRotation2 = odometry.rotation2
            + gaussian(randomEngine, std::sqrt(std::max(0.0, rotation2Variance)));
        const double sampledLateral = gaussian(
            randomEngine,
            std::sqrt(std::max(0.0, lateralVariance))
        );
        const double movementHeading = particle.pose.heading + sampledRotation1;

        particle.pose.position.x += static_cast<float>(
            sampledTranslation * std::cos(movementHeading)
            - sampledLateral * std::sin(movementHeading)
        );
        particle.pose.position.y += static_cast<float>(
            sampledTranslation * std::sin(movementHeading)
            + sampledLateral * std::cos(movementHeading)
        );
        particle.pose.heading = static_cast<float>(normalizeLocalizationAngle(
            particle.pose.heading + sampledRotation1 + sampledRotation2
        ));
    }
}

SensorUpdateResult ParticleFilter::sensorUpdate(
    const LaserScan& scan,
    const MapLikelihoodField& field,
    bool allowBeamSkipping
) {
    SensorUpdateResult result;
    result.totalBeams = scan.ranges.size();
    if (m_particles.empty() || !field.isValid() || scan.ranges.empty()
        || !std::isfinite(scan.angleMin) || !std::isfinite(scan.angleIncrement)
        || !std::isfinite(scan.minRange) || !std::isfinite(scan.maxRange)
        || !std::isfinite(scan.sensorOffsetX) || !std::isfinite(scan.sensorOffsetY)
        || !std::isfinite(scan.sensorYawOffset)
        || scan.minRange < 0.0f || scan.minRange >= scan.maxRange) {
        return result;
    }

    const std::vector<std::size_t> selectedIndices = selectEvenlySpacedBeamIndices(
        scan, m_config.maxBeams
    );
    result.selectedBeams = selectedIndices.size();
    std::vector<std::size_t> candidateIndices;
    candidateIndices.reserve(selectedIndices.size());
    for (const std::size_t scanIndex : selectedIndices) {
        const double range = scan.ranges[scanIndex];
        if (!std::isfinite(range) || range < scan.minRange) {
            ++result.invalidBeams;
        } else if (range >= scan.maxRange) {
            ++result.maxRangeBeams;
        } else {
            candidateIndices.push_back(scanIndex);
        }
    }
    if (candidateIndices.empty()) {
        return result;
    }
    normalizeWeights(m_particles);

    std::vector<std::vector<double>> endpointDistances(
        candidateIndices.size(), std::vector<double>(m_particles.size(), m_config.likelihoodMaxDistance)
    );
    std::vector<bool> validParticle(m_particles.size(), true);
    std::size_t validParticleCount = 0;
    for (std::size_t particleIndex = 0; particleIndex < m_particles.size(); ++particleIndex) {
        const Pose2D& pose = m_particles[particleIndex].pose;
        const double cosine = std::cos(pose.heading);
        const double sine = std::sin(pose.heading);
        const sf::Vector2f sensorOrigin(
            pose.position.x + static_cast<float>(cosine * scan.sensorOffsetX - sine * scan.sensorOffsetY),
            pose.position.y + static_cast<float>(sine * scan.sensorOffsetX + cosine * scan.sensorOffsetY)
        );
        validParticle[particleIndex] = field.isFree(pose.position)
            && field.isFree(sensorOrigin);
        validParticleCount += validParticle[particleIndex] ? 1 : 0;
    }
    if (validParticleCount == 0) {
        result.usedBeams = candidateIndices.size();
        result.updated = true;
        return result;
    }
    for (std::size_t candidate = 0; candidate < candidateIndices.size(); ++candidate) {
        const std::size_t scanIndex = candidateIndices[candidate];
        const double range = scan.ranges[scanIndex];
        for (std::size_t particleIndex = 0; particleIndex < m_particles.size(); ++particleIndex) {
            if (!validParticle[particleIndex]) {
                continue;
            }
            const Pose2D& pose = m_particles[particleIndex].pose;
            const double cosine = std::cos(pose.heading);
            const double sine = std::sin(pose.heading);
            const sf::Vector2f sensorOrigin(
                pose.position.x + static_cast<float>(cosine * scan.sensorOffsetX - sine * scan.sensorOffsetY),
                pose.position.y + static_cast<float>(sine * scan.sensorOffsetX + cosine * scan.sensorOffsetY)
            );
            const double beamAngle = pose.heading + scan.sensorYawOffset
                + laserScanBeamAngle(scan, scanIndex);
            const sf::Vector2f endpoint(
                sensorOrigin.x + static_cast<float>(range * std::cos(beamAngle)),
                sensorOrigin.y + static_cast<float>(range * std::sin(beamAngle))
            );
            endpointDistances[candidate][particleIndex] = std::min(
                field.distanceAt(endpoint), m_config.likelihoodMaxDistance
            );
        }
    }

    std::vector<bool> useCandidate(candidateIndices.size(), true);
    if (m_config.doBeamSkip && allowBeamSkipping) {
        for (std::size_t candidate = 0; candidate < candidateIndices.size(); ++candidate) {
            std::size_t agreeingParticles = 0;
            for (std::size_t particleIndex = 0; particleIndex < m_particles.size(); ++particleIndex) {
                if (validParticle[particleIndex]
                    && endpointDistances[candidate][particleIndex] <= m_config.beamSkipDistance) {
                    ++agreeingParticles;
                }
            }
            const double agreeingFraction = static_cast<double>(agreeingParticles)
                / static_cast<double>(validParticleCount);
            if (agreeingFraction < m_config.beamSkipThreshold) {
                useCandidate[candidate] = false;
                ++result.skippedBeams;
            }
        }
        const double skippedRatio = static_cast<double>(result.skippedBeams)
            / static_cast<double>(candidateIndices.size());
        if (skippedRatio > m_config.beamSkipErrorThreshold) {
            std::fill(useCandidate.begin(), useCandidate.end(), true);
            result.skippedBeams = 0;
            result.beamSkipFallback = true;
        }
    }
    result.usedBeams = candidateIndices.size() - result.skippedBeams;
    if (result.usedBeams == 0) {
        return result;
    }

    std::vector<double> logWeights(m_particles.size(), -std::numeric_limits<double>::infinity());
    double maximumLogWeight = -std::numeric_limits<double>::infinity();
    double qualitySum = 0.0;
    double qualityWeightSum = 0.0;
    double maximumAverageLikelihood = 0.0;

    for (std::size_t particleIndex = 0; particleIndex < m_particles.size(); ++particleIndex) {
        const Particle& particle = m_particles[particleIndex];
        if (!validParticle[particleIndex]) {
            continue;
        }
        double beamLogLikelihood = 0.0;
        std::size_t usedBeams = 0;

        for (std::size_t candidate = 0; candidate < candidateIndices.size(); ++candidate) {
            if (!useCandidate[candidate]) {
                continue;
            }
            const double distance = endpointDistances[candidate][particleIndex];
            const double hitProbability = std::exp(
                -0.5 * distance * distance / (m_config.sigmaHit * m_config.sigmaHit)
            );
            const double randomProbability = 1.0
                / static_cast<double>(scan.maxRange - scan.minRange);
            const double likelihood = std::max(
                kWeightFloor,
                m_config.zHit * hitProbability + m_config.zRand * randomProbability
            );
            beamLogLikelihood += std::log(likelihood);
            ++usedBeams;
        }

        if (usedBeams == 0) {
            continue;
        }

        const double averageBeamLogLikelihood = beamLogLikelihood
            / static_cast<double>(usedBeams);
        const double averageLikelihood = std::exp(averageBeamLogLikelihood);
        qualitySum += particle.weight * averageLikelihood;
        qualityWeightSum += particle.weight;
        maximumAverageLikelihood = std::max(maximumAverageLikelihood, averageLikelihood);
        const double priorLogWeight = std::log(std::max(kWeightFloor, particle.weight));
        logWeights[particleIndex] = priorLogWeight + beamLogLikelihood;
        maximumLogWeight = std::max(maximumLogWeight, logWeights[particleIndex]);
    }

    if (!std::isfinite(maximumLogWeight)) {
        return result;
    }

    for (std::size_t index = 0; index < m_particles.size(); ++index) {
        m_particles[index].weight = std::isfinite(logWeights[index])
            ? std::exp(logWeights[index] - maximumLogWeight)
            : 0.0;
    }
    normalizeWeights(m_particles);
    result.updated = true;
    result.observationQuality = qualityWeightSum > 0.0 ? qualitySum / qualityWeightSum : 0.0;
    result.likelihoodContrast = result.observationQuality > kWeightFloor
        ? maximumAverageLikelihood / result.observationQuality
        : 0.0;
    return result;
}

void ParticleFilter::adaptiveResample(
    const MapData& mapData,
    double randomInjectionProbability,
    std::mt19937& randomEngine
) {
    if (m_particles.empty()) {
        return;
    }
    normalizeWeights(m_particles);
    const double safeProbability = std::clamp(randomInjectionProbability, 0.0, 1.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    std::vector<double> cumulativeWeights;
    cumulativeWeights.reserve(m_particles.size());
    double cumulative = 0.0;
    for (const Particle& particle : m_particles) {
        cumulative += particle.weight;
        cumulativeWeights.push_back(cumulative);
    }
    cumulativeWeights.back() = 1.0;

    auto drawWeightedParticle = [&]() {
        const double target = unit(randomEngine);
        const auto found = std::lower_bound(
            cumulativeWeights.begin(), cumulativeWeights.end(), target
        );
        const std::size_t index = found == cumulativeWeights.end()
            ? m_particles.size() - 1
            : static_cast<std::size_t>(found - cumulativeWeights.begin());
        return m_particles[index];
    };
    auto particleBin = [&](const Pose2D& pose) {
        return std::tuple<int, int, int>{
            static_cast<int>(std::floor(pose.position.x / m_config.kldBinSizeX)),
            static_cast<int>(std::floor(pose.position.y / m_config.kldBinSizeY)),
            static_cast<int>(std::floor(
                (normalizeLocalizationAngle(pose.heading) + kLocalizationPi)
                / m_config.kldBinSizeYaw
            ))
        };
    };

    std::vector<Particle> resampled;
    resampled.reserve(m_config.maxParticles);
    std::set<std::tuple<int, int, int>> occupiedBins;
    std::size_t requiredCount = m_config.minParticles;
    while (resampled.size() < m_config.maxParticles
        && (resampled.size() < m_config.minParticles || resampled.size() < requiredCount)) {
        Particle particle = drawWeightedParticle();
        if (safeProbability > 0.0 && unit(randomEngine) < safeProbability) {
            Pose2D randomPose;
            if (sampleFreePose(mapData, randomPose, randomEngine)) {
                particle.pose = randomPose;
            }
        }
        particle.weight = 0.0;
        resampled.push_back(particle);
        if (occupiedBins.insert(particleBin(particle.pose)).second) {
            requiredCount = requiredKldSamples(occupiedBins.size(), m_config);
        }
    }

    const double uniformWeight = 1.0 / static_cast<double>(resampled.size());
    for (Particle& particle : resampled) {
        particle.weight = uniformWeight;
    }
    m_particles = std::move(resampled);
}

const std::vector<Particle>& ParticleFilter::getParticles() const {
    return m_particles;
}

LocalizationEstimate ParticleFilter::estimate() const {
    return estimateParticles(m_particles);
}

bool ParticleFilter::normalizeWeights(std::vector<Particle>& particles) {
    if (particles.empty()) {
        return false;
    }
    double total = 0.0;
    for (Particle& particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
            particle.weight = 0.0;
        }
        total += particle.weight;
    }
    if (!std::isfinite(total) || total <= std::numeric_limits<double>::min()) {
        const double uniformWeight = 1.0 / static_cast<double>(particles.size());
        for (Particle& particle : particles) {
            particle.weight = uniformWeight;
        }
        return false;
    }
    for (Particle& particle : particles) {
        particle.weight /= total;
    }
    return true;
}

double ParticleFilter::effectiveSampleSize(const std::vector<Particle>& particles) {
    double squaredWeightSum = 0.0;
    for (const Particle& particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
            return 0.0;
        }
        squaredWeightSum += particle.weight * particle.weight;
    }
    return squaredWeightSum > 0.0 && std::isfinite(squaredWeightSum)
        ? 1.0 / squaredWeightSum
        : 0.0;
}

LocalizationEstimate ParticleFilter::estimateParticles(
    const std::vector<Particle>& particles
) {
    LocalizationEstimate estimate;
    if (particles.empty()) {
        return estimate;
    }

    double weightSum = 0.0;
    double meanX = 0.0;
    double meanY = 0.0;
    double cosineSum = 0.0;
    double sineSum = 0.0;
    for (const Particle& particle : particles) {
        if (!finitePose(particle.pose) || !std::isfinite(particle.weight)
            || particle.weight < 0.0) {
            return estimate;
        }
        weightSum += particle.weight;
        meanX += particle.weight * particle.pose.position.x;
        meanY += particle.weight * particle.pose.position.y;
        cosineSum += particle.weight * std::cos(particle.pose.heading);
        sineSum += particle.weight * std::sin(particle.pose.heading);
    }
    if (!std::isfinite(weightSum) || weightSum <= std::numeric_limits<double>::min()) {
        return estimate;
    }

    meanX /= weightSum;
    meanY /= weightSum;
    const double meanYaw = std::atan2(sineSum / weightSum, cosineSum / weightSum);
    double covariance[3][3]{};
    for (const Particle& particle : particles) {
        const double normalizedWeight = particle.weight / weightSum;
        const double residual[3] = {
            particle.pose.position.x - meanX,
            particle.pose.position.y - meanY,
            normalizeLocalizationAngle(particle.pose.heading - meanYaw)
        };
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                covariance[row][column] += normalizedWeight
                    * residual[row] * residual[column];
            }
        }
    }

    estimate.pose.position = sf::Vector2f(
        static_cast<float>(meanX),
        static_cast<float>(meanY)
    );
    estimate.pose.heading = static_cast<float>(normalizeLocalizationAngle(meanYaw));
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            const double value = covariance[row][column];
            estimate.covariance.values[static_cast<std::size_t>(row * 3 + column)] =
                std::abs(value) < 1e-15 ? 0.0 : value;
        }
    }
    estimate.valid = true;
    estimate.particleCount = particles.size();
    estimate.effectiveSampleSize = effectiveSampleSize(particles);
    return estimate;
}

std::vector<ParticleCluster> ParticleFilter::clusterParticles(
    const std::vector<Particle>& particles,
    const AmclConfig& config
) {
    using Bin = std::tuple<int, int, int>;
    if (particles.empty()) {
        return {};
    }
    double totalWeight = 0.0;
    for (const Particle& particle : particles) {
        if (!finitePose(particle.pose) || !std::isfinite(particle.weight)
            || particle.weight < 0.0) {
            return {};
        }
        totalWeight += particle.weight;
    }
    if (!std::isfinite(totalWeight) || totalWeight <= std::numeric_limits<double>::min()) {
        return {};
    }

    const int yawBinCount = std::max(1, static_cast<int>(std::ceil(
        2.0 * kLocalizationPi / config.clusterBinSizeYaw
    )));
    auto binFor = [&](const Pose2D& pose) {
        int yawBin = static_cast<int>(std::floor(
            (normalizeLocalizationAngle(pose.heading) + kLocalizationPi)
            / config.clusterBinSizeYaw
        ));
        yawBin = std::clamp(yawBin, 0, yawBinCount - 1);
        return Bin{
            static_cast<int>(std::floor(pose.position.x / config.clusterBinSizeX)),
            static_cast<int>(std::floor(pose.position.y / config.clusterBinSizeY)),
            yawBin
        };
    };

    std::map<Bin, std::vector<std::size_t>> allBins;
    std::map<Bin, double> binWeights;
    for (std::size_t index = 0; index < particles.size(); ++index) {
        const Bin bin = binFor(particles[index].pose);
        allBins[bin].push_back(index);
        binWeights[bin] += particles[index].weight / totalWeight;
    }
    double maximumBinWeight = 0.0;
    for (const auto& [bin, weight] : binWeights) {
        (void)bin;
        maximumBinWeight = std::max(maximumBinWeight, weight);
    }
    const double connectivityThreshold = maximumBinWeight
        * config.clusterMinimumBinWeightRatio;
    std::map<Bin, std::vector<std::size_t>> bins;
    for (const auto& [bin, indices] : allBins) {
        if (binWeights[bin] + 1e-15 >= connectivityThreshold) {
            bins.emplace(bin, indices);
        }
    }

    std::set<Bin> visited;
    std::vector<ParticleCluster> clusters;
    for (const auto& [startBin, ignored] : bins) {
        (void)ignored;
        if (!visited.insert(startBin).second) {
            continue;
        }
        std::queue<Bin> pending;
        pending.push(startBin);
        std::vector<std::size_t> indices;
        while (!pending.empty()) {
            const Bin current = pending.front();
            pending.pop();
            const auto found = bins.find(current);
            if (found == bins.end()) {
                continue;
            }
            indices.insert(indices.end(), found->second.begin(), found->second.end());
            const auto [x, y, yaw] = current;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dyaw = -1; dyaw <= 1; ++dyaw) {
                        if (dx == 0 && dy == 0 && dyaw == 0) {
                            continue;
                        }
                        int neighborYaw = (yaw + dyaw) % yawBinCount;
                        if (neighborYaw < 0) {
                            neighborYaw += yawBinCount;
                        }
                        const Bin neighbor{x + dx, y + dy, neighborYaw};
                        if (bins.find(neighbor) != bins.end()
                            && visited.insert(neighbor).second) {
                            pending.push(neighbor);
                        }
                    }
                }
            }
        }

        std::vector<Particle> members;
        members.reserve(indices.size());
        double clusterWeight = 0.0;
        float minX = std::numeric_limits<float>::infinity();
        float minY = std::numeric_limits<float>::infinity();
        float maxX = -std::numeric_limits<float>::infinity();
        float maxY = -std::numeric_limits<float>::infinity();
        for (const std::size_t index : indices) {
            members.push_back(particles[index]);
            clusterWeight += particles[index].weight;
            minX = std::min(minX, particles[index].pose.position.x);
            minY = std::min(minY, particles[index].pose.position.y);
            maxX = std::max(maxX, particles[index].pose.position.x);
            maxY = std::max(maxY, particles[index].pose.position.y);
        }
        const LocalizationEstimate clusterEstimate = estimateParticles(members);
        if (!clusterEstimate.valid) {
            continue;
        }
        ParticleCluster cluster;
        cluster.particleCount = members.size();
        cluster.weight = clusterWeight / totalWeight;
        cluster.pose = clusterEstimate.pose;
        cluster.covariance = clusterEstimate.covariance;
        cluster.spatialExtent = sf::FloatRect(
            sf::Vector2f(minX, minY), sf::Vector2f(maxX - minX, maxY - minY)
        );
        double cosine = 0.0;
        double sine = 0.0;
        for (const Particle& member : members) {
            const double normalizedWeight = member.weight / clusterWeight;
            cosine += normalizedWeight * std::cos(member.pose.heading);
            sine += normalizedWeight * std::sin(member.pose.heading);
            cluster.headingExtent = std::max(
                cluster.headingExtent,
                std::abs(normalizeLocalizationAngle(member.pose.heading - cluster.pose.heading))
            );
        }
        cluster.headingResultant = std::hypot(cosine, sine);
        clusters.push_back(cluster);
    }

    std::stable_sort(clusters.begin(), clusters.end(), [](const ParticleCluster& first, const ParticleCluster& second) {
        if (std::abs(first.weight - second.weight) > 1e-12) {
            return first.weight > second.weight;
        }
        if (first.particleCount != second.particleCount) {
            return first.particleCount > second.particleCount;
        }
        if (first.pose.position.x != second.pose.position.x) {
            return first.pose.position.x < second.pose.position.x;
        }
        if (first.pose.position.y != second.pose.position.y) {
            return first.pose.position.y < second.pose.position.y;
        }
        return first.pose.heading < second.pose.heading;
    });
    return clusters;
}

double ParticleFilter::particleEntropy(const std::vector<Particle>& particles) {
    if (particles.size() <= 1) {
        return 0.0;
    }
    double total = 0.0;
    for (const Particle& particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
            return 0.0;
        }
        total += particle.weight;
    }
    if (total <= std::numeric_limits<double>::min() || !std::isfinite(total)) {
        return 0.0;
    }
    double entropy = 0.0;
    for (const Particle& particle : particles) {
        const double probability = particle.weight / total;
        if (probability > 0.0) {
            entropy -= probability * std::log(probability);
        }
    }
    return entropy / std::log(static_cast<double>(particles.size()));
}

std::size_t ParticleFilter::requiredKldSamples(
    std::size_t occupiedBinCount,
    const AmclConfig& config
) {
    if (occupiedBinCount <= 1) {
        return config.minParticles;
    }
    const double degreesOfFreedom = static_cast<double>(occupiedBinCount - 1);
    const double correction = 1.0
        - 2.0 / (9.0 * degreesOfFreedom)
        + config.pfZ * std::sqrt(2.0 / (9.0 * degreesOfFreedom));
    const double required = degreesOfFreedom
        / (2.0 * config.pfErr)
        * correction * correction * correction;
    if (!std::isfinite(required)) {
        return config.maxParticles;
    }
    const std::size_t rounded = required <= 0.0
        ? config.minParticles
        : static_cast<std::size_t>(std::ceil(required));
    return std::clamp(rounded, config.minParticles, config.maxParticles);
}

std::vector<Particle> ParticleFilter::systematicResample(
    const std::vector<Particle>& particles,
    std::size_t outputCount,
    std::mt19937& randomEngine
) {
    if (particles.empty() || outputCount == 0) {
        return {};
    }
    std::vector<Particle> normalized = particles;
    normalizeWeights(normalized);
    std::vector<Particle> result;
    result.reserve(outputCount);
    const double step = 1.0 / static_cast<double>(outputCount);
    const double start = std::uniform_real_distribution<double>(0.0, step)(randomEngine);
    std::size_t sourceIndex = 0;
    double cumulative = normalized.front().weight;
    for (std::size_t outputIndex = 0; outputIndex < outputCount; ++outputIndex) {
        const double target = start + static_cast<double>(outputIndex) * step;
        while (target > cumulative && sourceIndex + 1 < normalized.size()) {
            ++sourceIndex;
            cumulative += normalized[sourceIndex].weight;
        }
        Particle sampled = normalized[sourceIndex];
        sampled.weight = step;
        result.push_back(sampled);
    }
    return result;
}

bool ParticleFilter::sampleFreePose(
    const MapData& mapData,
    Pose2D& pose,
    std::mt19937& randomEngine
) const {
    const sf::FloatRect& boundary = mapData.getWorldBoundary();
    if (!std::isfinite(boundary.position.x) || !std::isfinite(boundary.position.y)
        || !std::isfinite(boundary.size.x) || !std::isfinite(boundary.size.y)
        || boundary.size.x <= 0.0f || boundary.size.y <= 0.0f) {
        return false;
    }
    std::uniform_real_distribution<double> sampleX(
        boundary.position.x,
        boundary.position.x + boundary.size.x
    );
    std::uniform_real_distribution<double> sampleY(
        boundary.position.y,
        boundary.position.y + boundary.size.y
    );
    std::uniform_real_distribution<double> sampleYaw(-kLocalizationPi, kLocalizationPi);
    for (std::size_t attempt = 0; attempt < 10000; ++attempt) {
        Pose2D candidate;
        candidate.position = sf::Vector2f(
            static_cast<float>(sampleX(randomEngine)),
            static_cast<float>(sampleY(randomEngine))
        );
        candidate.heading = static_cast<float>(sampleYaw(randomEngine));
        if (isPoseFree(candidate, mapData)) {
            pose = candidate;
            return true;
        }
    }
    return false;
}

bool ParticleFilter::isPoseFree(const Pose2D& pose, const MapData& mapData) const {
    return finitePose(pose)
        && mapData.containsWorldPoint(pose.position)
        && !mapData.isObstacleAt(pose.position);
}
