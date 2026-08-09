#include "ParticleFilter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
        && config.zHit + config.zRand > 0.0
        && config.likelihoodMaxDistance > 0.0
        && std::isfinite(config.likelihoodMaxDistance)
        && config.maxBeams > 0
        && config.kldBinSizeX > 0.0 && std::isfinite(config.kldBinSizeX)
        && config.kldBinSizeY > 0.0 && std::isfinite(config.kldBinSizeY)
        && config.kldBinSizeYaw > 0.0 && std::isfinite(config.kldBinSizeYaw)
        && config.pfErr > 0.0 && std::isfinite(config.pfErr)
        && config.pfZ > 0.0 && std::isfinite(config.pfZ);
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
    const MapLikelihoodField& field
) {
    if (m_particles.empty() || !field.isValid() || scan.ranges.empty()
        || !std::isfinite(scan.angleMin) || !std::isfinite(scan.angleIncrement)
        || !std::isfinite(scan.minRange) || !std::isfinite(scan.maxRange)
        || scan.minRange < 0.0f || scan.minRange >= scan.maxRange) {
        return {};
    }

    const std::size_t selectedCount = std::min(m_config.maxBeams, scan.ranges.size());
    std::vector<std::size_t> selectedIndices;
    selectedIndices.reserve(selectedCount);
    if (selectedCount == 1) {
        selectedIndices.push_back(scan.ranges.size() / 2);
    } else {
        for (std::size_t selected = 0; selected < selectedCount; ++selected) {
            const double position = static_cast<double>(selected)
                * static_cast<double>(scan.ranges.size() - 1)
                / static_cast<double>(selectedCount - 1);
            selectedIndices.push_back(static_cast<std::size_t>(std::llround(position)));
        }
    }

    const bool hasUsableBeam = std::any_of(
        selectedIndices.begin(), selectedIndices.end(), [&](std::size_t scanIndex) {
            const double range = scan.ranges[scanIndex];
            return std::isfinite(range) && range >= scan.minRange && range < scan.maxRange;
        }
    );
    if (!hasUsableBeam) {
        return {};
    }
    normalizeWeights(m_particles);

    std::vector<double> logWeights(m_particles.size(), -std::numeric_limits<double>::infinity());
    double maximumLogWeight = -std::numeric_limits<double>::infinity();
    double qualitySum = 0.0;
    double qualityWeightSum = 0.0;

    for (std::size_t particleIndex = 0; particleIndex < m_particles.size(); ++particleIndex) {
        const Particle& particle = m_particles[particleIndex];
        double beamLogLikelihood = 0.0;
        std::size_t usedBeams = 0;

        for (const std::size_t scanIndex : selectedIndices) {
            const double range = scan.ranges[scanIndex];
            if (!std::isfinite(range) || range < scan.minRange || range >= scan.maxRange) {
                continue;
            }

            const double beamAngle = particle.pose.heading
                + scan.angleMin
                + scan.angleIncrement * static_cast<double>(scanIndex);
            const sf::Vector2f endpoint(
                particle.pose.position.x + static_cast<float>(range * std::cos(beamAngle)),
                particle.pose.position.y + static_cast<float>(range * std::sin(beamAngle))
            );
            const double distance = std::min(
                field.distanceAt(endpoint),
                m_config.likelihoodMaxDistance
            );
            const double hitProbability = std::exp(
                -0.5 * distance * distance / (m_config.sigmaHit * m_config.sigmaHit)
            );
            const double randomProbability = 1.0 / static_cast<double>(scan.maxRange);
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
        qualitySum += particle.weight * std::exp(averageBeamLogLikelihood);
        qualityWeightSum += particle.weight;
        const double priorLogWeight = std::log(std::max(kWeightFloor, particle.weight));
        logWeights[particleIndex] = priorLogWeight + beamLogLikelihood;
        maximumLogWeight = std::max(maximumLogWeight, logWeights[particleIndex]);
    }

    if (!std::isfinite(maximumLogWeight)) {
        return {};
    }

    for (std::size_t index = 0; index < m_particles.size(); ++index) {
        m_particles[index].weight = std::isfinite(logWeights[index])
            ? std::exp(logWeights[index] - maximumLogWeight)
            : 0.0;
    }
    normalizeWeights(m_particles);
    return SensorUpdateResult{
        true,
        qualityWeightSum > 0.0 ? qualitySum / qualityWeightSum : 0.0
    };
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
