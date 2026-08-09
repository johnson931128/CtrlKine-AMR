#include "AmclLocalizer.hpp"
#include "LidarSimulator.hpp"
#include "MapLikelihoodField.hpp"
#include "ParticleFilter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "TestSupport.hpp"

namespace {
bool near(double first, double second, double tolerance = 0.001) {
    return std::abs(first - second) <= tolerance;
}

MapData makeAsymmetricMap() {
    MapData map(50.0f);
    map.setWorldBoundary(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(1000.0f, 800.0f)));
    for (int row = 1; row <= 6; ++row) {
        map.addObstacle(GridCoord{6, row});
    }
    for (int col = 7; col <= 15; ++col) {
        map.addObstacle(GridCoord{col, 10});
    }
    map.addObstacle(GridCoord{14, 2});
    map.addObstacle(GridCoord{16, 4});
    map.addObstacle(GridCoord{4, 12});
    return map;
}

AmclConfig exactConfig(std::size_t count = 100) {
    AmclConfig config;
    config.minParticles = count;
    config.maxParticles = count;
    config.initialParticleCount = count;
    config.initialStdDevX = 0.0;
    config.initialStdDevY = 0.0;
    config.initialStdDevYaw = 0.0;
    config.alpha1 = 0.0;
    config.alpha2 = 0.0;
    config.alpha3 = 0.0;
    config.alpha4 = 0.0;
    config.alpha5 = 0.0;
    config.maxBeams = 31;
    config.sigmaHit = 20.0;
    return config;
}

LidarSimulator deterministicLidar() {
    LidarConfig config;
    config.beamCount = 61;
    config.fieldOfView = 1.5 * kLocalizationPi;
    config.minRange = 1.0;
    config.maxRange = 900.0;
    config.rangeNoiseStdDev = 0.0;
    return LidarSimulator(config);
}
}

int main() {
    TestSuite suite;

    runTest(suite, "PF-001", "normalization produces a finite unit weight sum", [] {
        std::vector<Particle> particles = {
            Particle{Pose2D{}, 1.0},
            Particle{Pose2D{}, 3.0}
        };
        const bool normal = ParticleFilter::normalizeWeights(particles);
        return normal && near(particles[0].weight, 0.25)
            && near(particles[1].weight, 0.75)
            && near(particles[0].weight + particles[1].weight, 1.0);
    });

    runTest(suite, "PF-002", "zero and non-finite weights safely fall back to uniform", [] {
        std::vector<Particle> particles = {
            Particle{Pose2D{}, 0.0},
            Particle{Pose2D{}, std::numeric_limits<double>::infinity()},
            Particle{Pose2D{}, -1.0}
        };
        const bool normal = ParticleFilter::normalizeWeights(particles);
        return !normal && near(particles[0].weight, 1.0 / 3.0)
            && near(particles[1].weight, 1.0 / 3.0)
            && near(particles[2].weight, 1.0 / 3.0);
    });

    runTest(suite, "PF-003", "effective sample size follows normalized weights", [] {
        const std::vector<Particle> uniform = {
            Particle{Pose2D{}, 0.5}, Particle{Pose2D{}, 0.5}
        };
        const std::vector<Particle> skewed = {
            Particle{Pose2D{}, 0.9}, Particle{Pose2D{}, 0.1}
        };
        return near(ParticleFilter::effectiveSampleSize(uniform), 2.0)
            && near(ParticleFilter::effectiveSampleSize(skewed), 1.0 / 0.82);
    });

    runTest(suite, "PF-004", "systematic resampling replicates a dominant hypothesis in linear time", [] {
        std::vector<Particle> particles;
        particles.push_back(Particle{Pose2D{sf::Vector2f(1.0f, 0.0f), 0.0f}, 0.95});
        for (int index = 0; index < 5; ++index) {
            particles.push_back(Particle{
                Pose2D{sf::Vector2f(static_cast<float>(index + 2), 0.0f), 0.0f},
                0.01
            });
        }
        std::mt19937 randomEngine(42);
        const std::vector<Particle> resampled = ParticleFilter::systematicResample(
            particles, 100, randomEngine
        );
        const std::size_t dominantCount = static_cast<std::size_t>(std::count_if(
            resampled.begin(), resampled.end(), [](const Particle& particle) {
                return particle.pose.position.x == 1.0f;
            }
        ));
        return resampled.size() == 100 && dominantCount >= 94
            && std::all_of(resampled.begin(), resampled.end(), [](const Particle& particle) {
                return near(particle.weight, 0.01);
            });
    });

    runTest(suite, "KLD-001", "KLD required count obeys min, diversity growth, and max bounds", [] {
        AmclConfig config;
        config.minParticles = 100;
        config.maxParticles = 1000;
        config.initialParticleCount = 500;
        const std::size_t concentrated = ParticleFilter::requiredKldSamples(1, config);
        const std::size_t moderate = ParticleFilter::requiredKldSamples(10, config);
        const std::size_t broad = ParticleFilter::requiredKldSamples(10000, config);
        return concentrated == 100 && moderate > concentrated && moderate < 1000
            && broad == 1000;
    });

    runTest(suite, "KLD-002", "adaptive resampling uses fewer particles for concentrated belief", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config;
        config.minParticles = 100;
        config.maxParticles = 800;
        config.initialParticleCount = 400;
        config.initialStdDevX = 0.0;
        config.initialStdDevY = 0.0;
        config.initialStdDevYaw = 0.0;
        ParticleFilter concentrated(config);
        ParticleFilter broad(config);
        ParticleFilter broadTwin(config);
        std::mt19937 concentratedEngine(7);
        std::mt19937 broadEngine(8);
        std::mt19937 broadTwinEngine(8);
        concentrated.initializeLocal(
            Pose2D{sf::Vector2f(150.0f, 150.0f), 0.0f}, map, concentratedEngine
        );
        broad.initializeGlobal(map, broadEngine);
        broadTwin.initializeGlobal(map, broadTwinEngine);
        concentrated.adaptiveResample(map, 0.0, concentratedEngine);
        broad.adaptiveResample(map, 0.0, broadEngine);
        broadTwin.adaptiveResample(map, 0.0, broadTwinEngine);
        const bool deterministic = broad.getParticles().size() == broadTwin.getParticles().size()
            && std::equal(
                broad.getParticles().begin(), broad.getParticles().end(),
                broadTwin.getParticles().begin(), [](const Particle& first, const Particle& second) {
                    return near(first.pose.position.x, second.pose.position.x, 1e-12)
                        && near(first.pose.position.y, second.pose.position.y, 1e-12)
                        && near(first.pose.heading, second.pose.heading, 1e-12);
                }
            );
        return concentrated.getParticles().size() == config.minParticles
            && broad.getParticles().size() > concentrated.getParticles().size()
            && broad.getParticles().size() <= config.maxParticles
            && deterministic;
    });

    runTest(suite, "EST-001", "weighted position and circular heading mean are correct", [] {
        const std::vector<Particle> particles = {
            Particle{Pose2D{sf::Vector2f(0.0f, 0.0f), static_cast<float>(kLocalizationPi - 0.1)}, 0.5},
            Particle{Pose2D{sf::Vector2f(10.0f, 20.0f), static_cast<float>(-kLocalizationPi + 0.1)}, 0.5}
        };
        const LocalizationEstimate estimate = ParticleFilter::estimateParticles(particles);
        const bool covarianceFinite = std::all_of(
            estimate.covariance.values.begin(), estimate.covariance.values.end(),
            [](double value) { return std::isfinite(value); }
        );
        return estimate.valid && near(estimate.pose.position.x, 5.0)
            && near(estimate.pose.position.y, 10.0)
            && near(std::abs(estimate.pose.heading), kLocalizationPi, 0.001)
            && near(estimate.covariance.xx(), 25.0)
            && near(estimate.covariance.yy(), 100.0)
            && estimate.covariance.yawYaw() >= 0.0
            && estimate.covariance.xx() >= 0.0
            && estimate.covariance.yy() >= 0.0
            && covarianceFinite;
    });

    runTest(suite, "MOTION-001", "zero motion is stable and noise-free forward motion propagates particles", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(50);
        ParticleFilter filter(config);
        std::mt19937 randomEngine(10);
        if (!filter.initializeLocal(
                Pose2D{sf::Vector2f(150.0f, 150.0f), 0.0f}, map, randomEngine)) {
            return false;
        }
        filter.motionUpdate(OdometryDelta{}, randomEngine);
        const float before = filter.getParticles().front().pose.position.x;
        filter.motionUpdate(OdometryDelta{0.0, 50.0, 0.0, true}, randomEngine);
        return near(before, 150.0)
            && std::all_of(filter.getParticles().begin(), filter.getParticles().end(), [](const Particle& particle) {
                return near(particle.pose.position.x, 200.0)
                    && near(particle.pose.position.y, 150.0)
                    && near(particle.pose.heading, 0.0);
            });
    });

    runTest(suite, "MOTION-002", "in-place rotation normalizes every particle heading", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(30);
        ParticleFilter filter(config);
        std::mt19937 randomEngine(11);
        filter.initializeLocal(
            Pose2D{sf::Vector2f(150.0f, 150.0f), 3.0f}, map, randomEngine
        );
        filter.motionUpdate(OdometryDelta{0.0, 0.0, 1.0, true}, randomEngine);
        return std::all_of(filter.getParticles().begin(), filter.getParticles().end(), [](const Particle& particle) {
            return particle.pose.heading > -kLocalizationPi
                && particle.pose.heading <= kLocalizationPi
                && near(particle.pose.heading, normalizeLocalizationAngle(4.0), 1e-5);
        });
    });

    runTest(suite, "MOTION-003", "seeded motion noise creates deterministic nonzero spread", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(200);
        config.alpha3 = 0.01;
        ParticleFilter first(config);
        ParticleFilter second(config);
        std::mt19937 firstEngine(123);
        std::mt19937 secondEngine(123);
        const Pose2D start{sf::Vector2f(150.0f, 150.0f), 0.0f};
        first.initializeLocal(start, map, firstEngine);
        second.initializeLocal(start, map, secondEngine);
        const OdometryDelta motion{0.0, 50.0, 0.0, true};
        first.motionUpdate(motion, firstEngine);
        second.motionUpdate(motion, secondEngine);
        double meanX = 0.0;
        for (std::size_t index = 0; index < first.getParticles().size(); ++index) {
            if (!near(
                    first.getParticles()[index].pose.position.x,
                    second.getParticles()[index].pose.position.x,
                    1e-12)) {
                return false;
            }
            meanX += first.getParticles()[index].pose.position.x;
        }
        meanX /= static_cast<double>(first.getParticles().size());
        double variance = 0.0;
        for (const Particle& particle : first.getParticles()) {
            const double residual = particle.pose.position.x - meanX;
            variance += residual * residual;
        }
        variance /= static_cast<double>(first.getParticles().size());
        return variance > 1.0;
    });

    runTest(suite, "INIT-001", "global initialization samples only valid free map space", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config;
        config.minParticles = 200;
        config.maxParticles = 400;
        config.initialParticleCount = 300;
        ParticleFilter filter(config);
        std::mt19937 randomEngine(12);
        if (!filter.initializeGlobal(map, randomEngine)) {
            return false;
        }
        return filter.getParticles().size() == 300
            && std::all_of(filter.getParticles().begin(), filter.getParticles().end(), [&](const Particle& particle) {
                return map.containsWorldPoint(particle.pose.position)
                    && !map.isObstacleAt(particle.pose.position)
                    && particle.pose.heading >= -kLocalizationPi
                    && particle.pose.heading < kLocalizationPi
                    && near(particle.weight, 1.0 / 300.0, 1e-12);
            });
    });

    runTest(suite, "INIT-002", "impossible local initialization fails without retaining partial belief", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(50.0f, 50.0f)));
        map.addObstacle(GridCoord{0, 0});
        AmclConfig config = exactConfig(10);
        ParticleFilter filter(config);
        std::mt19937 randomEngine(13);
        const bool initialized = filter.initializeLocal(
            Pose2D{sf::Vector2f(25.0f, 25.0f), 0.0f}, map, randomEngine
        );
        return !initialized && filter.getParticles().empty();
    });

    runTest(suite, "SENSOR-001", "correct-pose scan scores better than a clearly wrong pose", [] {
        MapData map = makeAsymmetricMap();
        MapLikelihoodField field;
        AmclConfig config = exactConfig(100);
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = deterministicLidar();
        std::mt19937 scanEngine(20);
        const Pose2D truth{sf::Vector2f(170.0f, 210.0f), 0.35f};
        const LaserScan scan = lidar.simulate(truth, map, scanEngine);
        ParticleFilter correct(config);
        ParticleFilter wrong(config);
        std::mt19937 correctEngine(21);
        std::mt19937 wrongEngine(22);
        correct.initializeLocal(truth, map, correctEngine);
        wrong.initializeLocal(Pose2D{sf::Vector2f(760.0f, 650.0f), -1.2f}, map, wrongEngine);
        const SensorUpdateResult correctResult = correct.sensorUpdate(scan, field);
        const SensorUpdateResult wrongResult = wrong.sensorUpdate(scan, field);
        const double weightSum = [&] {
            double sum = 0.0;
            for (const Particle& particle : correct.getParticles()) sum += particle.weight;
            return sum;
        }();
        return correctResult.updated && wrongResult.updated
            && correctResult.observationQuality > wrongResult.observationQuality
            && std::isfinite(correctResult.observationQuality)
            && std::isfinite(wrongResult.observationQuality)
            && near(weightSum, 1.0, 1e-9);
    });

    runTest(suite, "SENSOR-002", "beamless scan is rejected without consuming pending odometry", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(80);
        config.updateMinTranslation = 0.0;
        config.updateMinRotation = 0.0;
        config.resampleInterval = 100;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = deterministicLidar();
        const Pose2D truth{sf::Vector2f(170.0f, 210.0f), 0.35f};
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(23);
        localizer.initializeLocal(truth, map, randomEngine);
        LaserScan validScan = lidar.simulate(truth, map, randomEngine);
        localizer.updateWithScan(validScan, field, map, randomEngine);
        const std::size_t previousUpdates = localizer.getStatistics().sensorUpdateCount;
        localizer.accumulateOdometry(OdometryDelta{0.0, 20.0, 0.0, true});
        LaserScan beamless = validScan;
        std::fill(beamless.ranges.begin(), beamless.ranges.end(), beamless.maxRange);
        const bool updated = localizer.updateWithScan(beamless, field, map, randomEngine);
        return !updated && localizer.needsSensorUpdate()
            && localizer.getStatistics().sensorUpdateCount == previousUpdates;
    });

    runTest(suite, "AMCL-001", "first scan bypasses thresholds and small odometry increments accumulate", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(80);
        config.updateMinTranslation = 10.0;
        config.updateMinRotation = 1.0;
        config.resampleInterval = 100;
        AmclLocalizer localizer(config);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = deterministicLidar();
        const Pose2D truth{sf::Vector2f(170.0f, 210.0f), 0.35f};
        std::mt19937 randomEngine(30);
        if (!localizer.initializeLocal(truth, map, randomEngine)
            || !localizer.needsSensorUpdate()) {
            return false;
        }
        const LaserScan scan = lidar.simulate(truth, map, randomEngine);
        if (!localizer.updateWithScan(scan, field, map, randomEngine)) {
            return false;
        }
        const bool afterFour = localizer.accumulateOdometry(OdometryDelta{0.0, 4.0, 0.0, true});
        const bool afterEight = localizer.accumulateOdometry(OdometryDelta{0.0, 4.0, 0.0, true});
        const bool afterEleven = localizer.accumulateOdometry(OdometryDelta{0.0, 3.0, 0.0, true});
        return !afterFour && !afterEight && afterEleven;
    });

    runTest(suite, "RECOVERY-001", "severe scan mismatch raises recovery injection probability", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(200);
        config.recoveryAlphaSlow = 0.10;
        config.recoveryAlphaFast = 0.90;
        config.resampleInterval = 100;
        AmclLocalizer localizer(config);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = deterministicLidar();
        const Pose2D poseA{sf::Vector2f(170.0f, 210.0f), 0.35f};
        const Pose2D poseB{sf::Vector2f(780.0f, 650.0f), -1.1f};
        std::mt19937 randomEngine(40);
        localizer.initializeLocal(poseA, map, randomEngine);
        const LaserScan scanA = lidar.simulate(poseA, map, randomEngine);
        localizer.updateWithScan(scanA, field, map, randomEngine);
        const double healthyProbability = localizer.getStatistics().recoveryProbability;
        localizer.forceSensorUpdate();
        const LaserScan scanB = lidar.simulate(poseB, map, randomEngine);
        localizer.updateWithScan(scanB, field, map, randomEngine);
        return healthyProbability == 0.0
            && localizer.getStatistics().recoveryProbability > 0.05
            && localizer.getStatistics().state == LocalizationState::Recovering;
    });

    runTest(suite, "RECOVERY-002", "zero recovery alphas keep random injection disabled", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(100);
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        AmclLocalizer localizer(config);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = deterministicLidar();
        const Pose2D poseA{sf::Vector2f(170.0f, 210.0f), 0.35f};
        const Pose2D poseB{sf::Vector2f(780.0f, 650.0f), -1.1f};
        std::mt19937 randomEngine(50);
        localizer.initializeLocal(poseA, map, randomEngine);
        localizer.updateWithScan(lidar.simulate(poseA, map, randomEngine), field, map, randomEngine);
        localizer.forceSensorUpdate();
        localizer.updateWithScan(lidar.simulate(poseB, map, randomEngine), field, map, randomEngine);
        return localizer.getStatistics().recoveryProbability == 0.0
            && localizer.getStatistics().state != LocalizationState::Recovering;
    });

    return suite.exitCode();
}
