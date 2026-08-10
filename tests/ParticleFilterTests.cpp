#include "AmclLocalizer.hpp"
#include "LidarSimulator.hpp"
#include "MapLikelihoodField.hpp"
#include "ParticleFilter.hpp"
#include "LocalizationVisualization.hpp"

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
    config.minimumLikelihoodContrast = 1.0;
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
        ParticleFilter smallError(config);
        ParticleFilter wrong(config);
        std::mt19937 correctEngine(21);
        std::mt19937 smallEngine(21);
        std::mt19937 wrongEngine(22);
        correct.initializeLocal(truth, map, correctEngine);
        smallError.initializeLocal(
            Pose2D{sf::Vector2f(195.0f, 225.0f), 0.45f}, map, smallEngine
        );
        wrong.initializeLocal(Pose2D{sf::Vector2f(760.0f, 650.0f), -1.2f}, map, wrongEngine);
        const SensorUpdateResult correctResult = correct.sensorUpdate(scan, field);
        const SensorUpdateResult smallResult = smallError.sensorUpdate(scan, field);
        const SensorUpdateResult wrongResult = wrong.sensorUpdate(scan, field);
        const double weightSum = [&] {
            double sum = 0.0;
            for (const Particle& particle : correct.getParticles()) sum += particle.weight;
            return sum;
        }();
        return correctResult.updated && smallResult.updated && wrongResult.updated
            && correctResult.observationQuality > smallResult.observationQuality
            && smallResult.observationQuality > wrongResult.observationQuality
            && std::isfinite(correctResult.observationQuality)
            && std::isfinite(wrongResult.observationQuality)
            && near(weightSum, 1.0, 1e-9);
    });

    runTest(suite, "SENSOR-002", "all-max scan commits pending motion without claiming sensor evidence", [] {
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
        const Pose2D beforePose = localizer.getEstimate().pose;
        localizer.accumulateOdometry(OdometryDelta{0.0, 20.0, 0.0, true});
        LaserScan beamless = validScan;
        std::fill(beamless.ranges.begin(), beamless.ranges.end(), beamless.maxRange);
        const bool updated = localizer.updateWithScan(beamless, field, map, randomEngine);
        const Pose2D afterPose = localizer.getEstimate().pose;
        const bool passed = updated && !localizer.needsSensorUpdate()
            && localizer.getStatistics().sensorUpdateCount == previousUpdates
            && near(afterPose.position.x,
                beforePose.position.x + 20.0 * std::cos(beforePose.heading), 1e-4)
            && near(afterPose.position.y,
                beforePose.position.y + 20.0 * std::sin(beforePose.heading), 1e-4)
            && localizer.getStatistics().sensor.maxRangeBeams
                == localizer.getStatistics().sensor.selectedBeams;
        if (!passed) {
            std::cerr << "All-max metrics: updated=" << updated
                      << " needs=" << localizer.needsSensorUpdate()
                      << " updates=" << localizer.getStatistics().sensorUpdateCount
                      << " previous=" << previousUpdates
                      << " before=" << beforePose.position.x
                      << " after=" << afterPose.position.x
                      << " max=" << localizer.getStatistics().sensor.maxRangeBeams
                      << " selected=" << localizer.getStatistics().sensor.selectedBeams << "\n";
        }
        return passed;
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

    runTest(suite, "CLUSTER-001", "separated equal modes remain deterministic clusters", [] {
        AmclConfig config;
        config.clusterBinSizeX = 50.0;
        config.clusterBinSizeY = 50.0;
        config.clusterBinSizeYaw = 0.25;
        const std::vector<Particle> particles = {
            Particle{Pose2D{sf::Vector2f(0.0f, 0.0f), 0.0f}, 0.25},
            Particle{Pose2D{sf::Vector2f(10.0f, 0.0f), 0.0f}, 0.25},
            Particle{Pose2D{sf::Vector2f(300.0f, 0.0f), 0.0f}, 0.25},
            Particle{Pose2D{sf::Vector2f(310.0f, 0.0f), 0.0f}, 0.25}
        };
        const std::vector<ParticleCluster> clusters = ParticleFilter::clusterParticles(
            particles, config
        );
        return clusters.size() == 2
            && near(clusters[0].weight, 0.5)
            && near(clusters[1].weight, 0.5)
            && near(clusters[0].pose.position.x, 5.0)
            && near(clusters[1].pose.position.x, 305.0);
    });

    runTest(suite, "CLUSTER-002", "yaw bins wrap across negative and positive pi", [] {
        AmclConfig config;
        config.clusterBinSizeX = 50.0;
        config.clusterBinSizeY = 50.0;
        config.clusterBinSizeYaw = 0.20;
        const std::vector<Particle> particles = {
            Particle{Pose2D{sf::Vector2f(10.0f, 10.0f), static_cast<float>(kLocalizationPi - 0.03)}, 0.5},
            Particle{Pose2D{sf::Vector2f(10.0f, 10.0f), static_cast<float>(-kLocalizationPi + 0.03)}, 0.5}
        };
        const std::vector<ParticleCluster> clusters = ParticleFilter::clusterParticles(
            particles, config
        );
        return clusters.size() == 1 && clusters.front().headingResultant > 0.99
            && near(std::abs(clusters.front().pose.heading), kLocalizationPi, 0.01);
    });

    runTest(suite, "CLUSTER-003", "negligible bridge particles cannot merge supported modes", [] {
        AmclConfig config;
        config.clusterBinSizeX = 50.0;
        config.clusterBinSizeY = 50.0;
        config.clusterBinSizeYaw = 0.25;
        config.clusterMinimumBinWeightRatio = 0.05;
        std::vector<Particle> particles = {
            Particle{Pose2D{sf::Vector2f(0.0f, 0.0f), 0.0f}, 0.495},
            Particle{Pose2D{sf::Vector2f(300.0f, 0.0f), 0.0f}, 0.495}
        };
        for (int bin = 1; bin <= 5; ++bin) {
            particles.push_back(Particle{
                Pose2D{sf::Vector2f(static_cast<float>(bin * 50 + 1), 0.0f), 0.0f},
                0.002
            });
        }
        const auto clusters = ParticleFilter::clusterParticles(particles, config);
        return clusters.size() == 2
            && near(clusters[0].weight, 0.495)
            && near(clusters[1].weight, 0.495)
            && near(clusters[0].pose.position.x, 0.0)
            && near(clusters[1].pose.position.x, 300.0);
    });

    runTest(suite, "CLUSTER-004", "unequal modes expose deterministic dominant statistics", [] {
        AmclConfig config;
        const std::vector<Particle> particles = {
            Particle{Pose2D{sf::Vector2f(0.0f, 0.0f), -0.1f}, 0.375},
            Particle{Pose2D{sf::Vector2f(10.0f, 0.0f), 0.1f}, 0.375},
            Particle{Pose2D{sf::Vector2f(300.0f, 0.0f), 0.0f}, 0.125},
            Particle{Pose2D{sf::Vector2f(310.0f, 0.0f), 0.0f}, 0.125}
        };
        const auto clusters = ParticleFilter::clusterParticles(particles, config);
        return clusters.size() == 2
            && near(clusters[0].weight, 0.75)
            && near(clusters[1].weight, 0.25)
            && near(clusters[0].pose.position.x, 5.0)
            && clusters[0].covariance.xx() > 0.0
            && near(clusters[0].spatialExtent.size.x, 10.0)
            && clusters[0].headingExtent > 0.09;
    });

    runTest(suite, "SENSOR-003", "beam accounting is bounded and outlier skipping is deterministic", [] {
        MapData map = makeAsymmetricMap();
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        AmclConfig config = exactConfig(80);
        config.maxBeams = 31;
        config.doBeamSkip = true;
        config.beamSkipThreshold = 0.80;
        config.beamSkipErrorThreshold = 0.95;
        ParticleFilter first(config);
        ParticleFilter second(config);
        std::mt19937 firstEngine(700);
        std::mt19937 secondEngine(700);
        const Pose2D truth{sf::Vector2f(175.0f, 175.0f), 0.15f};
        first.initializeLocal(truth, map, firstEngine);
        second.initializeLocal(truth, map, secondEngine);
        std::mt19937 scanEngine(701);
        LaserScan scan = deterministicLidar().simulate(truth, map, scanEngine);
        scan.ranges[scan.ranges.size() / 2] = 37.0f;
        const SensorUpdateResult a = first.sensorUpdate(scan, field);
        const SensorUpdateResult b = second.sensorUpdate(scan, field);
        return a.updated && b.updated && a.skippedBeams > 0
            && a.skippedBeams == b.skippedBeams
            && a.usedBeams + a.skippedBeams + a.invalidBeams + a.maxRangeBeams
                <= a.selectedBeams
            && a.selectedBeams <= a.totalBeams;
    });

    runTest(suite, "SUPPORT-001", "zero-obstacle map cannot report confident convergence", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(500.0f, 500.0f)
        ));
        AmclConfig config = exactConfig(80);
        config.minimumSensorUpdatesForConvergence = 1;
        AmclLocalizer localizer(config);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        std::mt19937 randomEngine(800);
        const Pose2D truth{sf::Vector2f(125.0f, 175.0f), 0.2f};
        if (!localizer.initializeLocal(truth, map, randomEngine)) return false;
        const LaserScan scan = deterministicLidar().simulate(truth, map, randomEngine);
        if (!localizer.updateWithScan(scan, field, map, randomEngine)) return false;
        return localizer.getStatistics().support == LocalizationSupport::Insufficient
            && localizer.getStatistics().state != LocalizationState::Converged
            && !localizer.getEstimate().converged;
    });

    runTest(suite, "SENSOR-004", "majority-corrupt beam skipping falls back safely", [] {
        MapData map = makeAsymmetricMap();
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        AmclConfig config = exactConfig(60);
        config.maxBeams = 31;
        config.doBeamSkip = true;
        config.beamSkipThreshold = 0.95;
        config.beamSkipErrorThreshold = 0.50;
        ParticleFilter filter(config);
        std::mt19937 engine(901);
        const Pose2D truth{sf::Vector2f(175.0f, 175.0f), 0.15f};
        if (!filter.initializeLocal(truth, map, engine)) return false;
        LaserScan scan = deterministicLidar().simulate(truth, map, engine);
        std::fill(scan.ranges.begin(), scan.ranges.end(), 37.0f);
        const SensorUpdateResult result = filter.sensorUpdate(scan, field);
        return result.updated && result.beamSkipFallback
            && result.skippedBeams == 0 && result.usedBeams == result.selectedBeams;
    });

    runTest(suite, "SENSOR-005", "disabled beam skipping reproduces the non-skip path", [] {
        MapData map = makeAsymmetricMap();
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        AmclConfig config = exactConfig(60);
        config.initialStdDevX = 20.0;
        config.initialStdDevY = 20.0;
        config.initialStdDevYaw = 0.1;
        ParticleFilter first(config);
        ParticleFilter second(config);
        std::mt19937 firstEngine(902);
        std::mt19937 secondEngine(902);
        const Pose2D truth{sf::Vector2f(175.0f, 175.0f), 0.15f};
        if (!first.initializeLocal(truth, map, firstEngine)
            || !second.initializeLocal(truth, map, secondEngine)) return false;
        std::mt19937 scanEngine(903);
        const LaserScan scan = deterministicLidar().simulate(truth, map, scanEngine);
        const SensorUpdateResult a = first.sensorUpdate(scan, field, false);
        const SensorUpdateResult b = second.sensorUpdate(scan, field, false);
        return a.updated && b.updated && a.skippedBeams == 0 && b.skippedBeams == 0
            && std::equal(
                first.getParticles().begin(), first.getParticles().end(),
                second.getParticles().begin(), [](const Particle& x, const Particle& y) {
                    return near(x.weight, y.weight, 1e-12);
                }
            );
    });

    runTest(suite, "KLD-003", "KLD equation and parameter directions match reference values", [] {
        AmclConfig loose;
        loose.minParticles = 50;
        loose.maxParticles = 5000;
        loose.initialParticleCount = 500;
        AmclConfig tighter = loose;
        tighter.pfErr = loose.pfErr * 0.5;
        AmclConfig higherConfidence = loose;
        higherConfidence.pfZ = 2.8;
        const std::size_t baseline = ParticleFilter::requiredKldSamples(20, loose);
        return ParticleFilter::requiredKldSamples(1, loose) == 50
            && ParticleFilter::requiredKldSamples(2, loose) == 66
            && ParticleFilter::requiredKldSamples(5, loose) == 134
            && baseline == 363
            && ParticleFilter::requiredKldSamples(100, loose) == 1347
            && ParticleFilter::requiredKldSamples(1000, loose) == 5000
            && ParticleFilter::requiredKldSamples(20, tighter) == 725
            && ParticleFilter::requiredKldSamples(20, higherConfidence) == 409
            && ParticleFilter::requiredKldSamples(20, tighter) > baseline
            && ParticleFilter::requiredKldSamples(20, higherConfidence) > baseline;
    });

    runTest(suite, "HISTORY-001", "localization history is bounded by configured capacity", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(20);
        config.historyCapacity = 3;
        AmclLocalizer localizer(config);
        std::mt19937 engine(904);
        const Pose2D pose{sf::Vector2f(175.0f, 175.0f), 0.1f};
        if (!localizer.initializeLocal(pose, map, engine)) return false;
        for (int index = 0; index < 5; ++index) {
            localizer.appendHistory(Pose2D{
                sf::Vector2f(static_cast<float>(index), 0.0f), 0.0f
            });
        }
        return localizer.getHistory().size() == 3
            && near(localizer.getHistory().front().odometry.position.x, 2.0)
            && near(localizer.getHistory().back().odometry.position.x, 4.0);
    });

    runTest(suite, "MOTION-004", "scan-to-scan motion is invariant to frame segmentation", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(40);
        config.alpha1 = 0.02;
        config.alpha2 = 0.001;
        config.alpha3 = 0.01;
        config.alpha4 = 0.02;
        config.alpha5 = 0.005;
        config.updateMinTranslation = 0.0;
        config.resampleInterval = 100;
        AmclLocalizer oneFrame(config);
        AmclLocalizer tenFrames(config);
        std::mt19937 oneEngine(905);
        std::mt19937 tenEngine(905);
        const Pose2D start{sf::Vector2f(175.0f, 175.0f), 0.1f};
        if (!oneFrame.initializeLocal(start, map, oneEngine)
            || !tenFrames.initializeLocal(start, map, tenEngine)) return false;
        oneFrame.accumulateOdometry(OdometryDelta{0.0, 100.0, 0.0, true});
        for (int frame = 0; frame < 10; ++frame) {
            tenFrames.accumulateOdometry(OdometryDelta{0.0, 10.0, 0.0, true});
        }
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        std::mt19937 scanEngine(906);
        const LaserScan scan = deterministicLidar().simulate(
            Pose2D{sf::Vector2f(274.5f, 185.0f), 0.1f}, map, scanEngine
        );
        if (!oneFrame.updateWithScan(scan, field, map, oneEngine)
            || !tenFrames.updateWithScan(scan, field, map, tenEngine)) return false;
        return std::equal(
            oneFrame.getParticles().begin(), oneFrame.getParticles().end(),
            tenFrames.getParticles().begin(), [](const Particle& first, const Particle& second) {
                return near(first.pose.position.x, second.pose.position.x, 1e-12)
                    && near(first.pose.position.y, second.pose.position.y, 1e-12)
                    && near(first.pose.heading, second.pose.heading, 1e-12)
                    && near(first.weight, second.weight, 1e-12);
            }
        );
    });

    runTest(suite, "INIT-003", "global initialization clears prior convergence lifecycle", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(80);
        config.minimumSensorUpdatesForConvergence = 1;
        config.minimumGlobalSensorUpdatesForConvergence = 3;
        AmclLocalizer localizer(config);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        std::mt19937 engine(907);
        const Pose2D truth{sf::Vector2f(175.0f, 175.0f), 0.15f};
        if (!localizer.initializeLocal(truth, map, engine)) return false;
        const LaserScan scan = deterministicLidar().simulate(truth, map, engine);
        if (!localizer.updateWithScan(scan, field, map, engine)) return false;
        if (localizer.getStatistics().state != LocalizationState::Converged) return false;
        if (!localizer.initializeGlobal(map, engine)) return false;
        return localizer.getStatistics().initialization == LocalizationInitialization::Global
            && localizer.getStatistics().sensorUpdateCount == 0
            && localizer.getStatistics().state != LocalizationState::Converged
            && !localizer.getEstimate().converged;
    });

    runTest(suite, "RNG-001", "visualization options do not consume inference randomness", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config = exactConfig(40);
        config.initialStdDevX = 15.0;
        config.initialStdDevY = 15.0;
        config.initialStdDevYaw = 0.1;
        ParticleFilter visible(config);
        ParticleFilter hidden(config);
        std::mt19937 visibleEngine(908);
        std::mt19937 hiddenEngine(908);
        const Pose2D start{sf::Vector2f(175.0f, 175.0f), 0.15f};
        if (!visible.initializeLocal(start, map, visibleEngine)
            || !hidden.initializeLocal(start, map, hiddenEngine)) return false;
        LocalizationViewOptions visibleView;
        LocalizationViewOptions hiddenView = visibleView;
        hiddenView.particles = false;
        const sf::VertexArray visibleVertices = buildParticleVertices(
            visible.getParticles(), visibleView, sf::Color::Magenta
        );
        const sf::VertexArray hiddenVertices = buildParticleVertices(
            hidden.getParticles(), hiddenView, sf::Color::Magenta
        );
        visible.motionUpdate(OdometryDelta{0.0, 20.0, 0.0, true}, visibleEngine);
        hidden.motionUpdate(OdometryDelta{0.0, 20.0, 0.0, true}, hiddenEngine);
        return visibleVertices.getVertexCount() == visible.getParticles().size()
            && hiddenVertices.getVertexCount() == 0
            && std::equal(
            visible.getParticles().begin(), visible.getParticles().end(),
            hidden.getParticles().begin(), [](const Particle& first, const Particle& second) {
                return near(first.pose.position.x, second.pose.position.x, 1e-12)
                    && near(first.pose.position.y, second.pose.position.y, 1e-12)
                    && near(first.pose.heading, second.pose.heading, 1e-12)
                    && near(first.weight, second.weight, 1e-12);
            }
        );
    });

    runTest(suite, "SENSOR-006", "beam skipping preserves pose estimate under deterministic outliers", [] {
        MapData map = makeAsymmetricMap();
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        AmclConfig enabledConfig;
        enabledConfig.minParticles = 300;
        enabledConfig.maxParticles = 300;
        enabledConfig.initialParticleCount = 300;
        enabledConfig.initialStdDevX = 90.0;
        enabledConfig.initialStdDevY = 90.0;
        enabledConfig.initialStdDevYaw = 0.5;
        enabledConfig.maxBeams = 31;
        enabledConfig.doBeamSkip = true;
        enabledConfig.beamSkipThreshold = 0.60;
        enabledConfig.beamSkipErrorThreshold = 0.80;
        AmclConfig disabledConfig = enabledConfig;
        disabledConfig.doBeamSkip = false;
        ParticleFilter enabled(enabledConfig);
        ParticleFilter disabled(disabledConfig);
        std::mt19937 enabledEngine(909);
        std::mt19937 disabledEngine(909);
        const Pose2D truth{sf::Vector2f(170.0f, 210.0f), 0.35f};
        if (!enabled.initializeLocal(truth, map, enabledEngine)
            || !disabled.initializeLocal(truth, map, disabledEngine)) return false;
        std::mt19937 scanEngine(910);
        LaserScan scan = deterministicLidar().simulate(truth, map, scanEngine);
        for (std::size_t index = 0; index < scan.ranges.size(); index += 10) {
            scan.ranges[index] = scan.minRange + 1.0f;
        }
        const SensorUpdateResult enabledResult = enabled.sensorUpdate(scan, field);
        const SensorUpdateResult disabledResult = disabled.sensorUpdate(scan, field);
        const auto poseError = [&](const LocalizationEstimate& estimate) {
            return std::hypot(
                estimate.pose.position.x - truth.position.x,
                estimate.pose.position.y - truth.position.y
            );
        };
        const double enabledError = poseError(enabled.estimate());
        const double disabledError = poseError(disabled.estimate());
        const bool passed = enabledResult.updated && disabledResult.updated
            && enabledResult.skippedBeams > 0
            && enabledError <= disabledError + 1e-6;
        if (!passed) {
            std::cerr << "Beam-skip metrics: skipped=" << enabledResult.skippedBeams
                      << " fallback=" << enabledResult.beamSkipFallback
                      << " enabledError=" << enabledError
                      << " disabledError=" << disabledError << "\n";
        }
        return passed;
    });

    runTest(suite, "SENSOR-007", "clean scan keeps all consistent beams when skipping is enabled", [] {
        MapData map = makeAsymmetricMap();
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        AmclConfig config = exactConfig(60);
        config.doBeamSkip = true;
        config.beamSkipThreshold = 0.9;
        ParticleFilter filter(config);
        std::mt19937 engine(911);
        const Pose2D truth{sf::Vector2f(170.0f, 210.0f), 0.35f};
        if (!filter.initializeLocal(truth, map, engine)) return false;
        const LaserScan scan = deterministicLidar().simulate(truth, map, engine);
        const SensorUpdateResult result = filter.sensorUpdate(scan, field);
        return result.updated && result.skippedBeams == 0
            && result.usedBeams > 0 && !result.beamSkipFallback;
    });

    runTest(suite, "SENSOR-008", "particle prediction applies the same LiDAR extrinsics as simulation", [] {
        MapData map = makeAsymmetricMap();
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        LidarConfig lidarConfig;
        lidarConfig.beamCount = 61;
        lidarConfig.fieldOfView = 1.5 * kLocalizationPi;
        lidarConfig.minRange = 1.0;
        lidarConfig.maxRange = 900.0;
        lidarConfig.offsetX = 35.0;
        lidarConfig.offsetY = -12.0;
        lidarConfig.yawOffset = 0.28;
        lidarConfig.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(lidarConfig);
        AmclConfig config = exactConfig(40);
        ParticleFilter correct(config);
        ParticleFilter zeroExtrinsic(config);
        std::mt19937 engineA(912);
        std::mt19937 engineB(912);
        const Pose2D truth{sf::Vector2f(170.0f, 210.0f), 0.35f};
        if (!correct.initializeLocal(truth, map, engineA)
            || !zeroExtrinsic.initializeLocal(truth, map, engineB)) return false;
        LaserScan scan = lidar.simulate(truth, map, engineA);
        LaserScan wrongScan = scan;
        wrongScan.sensorOffsetX = 0.0;
        wrongScan.sensorOffsetY = 0.0;
        wrongScan.sensorYawOffset = 0.0;
        const SensorUpdateResult right = correct.sensorUpdate(scan, field);
        const SensorUpdateResult wrong = zeroExtrinsic.sensorUpdate(wrongScan, field);
        return right.updated && wrong.updated
            && right.observationQuality > wrong.observationQuality;
    });

    runTest(suite, "SUPPORT-002", "unobserved obstacle cannot make an all-max scan well supported", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(2000.0f, 2000.0f)
        ));
        map.addObstacle(GridCoord{1, 1});
        AmclConfig config = exactConfig(60);
        config.minimumSensorUpdatesForConvergence = 1;
        AmclLocalizer localizer(config);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarConfig lidarConfig;
        lidarConfig.beamCount = 31;
        lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
        lidarConfig.minRange = 1.0;
        lidarConfig.maxRange = 100.0;
        lidarConfig.rangeNoiseStdDev = 0.0;
        std::mt19937 engine(913);
        const Pose2D truth{sf::Vector2f(1000.0f, 1000.0f), 0.2f};
        if (!localizer.initializeLocal(truth, map, engine)) return false;
        const LaserScan scan = LidarSimulator(lidarConfig).simulate(truth, map, engine);
        return localizer.updateWithScan(scan, field, map, engine)
            && localizer.getStatistics().support == LocalizationSupport::Weak
            && localizer.getStatistics().state != LocalizationState::Converged
            && localizer.getStatistics().sensor.maxRangeBeams
                == localizer.getStatistics().sensor.selectedBeams;
    });

    runTest(suite, "GLOBAL-001", "global warm-up still handles ESS depletion", [] {
        MapData map = makeAsymmetricMap();
        AmclConfig config;
        config.minParticles = 200;
        config.maxParticles = 1000;
        config.initialParticleCount = 800;
        config.resampleInterval = 1;
        config.minimumGlobalSensorUpdatesForConvergence = 15;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        AmclLocalizer localizer(config);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        std::mt19937 engine(914);
        const Pose2D truth{sf::Vector2f(170.0f, 210.0f), 0.35f};
        if (!localizer.initializeGlobal(map, engine)) return false;
        const LaserScan scan = deterministicLidar().simulate(truth, map, engine);
        if (!localizer.updateWithScan(scan, field, map, engine)) return false;
        const LocalizationStatistics& stats = localizer.getStatistics();
        return stats.sensorUpdateCount == 1
            && stats.preResampleEffectiveSampleSize
                < 0.9 * static_cast<double>(config.initialParticleCount)
            && near(stats.effectiveSampleSize, static_cast<double>(stats.particleCount), 1e-6)
            && stats.state != LocalizationState::Converged;
    });

    return suite.exitCode();
}
