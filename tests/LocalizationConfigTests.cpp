#include "LocalizationConfig.hpp"

#include <cstdio>
#include <fstream>
#include <limits>
#include <vector>

#include "TestSupport.hpp"

namespace {
bool writeFile(const std::string& filename, const std::string& content) {
    std::ofstream output(filename, std::ios::trunc);
    output << content;
    return output.good();
}
}

int main() {
    TestSuite suite;
    const std::string filename = "build/tests/localization_config_test.cfg";

    runTest(suite, "CONFIG-001", "valid localization config parses explicit values", [&] {
        if (!writeFile(filename,
                "minParticles 20\nmaxParticles 80\ninitialParticleCount 40\n"
                "doBeamSkip false\nclusterMinimumBinWeightRatio 0.08\n"
                "minimumSupportBeams 4\nminimumLikelihoodContrast 1.03\n"
                "lidarBeamCount 31\nlidarOffsetX 12.5\nrandomSeed 77\n")) {
            return false;
        }
        LocalizationConfigSet config;
        config.amcl.minParticles = 20;
        config.amcl.maxParticles = 80;
        config.amcl.initialParticleCount = 40;
        std::string error;
        const bool loaded = loadLocalizationConfigFile(filename, config, error);
        return loaded && error.empty() && !config.amcl.doBeamSkip
            && config.amcl.clusterMinimumBinWeightRatio == 0.08
            && config.amcl.minimumSupportBeams == 4
            && config.amcl.minimumLikelihoodContrast == 1.03
            && config.lidar.beamCount == 31 && config.lidar.offsetX == 12.5
            && config.amcl.randomSeed == 77;
    });

    runTest(suite, "CONFIG-002", "unknown key is rejected atomically", [&] {
        if (!writeFile(filename, "minParticles 10\nunknownParameter 4\n")) return false;
        LocalizationConfigSet config;
        const std::size_t original = config.amcl.minParticles;
        std::string error;
        return !loadLocalizationConfigFile(filename, config, error)
            && !error.empty() && config.amcl.minParticles == original;
    });

    runTest(suite, "CONFIG-003", "non-finite and inconsistent values are rejected", [&] {
        LocalizationConfigSet config;
        std::string error;
        const bool wroteNan = writeFile(filename, "sigmaHit nan\n");
        const bool nanRejected = wroteNan
            && !loadLocalizationConfigFile(filename, config, error);
        const bool wroteCounts = writeFile(
            filename, "minParticles 500\nmaxParticles 100\ninitialParticleCount 200\n"
        );
        const bool countsRejected = wroteCounts
            && !loadLocalizationConfigFile(filename, config, error);
        return nanRejected && countsRejected;
    });

    runTest(suite, "CONFIG-004", "central validation rejects invalid mixture weights", [] {
        LocalizationConfigSet config;
        config.amcl.zHit = 0.9;
        config.amcl.zRand = 0.9;
        std::string error;
        return !validateLocalizationConfig(config, error) && !error.empty();
    });

    runTest(suite, "CONFIG-005", "central validation rejects every parameter family", [] {
        std::vector<LocalizationConfigSet> invalid(11);
        invalid[0].lidar.fieldOfView = 7.0;
        invalid[1].lidar.minRange = 100.0;
        invalid[1].lidar.maxRange = 50.0;
        invalid[2].lidar.rangeNoiseStdDev = -1.0;
        invalid[3].odometry.rotationStdDevPerDistance = -0.1;
        invalid[4].amcl.kldBinSizeYaw = 0.0;
        invalid[5].amcl.convergenceHeadingStdDev = -0.1;
        invalid[6].amcl.clusterMinimumBinWeightRatio = 0.0;
        invalid[7].amcl.beamSkipThreshold = 1.1;
        invalid[8].amcl.recoveryAlphaSlow = 0.2;
        invalid[8].amcl.recoveryAlphaFast = 0.1;
        invalid[9].amcl.updateMinTranslation = -1.0;
        invalid[10].lidar.offsetX = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t index = 0; index < invalid.size(); ++index) {
            const LocalizationConfigSet& candidate = invalid[index];
            std::string error;
            if (validateLocalizationConfig(candidate, error) || error.empty()) {
                std::cerr << "Accepted invalid config family " << index << "\n";
                return false;
            }
        }
        return true;
    });

    runTest(suite, "CONFIG-006", "malformed records preserve the complete prior config", [&] {
        LocalizationConfigSet config;
        config.amcl.minParticles = 321;
        config.amcl.randomSeed = 42;
        config.lidar.offsetX = 17.0;
        config.odometry.translationStdDevPerDistance = 0.123;
        const bool wrote = writeFile(
            filename, "minParticles 400\nlidarOffsetX 99\nalpha1\n"
        );
        std::string error;
        return wrote && !loadLocalizationConfigFile(filename, config, error)
            && config.amcl.minParticles == 321
            && config.amcl.randomSeed == 42
            && config.lidar.offsetX == 17.0
            && config.odometry.translationStdDevPerDistance == 0.123;
    });

    std::remove(filename.c_str());
    return suite.exitCode();
}
