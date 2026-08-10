#include "LocalizationConfig.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "AmclLocalizer.hpp"
#include "LidarSimulator.hpp"
#include "OdometrySimulator.hpp"

namespace {
bool parseDouble(const std::string& text, double& value) {
    try {
        std::size_t consumed = 0;
        value = std::stod(text, &consumed);
        return consumed == text.size() && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

bool parseSize(const std::string& text, std::size_t& value) {
    double parsed = 0.0;
    if (!parseDouble(text, parsed) || parsed < 0.0 || std::floor(parsed) != parsed
        || parsed > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    value = static_cast<std::size_t>(parsed);
    return true;
}

bool parseBool(const std::string& text, bool& value) {
    if (text == "true" || text == "1" || text == "on") {
        value = true;
        return true;
    }
    if (text == "false" || text == "0" || text == "off") {
        value = false;
        return true;
    }
    return false;
}
}

bool validateLocalizationConfig(const LocalizationConfigSet& config, std::string& error) {
    try {
        AmclLocalizer localizer(config.amcl);
        LidarSimulator lidar(config.lidar);
        OdometrySimulator odometry(config.odometry);
        (void)localizer;
        (void)lidar;
        (void)odometry;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
    error.clear();
    return true;
}

bool loadLocalizationConfigFile(
    const std::string& filename,
    LocalizationConfigSet& config,
    std::string& error
) {
    std::ifstream input(filename);
    if (!input.is_open()) {
        error = "Cannot open localization configuration: " + filename;
        return false;
    }
    LocalizationConfigSet parsed = config;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        std::istringstream fields(line);
        std::string key;
        std::string value;
        std::string extra;
        if (!(fields >> key)) {
            continue;
        }
        if (!(fields >> value) || (fields >> extra)) {
            error = "Invalid localization config record at line " + std::to_string(lineNumber);
            return false;
        }

        double number = 0.0;
        bool recognized = false;
        bool parsedValue = true;
#define SET_DOUBLE(name, target) if (key == name) { recognized = true; parsedValue = parseDouble(value, number); if (parsedValue) target = number; }
#define SET_SIZE(name, target) else if (key == name) { recognized = true; parsedValue = parseSize(value, target); }
#define SET_BOOL(name, target) else if (key == name) { recognized = true; parsedValue = parseBool(value, target); }
        SET_DOUBLE("alpha1", parsed.amcl.alpha1)
        else SET_DOUBLE("alpha2", parsed.amcl.alpha2)
        else SET_DOUBLE("alpha3", parsed.amcl.alpha3)
        else SET_DOUBLE("alpha4", parsed.amcl.alpha4)
        else SET_DOUBLE("alpha5", parsed.amcl.alpha5)
        SET_SIZE("minParticles", parsed.amcl.minParticles)
        SET_SIZE("maxParticles", parsed.amcl.maxParticles)
        SET_SIZE("initialParticleCount", parsed.amcl.initialParticleCount)
        SET_DOUBLE("initialStdDevX", parsed.amcl.initialStdDevX)
        else SET_DOUBLE("initialStdDevY", parsed.amcl.initialStdDevY)
        else SET_DOUBLE("initialStdDevYaw", parsed.amcl.initialStdDevYaw)
        else SET_DOUBLE("sigmaHit", parsed.amcl.sigmaHit)
        else SET_DOUBLE("zHit", parsed.amcl.zHit)
        else SET_DOUBLE("zRand", parsed.amcl.zRand)
        else SET_DOUBLE("likelihoodMaxDistance", parsed.amcl.likelihoodMaxDistance)
        SET_SIZE("maxBeams", parsed.amcl.maxBeams)
        SET_BOOL("doBeamSkip", parsed.amcl.doBeamSkip)
        SET_DOUBLE("beamSkipDistance", parsed.amcl.beamSkipDistance)
        else SET_DOUBLE("beamSkipThreshold", parsed.amcl.beamSkipThreshold)
        else SET_DOUBLE("beamSkipErrorThreshold", parsed.amcl.beamSkipErrorThreshold)
        else SET_DOUBLE("updateMinTranslation", parsed.amcl.updateMinTranslation)
        else SET_DOUBLE("updateMinRotation", parsed.amcl.updateMinRotation)
        SET_SIZE("resampleInterval", parsed.amcl.resampleInterval)
        SET_DOUBLE("resampleEssRatio", parsed.amcl.resampleEssRatio)
        else SET_DOUBLE("kldBinSizeX", parsed.amcl.kldBinSizeX)
        else SET_DOUBLE("kldBinSizeY", parsed.amcl.kldBinSizeY)
        else SET_DOUBLE("kldBinSizeYaw", parsed.amcl.kldBinSizeYaw)
        else SET_DOUBLE("pfErr", parsed.amcl.pfErr)
        else SET_DOUBLE("pfZ", parsed.amcl.pfZ)
        else SET_DOUBLE("recoveryAlphaSlow", parsed.amcl.recoveryAlphaSlow)
        else SET_DOUBLE("recoveryAlphaFast", parsed.amcl.recoveryAlphaFast)
        else SET_DOUBLE("convergencePositionStdDev", parsed.amcl.convergencePositionStdDev)
        else SET_DOUBLE("convergenceHeadingStdDev", parsed.amcl.convergenceHeadingStdDev)
        SET_SIZE("minimumSensorUpdatesForConvergence", parsed.amcl.minimumSensorUpdatesForConvergence)
        SET_SIZE("minimumGlobalSensorUpdatesForConvergence", parsed.amcl.minimumGlobalSensorUpdatesForConvergence)
        SET_DOUBLE("clusterBinSizeX", parsed.amcl.clusterBinSizeX)
        else SET_DOUBLE("clusterBinSizeY", parsed.amcl.clusterBinSizeY)
        else SET_DOUBLE("clusterBinSizeYaw", parsed.amcl.clusterBinSizeYaw)
        else SET_DOUBLE("clusterMinimumBinWeightRatio", parsed.amcl.clusterMinimumBinWeightRatio)
        else SET_DOUBLE("significantClusterWeight", parsed.amcl.significantClusterWeight)
        else SET_DOUBLE("dominantClusterWeight", parsed.amcl.dominantClusterWeight)
        else SET_DOUBLE("dominantToSecondRatio", parsed.amcl.dominantToSecondRatio)
        else SET_DOUBLE("dominantSwitchMargin", parsed.amcl.dominantSwitchMargin)
        else SET_DOUBLE("minimumHeadingResultant", parsed.amcl.minimumHeadingResultant)
        SET_SIZE("minimumSupportBeams", parsed.amcl.minimumSupportBeams)
        SET_DOUBLE("minimumLikelihoodContrast", parsed.amcl.minimumLikelihoodContrast)
        else SET_DOUBLE("recoveringProbabilityThreshold", parsed.amcl.recoveringProbabilityThreshold)
        else SET_DOUBLE("navigationDominantWeight", parsed.amcl.navigationDominantWeight)
        else SET_DOUBLE("navigationPositionStdDev", parsed.amcl.navigationPositionStdDev)
        else SET_DOUBLE("navigationHeadingStdDev", parsed.amcl.navigationHeadingStdDev)
        SET_SIZE("historyCapacity", parsed.amcl.historyCapacity)
        SET_SIZE("lidarBeamCount", parsed.lidar.beamCount)
        SET_DOUBLE("lidarFOV", parsed.lidar.fieldOfView)
        else SET_DOUBLE("lidarMinRange", parsed.lidar.minRange)
        else SET_DOUBLE("lidarMaxRange", parsed.lidar.maxRange)
        else SET_DOUBLE("lidarNoise", parsed.lidar.rangeNoiseStdDev)
        else SET_DOUBLE("lidarOffsetX", parsed.lidar.offsetX)
        else SET_DOUBLE("lidarOffsetY", parsed.lidar.offsetY)
        else SET_DOUBLE("lidarYawOffset", parsed.lidar.yawOffset)
        else SET_DOUBLE("odomTranslationPerDistance", parsed.odometry.translationStdDevPerDistance)
        else SET_DOUBLE("odomTranslationPerRotation", parsed.odometry.translationStdDevPerRotation)
        else SET_DOUBLE("odomRotationPerRotation", parsed.odometry.rotationStdDevPerRotation)
        else SET_DOUBLE("odomRotationPerDistance", parsed.odometry.rotationStdDevPerDistance)
        else if (key == "randomSeed") {
            recognized = true;
            std::size_t seed = 0;
            parsedValue = parseSize(value, seed) && seed <= std::numeric_limits<std::uint32_t>::max();
            if (parsedValue) parsed.amcl.randomSeed = static_cast<std::uint32_t>(seed);
        }
#undef SET_DOUBLE
#undef SET_SIZE
#undef SET_BOOL
        if (!recognized || !parsedValue) {
            error = "Invalid localization config key/value at line " + std::to_string(lineNumber);
            return false;
        }
    }
    if (!validateLocalizationConfig(parsed, error)) {
        return false;
    }
    config = parsed;
    return true;
}
