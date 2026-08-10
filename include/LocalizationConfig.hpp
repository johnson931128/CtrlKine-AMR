#pragma once

#include <string>

#include "LocalizationTypes.hpp"

struct LocalizationConfigSet {
    AmclConfig amcl;
    LidarConfig lidar;
    OdometryConfig odometry;
};

bool validateLocalizationConfig(const LocalizationConfigSet& config, std::string& error);
bool loadLocalizationConfigFile(
    const std::string& filename,
    LocalizationConfigSet& config,
    std::string& error
);
