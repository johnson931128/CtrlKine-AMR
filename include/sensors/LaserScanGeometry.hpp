#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "localization/LocalizationTypes.hpp"

inline double laserScanBeamAngle(const LaserScan& scan, std::size_t index) {
    return scan.angleMin + scan.angleIncrement * static_cast<double>(index);
}

inline bool isFullCircleScan(const LaserScan& scan) {
    if (scan.ranges.size() <= 1 || !std::isfinite(scan.angleIncrement)) {
        return false;
    }
    const double coverage = std::abs(
        static_cast<double>(scan.angleIncrement) * static_cast<double>(scan.ranges.size())
    );
    return std::abs(coverage - 2.0 * kLocalizationPi) <= 1e-4;
}

inline bool isLaserReturnHit(const LaserScan& scan, double range) {
    return std::isfinite(range) && range >= scan.minRange && range < scan.maxRange;
}

inline std::vector<std::size_t> selectEvenlySpacedBeamIndices(
    const LaserScan& scan,
    std::size_t maximumCount
) {
    const std::size_t count = std::min(maximumCount, scan.ranges.size());
    std::vector<std::size_t> indices;
    indices.reserve(count);
    if (count == 0) {
        return indices;
    }
    if (count == 1) {
        indices.push_back(isFullCircleScan(scan) ? 0 : scan.ranges.size() / 2);
        return indices;
    }

    const bool cyclic = isFullCircleScan(scan);
    for (std::size_t selected = 0; selected < count; ++selected) {
        if (cyclic) {
            indices.push_back(selected * scan.ranges.size() / count);
        } else {
            const double position = static_cast<double>(selected)
                * static_cast<double>(scan.ranges.size() - 1)
                / static_cast<double>(count - 1);
            indices.push_back(static_cast<std::size_t>(std::llround(position)));
        }
    }
    return indices;
}
