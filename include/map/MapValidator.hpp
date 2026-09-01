#pragma once

#include <string>
#include <vector>

#include "core/AMR.hpp"
#include "map/MapData.hpp"

enum class ValidationStatus {
    Valid,
    Warning,
    Error
};

struct ValidationResult {
    ValidationStatus status = ValidationStatus::Valid;
    std::vector<std::string> messages;
};

class MapValidator {
public:
    static ValidationResult validate(const MapData& mapData, const AMR& robot);
};
