#pragma once

#include <string>
#include <vector>

#include "AMR.hpp"
#include "MapData.hpp"

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
