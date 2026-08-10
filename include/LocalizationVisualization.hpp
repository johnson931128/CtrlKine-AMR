#pragma once

#include "LocalizationTypes.hpp"

struct LocalizationViewOptions {
    bool particles = true;
    bool lidarRays = false;
    bool lidarHitPoints = false;
    bool estimate = true;
    bool covariance = true;
    bool odometry = false;
    bool diagnostics = true;
    std::size_t renderedRayCount = 15;
};

struct CovarianceEllipse {
    bool valid = false;
    double majorRadius = 0.0;
    double minorRadius = 0.0;
    double rotation = 0.0;
};

CovarianceEllipse covarianceEllipse(const LocalizationCovariance& covariance, double sigmaScale = 2.0);
sf::VertexArray buildParticleVertices(
    const std::vector<Particle>& particles,
    const LocalizationViewOptions& options,
    const sf::Color& color
);
