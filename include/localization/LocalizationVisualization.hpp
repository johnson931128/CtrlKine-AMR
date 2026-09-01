#pragma once

#include "localization/LocalizationTypes.hpp"

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

class LocalizationVisualization {
public:
    const LocalizationViewOptions& getOptions() const;
    LocalizationViewOptions& getOptions();

    void rebuildParticles(const std::vector<Particle>& particles);
    void drawScan(
        sf::RenderWindow& window,
        const LaserScan& scan,
        const Pose2D& basePose
    ) const;
    void drawBelief(
        sf::RenderWindow& window,
        const LocalizationEstimate& estimate,
        const LocalizationStatistics& statistics,
        const Pose2D& odometryPose,
        bool odometryInitialized
    ) const;

private:
    LocalizationViewOptions m_options;
    sf::VertexArray m_particleVertices;
};

CovarianceEllipse covarianceEllipse(const LocalizationCovariance& covariance, double sigmaScale = 2.0);
sf::VertexArray buildParticleVertices(
    const std::vector<Particle>& particles,
    const LocalizationViewOptions& options,
    const sf::Color& color
);
