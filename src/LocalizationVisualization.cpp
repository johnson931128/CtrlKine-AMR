#include "LocalizationVisualization.hpp"

#include <algorithm>
#include <cmath>

CovarianceEllipse covarianceEllipse(
    const LocalizationCovariance& covariance,
    double sigmaScale
) {
    CovarianceEllipse ellipse;
    const double xx = covariance.values[0];
    const double xy = 0.5 * (covariance.values[1] + covariance.values[3]);
    const double yy = covariance.values[4];
    if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yy)
        || !std::isfinite(sigmaScale) || sigmaScale <= 0.0) {
        return ellipse;
    }
    const double traceHalf = 0.5 * (xx + yy);
    const double differenceHalf = 0.5 * (xx - yy);
    const double root = std::hypot(differenceHalf, xy);
    const double majorValue = std::max(0.0, traceHalf + root);
    const double minorValue = std::max(0.0, traceHalf - root);
    ellipse.majorRadius = sigmaScale * std::sqrt(majorValue);
    ellipse.minorRadius = sigmaScale * std::sqrt(minorValue);
    ellipse.rotation = 0.5 * std::atan2(2.0 * xy, xx - yy);
    ellipse.valid = std::isfinite(ellipse.majorRadius)
        && std::isfinite(ellipse.minorRadius)
        && std::isfinite(ellipse.rotation);
    return ellipse;
}

sf::VertexArray buildParticleVertices(
    const std::vector<Particle>& particles,
    const LocalizationViewOptions& options,
    const sf::Color& color
) {
    sf::VertexArray vertices(sf::PrimitiveType::Points);
    if (!options.particles) {
        return vertices;
    }
    vertices.resize(particles.size());
    for (std::size_t index = 0; index < particles.size(); ++index) {
        vertices[index] = sf::Vertex{particles[index].pose.position, color};
    }
    return vertices;
}
