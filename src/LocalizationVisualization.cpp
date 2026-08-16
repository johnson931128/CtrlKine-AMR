#include "LocalizationVisualization.hpp"

#include "LaserScanGeometry.hpp"

#include <algorithm>
#include <cmath>

namespace {
const sf::Color kParticleColor(138, 43, 226, 90);
const sf::Color kEstimateColor(255, 20, 147, 230);
const sf::Color kLidarColor(0, 170, 190, 45);
const sf::Color kOdometryColor(255, 140, 0, 220);
}

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

const LocalizationViewOptions& LocalizationVisualization::getOptions() const {
    return m_options;
}

LocalizationViewOptions& LocalizationVisualization::getOptions() {
    return m_options;
}

void LocalizationVisualization::rebuildParticles(const std::vector<Particle>& particles) {
    m_particleVertices = buildParticleVertices(particles, m_options, kParticleColor);
}

void LocalizationVisualization::drawScan(
    sf::RenderWindow& window,
    const LaserScan& scan,
    const Pose2D& basePose
) const {
    if (scan.ranges.empty() || (!m_options.lidarRays && !m_options.lidarHitPoints)) {
        return;
    }

    const std::vector<std::size_t> renderedIndices = selectEvenlySpacedBeamIndices(
        scan, std::max<std::size_t>(1, m_options.renderedRayCount)
    );
    sf::VertexArray rays(sf::PrimitiveType::Lines);
    sf::VertexArray hits(sf::PrimitiveType::Points);
    const double cosine = std::cos(basePose.heading);
    const double sine = std::sin(basePose.heading);
    const sf::Vector2f sensorOrigin(
        basePose.position.x + static_cast<float>(
            cosine * scan.sensorOffsetX - sine * scan.sensorOffsetY
        ),
        basePose.position.y + static_cast<float>(
            sine * scan.sensorOffsetX + cosine * scan.sensorOffsetY
        )
    );
    for (const std::size_t beam : renderedIndices) {
        const double range = scan.ranges[beam];
        if (!std::isfinite(range)) {
            continue;
        }
        const double angle = basePose.heading + scan.sensorYawOffset
            + laserScanBeamAngle(scan, beam);
        const sf::Vector2f endpoint(
            sensorOrigin.x + static_cast<float>(range * std::cos(angle)),
            sensorOrigin.y + static_cast<float>(range * std::sin(angle))
        );
        const bool maximumRange = range >= scan.maxRange;
        if (m_options.lidarRays) {
            const sf::Color rayColor = maximumRange
                ? sf::Color(110, 150, 160, 22)
                : kLidarColor;
            rays.append(sf::Vertex{
                sensorOrigin,
                sf::Color(rayColor.r, rayColor.g, rayColor.b, 12)
            });
            rays.append(sf::Vertex{endpoint, rayColor});
        }
        if (m_options.lidarHitPoints && isLaserReturnHit(scan, range)) {
            hits.append(sf::Vertex{
                endpoint,
                sf::Color(0, 150, 180, 210)
            });
        }
    }
    if (m_options.lidarRays) {
        window.draw(rays);
    }
    if (m_options.lidarHitPoints) {
        window.draw(hits);
    }
}

void LocalizationVisualization::drawBelief(
    sf::RenderWindow& window,
    const LocalizationEstimate& estimate,
    const LocalizationStatistics& statistics,
    const Pose2D& odometryPose,
    bool odometryInitialized
) const {
    if (m_options.particles && m_particleVertices.getVertexCount() > 0) {
        window.draw(m_particleVertices);
    }

    if (m_options.odometry && odometryInitialized) {
        sf::CircleShape odometryMarker(8.0f, 3);
        odometryMarker.setOrigin(sf::Vector2f(8.0f, 8.0f));
        odometryMarker.setPosition(odometryPose.position);
        odometryMarker.setRotation(sf::radians(
            odometryPose.heading + static_cast<float>(kLocalizationPi / 2.0)
        ));
        odometryMarker.setFillColor(sf::Color::Transparent);
        odometryMarker.setOutlineThickness(2.0f);
        odometryMarker.setOutlineColor(kOdometryColor);
        window.draw(odometryMarker);
    }

    if (!estimate.valid) {
        return;
    }

    if (m_options.covariance) {
        const CovarianceEllipse ellipse = covarianceEllipse(estimate.covariance);
        if (ellipse.valid && ellipse.majorRadius > 0.0 && ellipse.minorRadius > 0.0) {
            constexpr std::size_t kSegments = 48;
            sf::VertexArray outline(sf::PrimitiveType::LineStrip, kSegments + 1);
            const sf::Color covarianceColor = statistics.state == LocalizationState::Recovering
                ? sf::Color(255, 140, 0, 150)
                : statistics.state == LocalizationState::Ambiguous
                    ? sf::Color(210, 150, 30, 130)
                    : sf::Color(255, 20, 147, 120);
            for (std::size_t index = 0; index <= kSegments; ++index) {
                const double angle = 2.0 * kLocalizationPi * static_cast<double>(index)
                    / static_cast<double>(kSegments);
                const double localX = ellipse.majorRadius * std::cos(angle);
                const double localY = ellipse.minorRadius * std::sin(angle);
                const double rotatedX = localX * std::cos(ellipse.rotation)
                    - localY * std::sin(ellipse.rotation);
                const double rotatedY = localX * std::sin(ellipse.rotation)
                    + localY * std::cos(ellipse.rotation);
                outline[index] = sf::Vertex{
                    estimate.pose.position + sf::Vector2f(
                        static_cast<float>(rotatedX), static_cast<float>(rotatedY)
                    ),
                    covarianceColor
                };
            }
            window.draw(outline);
        }
    }

    if (!m_options.estimate) {
        return;
    }

    sf::CircleShape marker(9.0f, 24);
    marker.setOrigin(sf::Vector2f(9.0f, 9.0f));
    marker.setPosition(estimate.pose.position);
    marker.setFillColor(sf::Color::Transparent);
    marker.setOutlineThickness(statistics.state == LocalizationState::Converged ? 3.0f : 2.0f);
    const sf::Color estimateColor = statistics.state == LocalizationState::Recovering
        ? sf::Color(255, 140, 0, 210)
        : statistics.state == LocalizationState::Ambiguous
            ? sf::Color(210, 150, 30, 180)
            : statistics.state == LocalizationState::Tracking
                ? sf::Color(255, 20, 147, 120)
                : kEstimateColor;
    marker.setOutlineColor(estimateColor);
    window.draw(marker);

    const sf::Vector2f headingEnd(
        estimate.pose.position.x + std::cos(estimate.pose.heading) * 30.0f,
        estimate.pose.position.y + std::sin(estimate.pose.heading) * 30.0f
    );
    sf::VertexArray heading(sf::PrimitiveType::Lines, 2);
    heading[0] = sf::Vertex{estimate.pose.position, estimateColor};
    heading[1] = sf::Vertex{headingEnd, estimateColor};
    window.draw(heading);
}
