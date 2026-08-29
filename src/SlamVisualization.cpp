#include "SlamVisualization.hpp"

#include <cmath>

namespace {
const sf::Color kSlamFreeColor(80, 170, 210, 40);
const sf::Color kSlamOccupiedColor(25, 70, 95, 190);
const sf::Color kSlamPoseColor(20, 190, 110, 240);
const sf::Color kSlamPredictedColor(245, 175, 35, 190);

void drawPoseMarker(
    sf::RenderWindow& window,
    const Pose2D& pose,
    const sf::Color& color,
    float radius
) {
    sf::CircleShape marker(radius, 20);
    marker.setOrigin(sf::Vector2f(radius, radius));
    marker.setPosition(pose.position);
    marker.setFillColor(sf::Color::Transparent);
    marker.setOutlineThickness(2.0f);
    marker.setOutlineColor(color);
    window.draw(marker);

    sf::VertexArray heading(sf::PrimitiveType::Lines, 2);
    heading[0] = sf::Vertex{pose.position, color};
    heading[1] = sf::Vertex{
        pose.position + sf::Vector2f(
            std::cos(pose.heading) * 24.0f,
            std::sin(pose.heading) * 24.0f
        ),
        color
    };
    window.draw(heading);
}

void appendCell(
    sf::VertexArray& vertices,
    const SlamOccupancyGrid& map,
    const SlamGridCoord& cell,
    const Pose2D& displayOrigin,
    const sf::Color& color
) {
    const double half = map.getConfig().resolution * 0.5;
    const sf::Vector2f center = map.cellCenter(cell);
    const sf::Vector2f localCorners[4] = {
        center + sf::Vector2f(static_cast<float>(-half), static_cast<float>(-half)),
        center + sf::Vector2f(static_cast<float>(half), static_cast<float>(-half)),
        center + sf::Vector2f(static_cast<float>(half), static_cast<float>(half)),
        center + sf::Vector2f(static_cast<float>(-half), static_cast<float>(half))
    };
    sf::Vector2f displayCorners[4];
    for (int index = 0; index < 4; ++index) {
        displayCorners[index] = SlamVisualization::toDisplayPose(
            Pose2D{localCorners[index], 0.0f}, displayOrigin
        ).position;
    }
    for (const int index : {0, 1, 2, 0, 2, 3}) {
        vertices.append(sf::Vertex{displayCorners[index], color});
    }
}
}

const SlamViewOptions& SlamVisualization::getOptions() const {
    return m_options;
}

SlamViewOptions& SlamVisualization::getOptions() {
    return m_options;
}

Pose2D SlamVisualization::toDisplayPose(
    const Pose2D& localPose,
    const Pose2D& displayOrigin
) {
    const double cosine = std::cos(displayOrigin.heading);
    const double sine = std::sin(displayOrigin.heading);
    Pose2D display;
    display.position.x = displayOrigin.position.x + static_cast<float>(
        cosine * localPose.position.x - sine * localPose.position.y
    );
    display.position.y = displayOrigin.position.y + static_cast<float>(
        sine * localPose.position.x + cosine * localPose.position.y
    );
    display.heading = static_cast<float>(normalizeLocalizationAngle(
        displayOrigin.heading + localPose.heading
    ));
    return display;
}

void SlamVisualization::rebuildMapIfNeeded(
    const SlamOccupancyGrid& map,
    const Pose2D& displayOrigin
) {
    if (m_cachedRevision == map.getRevision()) {
        return;
    }
    m_freeCells = sf::VertexArray(sf::PrimitiveType::Triangles);
    m_occupiedCells = sf::VertexArray(sf::PrimitiveType::Triangles);
    const SlamOccupancyGridConfig& config = map.getConfig();
    for (std::size_t row = 0; row < config.height; ++row) {
        for (std::size_t col = 0; col < config.width; ++col) {
            const SlamGridCoord cell{static_cast<int>(col), static_cast<int>(row)};
            const OccupancyState state = map.getState(cell);
            if (state == OccupancyState::Unknown) {
                continue;
            }
            if (state == OccupancyState::Occupied) {
                appendCell(m_occupiedCells, map, cell, displayOrigin, kSlamOccupiedColor);
            } else {
                appendCell(m_freeCells, map, cell, displayOrigin, kSlamFreeColor);
            }
        }
    }
    m_cachedRevision = map.getRevision();
}

void SlamVisualization::clear() {
    m_freeCells.clear();
    m_occupiedCells.clear();
    m_cachedRevision = static_cast<std::uint64_t>(-1);
}

void SlamVisualization::draw(
    sf::RenderWindow& window,
    const SlamUpdateResult& update,
    const Pose2D& displayOrigin
) const {
    if (m_options.occupancyMap) {
        window.draw(m_freeCells);
        window.draw(m_occupiedCells);
    }
    if (!update.poseValid) {
        return;
    }
    if (m_options.predictedPose) {
        drawPoseMarker(
            window,
            toDisplayPose(update.predictedPose, displayOrigin),
            kSlamPredictedColor,
            7.0f
        );
    }
    if (m_options.pose) {
        drawPoseMarker(
            window,
            toDisplayPose(update.pose, displayOrigin),
            kSlamPoseColor,
            9.0f
        );
    }
}
