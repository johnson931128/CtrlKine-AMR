#include "AMR.hpp"

#include <algorithm>

namespace {
float normalizeAngle(float angleRadians) {
    constexpr float kPi = 3.14159265f;
    constexpr float kTwoPi = 6.28318531f;

    while (angleRadians <= -kPi) {
        angleRadians += kTwoPi;
    }
    while (angleRadians > kPi) {
        angleRadians -= kTwoPi;
    }
    return angleRadians;
}
}

AMR::AMR(const AMRConfig& config, sf::Vector2f startPos)
    : m_config(config), m_position(startPos), m_heading(0.0f) {
    m_bodyShape.setSize(sf::Vector2f(m_config.bodyLength, m_config.bodyWidth));
    m_bodyShape.setFillColor(m_config.bodyColor);
    m_bodyShape.setOrigin(sf::Vector2f(m_config.bodyLength / 2.0f, m_config.bodyWidth / 2.0f));
    m_bodyShape.setPosition(m_position);

    const sf::Vector2f wheelSize(m_config.wheelLength, m_config.wheelWidth);
    const sf::Vector2f wheelOrigin(m_config.wheelLength / 2.0f, m_config.wheelWidth / 2.0f);

    for (int i = 0; i < 4; ++i) {
        m_wheels[i].setSize(wheelSize);
        m_wheels[i].setOrigin(wheelOrigin);
        m_wheels[i].setFillColor(m_config.wheelColor);
    }

    const float halfBase = m_config.wheelBase / 2.0f;
    const float halfTrack = m_config.trackWidth / 2.0f;

    m_wheels[0].setPosition(m_position + sf::Vector2f(halfBase, -halfTrack));
    m_wheels[1].setPosition(m_position + sf::Vector2f(halfBase, halfTrack));
    m_wheels[2].setPosition(m_position + sf::Vector2f(-halfBase, -halfTrack));
    m_wheels[3].setPosition(m_position + sf::Vector2f(-halfBase, halfTrack));
}

void AMR::setPose(const sf::Vector2f& position, float heading) {
    m_position = position;
    m_heading = heading;
    syncShapes();
}

void AMR::update(float dt, float vL, float vR) {
    const float v = (vL + vR) / 2.0f;
    const float omega = (vL - vR) / m_config.trackWidth;

    m_heading += omega * dt;
    m_position.x += v * std::cos(m_heading) * dt;
    m_position.y += v * std::sin(m_heading) * dt;

    syncShapes();
}

bool AMR::moveToward(const sf::Vector2f& target, float dt, float maxSpeed, float arrivalTolerance) {
    const sf::Vector2f delta = target - m_position;
    const float distance = std::sqrt((delta.x * delta.x) + (delta.y * delta.y));
    const float safeTolerance = std::max(0.0f, arrivalTolerance);

    if (distance <= safeTolerance) {
        m_position = target;
        syncShapes();
        return true;
    }

    if (dt <= 0.0f || maxSpeed <= 0.0f) {
        return false;
    }

    m_heading = std::atan2(delta.y, delta.x);
    const float step = std::min(maxSpeed * dt, distance);
    m_position += delta * (step / distance);

    const bool reached = distance - step <= safeTolerance;
    if (reached) {
        m_position = target;
    }

    syncShapes();
    return reached;
}

bool AMR::moveToward(
    const sf::Vector2f& target,
    float dt,
    float maxSpeed,
    float maxAngularSpeed,
    float arrivalTolerance
) {
    if (!std::isfinite(target.x) || !std::isfinite(target.y)
        || !std::isfinite(m_position.x) || !std::isfinite(m_position.y)
        || !std::isfinite(arrivalTolerance)) {
        return false;
    }

    const sf::Vector2f delta = target - m_position;
    const float distance = std::hypot(delta.x, delta.y);
    const float safeTolerance = std::max(0.0f, arrivalTolerance);
    if (distance <= safeTolerance) {
        m_position = target;
        syncShapes();
        return true;
    }

    if (!std::isfinite(dt) || !std::isfinite(maxSpeed) || !std::isfinite(maxAngularSpeed)
        || dt <= 0.0f || maxSpeed <= 0.0f || maxAngularSpeed <= 0.0f
        || m_config.trackWidth <= 0.0f) {
        return false;
    }

    const float targetHeading = std::atan2(delta.y, delta.x);
    const float headingError = normalizeAngle(targetHeading - m_heading);
    const float maxHeadingStep = maxAngularSpeed * dt;

    if (std::abs(headingError) > maxHeadingStep) {
        const float omega = std::copysign(maxAngularSpeed, headingError);
        const float halfWheelDifference = omega * m_config.trackWidth * 0.5f;
        update(dt, halfWheelDifference, -halfWheelDifference);
        return false;
    }

    const float step = std::min(maxSpeed * dt, distance);
    const float linearSpeed = step / dt;
    const float omega = headingError / dt;
    const float halfWheelDifference = omega * m_config.trackWidth * 0.5f;
    update(
        dt,
        linearSpeed + halfWheelDifference,
        linearSpeed - halfWheelDifference
    );

    const bool reached = distance - step <= safeTolerance;
    if (reached) {
        m_position = target;
        syncShapes();
    }
    return reached;
}

void AMR::syncShapes() {
    m_bodyShape.setPosition(m_position);
    m_bodyShape.setRotation(sf::radians(m_heading));

    const float halfBase = m_config.wheelBase / 2.0f;
    const float halfTrack = m_config.trackWidth / 2.0f;
    const sf::Vector2f localOffsets[4] = {
        sf::Vector2f(halfBase, -halfTrack),
        sf::Vector2f(halfBase, halfTrack),
        sf::Vector2f(-halfBase, -halfTrack),
        sf::Vector2f(-halfBase, halfTrack)
    };

    for (int i = 0; i < 4; ++i) {
        const float rx = localOffsets[i].x * std::cos(m_heading) - localOffsets[i].y * std::sin(m_heading);
        const float ry = localOffsets[i].x * std::sin(m_heading) + localOffsets[i].y * std::cos(m_heading);
        m_wheels[i].setPosition(m_position + sf::Vector2f(rx, ry));
        m_wheels[i].setRotation(sf::radians(m_heading));
    }
}

float AMR::getConservativeBodyRadius() const {
    return std::hypot(m_config.bodyLength * 0.5f, m_config.bodyWidth * 0.5f);
}

std::vector<sf::Vector2f> AMR::getCorners() const {
    std::vector<sf::Vector2f> corners;
    const float halfLength = m_config.bodyLength / 2.0f;
    const float halfWidth = m_config.bodyWidth / 2.0f;
    const sf::Vector2f localCorners[4] = {
        sf::Vector2f(halfLength, -halfWidth),
        sf::Vector2f(halfLength, halfWidth),
        sf::Vector2f(-halfLength, halfWidth),
        sf::Vector2f(-halfLength, -halfWidth)
    };

    for (int i = 0; i < 4; ++i) {
        const float rx = localCorners[i].x * std::cos(m_heading) - localCorners[i].y * std::sin(m_heading);
        const float ry = localCorners[i].x * std::sin(m_heading) + localCorners[i].y * std::cos(m_heading);
        corners.push_back(m_position + sf::Vector2f(rx, ry));
    }

    return corners;
}

bool AMR::containsPoint(const sf::Vector2f& worldPos) const {
    const sf::Vector2f delta = worldPos - m_position;
    const float cosHeading = std::cos(-m_heading);
    const float sinHeading = std::sin(-m_heading);
    const float localX = delta.x * cosHeading - delta.y * sinHeading;
    const float localY = delta.x * sinHeading + delta.y * cosHeading;
    const float halfLength = m_config.bodyLength * 0.5f;
    const float halfWidth = m_config.bodyWidth * 0.5f;

    return std::abs(localX) <= halfLength && std::abs(localY) <= halfWidth;
}

void AMR::draw(sf::RenderWindow& window) {
    for (int i = 0; i < 4; ++i) {
        window.draw(m_wheels[i]);
    }
    window.draw(m_bodyShape);
}
