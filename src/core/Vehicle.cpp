
// src/core/Vehicle.cpp
#include "core/Vehicle.h"
#include <sstream>
#include <iomanip>
#include <cmath>

Vehicle::Vehicle(uint32_t vehicleId, Direction dir, LaneId lane)
    : id(vehicleId)
    , direction(dir)
    , currentLane(lane)
    , waitTime(0.0f)
    , isProcessing(false)
    , turnProgress(0.0f)
    , hasStartedTurn(false)
    , speed(0.0f)
    , position(0.0f)
    , entryTime(std::chrono::steady_clock::now())
{
    // Initialize position
    pos.x = 0.0f;
    pos.y = 0.0f;
    pos.angle = 0.0f;
    pos.targetX = 0.0f;
    pos.targetY = 0.0f;
    pos.targetAngle = 0.0f;
}

void Vehicle::setProcessing(bool processing) {
    isProcessing = processing;
    if (processing) {
        speed = SimConstants::VEHICLE_BASE_SPEED;
    }
}

void Vehicle::updateWaitTime(float delta) {
    if (!isProcessing) {
        waitTime += delta;
    }
}

void Vehicle::updateTurnProgress(float delta) {
    if (hasStartedTurn && turnProgress < 1.0f) {
        turnProgress = std::min(1.0f, turnProgress + delta);
    }
}

void Vehicle::startTurn() {
    hasStartedTurn = true;
    turnProgress = 0.0f;
    speed = SimConstants::VEHICLE_TURN_SPEED;
}

void Vehicle::setSpeed(float newSpeed) {
    speed = newSpeed;
}

void Vehicle::setPosition(float newPos) {
    position = newPos;
}

void Vehicle::setTargetPosition(float x, float y, float angle) {
    pos.targetX = x;
    pos.targetY = y;
    pos.targetAngle = angle;
}

void Vehicle::updateMovement(float deltaTime) {
    if (!isProcessing) return;

    // Calculate distance to target
    float dx = pos.targetX - pos.x;
    float dy = pos.targetY - pos.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    // Update position if not at target
    if (distance > 0.1f) {
        float moveSpeed = speed * deltaTime;
        float moveRatio = std::min(1.0f, moveSpeed / distance);

        pos.x += dx * moveRatio;
        pos.y += dy * moveRatio;

        // Update angle smoothly using floats explicitly
        float targetAngle = std::atan2f(dy, dx);
        float angleDiff = targetAngle - pos.angle;

        // Normalize angle to [-π, π] using float constants
        while (angleDiff > static_cast<float>(M_PI)) {
            angleDiff -= 2.0f * static_cast<float>(M_PI);
        }
        while (angleDiff < -static_cast<float>(M_PI)) {
            angleDiff += 2.0f * static_cast<float>(M_PI);
        }

        pos.angle += angleDiff * moveRatio;
    }
}

bool Vehicle::hasReachedTarget() const {
    float dx = pos.targetX - pos.x;
    float dy = pos.targetY - pos.y;
    return std::sqrt(dx * dx + dy * dy) < 0.1f;
}

float Vehicle::calculateTurnRadius() const {
    switch (direction) {
        case Direction::LEFT:
            return SimConstants::TURN_GUIDE_RADIUS * 1.2f;
        case Direction::RIGHT:
            return SimConstants::TURN_GUIDE_RADIUS * 0.8f;
        default:
            return SimConstants::TURN_GUIDE_RADIUS;
    }
}

float Vehicle::calculateLanePosition(LaneId lane, size_t queuePosition) {
    using namespace SimConstants;
    float baseOffset = QUEUE_START_OFFSET + queuePosition * QUEUE_SPACING;

    switch (lane) {
        case LaneId::AL1_INCOMING:
        case LaneId::AL2_PRIORITY:
        case LaneId::AL3_FREELANE:
            return CENTER_X - baseOffset;

        case LaneId::BL1_INCOMING:
        case LaneId::BL2_NORMAL:
        case LaneId::BL3_FREELANE:
            return CENTER_Y - baseOffset;

        case LaneId::CL1_INCOMING:
        case LaneId::CL2_NORMAL:
        case LaneId::CL3_FREELANE:
            return CENTER_X + baseOffset;

        case LaneId::DL1_INCOMING:
        case LaneId::DL2_NORMAL:
        case LaneId::DL3_FREELANE:
            return CENTER_Y + baseOffset;

        default:
            return 0.0f;
    }
}

float Vehicle::calculateTurnAngle(Direction dir, LaneId fromLane, LaneId /*toLane*/) {
    const float WEST_ANGLE = 0.0f;
    const float NORTH_ANGLE = static_cast<float>(M_PI) / 2.0f;
    const float EAST_ANGLE = static_cast<float>(M_PI);
    const float SOUTH_ANGLE = -static_cast<float>(M_PI) / 2.0f;

    // Get base angle from source lane
    float baseAngle;
    if (fromLane <= LaneId::AL3_FREELANE) baseAngle = WEST_ANGLE;
    else if (fromLane <= LaneId::BL3_FREELANE) baseAngle = NORTH_ANGLE;
    else if (fromLane <= LaneId::CL3_FREELANE) baseAngle = EAST_ANGLE;
    else baseAngle = SOUTH_ANGLE;

    // Adjust for turn direction
    switch (dir) {
        case Direction::LEFT:
            return baseAngle - static_cast<float>(M_PI) / 2.0f;
        case Direction::RIGHT:
            return baseAngle + static_cast<float>(M_PI) / 2.0f;
        default:
            return baseAngle;
    }
}

float Vehicle::getTimeInSystem() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<float>(now - entryTime).count();
}

std::string Vehicle::toString() const {
    std::stringstream ss;
    ss << "Vehicle[ID:" << id
       << ", Lane:" << static_cast<int>(currentLane)
       << ", Dir:" << static_cast<int>(direction)
       << ", Pos:(" << std::fixed << std::setprecision(1)
       << pos.x << "," << pos.y << ")"
       << ", Wait:" << std::setprecision(1) << waitTime << "s"
       << ", Turn:" << (hasStartedTurn ? "Yes" : "No")
       << ", Progress:" << std::setprecision(2) << turnProgress * 100 << "%]";
    return ss.str();
}