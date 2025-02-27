// src/core/Vehicle.cpp
#include "core/Vehicle.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

// Add this to the Vehicle constructor:
Vehicle::Vehicle(uint32_t vehicleId, Direction dir, LaneId lane, bool emergency)
    : id(vehicleId)
    , direction(dir)
    , currentLane(lane)
    , targetLane(lane)
    , waitTime(0.0f)
    , isProcessing(false)
    , turnProgress(0.0f)
    , hasStartedTurn(false)
    , speed(0.0f)
    , position(0.0f)
    , entryTime(std::chrono::steady_clock::now())
    , isEmergency(emergency)
    , turning(false)
{
    // Initialize position parameters
    pos.x = 0.0f;
    pos.y = 0.0f;
    pos.angle = 0.0f;
    pos.targetX = 0.0f;
    pos.targetY = 0.0f;
    pos.targetAngle = 0.0f;
    pos.turnPosX = 0.0f;
    pos.turnPosY = 0.0f;
    pos.turnCenter_x = 0.0f;
    pos.turnCenter_y = 0.0f;
    pos.startAngle = 0.0f;
    pos.endAngle = 0.0f;
    pos.turnRadius = 0.0f;

    // Set initial position based on lane
    initializePosition();
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
        turnProgress = std::min(1.0f, turnProgress + delta * SimConstants::TURN_SPEED);
    }
}

void Vehicle::startTurn() {
    hasStartedTurn = true;
    turning = true;
    turnProgress = 0.0f;
    speed = SimConstants::VEHICLE_TURN_SPEED;
}


void Vehicle::setupTurn(float centerX, float centerY, float radius, float startAng, float endAng) {
    pos.turnCenter_x = centerX;
    pos.turnCenter_y = centerY;
    pos.turnRadius = radius;
    pos.startAngle = startAng;
    pos.endAngle = endAng;
    pos.turnPosX = pos.x;
    pos.turnPosY = pos.y;
}


void Vehicle::calculateNewTargetPosition() {
    using namespace SimConstants;

    // Similar to initializePosition but set targets for where to go after a turn
    int roadGroup = static_cast<int>(currentLane) / 3;
    int laneInRoad = static_cast<int>(currentLane) % 3;
    float laneOffset = (laneInRoad - 1) * LANE_WIDTH;

    switch (roadGroup) {
        case 0: // Road A - heading East
            pos.targetX = WINDOW_WIDTH;
            pos.targetY = CENTER_Y + laneOffset;
            pos.targetAngle = 0.0f;
            break;
        case 1: // Road B - heading South
            pos.targetX = CENTER_X + laneOffset;
            pos.targetY = WINDOW_HEIGHT;
            pos.targetAngle = M_PI/2.0f;
            break;
        case 2: // Road C - heading West
            pos.targetX = 0.0f;
            pos.targetY = CENTER_Y - laneOffset;
            pos.targetAngle = M_PI;
            break;
        case 3: // Road D - heading North
            pos.targetX = CENTER_X - laneOffset;
            pos.targetY = 0.0f;
            pos.targetAngle = -M_PI/2.0f;
            break;
    }
}

// Improve turn mechanics:
void Vehicle::updateTurn(float deltaTime) {
    if (!turning) return;

    // Calculate easedProgress for smoother turns
    turnProgress = std::min(1.0f, turnProgress + deltaTime * SimConstants::TURN_SPEED);
    float easedProgress = turnProgress * turnProgress * (3.0f - 2.0f * turnProgress);

    // Calculate current angle along the turn
    float currentAngle = pos.startAngle + (pos.endAngle - pos.startAngle) * easedProgress;

    // Calculate new position along the turn
    pos.turnPosX = pos.turnCenter_x + pos.turnRadius * cosf(currentAngle);
    pos.turnPosY = pos.turnCenter_y + pos.turnRadius * sinf(currentAngle);

    // Update position
    pos.x = pos.turnPosX;
    pos.y = pos.turnPosY;

    // Update angle to face tangent to circle
    pos.angle = currentAngle + static_cast<float>(M_PI)/2.0f;

    // Check if turn is complete
    if (turnProgress >= 1.0f) {
        turning = false;
        currentLane = targetLane;  // Update lane after completing turn

        // Set new target position based on new lane
        calculateNewTargetPosition();
    }
}

void Vehicle::setTurnPosition(float x, float y) {
    pos.turnPosX = x;
    pos.turnPosY = y;
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

    if (turning) {
        updateTurn(deltaTime);
        return;
    }

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

        // Update angle smoothly
        float targetAngle = std::atan2f(dy, dx);
        float angleDiff = targetAngle - pos.angle;

        // Normalize angle to [-π, π]
        while (angleDiff > static_cast<float>(M_PI)) {
            angleDiff -= 2.0f * static_cast<float>(M_PI);
        }
        while (angleDiff < -static_cast<float>(M_PI)) {
            angleDiff += 2.0f * static_cast<float>(M_PI);
        }

        pos.angle += angleDiff * 0.1f; // Smooth angle change
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

void Vehicle::calculateTurnParameters(float roadWidth, float laneWidth, float centerX, float centerY) {
    int quadrant = static_cast<int>(currentLane) / 3; // 0-3 for W,N,E,S
    bool isLeftTurn = direction == Direction::LEFT;
    bool isRightTurn = direction == Direction::RIGHT;
    float turnOffset = roadWidth * 0.25f;
    float turnRadius = SimConstants::TURN_GUIDE_RADIUS;

    // Adjust radius based on turn direction
    if (isLeftTurn) turnRadius *= 1.2f;
    if (isRightTurn) turnRadius *= 0.8f;

    // Set turn center and angle based on approach direction and turn type
    switch (quadrant) {
        case 0: // From West (A road)
            if (isLeftTurn) {
                setupTurn(
                    centerX - roadWidth/2.0f + turnOffset,
                    centerY - roadWidth/2.0f - turnOffset,
                    turnRadius,
                    0.0f,
                    -static_cast<float>(M_PI)/2.0f
                );
            } else if (isRightTurn) {
                setupTurn(
                    centerX - roadWidth/2.0f + turnOffset,
                    centerY + roadWidth/2.0f + turnOffset,
                    turnRadius,
                    0.0f,
                    static_cast<float>(M_PI)/2.0f
                );
            }
            break;

        case 1: // From North (B road)
            if (isLeftTurn) {
                setupTurn(
                    centerX + roadWidth/2.0f + turnOffset,
                    centerY - roadWidth/2.0f + turnOffset,
                    turnRadius,
                    static_cast<float>(M_PI)/2.0f,
                    0.0f
                );
            } else if (isRightTurn) {
                setupTurn(
                    centerX - roadWidth/2.0f - turnOffset,
                    centerY - roadWidth/2.0f + turnOffset,
                    turnRadius,
                    static_cast<float>(M_PI)/2.0f,
                    static_cast<float>(M_PI)
                );
            }
            break;

        case 2: // From East (C road)
            if (isLeftTurn) {
                setupTurn(
                    centerX + roadWidth/2.0f - turnOffset,
                    centerY + roadWidth/2.0f + turnOffset,
                    turnRadius,
                    static_cast<float>(M_PI),
                    static_cast<float>(M_PI)/2.0f
                );
            } else if (isRightTurn) {
                setupTurn(
                    centerX + roadWidth/2.0f - turnOffset,
                    centerY - roadWidth/2.0f - turnOffset,
                    turnRadius,
                    static_cast<float>(M_PI),
                    -static_cast<float>(M_PI)/2.0f
                );
            }
            break;

        case 3: // From South (D road)
            if (isLeftTurn) {
                setupTurn(
                    centerX - roadWidth/2.0f - turnOffset,
                    centerY + roadWidth/2.0f - turnOffset,
                    turnRadius,
                    -static_cast<float>(M_PI)/2.0f,
                    static_cast<float>(M_PI)
                );
            } else if (isRightTurn) {
                setupTurn(
                    centerX + roadWidth/2.0f + turnOffset,
                    centerY + roadWidth/2.0f - turnOffset,
                    turnRadius,
                    -static_cast<float>(M_PI)/2.0f,
                    0.0f
                );
            }
            break;
    }
}

// Initialize vehicle position based on lane
void Vehicle::initializePosition() {
    using namespace SimConstants;

    // Get base lane information
    int roadGroup = static_cast<int>(currentLane) / 3;  // 0=A, 1=B, 2=C, 3=D
    int laneInRoad = static_cast<int>(currentLane) % 3; // Lane position in road

    // Determine spawn position and angle based on road/lane
    float laneOffset = (laneInRoad - 1) * LANE_WIDTH; // -LANE_WIDTH, 0, or +LANE_WIDTH

    switch (roadGroup) {
        case 0: // Road A (West)
            pos.x = 0.0f;  // Left edge of screen
            pos.y = CENTER_Y + laneOffset;
            pos.angle = 0.0f;  // Facing right (East)
            break;

        case 1: // Road B (North)
            pos.x = CENTER_X + laneOffset;
            pos.y = 0.0f;  // Top edge of screen
            pos.angle = M_PI/2.0f;  // Facing down (South)
            break;

        case 2: // Road C (East)
            pos.x = WINDOW_WIDTH;  // Right edge of screen
            pos.y = CENTER_Y - laneOffset;  // Note sign change for consistent lane order
            pos.angle = M_PI;  // Facing left (West)
            break;

        case 3: // Road D (South)
            pos.x = CENTER_X - laneOffset;  // Note sign change for consistent lane order
            pos.y = WINDOW_HEIGHT;  // Bottom edge of screen
            pos.angle = -M_PI/2.0f;  // Facing up (North)
            break;
    }

    // Set initial target to same as current position
    pos.targetX = pos.x;
    pos.targetY = pos.y;
    pos.targetAngle = pos.angle;

    // Initialize animation position for queue
    if (roadGroup == 0 || roadGroup == 2) {
        // Horizontal roads (A and C) - use x for position
        position = pos.x;
    } else {
        // Vertical roads (B and D) - use y for position
        position = pos.y;
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

float Vehicle::calculateTurnAngle(Direction dir, LaneId fromLane, LaneId) {
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