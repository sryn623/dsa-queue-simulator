#include "vehicle.h"
#include <SDL3/SDL.h>

Vehicle::Vehicle(SDL_Renderer* renderer, int vehicle_id, LaneId startLane,
                Vector2D startPos, Direction facing)
    : m_renderer(renderer)
    , m_id(vehicle_id)
    , m_position(startPos)
    , m_velocity(0.0F, 0.0F)
    , m_status(VehicleStatus::MOVING)
    , m_currentLane(startLane)
    , m_targetLane(startLane)
    , m_facing(facing)
    , m_turnIntent(TurnBehaviour::STRAIGHT)
    , m_waitTime(0.0F)
    , m_isPriority(false) {
    checkPriority();
}

Vehicle::~Vehicle() = default;

void Vehicle::render() const {
    // Adjust vehicle dimensions
    const float carWidth = 20.0f;
    const float carLength = 40.0f;
    const float halfWidth = carWidth / 2.0f;
    const float halfLength = carLength / 2.0f;

    // Calculate corner positions based on direction
    SDL_Vertex vertices[6];
    SDL_FColor vehicleColor;
    if (m_isPriority) {
        vehicleColor = {1.0f, 0.0f, 0.0f, 1.0f}; // Red for priority
    } else {
        vehicleColor = {0.0f, 1.0f, 0.0f, 1.0f}; // Green for normal
    }

    switch(m_facing) {
        case Direction::NORTH:
        case Direction::SOUTH: {
            // For vertical movement, length is in Y direction
            vertices[0].position = {m_position.x - halfWidth, m_position.y - halfLength}; // Top left
            vertices[1].position = {m_position.x + halfWidth, m_position.y - halfLength}; // Top right
            vertices[2].position = {m_position.x - halfWidth, m_position.y + halfLength}; // Bottom left
            vertices[3].position = {m_position.x + halfWidth, m_position.y - halfLength}; // Top right
            vertices[4].position = {m_position.x + halfWidth, m_position.y + halfLength}; // Bottom right
            vertices[5].position = {m_position.x - halfWidth, m_position.y + halfLength}; // Bottom left
            break;
        }
        case Direction::EAST:
        case Direction::WEST: {
            // For horizontal movement, length is in X direction
            vertices[0].position = {m_position.x - halfLength, m_position.y - halfWidth}; // Left top
            vertices[1].position = {m_position.x + halfLength, m_position.y - halfWidth}; // Right top
            vertices[2].position = {m_position.x - halfLength, m_position.y + halfWidth}; // Left bottom
            vertices[3].position = {m_position.x + halfLength, m_position.y - halfWidth}; // Right top
            vertices[4].position = {m_position.x + halfLength, m_position.y + halfWidth}; // Right bottom
            vertices[5].position = {m_position.x - halfLength, m_position.y + halfWidth}; // Left bottom
            break;
        }
    }

    // Set colors for all vertices
    for (int i = 0; i < 6; i++) {
        vertices[i].color = vehicleColor;
    }

    // Draw vehicle body
    SDL_RenderGeometry(m_renderer, nullptr, vertices, 6, nullptr, 0);

    // Draw direction indicator (front of vehicle)
    SDL_Vertex indicator[3];
    SDL_FColor indicatorColor = {1.0f, 1.0f, 0.0f, 1.0f}; // Yellow

    float indicatorSize = 10.0f;
    switch(m_facing) {
        case Direction::NORTH: {
            indicator[0].position = {m_position.x, m_position.y - halfLength - 5};
            indicator[1].position = {m_position.x - 5, m_position.y - halfLength + 5};
            indicator[2].position = {m_position.x + 5, m_position.y - halfLength + 5};
            break;
        }
        case Direction::SOUTH: {
            indicator[0].position = {m_position.x, m_position.y + halfLength + 5};
            indicator[1].position = {m_position.x - 5, m_position.y + halfLength - 5};
            indicator[2].position = {m_position.x + 5, m_position.y + halfLength - 5};
            break;
        }
        case Direction::EAST: {
            indicator[0].position = {m_position.x + halfLength + 5, m_position.y};
            indicator[1].position = {m_position.x + halfLength - 5, m_position.y - 5};
            indicator[2].position = {m_position.x + halfLength - 5, m_position.y + 5};
            break;
        }
        case Direction::WEST: {
            indicator[0].position = {m_position.x - halfLength - 5, m_position.y};
            indicator[1].position = {m_position.x - halfLength + 5, m_position.y - 5};
            indicator[2].position = {m_position.x - halfLength + 5, m_position.y + 5};
            break;
        }
    }

    // Set indicator colors
    for (int i = 0; i < 3; i++) {
        indicator[i].color = indicatorColor;
    }

    // Draw direction indicator
    SDL_RenderGeometry(m_renderer, nullptr, indicator, 3, nullptr, 0);
}

void Vehicle::update(float deltaTime) {
    updateState();
    updatePosition(deltaTime);

    if (m_status == VehicleStatus::WAITING) {
        m_waitTime += deltaTime;
    }
}

void Vehicle::updatePosition(float deltaTime) {
    float speed = 100.0F;

    if (m_status == VehicleStatus::MOVING || m_status == VehicleStatus::TURNING) {
        switch (m_facing) {
            case Direction::NORTH:
                m_velocity = Vector2D(0.0F, -speed);
                break;
            case Direction::SOUTH:
                m_velocity = Vector2D(0.0F, speed);
                break;
            case Direction::EAST:
                m_velocity = Vector2D(speed, 0.0F);
                break;
            case Direction::WEST:
                m_velocity = Vector2D(-speed, 0.0F);
                break;
        }

        m_position.x += m_velocity.x * deltaTime;
        m_position.y += m_velocity.y * deltaTime;
    }
}

void Vehicle::updateState() {
    if (checkCollision()) {
        m_status = VehicleStatus::STOPPED;
    } else if (m_status == VehicleStatus::STOPPED) {
        m_status = VehicleStatus::MOVING;
    }

    if (m_status == VehicleStatus::TURNING && m_currentLane != m_targetLane) {
        m_currentLane = m_targetLane;
        m_status = VehicleStatus::MOVING;
    }
}

bool Vehicle::checkCollision() const {
    const float margin = 50.0F;
    return (m_position.x < -margin || m_position.x > 1280 + margin ||
            m_position.y < -margin || m_position.y > 720 + margin);
}

void Vehicle::changeLane(LaneId newLane) {
    if (m_currentLane != newLane) {
        m_targetLane = newLane;
        m_status = VehicleStatus::TURNING;
    }
}

void Vehicle::setTurnDirection(TurnBehaviour turn) {
    m_turnIntent = turn;
    m_status = VehicleStatus::TURNING;

    switch(turn) {
        case TurnBehaviour::TURNING_LEFT:
            switch(m_facing) {
                case Direction::NORTH: m_facing = Direction::WEST; break;
                case Direction::SOUTH: m_facing = Direction::EAST; break;
                case Direction::EAST: m_facing = Direction::NORTH; break;
                case Direction::WEST: m_facing = Direction::SOUTH; break;
            }
            break;
        case TurnBehaviour::TURNING_RIGHT:
            switch(m_facing) {
                case Direction::NORTH: m_facing = Direction::EAST; break;
                case Direction::SOUTH: m_facing = Direction::WEST; break;
                case Direction::EAST: m_facing = Direction::SOUTH; break;
                case Direction::WEST: m_facing = Direction::NORTH; break;
            }
            break;
        default:
            break;
    }
}

void Vehicle::checkPriority() {
    m_isPriority = (m_currentLane == LaneId::AL2_PRIORITY ||
                    m_currentLane == LaneId::BL2_PRIORITY ||
                    m_currentLane == LaneId::CL2_PRIORITY ||
                    m_currentLane == LaneId::DL2_PRIORITY);
}

bool Vehicle::needsTurnLeft(LaneId oldLane, LaneId newLane) const {
    return (oldLane != newLane);
}