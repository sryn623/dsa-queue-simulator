#include "lane.h"
#include <cmath>

Lane::Lane(SDL_Renderer* renderer, LaneId id, Vector2D start, Vector2D end, bool isPriority)
    : m_renderer(renderer)
    , m_id(id)
    , m_startPos(start)
    , m_endPos(end)
    , m_isPriority(isPriority) {
}

void Lane::addVehicle(Vehicle* vehicle) {
    if (vehicle) {
        m_vehicles.push_back(vehicle);
        positionVehicleInQueue(vehicle, m_vehicles.size() - 1);
    }
}

void Lane::update(float deltaTime) {
    for (size_t i = 0; i < m_vehicles.size(); ++i) {
        if (m_vehicles[i]) {
            positionVehicleInQueue(m_vehicles[i], i);
            m_vehicles[i]->update(deltaTime);
        }
    }
}

void Lane::render() const {
    // Draw lane boundaries
    SDL_FColor laneColor;
    if (m_isPriority) {
        laneColor = {1.0f, 1.0f, 0.0f, 1.0f}; // Yellow for priority
    } else {
        laneColor = {1.0f, 1.0f, 1.0f, 1.0f}; // White for normal
    }

    // Draw dashed lines
    const float dashLength = 20.0f;
    const float gapLength = 20.0f;
    const float lineWidth = 3.0f;

    Vector2D direction = {
        m_endPos.x - m_startPos.x,
        m_endPos.y - m_startPos.y
    };

    float totalLength = sqrt(direction.x * direction.x + direction.y * direction.y);
    direction.x /= totalLength;
    direction.y /= totalLength;

    Vector2D perp = {-direction.y, direction.x};

    float currentDist = 0;
    while (currentDist < totalLength) {
        float dashEnd = std::min(currentDist + dashLength, totalLength);

        SDL_Vertex lineVerts[6];
        Vector2D start = {
            m_startPos.x + direction.x * currentDist,
            m_startPos.y + direction.y * currentDist
        };
        Vector2D end = {
            m_startPos.x + direction.x * dashEnd,
            m_startPos.y + direction.y * dashEnd
        };

        // Create vertices for dashed line segment
        for (int i = 0; i < 6; i++) {
            lineVerts[i].color = laneColor;
        }

        lineVerts[0].position = {start.x + perp.x * lineWidth, start.y + perp.y * lineWidth};
        lineVerts[1].position = {end.x + perp.x * lineWidth, end.y + perp.y * lineWidth};
        lineVerts[2].position = {start.x - perp.x * lineWidth, start.y - perp.y * lineWidth};
        lineVerts[3].position = {end.x + perp.x * lineWidth, end.y + perp.y * lineWidth};
        lineVerts[4].position = {end.x - perp.x * lineWidth, end.y - perp.y * lineWidth};
        lineVerts[5].position = {start.x - perp.x * lineWidth, start.y - perp.y * lineWidth};

        SDL_RenderGeometry(m_renderer, nullptr, lineVerts, 6, nullptr, 0);

        currentDist += dashLength + gapLength;
    }

    // Render all vehicles
    for (const auto* vehicle : m_vehicles) {
        if (vehicle) {
            vehicle->render();
        }
    }
}

void Lane::positionVehicleInQueue(Vehicle* vehicle, size_t queuePosition) {
    if (!vehicle) return;

    const float vehicleSpacing = 60.0f;
    float offset = static_cast<float>(queuePosition) * vehicleSpacing;

    Vector2D direction = {
        m_endPos.x - m_startPos.x,
        m_endPos.y - m_startPos.y
    };

    float length = sqrt(direction.x * direction.x + direction.y * direction.y);
    direction.x /= length;
    direction.y /= length;

    Vector2D newPos = {
        m_startPos.x + direction.x * offset,
        m_startPos.y + direction.y * offset
    };

    vehicle->setPosition(newPos);
}