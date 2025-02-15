#include "road_system.h"

RoadSystem::RoadSystem(SDL_Renderer* renderer)
    : m_renderer(renderer) {
    initializeLanes();
}

void RoadSystem::initializeLanes() {
    // Window center points
    const float centerX = 640.0f;  // 1280/2
    const float centerY = 360.0f;  // 720/2

    // Define road parameters
    const float roadWidth = 180.0f;          // Total width for 3 lanes
    const float laneWidth = roadWidth / 3.0f; // Width of each individual lane
    const float halfRoadWidth = roadWidth / 2.0f;
    const float intersectionSize = roadWidth;

    // Road A (North)
    float northStart = centerX - halfRoadWidth;
    m_lanes[LaneId::AL1_INCOMING] = std::make_unique<Lane>(m_renderer, LaneId::AL1_INCOMING,
        Vector2D(northStart + laneWidth * 0.5f, 0),            // Incoming lane
        Vector2D(northStart + laneWidth * 0.5f, centerY - halfRoadWidth), true);

    m_lanes[LaneId::AL2_PRIORITY] = std::make_unique<Lane>(m_renderer, LaneId::AL2_PRIORITY,
        Vector2D(northStart + laneWidth * 1.5f, centerY - halfRoadWidth), // Priority lane (outgoing)
        Vector2D(northStart + laneWidth * 1.5f, 0), false);

    m_lanes[LaneId::AL3_FREELANE] = std::make_unique<Lane>(m_renderer, LaneId::AL3_FREELANE,
        Vector2D(northStart + laneWidth * 2.5f, centerY - halfRoadWidth), // Free left turn (outgoing)
        Vector2D(northStart + laneWidth * 2.5f, 0), false);

    // Road B (East)
    float eastStart = centerY - halfRoadWidth;
    m_lanes[LaneId::BL1_INCOMING] = std::make_unique<Lane>(m_renderer, LaneId::BL1_INCOMING,
        Vector2D(1280, eastStart + laneWidth * 0.5f),          // Incoming lane
        Vector2D(centerX + halfRoadWidth, eastStart + laneWidth * 0.5f), true);

    m_lanes[LaneId::BL2_PRIORITY] = std::make_unique<Lane>(m_renderer, LaneId::BL2_PRIORITY,
        Vector2D(centerX + halfRoadWidth, eastStart + laneWidth * 1.5f),  // Priority lane (outgoing)
        Vector2D(1280, eastStart + laneWidth * 1.5f), false);

    m_lanes[LaneId::BL3_FREELANE] = std::make_unique<Lane>(m_renderer, LaneId::BL3_FREELANE,
        Vector2D(centerX + halfRoadWidth, eastStart + laneWidth * 2.5f),  // Free left turn (outgoing)
        Vector2D(1280, eastStart + laneWidth * 2.5f), false);

    // Road C (South)
    float southStart = centerX - halfRoadWidth;
    m_lanes[LaneId::CL1_INCOMING] = std::make_unique<Lane>(m_renderer, LaneId::CL1_INCOMING,
        Vector2D(southStart + laneWidth * 0.5f, 720),          // Incoming lane
        Vector2D(southStart + laneWidth * 0.5f, centerY + halfRoadWidth), true);

    m_lanes[LaneId::CL2_PRIORITY] = std::make_unique<Lane>(m_renderer, LaneId::CL2_PRIORITY,
        Vector2D(southStart + laneWidth * 1.5f, centerY + halfRoadWidth), // Priority lane (outgoing)
        Vector2D(southStart + laneWidth * 1.5f, 720), false);

    m_lanes[LaneId::CL3_FREELANE] = std::make_unique<Lane>(m_renderer, LaneId::CL3_FREELANE,
        Vector2D(southStart + laneWidth * 2.5f, centerY + halfRoadWidth), // Free left turn (outgoing)
        Vector2D(southStart + laneWidth * 2.5f, 720), false);

    // Road D (West)
    float westStart = centerY - halfRoadWidth;
    m_lanes[LaneId::DL1_INCOMING] = std::make_unique<Lane>(m_renderer, LaneId::DL1_INCOMING,
        Vector2D(0, westStart + laneWidth * 0.5f),             // Incoming lane
        Vector2D(centerX - halfRoadWidth, westStart + laneWidth * 0.5f), true);

    m_lanes[LaneId::DL2_PRIORITY] = std::make_unique<Lane>(m_renderer, LaneId::DL2_PRIORITY,
        Vector2D(centerX - halfRoadWidth, westStart + laneWidth * 1.5f),  // Priority lane (outgoing)
        Vector2D(0, westStart + laneWidth * 1.5f), false);

    m_lanes[LaneId::DL3_FREELANE] = std::make_unique<Lane>(m_renderer, LaneId::DL3_FREELANE,
        Vector2D(centerX - halfRoadWidth, westStart + laneWidth * 2.5f),  // Free left turn (outgoing)
        Vector2D(0, westStart + laneWidth * 2.5f), false);
}

void RoadSystem::render() const {
    // Draw main roads background first
    const float centerX = 640.0f;
    const float centerY = 360.0f;
    const float roadWidth = 180.0f;
    const float halfRoadWidth = roadWidth / 2.0f;

    // Draw vertical road background
    SDL_FRect verticalRoad = {
        centerX - halfRoadWidth,
        0,
        roadWidth,
        720
    };

    // Draw horizontal road background
    SDL_FRect horizontalRoad = {
        0,
        centerY - halfRoadWidth,
        1280,
        roadWidth
    };

    // Set road color (dark gray)
    SDL_SetRenderDrawColor(m_renderer, 50, 50, 50, 255);
    SDL_RenderFillRect(m_renderer, &verticalRoad);
    SDL_RenderFillRect(m_renderer, &horizontalRoad);

    // Draw intersection with slightly lighter color
    SDL_FRect intersection = {
        centerX - halfRoadWidth,
        centerY - halfRoadWidth,
        roadWidth,
        roadWidth
    };
    SDL_SetRenderDrawColor(m_renderer, 70, 70, 70, 255);
    SDL_RenderFillRect(m_renderer, &intersection);

    // Render all lanes on top
    for (const auto& [id, lane] : m_lanes) {
        lane->render();
    }
}

void RoadSystem::update(float deltaTime) {
    // Update traffic manager first
    m_trafficManager.update(deltaTime);

    // Update all lanes
    for (auto& [id, lane] : m_lanes) {
        lane->update(deltaTime);
    }
}

Lane* RoadSystem::getLane(LaneId id) {
    auto it = m_lanes.find(id);
    return it != m_lanes.end() ? it->second.get() : nullptr;
}

std::vector<Lane*> RoadSystem::getPriorityLanes() {
    std::vector<Lane*> priorityLanes;
    for (auto& [id, lane] : m_lanes) {
        if (lane->isPriorityLane()) {
            priorityLanes.push_back(lane.get());
        }
    }
    return priorityLanes;
}

void RoadSystem::addVehicle(Vehicle* vehicle, LaneId lane) {
    // Add to traffic manager
    m_trafficManager.addVehicle(vehicle, lane);

    // Add to lane
    if (auto it = m_lanes.find(lane); it != m_lanes.end()) {
        it->second->addVehicle(vehicle);
    }
}