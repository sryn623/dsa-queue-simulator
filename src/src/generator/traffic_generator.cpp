#include "traffic_generator.h"

TrafficGenerator::TrafficGenerator(SDL_Renderer* renderer)
    : m_renderer(renderer)
    , m_rng(std::random_device{}())
    , m_vehicleCount(0) {
}

Vehicle* TrafficGenerator::generateVehicle() {
    // Only pick from incoming lanes (AL1, BL1, CL1, DL1)
    int road = m_rng() % 4;  // 0=North, 1=East, 2=South, 3=West

    Vector2D startPos;
    Direction facing;
    LaneId lane;

    const float centerX = 640.0f;
    const float centerY = 360.0f;
    const float roadWidth = 180.0f;
    const float laneWidth = roadWidth / 3.0f;
    const float halfRoadWidth = roadWidth / 2.0f;

    switch(road) {
        case 0: { // North Road (only AL1 incoming)
            float x = centerX - halfRoadWidth + (laneWidth * 0.5f); // First lane only
            startPos = Vector2D(x, -40);
            facing = Direction::SOUTH;
            lane = LaneId::AL1_INCOMING;
            break;
        }
        case 1: { // East Road (only BL1 incoming)
            float y = centerY - halfRoadWidth + (laneWidth * 0.5f); // First lane only
            startPos = Vector2D(1320, y);
            facing = Direction::WEST;
            lane = LaneId::BL1_INCOMING;
            break;
        }
        case 2: { // South Road (only CL1 incoming)
            float x = centerX - halfRoadWidth + (laneWidth * 0.5f); // First lane only
            startPos = Vector2D(x, 760);
            facing = Direction::NORTH;
            lane = LaneId::CL1_INCOMING;
            break;
        }
        default: { // West Road (only DL1 incoming)
            float y = centerY - halfRoadWidth + (laneWidth * 0.5f); // First lane only
            startPos = Vector2D(-40, y);
            facing = Direction::EAST;
            lane = LaneId::DL1_INCOMING;
            break;
        }
    }

    return new Vehicle(m_renderer, m_vehicleCount++, lane, startPos, facing);
}