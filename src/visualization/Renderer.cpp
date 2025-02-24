

#include "visualization/Renderer.h"
#include <iostream>
#include <cmath>
#include <cmath>
Renderer::Renderer()
    : window(nullptr)
    , renderer(nullptr)
    , debugMode(false) {
}

Renderer::~Renderer() {
    cleanup();
}

bool Renderer::initialize() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return false;
    }

    window = SDL_CreateWindow(
        "Traffic Junction Simulator",
        SimConstants::WINDOW_WIDTH,
        SimConstants::WINDOW_HEIGHT,
        SDL_WINDOW_RESIZABLE
    );

    if (!window) {
        std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
        return false;
    }

    renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        std::cerr << "Renderer creation failed: " << SDL_GetError() << std::endl;
        return false;
    }

    return true;
}

void Renderer::render(const TrafficManager& trafficManager) {
    // Clear screen with black background
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Core rendering
    renderBackground();
    renderRoads();
    renderLanes();
    renderIntersection();
    renderCrosswalks();
    renderStopLines();
    renderDirectionalArrows();

    // Traffic elements
    renderTrafficLights(trafficManager.getTrafficLights());
    renderVehicles(trafficManager.getActiveVehicles());

    // Priority mode indicators
    if (trafficManager.isInPriorityMode()) {
        renderPriorityLaneIndicator();
    }

    // Debug visualization
    if (debugMode) {
        if (showGrid) {
            drawDebugGrid();
        }
        renderLaneIdentifiers();
        renderVehicleCount(trafficManager);
        debugOverlay.render(renderer, trafficManager);
    }

    SDL_RenderPresent(renderer);
}


void Renderer::renderBackground() {
    // Create a gradient sky effect
    for (int y = 0; y < SimConstants::WINDOW_HEIGHT; ++y) {
        float t = static_cast<float>(y) / SimConstants::WINDOW_HEIGHT;
        uint8_t skyR = static_cast<uint8_t>(135 * (1 - t) + 30 * t);
        uint8_t skyG = static_cast<uint8_t>(206 * (1 - t) + 30 * t);
        uint8_t skyB = static_cast<uint8_t>(235 * (1 - t) + 30 * t);

        SDL_SetRenderDrawColor(renderer, skyR, skyG, skyB, 255);
        SDL_RenderLine(renderer, 0, y, SimConstants::WINDOW_WIDTH, y);
    }

    // Render grass areas with texture
    renderGrassAreas();
}

void Renderer::renderGrassAreas() {
    using namespace SimConstants;
    SDL_SetRenderDrawColor(renderer, 34, 139, 34, 255); // Base grass color

    // Define the four corner grass areas around the intersection
    SDL_FRect grassAreas[] = {
        // Top-left quadrant
        {0, 0, CENTER_X - ROAD_WIDTH/2.0f, CENTER_Y - ROAD_WIDTH/2.0f},
        // Top-right quadrant
        {CENTER_X + ROAD_WIDTH/2.0f, 0,
         static_cast<float>(WINDOW_WIDTH) - (CENTER_X + ROAD_WIDTH/2.0f),
         CENTER_Y - ROAD_WIDTH/2.0f},
        // Bottom-left quadrant
        {0, CENTER_Y + ROAD_WIDTH/2.0f,
         CENTER_X - ROAD_WIDTH/2.0f,
         static_cast<float>(WINDOW_HEIGHT) - (CENTER_Y + ROAD_WIDTH/2.0f)},
        // Bottom-right quadrant
        {CENTER_X + ROAD_WIDTH/2.0f, CENTER_Y + ROAD_WIDTH/2.0f,
         static_cast<float>(WINDOW_WIDTH) - (CENTER_X + ROAD_WIDTH/2.0f),
         static_cast<float>(WINDOW_HEIGHT) - (CENTER_Y + ROAD_WIDTH/2.0f)}
    };

    // Fill base grass areas
    for (const auto& area : grassAreas) {
        SDL_RenderFillRect(renderer, &area);
    }

    // Add grass texture variation using random dots
    SDL_SetRenderDrawColor(renderer, 28, 120, 28, 255);
    for (int i = 0; i < 2000; i++) {
        int x = rand() % WINDOW_WIDTH;
        int y = rand() % WINDOW_HEIGHT;

        // Check if point is in grass area (not on road)
        bool inRoad = (x > CENTER_X - ROAD_WIDTH/2 && x < CENTER_X + ROAD_WIDTH/2) ||
                     (y > CENTER_Y - ROAD_WIDTH/2 && y < CENTER_Y + ROAD_WIDTH/2);

        if (!inRoad) {
            SDL_RenderPoint(renderer, x, y);
        }
    }
}

void Renderer::renderRoads() {
    using namespace SimConstants;

    // Main road surface with asphalt texture
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);

    // Draw horizontal road
    SDL_FRect horizontalRoad = {
        0, CENTER_Y - ROAD_WIDTH/2.0f,
        static_cast<float>(WINDOW_WIDTH),
        static_cast<float>(ROAD_WIDTH)
    };
    SDL_RenderFillRect(renderer, &horizontalRoad);

    // Draw vertical road
    SDL_FRect verticalRoad = {
        CENTER_X - ROAD_WIDTH/2.0f, 0,
        static_cast<float>(ROAD_WIDTH),
        static_cast<float>(WINDOW_HEIGHT)
    };
    SDL_RenderFillRect(renderer, &verticalRoad);

    // Add road edges and curbs
    renderRoadEdges();
}

void Renderer::renderRoadEdges() {
    using namespace SimConstants;

    SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255);
    const float CURB_WIDTH = 4.0f;

    // Render curbs for all road edges
    SDL_FRect curbs[] = {
        // Horizontal road curbs
        {0, CENTER_Y - ROAD_WIDTH/2.0f,
         static_cast<float>(WINDOW_WIDTH), CURB_WIDTH},
        {0, CENTER_Y + ROAD_WIDTH/2.0f - CURB_WIDTH,
         static_cast<float>(WINDOW_WIDTH), CURB_WIDTH},

        // Vertical road curbs
        {CENTER_X - ROAD_WIDTH/2.0f, 0,
         CURB_WIDTH, static_cast<float>(WINDOW_HEIGHT)},
        {CENTER_X + ROAD_WIDTH/2.0f - CURB_WIDTH, 0,
         CURB_WIDTH, static_cast<float>(WINDOW_HEIGHT)}
    };

    for (const auto& curb : curbs) {
        SDL_RenderFillRect(renderer, &curb);
    }
}

void Renderer::renderLanes() {
    using namespace SimConstants;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    float laneWidth = ROAD_WIDTH / 3.0f;

    // Draw horizontal lane dividers
    for (int i = 1; i < 3; i++) {
        float y = CENTER_Y - ROAD_WIDTH/2.0f + i * laneWidth;
        renderDashedLine(0, y, WINDOW_WIDTH, y);
    }

    // Draw vertical lane dividers
    for (int i = 1; i < 3; i++) {
        float x = CENTER_X - ROAD_WIDTH/2.0f + i * laneWidth;
        renderDashedLine(x, 0, x, WINDOW_HEIGHT);
    }

    // Render special lane markings
    renderPriorityLane();
}

void Renderer::renderPriorityLane() {
    using namespace SimConstants;

    // Highlight AL2 priority lane with semi-transparent orange
    SDL_SetRenderDrawColor(renderer, 255, 165, 0, 100);
    float laneWidth = ROAD_WIDTH / 3.0f;

    SDL_FRect priorityLane = {
        0,
        CENTER_Y - laneWidth/2.0f,
        static_cast<float>(CENTER_X - ROAD_WIDTH/2.0f),
        laneWidth
    };

    SDL_RenderFillRect(renderer, &priorityLane);
}

void Renderer::renderTrafficLight(float x, float y, float rotation, LightState state) {
    // Constants for traffic light dimensions
    const float LIGHT_SPACING = 15.0f;        // Space between each light
    const float LIGHT_RADIUS = LIGHT_SIZE/2.0f;

    // Calculate oriented position for the traffic light housing
    float orientedX = x;
    float orientedY = y;

    // Apply rotation if needed to orient the traffic light correctly
    if (rotation != 0.0f) {
        SDL_FPoint rotated = rotatePoint(x, y,
            static_cast<float>(SimConstants::CENTER_X),
            static_cast<float>(SimConstants::CENTER_Y),
            rotation
        );
        orientedX = rotated.x;
        orientedY = rotated.y;
    }

    // Draw the traffic light housing (black background box)
    SDL_SetRenderDrawColor(renderer, 70, 70, 70, 255);  // Dark gray color
    SDL_FRect housing = {
        orientedX - HOUSING_PADDING,
        orientedY - HOUSING_PADDING,
        LIGHT_SIZE + (HOUSING_PADDING * 2.0f),
        (LIGHT_SIZE * 3.0f) + (HOUSING_PADDING * 4.0f)  // Room for three lights
    };
    SDL_RenderFillRect(renderer, &housing);

    // Draw outline for the housing
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);  // Lighter gray for border
    SDL_RenderRect(renderer, &housing);

    // Calculate positions for each light
    float centerX = orientedX + LIGHT_SIZE/2.0f;
    float redY = orientedY + LIGHT_SIZE/2.0f;
    float yellowY = redY + LIGHT_SIZE + LIGHT_SPACING;
    float greenY = yellowY + LIGHT_SIZE + LIGHT_SPACING;

    // Draw red light
    SDL_SetRenderDrawColor(renderer,
        state == LightState::RED ? 255 : 64,  // Bright red when active, dim when inactive
        0, 0, 255);
    renderCircle(centerX, redY, LIGHT_RADIUS);

    // Draw yellow light (always dim in two-state system)
    SDL_SetRenderDrawColor(renderer, 64, 64, 0, 255);
    renderCircle(centerX, yellowY, LIGHT_RADIUS);

    // Draw green light
    SDL_SetRenderDrawColor(renderer, 0,
        state == LightState::GREEN ? 255 : 64,  // Bright green when active, dim when inactive
        0, 255);
    renderCircle(centerX, greenY, LIGHT_RADIUS);
}

void Renderer::renderTrafficLights(const std::map<LaneId, TrafficLight>& lights) {
    using namespace SimConstants;

    // Define the standard positions for traffic lights
    struct LightPosition {
        float x;
        float y;
        float rotation;
        LaneId laneId;
    };

    // Define fixed positions for each traffic light
    const LightPosition positions[] = {
        // West approach (AL2 - Priority lane)
        {
            static_cast<float>(CENTER_X - ROAD_WIDTH/2.0f - 50.0f),
            static_cast<float>(CENTER_Y - LIGHT_SIZE * 3.0f),
            0.0f,
            LaneId::AL2_PRIORITY
        },
        // North approach (BL2)
        {
            static_cast<float>(CENTER_X - LIGHT_SIZE * 3.0f),
            static_cast<float>(CENTER_Y - ROAD_WIDTH/2.0f - 50.0f),
            90.0f * static_cast<float>(M_PI) / 180.0f,
            LaneId::BL2_NORMAL
        },
        // East approach (CL2)
        {
            static_cast<float>(CENTER_X + ROAD_WIDTH/2.0f + 50.0f),
            static_cast<float>(CENTER_Y - LIGHT_SIZE * 3.0f),
            180.0f * static_cast<float>(M_PI) / 180.0f,
            LaneId::CL2_NORMAL
        },
        // South approach (DL2)
        {
            static_cast<float>(CENTER_X - LIGHT_SIZE * 3.0f),
            static_cast<float>(CENTER_Y + ROAD_WIDTH/2.0f + 50.0f),
            270.0f * static_cast<float>(M_PI) / 180.0f,
            LaneId::DL2_NORMAL
        }
    };

    // Render each traffic light
    for (const auto& position : positions) {
        auto it = lights.find(position.laneId);
        if (it != lights.end()) {
            renderTrafficLight(
                position.x,
                position.y,
                position.rotation,
                it->second.getState()
            );
        }
    }
}


void Renderer::renderVehicles(const std::map<uint32_t, VehicleState>& vehicles) {
    for (const auto& [id, state] : vehicles) {
        renderVehicle(
            state.pos.x,          // Use pos.x instead of x
            state.pos.y,          // Use pos.y instead of y
            state.direction,
            state.vehicle->getCurrentLane() == LaneId::AL2_PRIORITY,
            state.turnAngle,
            state.isMoving
        );
    }
}

void Renderer::renderVehicle(float x, float y, Direction dir, bool isPriority, float angle, bool isMoving) {
    const float halfWidth = VEHICLE_WIDTH / 2.0f;
    const float halfHeight = VEHICLE_HEIGHT / 2.0f;

    // Create a smoother vehicle shape
    SDL_FPoint vertices[8] = {
        // Front
        {x + (halfWidth * 0.8f) * cosf(angle), y + (halfWidth * 0.8f) * sinf(angle)},
        // Front right
        {x + halfWidth * cosf(angle + 0.4f), y + halfWidth * sinf(angle + 0.4f)},
        // Right
        {x + halfWidth * cosf(angle + M_PI/2), y + halfWidth * sinf(angle + M_PI/2)},
        // Back right
        {x + halfWidth * cosf(angle + M_PI - 0.4f), y + halfWidth * sinf(angle + M_PI - 0.4f)},
        // Back
        {x - (halfWidth * 0.8f) * cosf(angle), y - (halfWidth * 0.8f) * sinf(angle)},
        // Back left
        {x + halfWidth * cosf(angle + M_PI + 0.4f), y + halfWidth * sinf(angle + M_PI + 0.4f)},
        // Left
        {x + halfWidth * cosf(angle - M_PI/2), y + halfWidth * sinf(angle - M_PI/2)},
        // Front left
        {x + halfWidth * cosf(angle - 0.4f), y + halfWidth * sinf(angle - 0.4f)}
    };

    // Draw shadow
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 100);
    for (int i = 0; i < 8; i++) {
        SDL_RenderLine(renderer,
            vertices[i].x + 2, vertices[i].y + 2,
            vertices[(i + 1) % 8].x + 2, vertices[(i + 1) % 8].y + 2);
    }

    // Vehicle body color
    if (isPriority) {
        SDL_SetRenderDrawColor(renderer, 255, 140, 0, 255); // Orange for priority
    } else {
        SDL_SetRenderDrawColor(renderer, 30, 144, 255, 255); // Blue for normal
    }

    // Draw vehicle body
    for (int i = 0; i < 8; i++) {
        SDL_RenderLine(renderer,
            vertices[i].x, vertices[i].y,
            vertices[(i + 1) % 8].x, vertices[(i + 1) % 8].y);
    }

    // Draw headlights
    SDL_SetRenderDrawColor(renderer, 255, 255, 200, 255);
    renderCircle(vertices[0].x - 5 * cosf(angle + 0.2f),
                vertices[0].y - 5 * sinf(angle + 0.2f), 3);
    renderCircle(vertices[0].x - 5 * cosf(angle - 0.2f),
                vertices[0].y - 5 * sinf(angle - 0.2f), 3);

    // Direction indicators
    if (dir != Direction::STRAIGHT) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 0, 200);
        if (dir == Direction::LEFT) {
            renderCircle(vertices[6].x, vertices[6].y, 4);
        } else {
            renderCircle(vertices[2].x, vertices[2].y, 4);
        }
    }

    // Movement trail
    if (isMoving) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 50);
        float t = static_cast<float>(SDL_GetTicks()) / 1000.0f;
        for (int i = 1; i <= 3; i++) {
            float offset = i * (5.0f + sinf(t * 4.0f) * 2.0f);
            float trailX = x - offset * cosf(angle);
            float trailY = y - offset * sinf(angle);
            renderCircle(trailX, trailY, 2);
        }
    }
}

void Renderer::renderIntersection() {
    using namespace SimConstants;

    // Draw intersection box with slightly darker asphalt
    SDL_SetRenderDrawColor(renderer, 45, 45, 45, 255);
    SDL_FRect intersection = {
        CENTER_X - ROAD_WIDTH/2.0f,
        CENTER_Y - ROAD_WIDTH/2.0f,
        static_cast<float>(ROAD_WIDTH),
        static_cast<float>(ROAD_WIDTH)
    };
    SDL_RenderFillRect(renderer, &intersection);

    // Draw intersection guidelines
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 100);

    // Center cross guidelines
    renderDashedLine(
        CENTER_X - ROAD_WIDTH/2.0f, CENTER_Y,
        CENTER_X + ROAD_WIDTH/2.0f, CENTER_Y
    );
    renderDashedLine(
        CENTER_X, CENTER_Y - ROAD_WIDTH/2.0f,
        CENTER_X, CENTER_Y + ROAD_WIDTH/2.0f
    );

    // Draw turning guide arcs
    renderTurningGuides();
}

void Renderer::renderTurningGuides() {
    using namespace SimConstants;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 50);
    const int SEGMENTS = 32;
    const float TURN_RADIUS = ROAD_WIDTH / 2.0f;

    // Draw turning guide arcs for each corner
    for (int corner = 0; corner < 4; corner++) {
        float centerX = CENTER_X + ((corner & 1) ? ROAD_WIDTH/4.0f : -ROAD_WIDTH/4.0f);
        float centerY = CENTER_Y + ((corner & 2) ? ROAD_WIDTH/4.0f : -ROAD_WIDTH/4.0f);

        for (int i = 0; i < SEGMENTS; i++) {
            float startAngle = (corner * 90 + i * 90.0f / SEGMENTS) * M_PI / 180.0f;
            float endAngle = (corner * 90 + (i + 1) * 90.0f / SEGMENTS) * M_PI / 180.0f;

            float x1 = centerX + TURN_RADIUS * cosf(startAngle);
            float y1 = centerY + TURN_RADIUS * sinf(startAngle);
            float x2 = centerX + TURN_RADIUS * cosf(endAngle);
            float y2 = centerY + TURN_RADIUS * sinf(endAngle);

            SDL_RenderLine(renderer, x1, y1, x2, y2);
        }
    }
}

void Renderer::renderStopLines() {
    using namespace SimConstants;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    const float STOP_LINE_WIDTH = 8.0f;
    const float OFFSET = ROAD_WIDTH/2.0f - 20.0f;

    // Draw stop lines at each intersection approach
    SDL_FRect stopLines[] = {
        // West approach
        {CENTER_X - OFFSET - STOP_LINE_WIDTH,
         CENTER_Y - LANE_WIDTH,
         STOP_LINE_WIDTH,
         LANE_WIDTH * 2},

        // North approach
        {CENTER_X - LANE_WIDTH,
         CENTER_Y - OFFSET - STOP_LINE_WIDTH,
         LANE_WIDTH * 2,
         STOP_LINE_WIDTH},

        // East approach
        {CENTER_X + OFFSET,
         CENTER_Y - LANE_WIDTH,
         STOP_LINE_WIDTH,
         LANE_WIDTH * 2},

        // South approach
        {CENTER_X - LANE_WIDTH,
         CENTER_Y + OFFSET,
         LANE_WIDTH * 2,
         STOP_LINE_WIDTH}
    };

    for (const auto& line : stopLines) {
        SDL_RenderFillRect(renderer, &line);
    }
}

void Renderer::renderDirectionalArrows() {
    using namespace SimConstants;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 128);
    const float ARROW_DISTANCE = 150.0f;

    // Draw direction arrows for each lane
    for (int lane = -1; lane <= 1; lane++) {
        float laneOffset = static_cast<float>(lane * LANE_WIDTH);

        // West approach (right-side driving)
        drawArrow(CENTER_X - ARROW_DISTANCE, CENTER_Y + laneOffset,
                 0.0f, Direction::STRAIGHT);

        // North approach
        drawArrow(CENTER_X + laneOffset, CENTER_Y - ARROW_DISTANCE,
                 static_cast<float>(M_PI / 2), Direction::STRAIGHT);

        // East approach
        drawArrow(CENTER_X + ARROW_DISTANCE, CENTER_Y + laneOffset,
                 static_cast<float>(M_PI), Direction::STRAIGHT);

        // South approach
        drawArrow(CENTER_X + laneOffset, CENTER_Y + ARROW_DISTANCE,
                 static_cast<float>(-M_PI / 2), Direction::STRAIGHT);
    }

    // Add left turn arrows for free lanes
    drawArrow(CENTER_X - ARROW_DISTANCE, CENTER_Y + LANE_WIDTH,
             0.0f, Direction::LEFT);
    drawArrow(CENTER_X + LANE_WIDTH, CENTER_Y - ARROW_DISTANCE,
             static_cast<float>(M_PI / 2), Direction::LEFT);
    drawArrow(CENTER_X + ARROW_DISTANCE, CENTER_Y + LANE_WIDTH,
             static_cast<float>(M_PI), Direction::LEFT);
    drawArrow(CENTER_X + LANE_WIDTH, CENTER_Y + ARROW_DISTANCE,
             static_cast<float>(-M_PI / 2), Direction::LEFT);
}

void Renderer::drawArrow(float x, float y, float angle, Direction dir) {
    const float ARROW_LENGTH = 30.0f;
    const float HEAD_SIZE = 10.0f;
    const float HEAD_ANGLE = static_cast<float>(M_PI / 6);

    float cosA = cosf(angle);
    float sinA = sinf(angle);

    // Draw arrow shaft
    float endX = x + ARROW_LENGTH * cosA;
    float endY = y + ARROW_LENGTH * sinA;
    SDL_RenderLine(renderer, x, y, endX, endY);

    // Draw arrow head
    float leftX = endX - HEAD_SIZE * cosf(angle + HEAD_ANGLE);
    float leftY = endY - HEAD_SIZE * sinf(angle + HEAD_ANGLE);
    float rightX = endX - HEAD_SIZE * cosf(angle - HEAD_ANGLE);
    float rightY = endY - HEAD_SIZE * sinf(angle - HEAD_ANGLE);

    SDL_RenderLine(renderer, endX, endY, leftX, leftY);
    SDL_RenderLine(renderer, endX, endY, rightX, rightY);

    // Add curved arrow for left turns
    if (dir == Direction::LEFT) {
        const float CURVE_RADIUS = 15.0f;
        const int SEGMENTS = 8;

        for (int i = 0; i < SEGMENTS; i++) {
            float startAngle = angle - M_PI/2 + (i * M_PI/2) / SEGMENTS;
            float endAngle = angle - M_PI/2 + ((i + 1) * M_PI/2) / SEGMENTS;

            float x1 = x + CURVE_RADIUS * cosf(startAngle);
            float y1 = y + CURVE_RADIUS * sinf(startAngle);
            float x2 = x + CURVE_RADIUS * cosf(endAngle);
            float y2 = y + CURVE_RADIUS * sinf(endAngle);

            SDL_RenderLine(renderer, x1, y1, x2, y2);
        }
    }
}

void Renderer::renderCircle(float x, float y, float radius) {
    const int SEGMENTS = 16;
    for (int i = 0; i < SEGMENTS; i++) {
        float angle1 = 2.0f * M_PI * i / SEGMENTS;
        float angle2 = 2.0f * M_PI * (i + 1) / SEGMENTS;

        SDL_RenderLine(renderer,
            x + radius * cosf(angle1),
            y + radius * sinf(angle1),
            x + radius * cosf(angle2),
            y + radius * sinf(angle2)
        );
    }
}

void Renderer::renderDashedLine(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float length = std::sqrt(dx * dx + dy * dy);
    float nx = dx / length;
    float ny = dy / length;

    float x = x1;
    float y = y1;
    bool drawing = true;
    float remainingLength = length;

    while (remainingLength > 0) {
        float segmentLength = std::min(drawing ? DASH_LENGTH : GAP_LENGTH, remainingLength);

        if (drawing) {
            float endX = x + nx * segmentLength;
            float endY = y + ny * segmentLength;
            SDL_RenderLine(renderer, x, y, endX, endY);
        }

        x += nx * segmentLength;
        y += ny * segmentLength;
        remainingLength -= segmentLength;
        drawing = !drawing;
    }
}

void Renderer::cleanup() {
    if (renderer) {
        SDL_DestroyRenderer(renderer);
        renderer = nullptr;
    }
    if (window) {
        SDL_DestroyWindow(window);
        window = nullptr;
    }
    SDL_Quit();
}

float Renderer::calculateTurningAngle(const VehicleState& state) const {
    float dx = state.targetPos.x - state.pos.x;
    float dy = state.targetPos.y - state.pos.y;
    return std::atan2f(dy, dx);
}

SDL_Color Renderer::getLaneColor(LaneId laneId, bool isActive) const {
    if (isActive) {
        if (laneId == LaneId::AL2_PRIORITY) {
            return {255, 165, 0, 255}; // Orange for active priority lane
        }
        return {0, 255, 0, 255}; // Green for active normal lanes
    }

    if (laneId == LaneId::AL2_PRIORITY) {
        return {255, 165, 0, 128}; // Semi-transparent orange for inactive priority lane
    }
    return {255, 255, 255, 128}; // Semi-transparent white for inactive normal lanes
}

void Renderer::drawDebugGrid() {
    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 64);

    // Draw vertical grid lines
    for (float x = 0.0f; x < static_cast<float>(SimConstants::WINDOW_WIDTH); x += 50.0f) {
        SDL_RenderLine(renderer,
            static_cast<int>(x), 0,
            static_cast<int>(x), SimConstants::WINDOW_HEIGHT
        );
    }

    // Draw horizontal grid lines
    for (float y = 0.0f; y < static_cast<float>(SimConstants::WINDOW_HEIGHT); y += 50.0f) {
        SDL_RenderLine(renderer,
            0, static_cast<int>(y),
            SimConstants::WINDOW_WIDTH, static_cast<int>(y)
        );
    }
}



void Renderer::renderCrosswalks() {
    using namespace SimConstants;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    const float STRIPE_WIDTH = 5.0f;
    const float STRIPE_LENGTH = 30.0f;
    const float STRIPE_GAP = 5.0f;
    const float CROSSWALK_WIDTH = 20.0f;

    // Render crosswalks on all four sides of the intersection
    for (int i = 0; i < 4; ++i) {
        float angle = static_cast<float>(i * 90) * static_cast<float>(M_PI) / 180.0f;
        float baseX = CENTER_X + cosf(angle) * (ROAD_WIDTH / 2.0f - CROSSWALK_WIDTH);
        float baseY = CENTER_Y + sinf(angle) * (ROAD_WIDTH / 2.0f - CROSSWALK_WIDTH);

        // Draw zebra stripes
        for (float offset = 0; offset < ROAD_WIDTH; offset += STRIPE_WIDTH + STRIPE_GAP) {
            SDL_FPoint p1 = rotatePoint(baseX, baseY + offset, CENTER_X, CENTER_Y, angle);
            SDL_FPoint p2 = rotatePoint(baseX + STRIPE_LENGTH, baseY + offset, CENTER_X, CENTER_Y, angle);

            SDL_FRect stripe = {
                p1.x, p1.y,
                STRIPE_WIDTH,
                p2.y - p1.y
            };
            SDL_RenderFillRect(renderer, &stripe);
        }
    }
}

SDL_FPoint Renderer::rotatePoint(float x, float y, float cx, float cy, float angle) {
    // First, translate point back to origin by subtracting center coordinates
    float translatedX = x - cx;
    float translatedY = y - cy;

    // Perform the rotation using the rotation matrix:
    // | cos(θ) -sin(θ) |
    // | sin(θ)  cos(θ) |
    float rotatedX = translatedX * cosf(angle) - translatedY * sinf(angle);
    float rotatedY = translatedX * sinf(angle) + translatedY * cosf(angle);

    // Translate back to original position by adding center coordinates
    SDL_FPoint result = {
        rotatedX + cx,
        rotatedY + cy
    };

    return result;
}

void Renderer::renderPriorityLaneIndicator() {
    using namespace SimConstants;

    // Draw priority mode indicator in top-left corner
    const float INDICATOR_SIZE = 30.0f;
    const float PADDING = 10.0f;

    SDL_SetRenderDrawColor(renderer, 255, 69, 0, 255); // Orange for priority
    SDL_FRect indicator = {
        PADDING,
        PADDING,
        INDICATOR_SIZE,
        INDICATOR_SIZE
    };
    SDL_RenderFillRect(renderer, &indicator);

    // Add pulsing effect
    float t = static_cast<float>(SDL_GetTicks()) / 1000.0f;
    uint8_t alpha = static_cast<uint8_t>(128 + 127 * sinf(t * 2.0f));
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, alpha);
    SDL_RenderRect(renderer, &indicator);
}

void Renderer::renderLaneIdentifiers() {
    using namespace SimConstants;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    const float OFFSET = ROAD_WIDTH / 2.0f + 30.0f;

    // Define lane positions and labels
    struct LaneLabel {
        float x, y;
        LaneId id;
    } labels[] = {
        {CENTER_X - OFFSET, CENTER_Y - LANE_WIDTH, LaneId::AL1_INCOMING},
        {CENTER_X - OFFSET, CENTER_Y, LaneId::AL2_PRIORITY},
        {CENTER_X - OFFSET, CENTER_Y + LANE_WIDTH, LaneId::AL3_FREELANE},

        {CENTER_X - LANE_WIDTH, CENTER_Y - OFFSET, LaneId::BL1_INCOMING},
        {CENTER_X, CENTER_Y - OFFSET, LaneId::BL2_NORMAL},
        {CENTER_X + LANE_WIDTH, CENTER_Y - OFFSET, LaneId::BL3_FREELANE},

        {CENTER_X + OFFSET, CENTER_Y - LANE_WIDTH, LaneId::CL1_INCOMING},
        {CENTER_X + OFFSET, CENTER_Y, LaneId::CL2_NORMAL},
        {CENTER_X + OFFSET, CENTER_Y + LANE_WIDTH, LaneId::CL3_FREELANE},

        {CENTER_X - LANE_WIDTH, CENTER_Y + OFFSET, LaneId::DL1_INCOMING},
        {CENTER_X, CENTER_Y + OFFSET, LaneId::DL2_NORMAL},
        {CENTER_X + LANE_WIDTH, CENTER_Y + OFFSET, LaneId::DL3_FREELANE}
    };

    // Draw background rectangles for labels
    for (const auto& label : labels) {
        SDL_FRect bg = {
            label.x - 25.0f,
            label.y - 12.0f,
            50.0f,
            24.0f
        };

        // Different colors for different lane types
        if (label.id == LaneId::AL2_PRIORITY) {
            SDL_SetRenderDrawColor(renderer, 255, 165, 0, 128); // Orange for priority
        } else if (static_cast<int>(label.id) % 3 == 2) {
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 128); // Green for free lanes
        } else {
            SDL_SetRenderDrawColor(renderer, 100, 100, 100, 128); // Gray for normal lanes
        }

        SDL_RenderFillRect(renderer, &bg);
    }
}

void Renderer::renderVehicleCount(const TrafficManager& trafficManager) {
    using namespace SimConstants;

    const float PADDING = 10.0f;
    const float BOX_WIDTH = 150.0f;
    const float BOX_HEIGHT = 80.0f;

    // Draw background panel
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 200);
    SDL_FRect countBox = {
        PADDING,
        WINDOW_HEIGHT - BOX_HEIGHT - PADDING,
        BOX_WIDTH,
        BOX_HEIGHT
    };
    SDL_RenderFillRect(renderer, &countBox);

    // Draw separator lines
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 128);
    SDL_RenderLine(renderer,
        countBox.x,
        countBox.y + BOX_HEIGHT / 2.0f,
        countBox.x + BOX_WIDTH,
        countBox.y + BOX_HEIGHT / 2.0f
    );

    // Vehicle counts are rendered here
    // Note: Actual text rendering would require SDL_ttf setup
    // For now, we just show the box layout
}

// End of Renderer.cpp