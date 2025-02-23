#include "managers/TrafficManager.h"
#include "managers/FileHandler.h"
#include <algorithm>
#include <cmath>
#include <iostream>

TrafficManager::TrafficManager()
    : inPriorityMode(false)
    , stateTimer(0.0f)
    , lastUpdateTime(0.0f) {

    // Initialize all lanes with their respective configurations
    lanes.push_back(std::make_unique<Lane>(LaneId::AL1_INCOMING, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::AL2_PRIORITY, true));  // Priority lane
    lanes.push_back(std::make_unique<Lane>(LaneId::AL3_FREELANE, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::BL1_INCOMING, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::BL2_NORMAL, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::BL3_FREELANE, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::CL1_INCOMING, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::CL2_NORMAL, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::CL3_FREELANE, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::DL1_INCOMING, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::DL2_NORMAL, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::DL3_FREELANE, false));

    // Initialize traffic lights for controlled lanes
    trafficLights[LaneId::AL2_PRIORITY] = TrafficLight();
    trafficLights[LaneId::BL2_NORMAL] = TrafficLight();
    trafficLights[LaneId::CL2_NORMAL] = TrafficLight();
    trafficLights[LaneId::DL2_NORMAL] = TrafficLight();

    // Set initial traffic light states
    synchronizeTrafficLights();
}

void TrafficManager::update(float deltaTime) {
    using namespace SimConstants;

    stateTimer += deltaTime;
    lastUpdateTime += deltaTime;

    static float fileCheckTimer = 0.0f;
    fileCheckTimer += deltaTime;

    // Check for new vehicles at regular intervals
    if (fileCheckTimer >= FILE_CHECK_INTERVAL / 1000.0f) {
        FileHandler fileHandler;
        auto newVehicles = fileHandler.readNewVehicles();

        for (const auto& [laneId, vehicle] : newVehicles) {
            addVehicleToLane(laneId, vehicle);
            addNewVehicleToState(vehicle, laneId);
        }
        fileCheckTimer = 0.0f;
    }

    // Update vehicle positions and states
    updateVehiclePositions(deltaTime);

    // Check and handle priority conditions
    if (checkPriorityConditions()) {
        if (!inPriorityMode) {
            synchronizeTrafficLights();
        }
        inPriorityMode = true;
        processPriorityLane();
    } else {
        if (inPriorityMode) {
            synchronizeTrafficLights();
        }
        inPriorityMode = false;

        // Process normal lanes at regular intervals
        if (lastUpdateTime >= VEHICLE_PROCESS_TIME) {
            size_t vehiclesToProcess = calculateVehiclesToProcess();
            processNormalLanes(vehiclesToProcess);
            lastUpdateTime = 0.0f;
        }
    }

    // Update traffic lights
    updateTrafficLights(deltaTime);

    // Process free lanes continuously
    for (auto& lane : lanes) {
        if (lane->getId() == LaneId::AL3_FREELANE ||
            lane->getId() == LaneId::BL3_FREELANE ||
            lane->getId() == LaneId::CL3_FREELANE ||
            lane->getId() == LaneId::DL3_FREELANE) {

            while (lane->getQueueSize() > 0) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    auto it = activeVehicles.find(vehicle->getId());
                    if (it != activeVehicles.end()) {
                        it->second.isMoving = true;
                    }
                }
            }
        }
    }
}

void TrafficManager::addVehicleToLane(LaneId laneId, std::shared_ptr<Vehicle> vehicle) {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [laneId](const auto& lane) { return lane->getId() == laneId; });

    if (it != lanes.end()) {
        (*it)->addVehicle(vehicle);
    }
}

void TrafficManager::addNewVehicleToState(std::shared_ptr<Vehicle> vehicle, LaneId laneId) {
    using namespace SimConstants;

    VehicleState state;
    state.vehicle = vehicle;
    state.speed = VEHICLE_BASE_SPEED;
    state.isMoving = false;
    state.direction = vehicle->getDirection();
    state.hasStartedTurn = false;
    state.turnProgress = 0.0f;
    state.waitTime = 0.0f;

    // Calculate initial position based on lane and queue position
    size_t queuePosition = getLaneSize(laneId);

    // Set position based on lane configuration
    switch (laneId) {
        case LaneId::AL1_INCOMING:
        case LaneId::AL2_PRIORITY:
        case LaneId::AL3_FREELANE: {
            float laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::AL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X - QUEUE_START_OFFSET - (queuePosition * QUEUE_SPACING);
            state.y = CENTER_Y - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            break;
        }

        case LaneId::BL1_INCOMING:
        case LaneId::BL2_NORMAL:
        case LaneId::BL3_FREELANE: {
            float laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::BL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.y = CENTER_Y - QUEUE_START_OFFSET - (queuePosition * QUEUE_SPACING);
            break;
        }

        case LaneId::CL1_INCOMING:
        case LaneId::CL2_NORMAL:
        case LaneId::CL3_FREELANE: {
            float laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::CL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X + QUEUE_START_OFFSET + (queuePosition * QUEUE_SPACING);
            state.y = CENTER_Y - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            break;
        }

        case LaneId::DL1_INCOMING:
        case LaneId::DL2_NORMAL:
        case LaneId::DL3_FREELANE: {
            float laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::DL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.y = CENTER_Y + QUEUE_START_OFFSET + (queuePosition * QUEUE_SPACING);
            break;
        }
    }

    // Calculate target position for vehicle's path
    calculateTargetPosition(state, laneId);

    std::cout << "Added vehicle " << vehicle->getId()
              << " to lane " << static_cast<int>(laneId)
              << " at position (" << state.x << "," << state.y << ")"
              << " with direction " << static_cast<int>(state.direction)
              << std::endl;

    activeVehicles[vehicle->getId()] = state;
}

void TrafficManager::updateVehiclePositions(float deltaTime) {
    auto it = activeVehicles.begin();
    while (it != activeVehicles.end()) {
        auto& state = it->second;

        if (state.isMoving) {
            float dx = state.targetX - state.x;
            float dy = state.targetY - state.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            // Remove vehicle if it reached its destination
            if (distance < 1.0f) {
                it = activeVehicles.erase(it);
                continue;
            }

            // Calculate movement with gradual speed increase
            float speedFactor = 1.0f - std::exp(-distance / 200.0f);
            float currentSpeed = state.speed * speedFactor;

            float moveX = (dx / distance) * currentSpeed * deltaTime;
            float moveY = (dy / distance) * currentSpeed * deltaTime;

            // Check for collisions before moving
            if (!checkCollision(state, state.x + moveX, state.y + moveY)) {
                state.x += moveX;
                state.y += moveY;
            }
            ++it;
        } else {
            // Update queue position for waiting vehicles
            updateVehicleQueuePosition(
                state,
                state.vehicle->getCurrentLane(),
                getLaneSize(state.vehicle->getCurrentLane())
            );
            ++it;
        }
    }
}

void TrafficManager::updateVehicleQueuePosition(VehicleState& state, LaneId laneId,
                                              size_t queuePosition) {
    using namespace SimConstants;

    float laneOffset = 0.0f;

    // Calculate lane-specific offset
    switch (laneId) {
        case LaneId::AL1_INCOMING:
        case LaneId::AL2_PRIORITY:
        case LaneId::AL3_FREELANE:
            laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::AL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X - QUEUE_START_OFFSET - (queuePosition * QUEUE_SPACING);
            state.y = CENTER_Y - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            break;

        case LaneId::BL1_INCOMING:
        case LaneId::BL2_NORMAL:
        case LaneId::BL3_FREELANE:
            laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::BL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.y = CENTER_Y - QUEUE_START_OFFSET - (queuePosition * QUEUE_SPACING);
            break;

        case LaneId::CL1_INCOMING:
        case LaneId::CL2_NORMAL:
        case LaneId::CL3_FREELANE:
            laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::CL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X + QUEUE_START_OFFSET + (queuePosition * QUEUE_SPACING);
            state.y = CENTER_Y - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            break;

        case LaneId::DL1_INCOMING:
        case LaneId::DL2_NORMAL:
        case LaneId::DL3_FREELANE:
            laneOffset = static_cast<float>(
                (static_cast<int>(laneId) - static_cast<int>(LaneId::DL1_INCOMING))
            ) * LANE_WIDTH;
            state.x = CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.y = CENTER_Y + QUEUE_START_OFFSET + (queuePosition * QUEUE_SPACING);
            break;
    }
}

void TrafficManager::calculateTargetPosition(VehicleState& state, LaneId laneId) {
    using namespace SimConstants;

    const float EXIT_DISTANCE = 450.0f;

    // Calculate target position based on direction and current lane
    switch (state.direction) {
        case Direction::STRAIGHT: {
            switch (laneId) {
                case LaneId::AL1_INCOMING:
                case LaneId::AL2_PRIORITY:
                case LaneId::AL3_FREELANE:
                    state.targetX = CENTER_X + EXIT_DISTANCE;
                    state.targetY = state.y;
                    break;

                case LaneId::BL1_INCOMING:
                case LaneId::BL2_NORMAL:
                case LaneId::BL3_FREELANE:
                    state.targetX = state.x;
                    state.targetY = CENTER_Y + EXIT_DISTANCE;
                    break;

                case LaneId::CL1_INCOMING:
                case LaneId::CL2_NORMAL:
                case LaneId::CL3_FREELANE:
                    state.targetX = CENTER_X - EXIT_DISTANCE;
                    state.targetY = state.y;
                    break;

                case LaneId::DL1_INCOMING:
                case LaneId::DL2_NORMAL:
                case LaneId::DL3_FREELANE:
                    state.targetX = state.x;
                    state.targetY = CENTER_Y - EXIT_DISTANCE;
                    break;
            }
            break;
        }

        case Direction::LEFT: {
            float turnRadius = TURN_GUIDE_RADIUS;
            switch (laneId) {
                case LaneId::AL1_INCOMING:
                case LaneId::AL2_PRIORITY:
                case LaneId::AL3_FREELANE:
                    state.targetX = state.x + turnRadius;
                    state.targetY = CENTER_Y - EXIT_DISTANCE;
                    break;

                case LaneId::BL1_INCOMING:
                case LaneId::BL2_NORMAL:
case LaneId::BL3_FREELANE:
                    state.targetX = CENTER_X - EXIT_DISTANCE;
                    state.targetY = state.y + turnRadius;
                    break;

                case LaneId::CL1_INCOMING:
                case LaneId::CL2_NORMAL:
                case LaneId::CL3_FREELANE:
                    state.targetX = state.x - turnRadius;
                    state.targetY = CENTER_Y + EXIT_DISTANCE;
                    break;

                case LaneId::DL1_INCOMING:
                case LaneId::DL2_NORMAL:
                case LaneId::DL3_FREELANE:
                    state.targetX = CENTER_X + EXIT_DISTANCE;
                    state.targetY = state.y - turnRadius;
                    break;
            }
            break;
        }

        case Direction::RIGHT: {
            float turnRadius = TURN_GUIDE_RADIUS * 0.6f;  // Tighter turn for right turns
            switch (laneId) {
                case LaneId::AL1_INCOMING:
                case LaneId::AL2_PRIORITY:
                case LaneId::AL3_FREELANE:
                    state.targetX = state.x + turnRadius;
                    state.targetY = CENTER_Y + EXIT_DISTANCE;
                    break;

                case LaneId::BL1_INCOMING:
                case LaneId::BL2_NORMAL:
                case LaneId::BL3_FREELANE:
                    state.targetX = CENTER_X + EXIT_DISTANCE;
                    state.targetY = state.y + turnRadius;
                    break;

                case LaneId::CL1_INCOMING:
                case LaneId::CL2_NORMAL:
                case LaneId::CL3_FREELANE:
                    state.targetX = state.x - turnRadius;
                    state.targetY = CENTER_Y - EXIT_DISTANCE;
                    break;

                case LaneId::DL1_INCOMING:
                case LaneId::DL2_NORMAL:
                case LaneId::DL3_FREELANE:
                    state.targetX = CENTER_X - EXIT_DISTANCE;
                    state.targetY = state.y - turnRadius;
                    break;
            }
            break;
        }
    }
}

bool TrafficManager::checkCollision(const VehicleState& state, float newX, float newY) const {
    using namespace SimConstants;

    const float MIN_DISTANCE = VEHICLE_WIDTH * 1.2f;  // Safe distance between vehicles

    // Check collision with all other active vehicles
    for (const auto& [otherId, otherState] : activeVehicles) {
        if (otherId != state.vehicle->getId()) {
            float dx = newX - otherState.x;
            float dy = newY - otherState.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < MIN_DISTANCE) {
                return true;  // Collision detected
            }
        }
    }
    return false;  // No collision
}

void TrafficManager::updateTrafficLights(float deltaTime) {
    // Update each traffic light's state
    for (auto& [_, light] : trafficLights) {
        light.update(deltaTime);
    }

    // Synchronize traffic lights based on priority mode
    if (!inPriorityMode) {
        // Normal mode: opposing lights are synchronized
        if (trafficLights[LaneId::BL2_NORMAL].getState() == LightState::GREEN) {
            trafficLights[LaneId::DL2_NORMAL].setState(LightState::GREEN);
            trafficLights[LaneId::AL2_PRIORITY].setState(LightState::RED);
            trafficLights[LaneId::CL2_NORMAL].setState(LightState::RED);
        } else {
            trafficLights[LaneId::AL2_PRIORITY].setState(LightState::GREEN);
            trafficLights[LaneId::CL2_NORMAL].setState(LightState::GREEN);
            trafficLights[LaneId::BL2_NORMAL].setState(LightState::RED);
            trafficLights[LaneId::DL2_NORMAL].setState(LightState::RED);
        }
    } else {
        // Priority mode: priority lane gets green, others red
        trafficLights[LaneId::AL2_PRIORITY].setState(LightState::GREEN);
        trafficLights[LaneId::BL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::CL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::DL2_NORMAL].setState(LightState::RED);
    }
}

void TrafficManager::synchronizeTrafficLights() {
    // Set initial states based on priority mode
    if (inPriorityMode) {
        trafficLights[LaneId::AL2_PRIORITY].setState(LightState::GREEN);
        trafficLights[LaneId::BL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::CL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::DL2_NORMAL].setState(LightState::RED);
    } else {
        // Start with North-South traffic flow
        trafficLights[LaneId::AL2_PRIORITY].setState(LightState::RED);
        trafficLights[LaneId::BL2_NORMAL].setState(LightState::GREEN);
        trafficLights[LaneId::CL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::DL2_NORMAL].setState(LightState::GREEN);
    }
}

bool TrafficManager::checkPriorityConditions() const {
    // Check if priority lane (AL2) has more than 10 vehicles
    auto priorityLane = std::find_if(lanes.begin(), lanes.end(),
        [](const auto& lane) {
            return lane->isPriorityLane() && lane->getQueueSize() > 10;
        });
    return priorityLane != lanes.end();
}

void TrafficManager::processPriorityLane() {
    // Process vehicles in priority lane until queue length is below threshold
    for (auto& lane : lanes) {
        if (lane->isPriorityLane() && lane->getQueueSize() > 5) {
            while (lane->getQueueSize() > 5) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    auto it = activeVehicles.find(vehicle->getId());
                    if (it != activeVehicles.end()) {
                        it->second.isMoving = true;
                    }
                }
            }
            break;
        }
    }
}

void TrafficManager::processNormalLanes(size_t vehicleCount) {
    if (vehicleCount == 0) return;

    // Process vehicles from normal lanes based on calculated count
    for (auto& lane : lanes) {
        if (!lane->isPriorityLane() &&
            lane->getId() != LaneId::AL3_FREELANE &&
            lane->getId() != LaneId::BL3_FREELANE &&
            lane->getId() != LaneId::CL3_FREELANE &&
            lane->getId() != LaneId::DL3_FREELANE) {

            // Process up to vehicleCount vehicles from each normal lane
            for (size_t i = 0; i < vehicleCount && lane->getQueueSize() > 0; ++i) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    auto it = activeVehicles.find(vehicle->getId());
                    if (it != activeVehicles.end()) {
                        it->second.isMoving = true;
                    }
                }
            }
        }
    }
}

size_t TrafficManager::calculateVehiclesToProcess() const {
    // Calculate average number of waiting vehicles in normal lanes
    size_t totalVehicles = 0;
    size_t normalLaneCount = 0;

    for (const auto& lane : lanes) {
        if (!lane->isPriorityLane() &&
            lane->getId() != LaneId::AL3_FREELANE &&
            lane->getId() != LaneId::BL3_FREELANE &&
            lane->getId() != LaneId::CL3_FREELANE &&
            lane->getId() != LaneId::DL3_FREELANE) {

            totalVehicles += lane->getQueueSize();
            normalLaneCount++;
        }
    }

    if (normalLaneCount == 0) return 0;

    // Calculate and return the number of vehicles to process
    float avgVehicles = static_cast<float>(totalVehicles) / static_cast<float>(normalLaneCount);
    float vehicleRatio = 0.3f;  // Process 30% of average waiting vehicles
    return static_cast<size_t>(std::ceil(avgVehicles * vehicleRatio));
}

size_t TrafficManager::getLaneSize(LaneId laneId) const {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [laneId](const auto& lane) { return lane->getId() == laneId; });

    return (it != lanes.end()) ? (*it)->getQueueSize() : 0;
}

float TrafficManager::calculateTurningRadius(Direction dir) const {
    using namespace SimConstants;
    return (dir == Direction::LEFT) ? INTERSECTION_RADIUS * 1.5f :
           (dir == Direction::RIGHT) ? INTERSECTION_RADIUS * 0.5f :
           INTERSECTION_RADIUS;
}