// TrafficManager.cpp
#include "managers/TrafficManager.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include<core/Constants.h>

TrafficManager::TrafficManager()
    : inPriorityMode(false)
    , stateTimer(0.0f)
    , lastUpdateTime(0.0f)
    , processingTimer(0.0f)
    , totalVehiclesProcessed(0)
    , averageWaitTime(0.0f)
{
    // Initialize lanes
    lanes.push_back(std::make_unique<Lane>(LaneId::AL1_INCOMING, false));
    lanes.push_back(std::make_unique<Lane>(LaneId::AL2_PRIORITY, true));
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

    // Initialize traffic lights
    trafficLights[LaneId::AL2_PRIORITY] = TrafficLight();
    trafficLights[LaneId::BL2_NORMAL] = TrafficLight();
    trafficLights[LaneId::CL2_NORMAL] = TrafficLight();
    trafficLights[LaneId::DL2_NORMAL] = TrafficLight();

    synchronizeTrafficLights();
}

void TrafficManager::update(float deltaTime) {
    updateTimers(deltaTime);
    processNewVehicles();
    handleStateTransition(deltaTime);
    updateVehiclePositions(deltaTime);
    updateTrafficLights(deltaTime);
    updateStatistics(deltaTime);

    // Process vehicles based on mode
    if (processingTimer >= VEHICLE_PROCESS_TIME) {
        if (inPriorityMode) {
            processPriorityLane();
        } else {
            processNormalLanes(calculateVehiclesToProcess());
        }
        processFreeLanes();
        processingTimer = 0.0f;
    }

    checkWaitTimes();
    cleanupRemovedVehicles();
}


void TrafficManager::updateVehicleMovement(VehicleState& state, float deltaTime) {
    if (!state.isMoving) return;

    using namespace SimConstants;

    // Check if vehicle should start turning
    if (!state.hasStartedTurn && state.direction != Direction::STRAIGHT) {
        float distToIntersection = getDistanceToIntersection(state);
        if (distToIntersection < TURN_ENTRY_DISTANCE) {
            state.hasStartedTurn = true;
            state.turnProgress = 0.0f;
            state.speed = VEHICLE_TURN_SPEED;
            calculateTurnParameters(state);
        }
    }

    if (state.hasStartedTurn) {
        // Execute turn
        updateTurningMovement(state, deltaTime);
    } else {
        // Normal straight movement
        updateStraightMovement(state, deltaTime);
    }
}


void TrafficManager::addVehicleToLane(LaneId laneId, std::shared_ptr<Vehicle> vehicle) {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [laneId](const auto& lane) { return lane->getId() == laneId; });

    if (it != lanes.end()) {
        (*it)->addVehicle(vehicle);
    }
}

size_t TrafficManager::getLaneSize(LaneId laneId) const {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [laneId](const auto& lane) { return lane->getId() == laneId; });
    return it != lanes.end() ? (*it)->getQueueSize() : 0;
}

bool TrafficManager::isFreeLane(LaneId laneId) const {
    return laneId == LaneId::AL3_FREELANE ||
           laneId == LaneId::BL3_FREELANE ||
           laneId == LaneId::CL3_FREELANE ||
           laneId == LaneId::DL3_FREELANE;
}

Lane* TrafficManager::getPriorityLane() const {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [](const auto& lane) { return lane->isPriorityLane(); });
    return it != lanes.end() ? it->get() : nullptr;
}

bool TrafficManager::isNearIntersection(const VehicleState& state) const {
    float dx = state.pos.x - SimConstants::CENTER_X;
    float dy = state.pos.y - SimConstants::CENTER_Y;
    float distance = std::sqrt(dx * dx + dy * dy);
    return distance < SimConstants::ROAD_WIDTH * 0.75f;
}


bool TrafficManager::isInIntersection(const Position& pos) const {
    using namespace SimConstants;
    float dx = pos.x - CENTER_X;
    float dy = pos.y - CENTER_Y;
    return std::sqrt(dx * dx + dy * dy) < ROAD_WIDTH/2.0f;
}


// TrafficManager.cpp continuation


void TrafficManager::updateVehiclePositions(float deltaTime) {
    for (auto& [_, state] : activeVehicles) {
        if (state.isMoving) {
            // Check if vehicle should start turning
            if (!state.hasStartedTurn && state.direction != Direction::STRAIGHT) {
                float distToIntersection = getDistanceToIntersection(state);
                if (distToIntersection < SimConstants::TURN_ENTRY_DISTANCE) {
                    state.hasStartedTurn = true;
                    state.turnProgress = 0.0f;
                    state.speed = SimConstants::VEHICLE_TURN_SPEED;
                    calculateTurnParameters(state);
                }
            }

            if (state.hasStartedTurn) {
                // Execute turn
                updateTurningMovement(state, deltaTime);
            } else {
                // Normal straight movement
                updateStraightMovement(state, deltaTime);
            }

            // Check for intermediate targets (lane changes)
            if (!state.intermediateTargets.empty()) {
                auto targetPos = state.intermediateTargets.front();
                float dx = targetPos.x - state.pos.x;
                float dy = targetPos.y - state.pos.y;
                float distanceToTarget = std::sqrt(dx * dx + dy * dy);

                if (distanceToTarget < 1.0f) {
                    state.intermediateTargets.erase(state.intermediateTargets.begin());
                } else {
                    float moveSpeed = state.speed * deltaTime;
                    float moveX = state.pos.x + (dx / distanceToTarget) * moveSpeed;
                    float moveY = state.pos.y + (dy / distanceToTarget) * moveSpeed;

                    if (!checkCollision(state, moveX, moveY)) {
                        state.pos.x = moveX;
                        state.pos.y = moveY;
                    }
                }
            }
        }
    }

    // Clean up vehicles that have reached their destination
    cleanupRemovedVehicles();
}


void TrafficManager::updateStraightMovement(VehicleState& state, float deltaTime) {
    if (!state.isMoving) return;

    float dx = state.targetPos.x - state.pos.x;
    float dy = state.targetPos.y - state.pos.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    if (distance > 0.1f) {
        float moveSpeed = state.speed * deltaTime;
        float moveRatio = std::min(1.0f, moveSpeed / distance);

        float newX = state.pos.x + dx * moveRatio;
        float newY = state.pos.y + dy * moveRatio;

        if (!checkCollision(state, newX, newY)) {
            state.pos.x = newX;
            state.pos.y = newY;
            state.turnAngle = std::atan2(dy, dx);
        } else {
            state.speed *= 0.5f; // Slow down when near collision
        }
    }
}


void TrafficManager::updateTurningMovement(VehicleState& state, float deltaTime) {
    if (!state.hasStartedTurn && isNearIntersection(state)) {
        state.hasStartedTurn = true;
        state.turnProgress = 0.0f;
        state.speed = SimConstants::VEHICLE_TURN_SPEED;
    }

    if (state.hasStartedTurn) {
        // Smooth turn progression
        float progressDelta = deltaTime * (1.0f - state.turnProgress * 0.5f);
        state.turnProgress = std::min(1.0f, state.turnProgress + progressDelta);

        // Use cubic easing for smoother turns
        float easedProgress = state.turnProgress * state.turnProgress * (3.0f - 2.0f * state.turnProgress);
        float currentAngle = state.startAngle + (state.endAngle - state.startAngle) * easedProgress;

        // Calculate new position along turn arc
        float newX = state.turnCenter.x + state.turnRadius * std::cos(currentAngle);
        float newY = state.turnCenter.y + state.turnRadius * std::sin(currentAngle);

        // Look ahead for collisions
        bool willCollide = false;
        const int LOOK_AHEAD_STEPS = 5;
        for (int i = 1; i <= LOOK_AHEAD_STEPS; i++) {
            float futureProgress = std::min(1.0f, state.turnProgress + progressDelta * i);
            float futureAngle = state.startAngle + (state.endAngle - state.startAngle) * futureProgress;
            float futureX = state.turnCenter.x + state.turnRadius * std::cos(futureAngle);
            float futureY = state.turnCenter.y + state.turnRadius * std::sin(futureAngle);

            if (checkCollision(state, futureX, futureY)) {
                willCollide = true;
                break;
            }
        }

        if (!willCollide && !checkCollision(state, newX, newY)) {
            state.pos.x = newX;
            state.pos.y = newY;
            state.turnAngle = currentAngle;
            state.inIntersection = true;
        } else {
            state.speed *= 0.8f; // Slow down if collision likely
        }
    } else {
        updateStraightMovement(state, deltaTime);
    }
}

bool TrafficManager::checkCollision(const VehicleState& state, float newX, float newY) const {
    using namespace SimConstants;

    const float MIN_DISTANCE = VEHICLE_WIDTH * 1.5f;

    for (const auto& [otherId, otherState] : activeVehicles) {
        if (otherId != state.vehicle->getId()) {
            float dx = newX - otherState.pos.x;
            float dy = newY - otherState.pos.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < MIN_DISTANCE) {
                return true;
            }
        }
    }
    return false;
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
    state.processingTime = 0.0f;
    state.inIntersection = false;
    state.isPassing = false;

    // Calculate lane offset from center (important for preventing collisions)
    float laneOffset = 0.0f;
    int lanePosition = static_cast<int>(laneId) % 3;

    // Proper lane spacing to prevent collisions
    if (lanePosition == 0) {  // Left lane
        laneOffset = -LANE_WIDTH;
    } else if (lanePosition == 1) {  // Middle lane
        laneOffset = 0.0f;
    } else {  // Right lane
        laneOffset = LANE_WIDTH;
    }

    // Calculate initial positions based on road
    int roadNum = static_cast<int>(laneId) / 3;
    switch(roadNum) {
        case 0: // West approach (A lanes)
            state.pos.x = -VEHICLE_WIDTH * 2;
            state.pos.y = CENTER_Y + laneOffset;
            state.turnAngle = 0.0f;
            break;

        case 1: // North approach (B lanes)
            state.pos.x = CENTER_X + laneOffset;
            state.pos.y = -VEHICLE_HEIGHT * 2;
            state.turnAngle = M_PI/2;
            break;

        case 2: // East approach (C lanes)
            state.pos.x = WINDOW_WIDTH + VEHICLE_WIDTH * 2;
            state.pos.y = CENTER_Y - laneOffset; // Note negative offset
            state.turnAngle = M_PI;
            break;

        case 3: // South approach (D lanes)
            state.pos.x = CENTER_X - laneOffset; // Note negative offset
            state.pos.y = WINDOW_HEIGHT + VEHICLE_HEIGHT * 2;
            state.turnAngle = -M_PI/2;
            break;
    }

    // Calculate target position based on direction

calculateTargetPosition(state, laneId);


    // Add queue offset for multiple vehicles
    size_t queuePosition = getLaneSize(laneId);
    float queueOffset = queuePosition * (VEHICLE_LENGTH + MIN_VEHICLE_SPACING);

    // Apply queue offset based on approach direction
    switch(roadNum) {
        case 0: state.pos.x -= queueOffset; break;
        case 1: state.pos.y -= queueOffset; break;
        case 2: state.pos.x += queueOffset; break;
        case 3: state.pos.y += queueOffset; break;
    }

    activeVehicles[vehicle->getId()] = state;
}// TrafficManager.cpp continuation



void TrafficManager::calculateTurnParameters(VehicleState& state) {
    using namespace SimConstants;

    // Get the quadrant (0-3 for W,N,E,S)
    int quadrant = static_cast<int>(state.vehicle->getCurrentLane()) / 3;

    // Calculate turn center and angles
    switch (quadrant) {
        case 0: // From West
            state.turnCenter.x = CENTER_X - ROAD_WIDTH/4;
            state.turnCenter.y = CENTER_Y;
            state.startAngle = 0.0f;
            state.endAngle = -M_PI/2;  // For right turns
            break;
        case 1: // From North
            state.turnCenter.x = CENTER_X;
            state.turnCenter.y = CENTER_Y - ROAD_WIDTH/4;
            state.startAngle = M_PI/2;
            state.endAngle = 0.0f;  // For right turns
            break;
        case 2: // From East
            state.turnCenter.x = CENTER_X + ROAD_WIDTH/4;
            state.turnCenter.y = CENTER_Y;
            state.startAngle = M_PI;
            state.endAngle = M_PI/2;  // For right turns
            break;
        case 3: // From South
            state.turnCenter.x = CENTER_X;
            state.turnCenter.y = CENTER_Y + ROAD_WIDTH/4;
            state.startAngle = -M_PI/2;
            state.endAngle = M_PI;  // For right turns
            break;
    }
}

void TrafficManager::updateTrafficLights(float deltaTime) {
    // Update each light's state
    for (auto& [_, light] : trafficLights) {
        light.update(deltaTime);
    }

    if (inPriorityMode) {
        // Priority mode: AL2 gets green, others red
        trafficLights[LaneId::AL2_PRIORITY].setState(LightState::GREEN);
        trafficLights[LaneId::BL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::CL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::DL2_NORMAL].setState(LightState::RED);
    } else {
        // Normal mode: Alternate between N-S and E-W traffic
        float cycleTime = 10.0f;
        bool northSouthGreen = std::fmod(stateTimer, cycleTime * 2) < cycleTime;

        trafficLights[LaneId::BL2_NORMAL].setState(northSouthGreen ? LightState::GREEN : LightState::RED);
        trafficLights[LaneId::DL2_NORMAL].setState(northSouthGreen ? LightState::GREEN : LightState::RED);
        trafficLights[LaneId::AL2_PRIORITY].setState(northSouthGreen ? LightState::RED : LightState::GREEN);
        trafficLights[LaneId::CL2_NORMAL].setState(northSouthGreen ? LightState::RED : LightState::GREEN);
    }
}

void TrafficManager::synchronizeTrafficLights() {
    if (inPriorityMode) {
        trafficLights[LaneId::AL2_PRIORITY].setState(LightState::GREEN);
        trafficLights[LaneId::BL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::CL2_NORMAL].setState(LightState::RED);
        trafficLights[LaneId::DL2_NORMAL].setState(LightState::RED);
        std::cout << "Priority mode: AL2 green, others red" << std::endl;
    } else {
        // Alternate N-S and E-W every 30s
        bool northSouthGreen = ((int)stateTimer % 60) < 30;

        trafficLights[LaneId::AL2_PRIORITY].setState(northSouthGreen ? LightState::RED : LightState::GREEN);
        trafficLights[LaneId::BL2_NORMAL].setState(northSouthGreen ? LightState::GREEN : LightState::RED);
        trafficLights[LaneId::CL2_NORMAL].setState(northSouthGreen ? LightState::RED : LightState::GREEN);
        trafficLights[LaneId::DL2_NORMAL].setState(northSouthGreen ? LightState::GREEN : LightState::RED);

        std::cout << "Normal mode: " << (northSouthGreen ? "N-S green, E-W red" : "E-W green, N-S red") << std::endl;
    }
}

void TrafficManager::handleStateTransition(float deltaTime) {
    bool shouldBePriority = checkPriorityConditions();

    if (shouldBePriority && !inPriorityMode) {
        if (stateTimer >= MIN_STATE_TIME) {
            inPriorityMode = true;
            stateTimer = 0.0f;
            synchronizeTrafficLights();
        }
    } else if (!shouldBePriority && inPriorityMode) {
        auto* priorityLane = getPriorityLane();
        if (priorityLane && priorityLane->getQueueSize() <= PRIORITY_RELEASE_THRESHOLD &&
            stateTimer >= MIN_STATE_TIME) {
            inPriorityMode = false;
            stateTimer = 0.0f;
            synchronizeTrafficLights();
        }
    }

    // Force state change if stuck too long
    if (stateTimer >= MAX_STATE_TIME) {
        inPriorityMode = !inPriorityMode;
        stateTimer = 0.0f;
        synchronizeTrafficLights();
    }
}



void TrafficManager::processNewVehicles() {
    auto newVehicles = fileHandler.readNewVehicles();

    for (const auto& [laneId, vehicle] : newVehicles) {
        std::cout << "Processing new vehicle " << vehicle->getId()
                  << " for lane " << static_cast<int>(laneId) << std::endl;

        LaneId optimalLane = determineOptimalLane(vehicle->getDirection(), laneId);

        if (isValidSpawnLane(optimalLane, vehicle->getDirection())) {
            addVehicleToLane(optimalLane, vehicle);
            addNewVehicleToState(vehicle, optimalLane);

            std::cout << "Vehicle " << vehicle->getId()
                      << " added to lane " << static_cast<int>(optimalLane) << std::endl;
        } else {
            std::cerr << "Invalid spawn configuration for vehicle "
                      << vehicle->getId() << std::endl;
        }
    }
}

LaneId TrafficManager::determineOptimalLane(Direction direction, LaneId sourceLane) const {
    int roadGroup = static_cast<int>(sourceLane) / 3;

    switch (direction) {
        case Direction::LEFT:
            return static_cast<LaneId>(roadGroup * 3 + 2);
        case Direction::RIGHT:
            return static_cast<LaneId>(roadGroup * 3);
        default: {
            LaneId lane1 = static_cast<LaneId>(roadGroup * 3);
            LaneId lane2 = static_cast<LaneId>(roadGroup * 3 + 1);
            size_t lane1Size = getLaneSize(lane1);
            size_t lane2Size = getLaneSize(lane2);

            if (lane1Size <= lane2Size) {
                return lane1;
            } else if (lane2Size <= 2) {
                return lane2;
            } else {
                return lane1;
            }
        }
    }
}
bool TrafficManager::isValidSpawnLane(LaneId laneId, Direction direction) const {
    int laneInRoad = static_cast<int>(laneId) % 3;

    switch (direction) {
        case Direction::LEFT:  return laneInRoad == 2;
        case Direction::RIGHT: return laneInRoad == 0;
        default:               return laneInRoad == 0 || laneInRoad == 1;
    }
}

void TrafficManager::processPriorityLane() {
    auto* priorityLane = getPriorityLane();
    if (!priorityLane) return;

    while (priorityLane->getQueueSize() > PRIORITY_RELEASE_THRESHOLD) {
        auto vehicle = priorityLane->removeVehicle();
        if (vehicle) {
            auto it = activeVehicles.find(vehicle->getId());
            if (it != activeVehicles.end()) {
                it->second.isMoving = true;
            }
        }
    }
}


// TrafficManager.cpp final part

void TrafficManager::processNormalLanes(size_t vehicleCount) {
    if (vehicleCount == 0) return;

    for (auto& lane : lanes) {
        if (!lane->isPriorityLane() && !isFreeLane(lane->getId())) {
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

void TrafficManager::processFreeLanes() {
    for (auto& lane : lanes) {
        if (isFreeLane(lane->getId())) {
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

size_t TrafficManager::calculateVehiclesToProcess() const {
    size_t totalVehicles = 0;
    size_t normalLaneCount = 0;

    for (const auto& lane : lanes) {
        if (!lane->isPriorityLane() && !isFreeLane(lane->getId())) {
            totalVehicles += lane->getQueueSize();
            normalLaneCount++;
        }
    }

    return normalLaneCount > 0 ?
        static_cast<size_t>(std::ceil(static_cast<float>(totalVehicles) / normalLaneCount)) : 0;
}

void TrafficManager::checkWaitTimes() {
    using namespace SimConstants;

    for (auto& lane : lanes) {
        if (lane->getQueueSize() > 0 && !isFreeLane(lane->getId())) {
            bool needsProcessing = false;

            // Get the first vehicle's wait time
            auto firstVehicleIt = std::find_if(activeVehicles.begin(), activeVehicles.end(),
                [&lane](const auto& pair) {
                    return pair.second.vehicle->getCurrentLane() == lane->getId() &&
                           !pair.second.isMoving;
                });

            if (firstVehicleIt != activeVehicles.end()) {
                if (firstVehicleIt->second.waitTime > MAX_WAIT_TIME) {
                    needsProcessing = true;
                }
            }

            if (needsProcessing || lane->getQueueSize() >= 8) {
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

void TrafficManager::updateTimers(float deltaTime) {
    stateTimer += deltaTime;
    processingTimer += deltaTime;
    lastUpdateTime += deltaTime;

    // Update wait times for vehicles
    for (auto& [_, state] : activeVehicles) {
        if (!state.isMoving) {
            state.waitTime += deltaTime;
        }
    }
}

void TrafficManager::updateStatistics(float deltaTime) {
    // Update average wait time
    float totalWaitTime = 0.0f;
    size_t waitingVehicles = 0;

    for (const auto& [_, state] : activeVehicles) {
        if (!state.isMoving) {
            totalWaitTime += state.waitTime;
            waitingVehicles++;
        }
    }

    if (waitingVehicles > 0) {
        averageWaitTime = totalWaitTime / static_cast<float>(waitingVehicles);
    }
}

float TrafficManager::calculateAverageWaitTime() const {
    float totalWaitTime = 0.0f;
    size_t vehicleCount = 0;

    for (const auto& [_, state] : activeVehicles) {
        if (!state.isMoving) {
            totalWaitTime += state.waitTime;
            vehicleCount++;
        }
    }

    return vehicleCount > 0 ? totalWaitTime / static_cast<float>(vehicleCount) : 0.0f;
}

size_t TrafficManager::getQueuedVehicleCount() const {
    size_t count = 0;
    for (const auto& lane : lanes) {
        count += lane->getQueueSize();
    }
    return count;
}

void TrafficManager::cleanupRemovedVehicles() {
    auto it = activeVehicles.begin();
    while (it != activeVehicles.end()) {
        if (hasReachedDestination(it->second)) {
            it = activeVehicles.erase(it);
        } else {
            ++it;
        }
    }
}

bool TrafficManager::checkPriorityConditions() const {
    auto* priorityLane = getPriorityLane();
    if (!priorityLane) return false;

    return priorityLane->getQueueSize() > PRIORITY_THRESHOLD;
}

float TrafficManager::calculateTurningRadius(Direction dir) const {
    using namespace SimConstants;
    switch (dir) {
        case Direction::LEFT:
            return TURN_GUIDE_RADIUS * 1.2f;  // Wider radius for left turns
        case Direction::RIGHT:
            return TURN_GUIDE_RADIUS * 0.8f;  // Tighter radius for right turns
        default:
            return TURN_GUIDE_RADIUS;
    }
}

Position TrafficManager::calculateLaneEndpoint(LaneId laneId) const {
    using namespace SimConstants;
    const float EXIT_DISTANCE = QUEUE_START_OFFSET * 1.5f;

    float laneOffset = static_cast<float>((static_cast<int>(laneId) % 3)) * LANE_WIDTH;
    float baseY = CENTER_Y - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;

    switch (laneId) {
        case LaneId::AL1_INCOMING:
        case LaneId::AL2_PRIORITY:
        case LaneId::AL3_FREELANE:
            return Position(CENTER_X + EXIT_DISTANCE, baseY);

        case LaneId::BL1_INCOMING:
        case LaneId::BL2_NORMAL:
        case LaneId::BL3_FREELANE:
            return Position(CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset,
                          CENTER_Y + EXIT_DISTANCE);

        case LaneId::CL1_INCOMING:
        case LaneId::CL2_NORMAL:
        case LaneId::CL3_FREELANE:
            return Position(CENTER_X - EXIT_DISTANCE, baseY);

        case LaneId::DL1_INCOMING:
        case LaneId::DL2_NORMAL:
        case LaneId::DL3_FREELANE:
            return Position(CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset,
                          CENTER_Y - EXIT_DISTANCE);

        default:
            return Position(CENTER_X, CENTER_Y);
    }
}

bool TrafficManager::hasReachedDestination(const VehicleState& state) const {
    float dx = state.pos.x - state.targetPos.x;
    float dy = state.pos.y - state.targetPos.y;
    return std::sqrt(dx * dx + dy * dy) < 1.0f;
}



void TrafficManager::calculateTurnPath(VehicleState& state) {
    using namespace SimConstants;
    float turnOffset = ROAD_WIDTH * 0.25f;
    state.turnRadius = calculateTurningRadius(state.direction);

    LaneId laneId = state.vehicle->getCurrentLane();
    bool isLeftTurn = state.direction == Direction::LEFT;

    // Set turn center based on lane and turn direction
    switch (laneId) {
        case LaneId::AL1_INCOMING:
        case LaneId::AL2_PRIORITY:
        case LaneId::AL3_FREELANE:
            state.turnCenter.x = CENTER_X - ROAD_WIDTH/2.0f + turnOffset;
            state.turnCenter.y = isLeftTurn ?
                CENTER_Y - ROAD_WIDTH/2.0f - turnOffset :
                CENTER_Y + ROAD_WIDTH/2.0f + turnOffset;
            state.startAngle = 0.0f;
            state.endAngle = isLeftTurn ? -M_PI/2.0f : M_PI/2.0f;
            break;

        case LaneId::BL1_INCOMING:
        case LaneId::BL2_NORMAL:
        case LaneId::BL3_FREELANE:
            state.turnCenter.x = isLeftTurn ?
                CENTER_X + ROAD_WIDTH/2.0f + turnOffset :
                CENTER_X - ROAD_WIDTH/2.0f - turnOffset;
            state.turnCenter.y = CENTER_Y - ROAD_WIDTH/2.0f + turnOffset;
            state.startAngle = M_PI/2.0f;
            state.endAngle = isLeftTurn ? 0.0f : M_PI;
            break;

        case LaneId::CL1_INCOMING:
        case LaneId::CL2_NORMAL:
        case LaneId::CL3_FREELANE:
            state.turnCenter.x = CENTER_X + ROAD_WIDTH/2.0f - turnOffset;
            state.turnCenter.y = isLeftTurn ?
                CENTER_Y + ROAD_WIDTH/2.0f + turnOffset :
                CENTER_Y - ROAD_WIDTH/2.0f - turnOffset;
            state.startAngle = M_PI;
            state.endAngle = isLeftTurn ? M_PI/2.0f : -M_PI/2.0f;
            break;

        case LaneId::DL1_INCOMING:
        case LaneId::DL2_NORMAL:
        case LaneId::DL3_FREELANE:
            state.turnCenter.x = isLeftTurn ?
                CENTER_X - ROAD_WIDTH/2.0f - turnOffset :
                CENTER_X + ROAD_WIDTH/2.0f + turnOffset;
            state.turnCenter.y = CENTER_Y + ROAD_WIDTH/2.0f - turnOffset;
            state.startAngle = -M_PI/2.0f;
            state.endAngle = isLeftTurn ? M_PI : 0.0f;
            break;
    }
}

// Add these implementations to TrafficManager.cpp

void TrafficManager::updateVehicleQueuePosition(VehicleState& state, LaneId laneId, size_t queuePosition) {
    using namespace SimConstants;

    float laneOffset = static_cast<float>((static_cast<int>(laneId) % 3)) * LANE_WIDTH;
    float queueOffset = QUEUE_START_OFFSET + (queuePosition * QUEUE_SPACING);

    // Calculate target position based on lane
    switch (laneId) {
        case LaneId::AL1_INCOMING:
        case LaneId::AL2_PRIORITY:
        case LaneId::AL3_FREELANE: {
            state.pos.x = CENTER_X - queueOffset;
            state.pos.y = CENTER_Y - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.turnAngle = 0.0f;
            break;
        }
        case LaneId::BL1_INCOMING:
        case LaneId::BL2_NORMAL:
        case LaneId::BL3_FREELANE: {
            state.pos.x = CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.pos.y = CENTER_Y - queueOffset;
            state.turnAngle = static_cast<float>(M_PI) / 2.0f;
            break;
        }
        case LaneId::CL1_INCOMING:
        case LaneId::CL2_NORMAL:
        case LaneId::CL3_FREELANE: {
            state.pos.x = CENTER_X + queueOffset;
            state.pos.y = CENTER_Y - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.turnAngle = static_cast<float>(M_PI);
            break;
        }
        case LaneId::DL1_INCOMING:
        case LaneId::DL2_NORMAL:
        case LaneId::DL3_FREELANE: {
            state.pos.x = CENTER_X - ROAD_WIDTH/2.0f + LANE_WIDTH/2.0f + laneOffset;
            state.pos.y = CENTER_Y + queueOffset;
            state.turnAngle = -static_cast<float>(M_PI) / 2.0f;
            break;
        }
    }
}



void TrafficManager::calculateTargetPosition(VehicleState& state, LaneId laneId) {
    using namespace SimConstants;

    // Get road number (0-3) and lane position (0-2)
    int roadNum = static_cast<int>(laneId) / 3;
    int lanePosition = static_cast<int>(laneId) % 3;

    if (state.direction == Direction::STRAIGHT) {
        switch(roadNum) {
            case 0: // West to East
                state.targetPos.x = WINDOW_WIDTH + VEHICLE_WIDTH;
                state.targetPos.y = CENTER_Y + (lanePosition - 1) * LANE_WIDTH;
                break;
            case 1: // North to South
                state.targetPos.x = CENTER_X + (lanePosition - 1) * LANE_WIDTH;
                state.targetPos.y = WINDOW_HEIGHT + VEHICLE_HEIGHT;
                break;
            case 2: // East to West
                state.targetPos.x = -VEHICLE_WIDTH;
                state.targetPos.y = CENTER_Y + (1 - lanePosition) * LANE_WIDTH;
                break;
            case 3: // South to North
                state.targetPos.x = CENTER_X + (1 - lanePosition) * LANE_WIDTH;
                state.targetPos.y = -VEHICLE_HEIGHT;
                break;
        }
    }
    else if (state.direction == Direction::RIGHT) {
        switch(roadNum) {
            case 0: // West to North
                state.targetPos.x = CENTER_X + LANE_WIDTH;
                state.targetPos.y = -VEHICLE_HEIGHT;
                break;
            case 1: // North to East
                state.targetPos.x = WINDOW_WIDTH + VEHICLE_WIDTH;
                state.targetPos.y = CENTER_Y - LANE_WIDTH;
                break;
            case 2: // East to South
                state.targetPos.x = CENTER_X - LANE_WIDTH;
                state.targetPos.y = WINDOW_HEIGHT + VEHICLE_HEIGHT;
                break;
            case 3: // South to West
                state.targetPos.x = -VEHICLE_WIDTH;
                state.targetPos.y = CENTER_Y + LANE_WIDTH;
                break;
        }
        state.turnRadius = ROAD_WIDTH * 0.75f;
    }
}
LaneId TrafficManager::determineTargetLane(LaneId currentLane, Direction direction) const {
    // Return appropriate target lane based on current lane and intended direction
    switch(direction) {
        case Direction::LEFT:
            // Third lane only for left turns
            switch(currentLane) {
                case LaneId::AL3_FREELANE: return LaneId::BL3_FREELANE;
                case LaneId::BL3_FREELANE: return LaneId::CL3_FREELANE;
                case LaneId::CL3_FREELANE: return LaneId::DL3_FREELANE;
                case LaneId::DL3_FREELANE: return LaneId::AL3_FREELANE;
                default: return currentLane; // Should not happen
            }

        case Direction::RIGHT:
            // First lane only for right turns
            switch(currentLane) {
                case LaneId::AL1_INCOMING: return LaneId::DL1_INCOMING;
                case LaneId::BL1_INCOMING: return LaneId::AL1_INCOMING;
                case LaneId::CL1_INCOMING: return LaneId::BL1_INCOMING;
                case LaneId::DL1_INCOMING: return LaneId::CL1_INCOMING;
                default: return currentLane;
            }

        default: // STRAIGHT
            // Can use first or second lane
            return currentLane;
    }
}


void TrafficManager::changeLaneToFree(VehicleState& state) {
    float targetLaneY = state.pos.y;
    if (state.pos.x < SimConstants::CENTER_X) targetLaneY += SimConstants::LANE_WIDTH;
    else targetLaneY -= SimConstants::LANE_WIDTH;

    state.intermediateTargets.push_back({state.pos.x, targetLaneY});
}

void TrafficManager::changeLaneToFirst(VehicleState& state) {
    float targetLaneY = state.pos.y;
    if (state.pos.x < SimConstants::CENTER_X) targetLaneY -= SimConstants::LANE_WIDTH;
    else targetLaneY += SimConstants::LANE_WIDTH;

    state.intermediateTargets.push_back({state.pos.x, targetLaneY});
}



bool TrafficManager::canVehicleMove(const VehicleState& state) const {
    using namespace SimConstants;  // Add this for constant access

    // Check traffic light state
    auto lightIt = trafficLights.find(state.vehicle->getCurrentLane());
    if (lightIt != trafficLights.end() && lightIt->second.getState() == LightState::RED
        && !state.inIntersection) {
        // Handle red light stop
        float distToIntersection = getDistanceToIntersection(state);
        if (distToIntersection < STOP_LINE_OFFSET) {
            std::cout << "Vehicle " << state.vehicle->getId() << " stopped at red light" << std::endl;
            return false;
        }
    }

    return !hasVehicleAhead(state);
}

LightState TrafficManager::getLightStateForLane(LaneId laneId) const {
    auto it = trafficLights.find(laneId);
    return it != trafficLights.end() ? it->second.getState() : LightState::RED;
}

float TrafficManager::getDistanceToIntersection(const VehicleState& state) const {
    using namespace SimConstants;

    float dx = CENTER_X - state.pos.x;
    float dy = CENTER_Y - state.pos.y;
    return std::sqrt(dx * dx + dy * dy) - INTERSECTION_RADIUS;
}

bool TrafficManager::hasVehicleAhead(const VehicleState& state) const {
    for (const auto& [_, otherState] : activeVehicles) {
        if (state.vehicle->getId() != otherState.vehicle->getId() &&
            state.vehicle->getCurrentLane() == otherState.vehicle->getCurrentLane()) {

            float dx = otherState.pos.x - state.pos.x;
            float dy = otherState.pos.y - state.pos.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < SimConstants::VEHICLE_MIN_SPACING &&
                isVehicleAhead(state, otherState)) {
                return true;
            }
        }
    }
    return false;
}

bool TrafficManager::isVehicleAhead(const VehicleState& first,
                                  const VehicleState& second) const {
    // Determine if second vehicle is ahead of first based on road direction
    LaneId laneId = first.vehicle->getCurrentLane();

    if (laneId <= LaneId::AL3_FREELANE) {
        return second.pos.x > first.pos.x;
    } else if (laneId <= LaneId::BL3_FREELANE) {
        return second.pos.y > first.pos.y;
    } else if (laneId <= LaneId::CL3_FREELANE) {
        return second.pos.x < first.pos.x;
    } else {
        return second.pos.y < first.pos.y;
    }
}


void TrafficManager::calculateLeftTurnPath(VehicleState& state) {
    using namespace SimConstants;

    // Calculate larger radius for left turns
    state.turnRadius = TURN_GUIDE_RADIUS * 1.2f;

    // Calculate turn center and angles based on approach direction
    LaneId laneId = state.vehicle->getCurrentLane();

    if (laneId <= LaneId::AL3_FREELANE) {  // Coming from West
        // Turn center will be north-west of intersection
        state.turnCenter.x = CENTER_X - ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y - ROAD_WIDTH/2.0f;
        state.startAngle = 0.0f;  // Start facing east
        state.endAngle = -M_PI/2.0f;  // End facing north
        state.targetPos.x = CENTER_X;
        state.targetPos.y = -VEHICLE_HEIGHT;  // Exit north
    }
    else if (laneId <= LaneId::BL3_FREELANE) {  // Coming from North
        // Turn center will be north-east of intersection
        state.turnCenter.x = CENTER_X + ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y - ROAD_WIDTH/2.0f;
        state.startAngle = M_PI/2.0f;  // Start facing south
        state.endAngle = 0.0f;  // End facing east
        state.targetPos.x = WINDOW_WIDTH + VEHICLE_WIDTH;  // Exit east
        state.targetPos.y = CENTER_Y;
    }
    else if (laneId <= LaneId::CL3_FREELANE) {  // Coming from East
        // Turn center will be south-east of intersection
        state.turnCenter.x = CENTER_X + ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y + ROAD_WIDTH/2.0f;
        state.startAngle = M_PI;  // Start facing west
        state.endAngle = M_PI/2.0f;  // End facing south
        state.targetPos.x = CENTER_X;
        state.targetPos.y = WINDOW_HEIGHT + VEHICLE_HEIGHT;  // Exit south
    }
    else {  // Coming from South
        // Turn center will be south-west of intersection
        state.turnCenter.x = CENTER_X - ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y + ROAD_WIDTH/2.0f;
        state.startAngle = -M_PI/2.0f;  // Start facing north
        state.endAngle = M_PI;  // End facing west
        state.targetPos.x = -VEHICLE_WIDTH;  // Exit west
        state.targetPos.y = CENTER_Y;
    }

    // Adjust speed for turning
    state.speed = VEHICLE_TURN_SPEED;
    state.turnProgress = 0.0f;
    state.hasStartedTurn = true;
}

void TrafficManager::calculateRightTurnPath(VehicleState& state) {
    using namespace SimConstants;

    // Calculate smaller radius for right turns
    state.turnRadius = TURN_GUIDE_RADIUS * 0.8f;

    // Calculate turn center and angles based on approach direction
    LaneId laneId = state.vehicle->getCurrentLane();

    if (laneId <= LaneId::AL3_FREELANE) {  // Coming from West
        // Turn center will be south-west of intersection
        state.turnCenter.x = CENTER_X - ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y + ROAD_WIDTH/2.0f;
        state.startAngle = 0.0f;  // Start facing east
        state.endAngle = M_PI/2.0f;  // End facing south
        state.targetPos.x = CENTER_X;
        state.targetPos.y = WINDOW_HEIGHT + VEHICLE_HEIGHT;  // Exit south
    }
    else if (laneId <= LaneId::BL3_FREELANE) {  // Coming from North
        // Turn center will be north-west of intersection
        state.turnCenter.x = CENTER_X - ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y - ROAD_WIDTH/2.0f;
        state.startAngle = M_PI/2.0f;  // Start facing south
        state.endAngle = M_PI;  // End facing west
        state.targetPos.x = -VEHICLE_WIDTH;  // Exit west
        state.targetPos.y = CENTER_Y;
    }
    else if (laneId <= LaneId::CL3_FREELANE) {  // Coming from East
        // Turn center will be north-east of intersection
        state.turnCenter.x = CENTER_X + ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y - ROAD_WIDTH/2.0f;
        state.startAngle = M_PI;  // Start facing west
        state.endAngle = -M_PI/2.0f;  // End facing north
        state.targetPos.x = CENTER_X;
        state.targetPos.y = -VEHICLE_HEIGHT;  // Exit north
    }
    else {  // Coming from South
        // Turn center will be south-east of intersection
        state.turnCenter.x = CENTER_X + ROAD_WIDTH/2.0f;
        state.turnCenter.y = CENTER_Y + ROAD_WIDTH/2.0f;
        state.startAngle = -M_PI/2.0f;  // Start facing north
        state.endAngle = 0.0f;  // End facing east
        state.targetPos.x = WINDOW_WIDTH + VEHICLE_WIDTH;  // Exit east
        state.targetPos.y = CENTER_Y;
    }

    // Adjust speed for turning
    state.speed = VEHICLE_TURN_SPEED;
    state.turnProgress = 0.0f;
    state.hasStartedTurn = true;
}