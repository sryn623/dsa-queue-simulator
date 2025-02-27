
// src/managers/TrafficManager.cpp
#include "managers/TrafficManager.h"
#include <algorithm>
#include <iostream>

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

    // Initialize traffic lights with their controlled lanes
    trafficLights[LaneId::AL2_PRIORITY] = TrafficLight(LaneId::AL2_PRIORITY);
    trafficLights[LaneId::BL2_NORMAL] = TrafficLight(LaneId::BL2_NORMAL);
    trafficLights[LaneId::CL2_NORMAL] = TrafficLight(LaneId::CL2_NORMAL);
    trafficLights[LaneId::DL2_NORMAL] = TrafficLight(LaneId::DL2_NORMAL);

    // Initialize lane priority queue
    updateLaneQueue();
    synchronizeTrafficLights();
}

// Fix for the update method to make it safer
void TrafficManager::update(float deltaTime) {
    // Update timers first to avoid calling methods that might lock mutexes while holding a lock
    updateTimers(deltaTime);

    // These operations can be done independently with no risk of deadlock
    processNewVehicles();
    handleStateTransition(deltaTime);
    updateVehiclePositions(deltaTime);
    updateTrafficLights(deltaTime);

    // Calculate stats without modifying data
    updateStatistics(deltaTime);

    // Process vehicles based on mode
    if (processingTimer >= SimConstants::VEHICLE_PROCESS_TIME) {
        if (inPriorityMode) {
            processPriorityLane();
        } else {
            processNormalLanes(calculateVehiclesToProcess());
        }
        processFreeLanes();
        processingTimer = 0.0f;

        // Update lane queue after processing
        updateLaneQueue();
    }

    // Check wait times and clean up
    checkWaitTimes();
    cleanupFinishedVehicles();

    // Update individual lanes
    for (auto& lane : lanes) {
        lane->update(deltaTime);
    }
}

// Add this implementation to the updateLaneQueue method
void TrafficManager::updateLaneQueue() {
    // Create a temporary new queue instead of modifying the existing one in-place
    PriorityQueue<LaneId> newQueue;

    // Add lanes with calculated priorities
    for (const auto& lane : lanes) {
        if (!isFreeLane(lane->getId())) {
            int priority = 1; // Default priority

            if (lane->isPriorityLane() && lane->getQueueSize() > SimConstants::PRIORITY_THRESHOLD) {
                priority = 3; // Highest priority
            }
            else if (lane->getQueueSize() > 8) {
                priority = 2; // Medium priority
            }

            newQueue.enqueuePriority(lane->getId(), priority);
        }
    }

    // Move the new queue to replace the existing one (no mutex deadlock issue)
    laneQueue = std::move(newQueue);
}

void TrafficManager::addVehicleToLane(LaneId laneId, std::shared_ptr<Vehicle> vehicle) {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [laneId](const auto& lane) { return lane->getId() == laneId; });

    if (it != lanes.end()) {
        (*it)->addVehicle(vehicle);
        activeVehicles[vehicle->getId()] = vehicle;

        // Position vehicle based on lane
        setupVehicleTurn(vehicle);
    }
}

void TrafficManager::setupVehicleTurn(std::shared_ptr<Vehicle> vehicle) {
    using namespace SimConstants;

    // Initialize vehicle position based on lane
    LaneId laneId = vehicle->getCurrentLane();
    int roadNum = static_cast<int>(laneId) / 3; // 0-3 for W,N,E,S
    int lanePosition = static_cast<int>(laneId) % 3; // 0-2 for lane position

    // Calculate lane offset from center
    float laneOffset = (lanePosition - 1) * LANE_WIDTH;

    // Set initial position based on approach direction
    float initialX = CENTER_X;
    float initialY = CENTER_Y;
    float initialAngle = 0.0f;

    switch (roadNum) {
        case 0: // West approach (A lanes)
            initialX = -VEHICLE_WIDTH;
            initialY = CENTER_Y + laneOffset;
            initialAngle = 0.0f; // Facing east
            break;

        case 1: // North approach (B lanes)
            initialX = CENTER_X + laneOffset;
            initialY = -VEHICLE_HEIGHT;
            initialAngle = M_PI/2.0f; // Facing south
            break;

        case 2: // East approach (C lanes)
            initialX = WINDOW_WIDTH + VEHICLE_WIDTH;
            initialY = CENTER_Y - laneOffset; // Note negative offset
            initialAngle = M_PI; // Facing west
            break;

        case 3: // South approach (D lanes)
            initialX = CENTER_X - laneOffset; // Note negative offset
            initialY = WINDOW_HEIGHT + VEHICLE_HEIGHT;
            initialAngle = -M_PI/2.0f; // Facing north
            break;
    }

    // Set vehicle position and targets
    vehicle->setTargetPosition(initialX, initialY, initialAngle);

    // Calculate turn parameters if needed
    if (vehicle->getDirection() != Direction::STRAIGHT) {
        vehicle->calculateTurnParameters(ROAD_WIDTH, LANE_WIDTH, CENTER_X, CENTER_Y);
    }
}

size_t TrafficManager::getLaneSize(LaneId laneId) const {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [laneId](const auto& lane) { return lane->getId() == laneId; });
    return it != lanes.end() ? (*it)->getQueueSize() : 0;
}

void TrafficManager::updateVehiclePositions(float deltaTime) {
    // Update position for each active vehicle
    for (auto& [_, vehicle] : activeVehicles) {
        if (vehicle->isInProcess()) {
            if (vehicle->isTurning()) {
                vehicle->updateTurn(deltaTime);
            } else {
                vehicle->updateMovement(deltaTime);
            }
        } else {
            // Update wait time for non-moving vehicles
            vehicle->updateWaitTime(deltaTime);
        }
    }

    // Update turning progress
    updateVehicleTurns(deltaTime);
}

void TrafficManager::updateVehicleTurns(float deltaTime) {
    for (auto& [_, vehicle] : activeVehicles) {
        if (vehicle->isInProcess() && vehicle->hasTurnStarted()) {
            vehicle->updateTurnProgress(deltaTime);

            // Check if turn is complete
            if (vehicle->getTurnProgress() >= 1.0f) {
                // Update lane based on turn direction
                LaneId currentLane = vehicle->getCurrentLane();
                Direction direction = vehicle->getDirection();

                // Determine target lane based on current lane and direction
                LaneId targetLane = currentLane;
                int roadGroup = static_cast<int>(currentLane) / 3;

                switch (direction) {
                    case Direction::LEFT:
                        // Left turns go to the next road counterclockwise
                        targetLane = static_cast<LaneId>((roadGroup + 3) % 4 * 3 + 1); // Middle lane
                        break;

                    case Direction::RIGHT:
                        // Right turns go to the next road clockwise
                        targetLane = static_cast<LaneId>((roadGroup + 1) % 4 * 3 + 1); // Middle lane
                        break;

                    default:
                        // Straight continues to opposite road
                        targetLane = static_cast<LaneId>((roadGroup + 2) % 4 * 3 + 1); // Middle lane
                        break;
                }

                vehicle->setTargetLane(targetLane);
            }
        }
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
        float cycleTime = 15.0f;
        bool northSouthGreen = std::fmod(stateTimer, cycleTime * 2) < cycleTime;

        trafficLights[LaneId::BL2_NORMAL].setState(northSouthGreen ? LightState::GREEN : LightState::RED);
        trafficLights[LaneId::DL2_NORMAL].setState(northSouthGreen ? LightState::GREEN : LightState::RED);
        trafficLights[LaneId::AL2_PRIORITY].setState(northSouthGreen ? LightState::RED : LightState::GREEN);
        trafficLights[LaneId::CL2_NORMAL].setState(northSouthGreen ? LightState::RED : LightState::GREEN);
    }
}

void TrafficManager::synchronizeTrafficLights() {
    // All traffic lights should have a consistent state
    if (inPriorityMode) {
        // In priority mode, only AL2 gets green
        for (auto& [laneId, light] : trafficLights) {
            light.setPriorityMode(true);

            if (laneId == LaneId::AL2_PRIORITY) {
                light.setState(LightState::GREEN);
            } else {
                light.setState(LightState::RED);
            }
        }
    } else {
        // In normal mode, we alternate N-S and E-W
        for (auto& [_, light] : trafficLights) {
            light.setPriorityMode(false);
        }

        // Initial setup: N-S green
        trafficLights[LaneId::BL2_NORMAL].setState(LightState::GREEN);
        trafficLights[LaneId::DL2_NORMAL].setState(LightState::GREEN);
        trafficLights[LaneId::AL2_PRIORITY].setState(LightState::RED);
        trafficLights[LaneId::CL2_NORMAL].setState(LightState::RED);
    }
}

void TrafficManager::handleStateTransition(float deltaTime) {
    bool shouldBePriority = checkPriorityConditions();

    if (shouldBePriority && !inPriorityMode) {
        if (stateTimer >= SimConstants::MIN_STATE_TIME) {
            inPriorityMode = true;
            stateTimer = 0.0f;
            synchronizeTrafficLights();
            std::cout << "Switching to PRIORITY mode" << std::endl;
        }
    } else if (!shouldBePriority && inPriorityMode) {
        auto* priorityLane = getPriorityLane();
        if (priorityLane && priorityLane->getQueueSize() <= SimConstants::NORMAL_THRESHOLD &&
            stateTimer >= SimConstants::MIN_STATE_TIME) {
            inPriorityMode = false;
            stateTimer = 0.0f;
            synchronizeTrafficLights();
            std::cout << "Switching to NORMAL mode" << std::endl;
        }
    }

    // Force state change if stuck too long
    if (stateTimer >= SimConstants::MAX_WAIT_TIME) {
        inPriorityMode = !inPriorityMode;
        stateTimer = 0.0f;
        synchronizeTrafficLights();
        std::cout << "FORCED mode switch after timeout" << std::endl;
    }
}

bool TrafficManager::checkPriorityConditions() const {
    // Check if priority lane exceeds threshold
    auto* priorityLane = getPriorityLane();
    if (!priorityLane) return false;

    return priorityLane->getQueueSize() > SimConstants::PRIORITY_THRESHOLD;
}

void TrafficManager::processNewVehicles() {
    // Get new vehicles from files
    auto newVehicles = fileHandler.readNewVehicles();

    for (const auto& [laneId, vehicle] : newVehicles) {
        // Check if this is a valid lane for the vehicle's direction
        LaneId optimalLane = determineOptimalLane(vehicle->getDirection(), laneId);

        if (isValidSpawnLane(optimalLane, vehicle->getDirection())) {
            addVehicleToLane(optimalLane, vehicle);
            std::cout << "Vehicle " << vehicle->getId() << " added to lane "
                     << static_cast<int>(optimalLane) << std::endl;
        } else {
            std::cerr << "Invalid spawn configuration for vehicle " << vehicle->getId() << std::endl;
        }
    }
}

LaneId TrafficManager::determineOptimalLane(Direction direction, LaneId sourceLane) const {
    int roadGroup = static_cast<int>(sourceLane) / 3;

    switch (direction) {
        case Direction::LEFT:
            // Left turns use the 3rd lane (freelane)
            return static_cast<LaneId>(roadGroup * 3 + 2);

        case Direction::RIGHT:
            // Right turns use the 1st lane
            return static_cast<LaneId>(roadGroup * 3);

        default: // STRAIGHT
            // Check queue sizes of 1st and 2nd lanes to balance
            LaneId lane1 = static_cast<LaneId>(roadGroup * 3);     // First lane
            LaneId lane2 = static_cast<LaneId>(roadGroup * 3 + 1); // Second lane

            size_t lane1Size = getLaneSize(lane1);
            size_t lane2Size = getLaneSize(lane2);

            // Choose lane with shortest queue
            return (lane1Size <= lane2Size) ? lane1 : lane2;
    }
}

bool TrafficManager::isValidSpawnLane(LaneId laneId, Direction direction) const {
    int laneInRoad = static_cast<int>(laneId) % 3;

    switch (direction) {
        case Direction::LEFT:  return laneInRoad == 2; // Only 3rd lane for left turns
        case Direction::RIGHT: return laneInRoad == 0; // 1st lane for right turns
        default:               return laneInRoad == 0 || laneInRoad == 1; // 1st or 2nd for straight
    }
}

void TrafficManager::processPriorityLane() {
    auto* priorityLane = getPriorityLane();
    if (!priorityLane) return;

    size_t initialSize = priorityLane->getQueueSize();
    size_t processCount = 0;

    // Process until below normal threshold or max count reached
    while (priorityLane->getQueueSize() > SimConstants::NORMAL_THRESHOLD &&
           processCount < initialSize) {
        auto vehicle = priorityLane->removeVehicle();
        if (vehicle) {
            vehicle->setProcessing(true);
            processCount++;
            totalVehiclesProcessed++;
        }
    }

    std::cout << "Priority mode: Processed " << processCount << " vehicles" << std::endl;
}

void TrafficManager::processNormalLanes(size_t vehicleCount) {
    if (vehicleCount == 0) return;

    // Count of vehicles to process per normal lane
    for (auto& lane : lanes) {
        if (!lane->isPriorityLane() && !isFreeLane(lane->getId())) {
            for (size_t i = 0; i < vehicleCount && lane->getQueueSize() > 0; ++i) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    vehicle->setProcessing(true);
                    totalVehiclesProcessed++;
                }
            }
        }
    }

    std::cout << "Normal mode: Processed " << vehicleCount << " vehicles per lane" << std::endl;
}

void TrafficManager::processFreeLanes() {
    // Free lanes (third lane of each road) always process all their vehicles
    for (auto& lane : lanes) {
        if (isFreeLane(lane->getId())) {
            size_t initialSize = lane->getQueueSize();
            size_t processCount = 0;

            while (lane->getQueueSize() > 0) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    vehicle->setProcessing(true);
                    processCount++;
                    totalVehiclesProcessed++;
                }
            }

            if (processCount > 0) {
                std::cout << "Free lane " << static_cast<int>(lane->getId())
                         << ": Processed " << processCount << " vehicles" << std::endl;
            }
        }
    }
}

size_t TrafficManager::calculateVehiclesToProcess() const {
    // Formula: |V| = (1/n) * Σ|Li|
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

    // Check if any vehicle has been waiting too long
    for (auto& lane : lanes) {
        if (lane->getQueueSize() > 0 && !isFreeLane(lane->getId())) {
            // Process vehicles with excessive wait time
            if (lane->getAverageWaitTime() > MAX_WAIT_TIME) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    vehicle->setProcessing(true);
                    totalVehiclesProcessed++;
                    std::cout << "Processing vehicle with excessive wait time: "
                             << vehicle->getId() << std::endl;
                }
            }
        }
    }
}

void TrafficManager::updateTimers(float deltaTime) {
    stateTimer += deltaTime;
    processingTimer += deltaTime;
    lastUpdateTime += deltaTime;
}

void TrafficManager::updateStatistics(float deltaTime) {
    // Update average wait time for vehicles
    float totalWaitTime = 0.0f;
    size_t waitingVehicles = 0;

    for (const auto& [_, vehicle] : activeVehicles) {
        if (!vehicle->isInProcess()) {
            totalWaitTime += vehicle->getWaitTime();
            waitingVehicles++;
        }
    }

    if (waitingVehicles > 0) {
        averageWaitTime = totalWaitTime / static_cast<float>(waitingVehicles);
    }
}

float TrafficManager::getAverageWaitingTime() const {
    return averageWaitTime;
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

void TrafficManager::cleanupFinishedVehicles() {
    // Remove vehicles that have left the scene
    std::vector<uint32_t> toRemove;

    for (const auto& [id, vehicle] : activeVehicles) {
        // Check if vehicle has exited the screen bounds
        float x = vehicle->getX();
        float y = vehicle->getY();

        bool outOfBounds = (
            x < -SimConstants::VEHICLE_WIDTH * 2 ||
            x > SimConstants::WINDOW_WIDTH + SimConstants::VEHICLE_WIDTH * 2 ||
            y < -SimConstants::VEHICLE_HEIGHT * 2 ||
            y > SimConstants::WINDOW_HEIGHT + SimConstants::VEHICLE_HEIGHT * 2
        );

        if (outOfBounds) {
            toRemove.push_back(id);
        }
    }

    // Remove the vehicles marked for removal
    for (uint32_t id : toRemove) {
        removeVehicle(id);
    }
}

void TrafficManager::removeVehicle(uint32_t vehicleId) {
    activeVehicles.erase(vehicleId);
}

bool TrafficManager::checkCollision(const std::shared_ptr<Vehicle>& vehicle, float newX, float newY) const {
    using namespace SimConstants;

    const float MIN_DISTANCE = VEHICLE_WIDTH * 1.5f;

    for (const auto& [otherId, otherVehicle] : activeVehicles) {
        if (otherId != vehicle->getId()) {
            float dx = newX - otherVehicle->getX();
            float dy = newY - otherVehicle->getY();
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < MIN_DISTANCE) {
                return true;
            }
        }
    }

    return false;
}