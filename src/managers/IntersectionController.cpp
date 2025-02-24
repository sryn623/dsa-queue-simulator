// IntersectionController.cpp
#include "managers/IntersectionController.h"
#include <algorithm>
#include <numeric>

IntersectionController::IntersectionController(std::vector<std::unique_ptr<Lane>>& lanes)
    : lanes(lanes)
    , isPriorityMode(false)
    , stateTimer(0.0f)
    , elapsedTime(0.0f)
    , processingTimer(0.0f)
    , vehiclesProcessedInState(0)
    , totalVehiclesProcessed(0)
{
    updateLaneQueue();
}

void IntersectionController::update(float deltaTime) {
    updateTimers(deltaTime);

    // Check state transitions
    handleStateTransition();

    // Process vehicles based on current state
    if (processingTimer >= calculateProcessingTime()) {
        if (isPriorityMode) {
            processPriorityLane();
        } else {
            processNormalLanes();
        }
        processingTimer = 0.0f;
    }

    // Always process free lanes
    processFreeLanes();

    // Check wait times and update priorities
    checkWaitTimes();
    updateLaneQueue();
}

void IntersectionController::updateTimers(float deltaTime) {
    stateTimer += deltaTime;
    elapsedTime += deltaTime;
    processingTimer += deltaTime;
}

void IntersectionController::handleStateTransition() {
    if (isPriorityMode && shouldSwitchToNormalMode()) {
        isPriorityMode = false;
        resetStateTimers();
    }
    else if (!isPriorityMode && shouldSwitchToPriorityMode()) {
        isPriorityMode = true;
        resetStateTimers();
    }

    // Force state change if stuck too long
    if (stateTimer >= MAX_STATE_TIME) {
        isPriorityMode = !isPriorityMode;
        resetStateTimers();
    }
}

void IntersectionController::updateLaneQueue() {
    // Clear existing queue
    while (!laneQueue.isEmpty()) {
        laneQueue.dequeue();
    }

    // Add lanes with calculated priorities
    for (const auto& lane : lanes) {
        if (!isFreeLane(lane->getId())) {
            int priority = calculateLanePriority(*lane);
            laneQueue.enqueuePriority(lane->getId(), priority);
        }
    }
}

int IntersectionController::calculateLanePriority(const Lane& lane) const {
    int priority = 1; // Base priority

    if (lane.isPriorityLane() && lane.getQueueSize() > PRIORITY_THRESHOLD) {
        priority = 3; // Highest priority
    }
    else if (lane.getQueueSize() > 8) {
        priority = 2; // Medium priority
    }

    return priority;
}

void IntersectionController::processPriorityLane() {
    Lane* priorityLane = getPriorityLane();
    if (!priorityLane) return;

    size_t initialSize = priorityLane->getQueueSize();
    while (priorityLane->getQueueSize() > PRIORITY_RELEASE_THRESHOLD) {
        auto vehicle = priorityLane->removeVehicle();
        if (vehicle) {
            vehiclesProcessedInState++;
            totalVehiclesProcessed++;
        }
    }
}

void IntersectionController::processNormalLanes() {
    size_t vehiclesToProcess = calculateVehiclesToProcess();

    for (auto& lane : lanes) {
        if (!isFreeLane(lane->getId()) && !lane->isPriorityLane()) {
            for (size_t i = 0; i < vehiclesToProcess && lane->getQueueSize() > 0; ++i) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    vehiclesProcessedInState++;
                    totalVehiclesProcessed++;
                }
            }
        }
    }
}

void IntersectionController::processFreeLanes() {
    for (auto& lane : lanes) {
        if (isFreeLane(lane->getId())) {
            while (lane->getQueueSize() > 0) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    vehiclesProcessedInState++;
                    totalVehiclesProcessed++;
                }
            }
        }
    }
}

size_t IntersectionController::calculateVehiclesToProcess() const {
    // |V| = 1/n Σ|Li| formula from assignment
    float avgVehicles = calculateAverageWaitingVehicles();
    return static_cast<size_t>(std::ceil(avgVehicles));
}

float IntersectionController::calculateAverageWaitingVehicles() const {
    size_t totalVehicles = 0;
    size_t normalLaneCount = 0;

    // Only count vehicles in normal lanes (not free or priority lanes)
    for (const auto& lane : lanes) {
        if (!isFreeLane(lane->getId()) && !lane->isPriorityLane()) {
            totalVehicles += lane->getQueueSize();
            normalLaneCount++;
        }
    }

    return normalLaneCount > 0 ?
        static_cast<float>(totalVehicles) / static_cast<float>(normalLaneCount) : 0.0f;
}

float IntersectionController::calculateProcessingTime() const {
    // According to assignment formula: T = |V| * t
    // where |V| = average number of waiting vehicles
    // and t = 2 seconds per vehicle

    if (isPriorityMode) {
        // In priority mode, only process priority lane
        Lane* priorityLane = getPriorityLane();
        if (priorityLane) {
            return priorityLane->getQueueSize() * SimConstants::VEHICLE_PROCESS_TIME;
        }
        return 0.0f;
    }

    // In normal mode, calculate average of normal lanes
    // |V| = 1/n Σ|Li| where n is number of normal lanes
    float avgVehicles = calculateAverageWaitingVehicles();
    return avgVehicles * SimConstants::VEHICLE_PROCESS_TIME;
}


bool IntersectionController::shouldSwitchToNormalMode() const {
    if (!isPriorityMode) return false;

    Lane* priorityLane = getPriorityLane();
    return priorityLane &&
           priorityLane->getQueueSize() <= SimConstants::NORMAL_THRESHOLD &&
           stateTimer >= MIN_STATE_TIME;
}

bool IntersectionController::shouldSwitchToPriorityMode() const {
    if (isPriorityMode) return false;

    Lane* priorityLane = getPriorityLane();
    return priorityLane &&
           priorityLane->getQueueSize() > SimConstants::PRIORITY_THRESHOLD &&
           stateTimer >= MIN_STATE_TIME;
}

void IntersectionController::resetStateTimers() {
    stateTimer = 0.0f;
    processingTimer = 0.0f;
    vehiclesProcessedInState = 0;
}

Lane* IntersectionController::getPriorityLane() const {
    auto it = std::find_if(lanes.begin(), lanes.end(),
        [](const auto& lane) { return lane->isPriorityLane(); });
    return it != lanes.end() ? it->get() : nullptr;
}

bool IntersectionController::isFreeLane(LaneId id) const {
    return id == LaneId::AL3_FREELANE ||
           id == LaneId::BL3_FREELANE ||
           id == LaneId::CL3_FREELANE ||
           id == LaneId::DL3_FREELANE;
}

void IntersectionController::checkWaitTimes() {
    for (const auto& lane : lanes) {
        if (lane->getQueueSize() > 0 && !isFreeLane(lane->getId())) {
            // Process lanes that have been waiting too long
            if (lane->getQueueSize() >= 8) {
                auto vehicle = lane->removeVehicle();
                if (vehicle) {
                    vehiclesProcessedInState++;
                    totalVehiclesProcessed++;
                }
            }
        }
    }
}

std::vector<IntersectionController::LaneStatus> IntersectionController::getLaneStatuses() const {
    std::vector<LaneStatus> statuses;

    for (const auto& lane : lanes) {
        LaneStatus status;
        status.id = lane->getId();
        status.queueSize = lane->getQueueSize();
        status.isActive = isPriorityMode ? lane->isPriorityLane() :
                         !isFreeLane(lane->getId());
        status.priority = calculateLanePriority(*lane);

        statuses.push_back(status);
    }

    return statuses;
}

IntersectionController::ProcessingStats IntersectionController::getStats() const {
    ProcessingStats stats;
    stats.totalVehiclesProcessed = totalVehiclesProcessed;
    stats.isPriorityMode = isPriorityMode;

    // Fill current queue sizes
    size_t laneIndex = 0;
    for (const auto& lane : lanes) {
        stats.currentQueueSizes[laneIndex++] = lane->getQueueSize();
    }

    return stats;
}