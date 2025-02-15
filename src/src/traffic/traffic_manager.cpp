#include "traffic_manager.h"

TrafficManager::TrafficManager() {
    // Initialize queues for all lanes
    for (int i = 0; i < 12; ++i) { // 12 lanes total (3 per road * 4 roads)
        LaneId laneId = static_cast<LaneId>(i);
        m_queues[laneId] = TrafficQueue();

        // Set priority status for AL2, BL2, CL2, DL2
        bool isPriority = (laneId == LaneId::AL2_PRIORITY ||
                         laneId == LaneId::BL2_PRIORITY ||
                         laneId == LaneId::CL2_PRIORITY ||
                         laneId == LaneId::DL2_PRIORITY);
        m_queues[laneId].setPriorityStatus(isPriority);
    }
}

void TrafficManager::addVehicle(Vehicle* vehicle, LaneId lane) {
    if (vehicle && m_queues.count(lane) > 0) {
        m_queues[lane].enqueue(vehicle);
        checkPriorityConditions();
    }
}

void TrafficManager::update(float deltaTime) {
    // Update all queues
    for (auto& [lane, queue] : m_queues) {
        queue.update(deltaTime);
    }

    // Process vehicles based on conditions
    processLanes();
}

void TrafficManager::processLanes() {
    if (shouldProcessPriorityLane()) {
        processPriorityCondition();
    } else {
        processNormalCondition();
    }
}

bool TrafficManager::shouldProcessPriorityLane() const {
    // Check if any priority lane has more than threshold vehicles
    for (const auto& [lane, queue] : m_queues) {
        if (queue.isPriorityQueue() && queue.size() >= PRIORITY_THRESHOLD_HIGH) {
            return true;
        }
    }
    return false;
}

void TrafficManager::checkPriorityConditions() {
    bool shouldBePriority = false;

    // Check all priority lanes
    for (const auto& [lane, queue] : m_queues) {
        if (queue.isPriorityQueue()) {
            if (queue.size() >= PRIORITY_THRESHOLD_HIGH) {
                shouldBePriority = true;
                break;
            }
        }
    }

    // Check if we should exit priority mode
    if (m_inPriorityMode) {
        bool allPriorityLowEnough = true;
        for (const auto& [lane, queue] : m_queues) {
            if (queue.isPriorityQueue() && queue.size() >= PRIORITY_THRESHOLD_LOW) {
                allPriorityLowEnough = false;
                break;
            }
        }
        if (allPriorityLowEnough) {
            shouldBePriority = false;
        }
    }

    m_inPriorityMode = shouldBePriority;
}

void TrafficManager::processNormalCondition() {
    float avgLength = calculateAverageQueueLength();
    if (avgLength <= 0) return;

    // Process normal lanes based on average queue length
    for (auto& [lane, queue] : m_queues) {
        if (!queue.isPriorityQueue() && !queue.empty()) {
            Vehicle* vehicle = queue.dequeue();
            if (vehicle) {
                // Here you would implement the logic to move the vehicle
                // through the intersection
            }
        }
    }
}

void TrafficManager::processPriorityCondition() {
    // Process only priority lanes that are above threshold
    for (auto& [lane, queue] : m_queues) {
        if (queue.isPriorityQueue() && queue.size() >= PRIORITY_THRESHOLD_LOW) {
            while (!queue.empty() && queue.size() >= PRIORITY_THRESHOLD_LOW) {
                Vehicle* vehicle = queue.dequeue();
                if (vehicle) {
                    // Here you would implement the logic to move the vehicle
                    // through the intersection
                }
            }
        }
    }
}

float TrafficManager::calculateAverageQueueLength() const {
    float totalLength = 0;
    int count = 0;

    for (const auto& [lane, queue] : m_queues) {
        if (!queue.isPriorityQueue()) {
            totalLength += queue.size();
            count++;
        }
    }

    return count > 0 ? totalLength / count : 0;
}