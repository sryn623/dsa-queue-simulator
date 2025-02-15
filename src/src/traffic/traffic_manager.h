#pragma once
#include "traffic_queue.h"
#include <unordered_map>
#include <vector>

class TrafficManager {
public:
    TrafficManager();

    // Queue management
    void addVehicle(Vehicle* vehicle, LaneId lane);
    void update(float deltaTime);

    // Lane processing
    void processLanes();
    bool shouldProcessPriorityLane() const;
    bool isInPriorityMode() const { return m_inPriorityMode; }

private:
    std::unordered_map<LaneId, TrafficQueue> m_queues;
    bool m_inPriorityMode = false;

    // Constants for priority handling
    static constexpr int PRIORITY_THRESHOLD_HIGH = 10;
    static constexpr int PRIORITY_THRESHOLD_LOW = 5;
    static constexpr float PROCESSING_TIME = 2.0f; // Time to process one vehicle

    // Helper methods
    void checkPriorityConditions();
    void processNormalCondition();
    void processPriorityCondition();
    float calculateAverageQueueLength() const;
};