#pragma once
#include "vehicle.h"
#include <deque>

class TrafficQueue {
public:
    void enqueue(Vehicle* vehicle);
    Vehicle* dequeue();
    Vehicle* front() const;
    bool empty() const { return m_vehicles.empty(); }
    size_t size() const { return m_vehicles.size(); }

    // Queue management
    const std::deque<Vehicle*>& getVehicles() const { return m_vehicles; }

    // Priority handling
    bool isPriorityQueue() const { return m_isPriority; }
    void setPriorityStatus(bool status) { m_isPriority = status; }

    // Queue processing
    void update(float deltaTime);
    bool readyToProcess() const { return m_processTimer >= PROCESS_TIME_REQUIRED; }

private:
    std::deque<Vehicle*> m_vehicles;
    bool m_isPriority = false;
    float m_processTimer = 0.0f;

    static constexpr float PROCESS_TIME_REQUIRED = 2.0f;
};