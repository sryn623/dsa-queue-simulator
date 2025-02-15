#include "traffic_queue.h"

void TrafficQueue::enqueue(Vehicle* vehicle) {
    if (vehicle) {
        m_vehicles.push_back(vehicle);
    }
}

Vehicle* TrafficQueue::dequeue() {
    if (m_vehicles.empty()) {
        return nullptr;
    }

    Vehicle* vehicle = m_vehicles.front();
    m_vehicles.pop_front();
    return vehicle;
}

Vehicle* TrafficQueue::front() const {
    return m_vehicles.empty() ? nullptr : m_vehicles.front();
}

void TrafficQueue::update(float deltaTime) {
    if (!m_vehicles.empty()) {
        m_processTimer += deltaTime;
    } else {
        m_processTimer = 0.0f;
    }
}