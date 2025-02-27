
// src/core/Lane.cpp
#include "core/Lane.h"
#include "core/Constants.h"
#include <filesystem>
#include <algorithm>

Lane::Lane(LaneId id, bool isPriority)
    : id(id), isPriority(isPriority), waitTime(0.0f)
{
    // Determine if this is a free lane (lane 3 of each road)
    isFreeLane = (
        id == LaneId::AL3_FREELANE ||
        id == LaneId::BL3_FREELANE ||
        id == LaneId::CL3_FREELANE ||
        id == LaneId::DL3_FREELANE
    );

    // Set up data file path based on lane ID
    std::string lanePrefix;
    switch(id) {
        case LaneId::AL1_INCOMING: lanePrefix = "a1"; break;
        case LaneId::AL2_PRIORITY: lanePrefix = "a2"; break;
        case LaneId::AL3_FREELANE: lanePrefix = "a3"; break;
        case LaneId::BL1_INCOMING: lanePrefix = "b1"; break;
        case LaneId::BL2_NORMAL: lanePrefix = "b2"; break;
        case LaneId::BL3_FREELANE: lanePrefix = "b3"; break;
        case LaneId::CL1_INCOMING: lanePrefix = "c1"; break;
        case LaneId::CL2_NORMAL: lanePrefix = "c2"; break;
        case LaneId::CL3_FREELANE: lanePrefix = "c3"; break;
        case LaneId::DL1_INCOMING: lanePrefix = "d1"; break;
        case LaneId::DL2_NORMAL: lanePrefix = "d2"; break;
        case LaneId::DL3_FREELANE: lanePrefix = "d3"; break;
        default: lanePrefix = "unknown";
    }
    dataFile = "data/lanes/lane_" + lanePrefix + ".txt";
}

Direction Lane::getVehicleDirection(size_t index) const {
    if (index >= vehicleQueue.getSize()) {
        return Direction::STRAIGHT;  // Default direction
    }

    return vehicleQueue.peek(index)->getDirection();
}

void Lane::addVehicle(std::shared_ptr<Vehicle> vehicle) {
    vehicleQueue.enqueue(vehicle);
}

std::shared_ptr<Vehicle> Lane::removeVehicle() {
    if (vehicleQueue.isEmpty()) return nullptr;
    return vehicleQueue.dequeue();
}

size_t Lane::getQueueSize() const {
    return vehicleQueue.getSize();
}

bool Lane::isPriorityLane() const {
    return isPriority;
}

LaneId Lane::getId() const {
    return id;
}

const std::string& Lane::getDataFile() const {
    return dataFile;
}

bool Lane::isFreeLaneType() const {
    return isFreeLane;
}

void Lane::update(float deltaTime) {
    // Update wait times for vehicles
    updateVehicleWaitTimes(deltaTime);

    // Free lanes (3rd lane of each road) process vehicles immediately
    if (isFreeLane) {
        while (!vehicleQueue.isEmpty()) {
            removeVehicle();
        }
    }
}

void Lane::updateVehicleWaitTimes(float deltaTime) {
    size_t count = vehicleQueue.getSize();
    if (count == 0) return;

    float totalWaitTime = 0.0f;
    for (size_t i = 0; i < count; i++) {
        auto vehicle = vehicleQueue.peek(i);
        vehicle->updateWaitTime(deltaTime);
        totalWaitTime += vehicle->getWaitTime();
    }

    // Update average wait time
    waitTime = totalWaitTime / static_cast<float>(count);
}

float Lane::getAverageWaitTime() const {
    return waitTime;
}

bool Lane::exceedsPriorityThreshold() const {
    return isPriority && vehicleQueue.getSize() > SimConstants::PRIORITY_THRESHOLD;
}

bool Lane::isBelowNormalThreshold() const {
    return isPriority && vehicleQueue.getSize() < SimConstants::NORMAL_THRESHOLD;
}