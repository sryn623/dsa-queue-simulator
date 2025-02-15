#pragma once
#include <SDL3/SDL.h>
#include "vehicle.h"
#include "traffic_queue.h"
#include <vector>

class Lane {
public:
    Lane(SDL_Renderer* renderer, LaneId id, Vector2D start, Vector2D end, bool isPriority = false);
    void render() const;
    void update(float deltaTime);

    // Lane management
    void addVehicle(Vehicle* vehicle);
    bool isPriorityLane() const { return m_isPriority; }
    int getVehicleCount() const { return static_cast<int>(m_vehicles.size()); }
    LaneId getId() const { return m_id; }

private:
    void positionVehicleInQueue(Vehicle* vehicle, size_t queuePosition = 0);

    SDL_Renderer* m_renderer;
    LaneId m_id;
    Vector2D m_startPos;
    Vector2D m_endPos;
    bool m_isPriority;
    std::vector<Vehicle*> m_vehicles;
};