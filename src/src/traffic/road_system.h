#pragma once
#include "lane.h"
#include "traffic_manager.h"
#include <map>
#include <memory>

class RoadSystem {
public:
    RoadSystem(SDL_Renderer* renderer);
    void render() const;
    void update(float deltaTime);

    Lane* getLane(LaneId id);
    std::vector<Lane*> getPriorityLanes();
    void addVehicle(Vehicle* vehicle, LaneId lane);

private:
    SDL_Renderer* m_renderer;
    std::map<LaneId, std::unique_ptr<Lane>> m_lanes;
    TrafficManager m_trafficManager;

    void initializeLanes();
    Vector2D calculateLaneStart(LaneId id) const;
    Vector2D calculateLaneEnd(LaneId id) const;

    static constexpr int ROAD_WIDTH = 180;  // Total width for 3 lanes
    static constexpr int LANE_WIDTH = 60;   // Width of each lane
    static constexpr int INTERSECTION_SIZE = 180;
};