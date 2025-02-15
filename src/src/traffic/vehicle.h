#pragma once
#include <SDL3/SDL.h>
#include <cstdint>

enum class VehicleStatus : std::uint8_t {
  WAITING,
  MOVING,
  TURNING,
  STOPPED,
  TURNING_LEFT,
  TURNING_RIGHT,
  EXISTING,
};

enum class LaneId : std::uint8_t {
  // lane 1
  AL1_INCOMING,
  AL2_PRIORITY,
  AL3_FREELANE,

  // lane 2
  BL1_INCOMING,
  BL2_PRIORITY,
  BL3_FREELANE,

  // lane 3
  CL1_INCOMING,
  CL2_PRIORITY,
  CL3_FREELANE,

  // lane 4
  DL1_INCOMING,
  DL2_PRIORITY,
  DL3_FREELANE,
};

enum class TurnBehaviour : std::uint8_t {
  TURNING_LEFT,
  TURNING_RIGHT,
  STRAIGHT,
};

enum class Direction : std::uint8_t {
  NORTH,
  SOUTH,
  EAST,
  WEST,
};

struct Vector2D {
  float x, y;
  Vector2D(float x_ = 0.0F, float y_ = 0.0F) : x(x_), y(y_) {}
};

class Vehicle {
public:
  Vehicle(SDL_Renderer *renderer, int vehicle_id, LaneId startLane,
          Vector2D startPos, Direction facing);
  ~Vehicle();

  void update(float deltaTime);
  void render() const; // Note: this is const
  void changeLane(LaneId newLane);
  void setTurnDirection(TurnBehaviour turn);
  void checkPriority();

  // Getter methods
  VehicleStatus getStatus() const { return m_status; }
  LaneId getCurrentLaneId() const { return m_currentLane; }
  Direction getFacingDirection() const { return m_facing; }
  Vector2D getPosition() const { return m_position; }
  float getWaitTime() const { return m_waitTime; }
  void setPosition(Vector2D newPos) { m_position = newPos; }

private:
  SDL_Renderer *m_renderer;
  int m_id;
  Vector2D m_position;
  Vector2D m_velocity;

  VehicleStatus m_status;
  LaneId m_currentLane;
  LaneId m_targetLane;
  Direction m_facing;
  TurnBehaviour m_turnIntent;

  float m_waitTime;
  bool m_isPriority;

  void updatePosition(float deltaTime);
  void updateState();
  bool checkCollision() const;
  bool needsTurnLeft(LaneId oldLane, LaneId newLane) const;
};