#pragma once
#include "../traffic/vehicle.h"
#include <SDL3/SDL.h>
#include <random>

class TrafficGenerator {
public:
  TrafficGenerator(SDL_Renderer *renderer);
  Vehicle *generateVehicle();

private:
  SDL_Renderer *m_renderer;
  std::mt19937 m_rng;
  int m_vehicleCount = 0;
};