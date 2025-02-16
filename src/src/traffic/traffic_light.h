#pragma once
#include <SDL3/SDL.h>
#include <cstdint>

enum class LightState : std::uint8_t {
    RED,
    GREEN,
    YELLOW
};

class TrafficLight {
public:
    TrafficLight(SDL_Renderer* renderer, float x, float y, bool isHorizontal);
    ~TrafficLight() = default;

    void update(float deltaTime);
    void render() const;
    void setState(LightState newState);

    LightState getState() const { return m_currentState; }
    bool isGreen() const { return m_currentState == LightState::GREEN; }
    float getTimeInState() const { return m_stateTimer; }

private:
    SDL_Renderer* m_renderer;
    LightState m_currentState;
    float m_stateTimer;
    float m_position[2];  // x, y
    bool m_isHorizontal;

    // Light timing constants
    static constexpr float GREEN_DURATION = 10.0f;   // 10 seconds green
    static constexpr float YELLOW_DURATION = 3.0f;   // 3 seconds yellow
    static constexpr float RED_DURATION = 13.0f;     // Matches other light's cycle

    void renderLight(float x, float y, SDL_FColor color) const;
};