#include "traffic_light.h"

TrafficLight::TrafficLight(SDL_Renderer* renderer, float x, float y, bool isHorizontal)
    : m_renderer(renderer)
    , m_currentState(LightState::RED)
    , m_stateTimer(0.0f)
    , m_position{x, y}
    , m_isHorizontal(isHorizontal) {
}

void TrafficLight::update(float deltaTime) {
    m_stateTimer += deltaTime;

    // State transitions
    switch(m_currentState) {
        case LightState::GREEN:
            if (m_stateTimer >= GREEN_DURATION) {
                setState(LightState::YELLOW);
            }
            break;

        case LightState::YELLOW:
            if (m_stateTimer >= YELLOW_DURATION) {
                setState(LightState::RED);
            }
            break;

        case LightState::RED:
            if (m_stateTimer >= RED_DURATION) {
                setState(LightState::GREEN);
            }
            break;
    }
}

void TrafficLight::setState(LightState newState) {
    m_currentState = newState;
    m_stateTimer = 0.0f;
}

void TrafficLight::render() const {
    const float LIGHT_SPACING = 40.0f;
    const float LIGHT_SIZE = 20.0f;

    // Base positions
    float xPos = m_position[0];
    float yPos = m_position[1];

    // Colors for each light state
    SDL_FColor redColor = {0.2f, 0.0f, 0.0f, 1.0f};
    SDL_FColor yellowColor = {0.2f, 0.2f, 0.0f, 1.0f};
    SDL_FColor greenColor = {0.0f, 0.2f, 0.0f, 1.0f};

    // Activate current light
    switch(m_currentState) {
        case LightState::RED:
            redColor.r = 1.0f;
            break;
        case LightState::YELLOW:
            yellowColor.g = 1.0f;
            break;
        case LightState::GREEN:
            greenColor.g = 1.0f;
            break;
    }

    if (m_isHorizontal) {
        // Horizontal layout
        renderLight(xPos, yPos, redColor);
        renderLight(xPos + LIGHT_SPACING, yPos, yellowColor);
        renderLight(xPos + LIGHT_SPACING * 2, yPos, greenColor);
    } else {
        // Vertical layout
        renderLight(xPos, yPos, redColor);
        renderLight(xPos, yPos + LIGHT_SPACING, yellowColor);
        renderLight(xPos, yPos + LIGHT_SPACING * 2, greenColor);
    }
}

void TrafficLight::renderLight(float x, float y, SDL_FColor color) const {
    const float LIGHT_SIZE = 20.0f;

    // Light background (black circle)
    SDL_Vertex bgVerts[32];  // More vertices for smoother circle
    SDL_FColor bgColor = {0.0f, 0.0f, 0.0f, 1.0f};

    // Create circle vertices
    for (int i = 0; i < 30; i++) {
        float angle = (i / 30.0f) * 2 * 3.14159f;
        bgVerts[i].position.x = x + cos(angle) * LIGHT_SIZE;
        bgVerts[i].position.y = y + sin(angle) * LIGHT_SIZE;
        bgVerts[i].color = bgColor;
    }

    // Center vertex for triangle fan
    bgVerts[30].position.x = x;
    bgVerts[30].position.y = y;
    bgVerts[30].color = bgColor;

    // Closing vertex
    bgVerts[31] = bgVerts[0];

    SDL_RenderGeometry(m_renderer, nullptr, bgVerts, 32, nullptr, 0);

    // Colored light (smaller circle)
    SDL_Vertex lightVerts[32];
    const float INNER_SIZE = LIGHT_SIZE * 0.7f;

    for (int i = 0; i < 30; i++) {
        float angle = (i / 30.0f) * 2 * 3.14159f;
        lightVerts[i].position.x = x + cos(angle) * INNER_SIZE;
        lightVerts[i].position.y = y + sin(angle) * INNER_SIZE;
        lightVerts[i].color = color;
    }

    lightVerts[30].position.x = x;
    lightVerts[30].position.y = y;
    lightVerts[30].color = color;
    lightVerts[31] = lightVerts[0];

    SDL_RenderGeometry(m_renderer, nullptr, lightVerts, 32, nullptr, 0);
}