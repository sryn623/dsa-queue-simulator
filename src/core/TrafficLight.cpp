// src/core/TrafficLight.cpp
#include "core/TrafficLight.h"
#include <cmath>

TrafficLight::TrafficLight(LaneId lane)
    : state(LightState::RED)
    , nextState(LightState::RED)
    , transitionProgress(0.0f)
    , transitionDuration(1.0f)
    , stateTimer(0.0f)
    , isTransitioning(false)
    , currentStateDuration(15.0f)  // Default duration for normal mode
    , isPriorityMode(false)
    , isForced(false)
    , controlledLane(lane)
{
}

void TrafficLight::update(float deltaTime) {
    // Don't update if state is being forced
    if (isForced) {
        return;
    }

    stateTimer += deltaTime;

    if (isTransitioning) {
        // Handle transition animation
        transitionProgress += deltaTime / transitionDuration;
        if (transitionProgress >= 1.0f) {
            state = nextState;
            isTransitioning = false;
            transitionProgress = 0.0f;
            stateTimer = 0.0f;
            currentStateDuration = getNextStateDuration();
        }
    }
    else if (stateTimer >= currentStateDuration) {
        // Time to change state
        LightState newState = (state == LightState::RED) ? LightState::GREEN : LightState::RED;
        startTransition(newState);
    }
}

void TrafficLight::setState(LightState newState) {
    if (state != newState && !isTransitioning) {
        startTransition(newState);
    }
}

void TrafficLight::forceState(LightState newState, bool force) {
    isForced = force;
    if (force) {
        state = newState;
        nextState = newState;
        isTransitioning = false;
        transitionProgress = 0.0f;
        stateTimer = 0.0f;
    } else {
        isForced = false;
        setState(newState);
    }
}

void TrafficLight::setPriorityMode(bool enabled) {
    isPriorityMode = enabled;
    // Adjust timings when priority mode changes
    currentStateDuration = getStateDuration();
}

float TrafficLight::getStateDuration() const {
    if (isPriorityMode) {
        // In priority mode, A2 gets longer green, others longer red
        if (controlledLane == LaneId::AL2_PRIORITY) {
            return (state == LightState::GREEN) ? 30.0f : 5.0f;  // Priority lane gets more green time
        } else {
            return (state == LightState::GREEN) ? 5.0f : 30.0f;  // Others get less green time
        }
    }

    // In normal mode, more balanced durations
    return 15.0f;
}

float TrafficLight::getNextStateDuration() const {
    if (isPriorityMode) {
        if (controlledLane == LaneId::AL2_PRIORITY) {
            return (nextState == LightState::GREEN) ? 30.0f : 5.0f;
        } else {
            return (nextState == LightState::GREEN) ? 5.0f : 30.0f;
        }
    }
    return 15.0f;
}

void TrafficLight::startTransition(LightState newState) {
    nextState = newState;
    isTransitioning = true;
    transitionProgress = 0.0f;
    stateTimer = 0.0f;
}

void TrafficLight::render(SDL_Renderer* renderer, float x, float y) const {
    // Constants for rendering
    const float LIGHT_SIZE = 30.0f;
    const float HOUSING_PADDING = 5.0f;
    const float HOUSING_WIDTH = LIGHT_SIZE + (HOUSING_PADDING * 2.0f);
    const float HOUSING_HEIGHT = (LIGHT_SIZE * 2.0f) + (HOUSING_PADDING * 3.0f);

    // Draw housing (dark gray background)
    SDL_FRect housing = {
        x - HOUSING_PADDING,
        y - HOUSING_PADDING,
        HOUSING_WIDTH,
        HOUSING_HEIGHT
    };
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_RenderFillRect(renderer, &housing);

    // Draw border
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_RenderRect(renderer, &housing);

    // Calculate light positions
    float redY = y;
    float greenY = y + LIGHT_SIZE + HOUSING_PADDING;

    // Draw RED light
    SDL_FRect redLight = {x, redY, LIGHT_SIZE, LIGHT_SIZE};
    float redIntensity;
    if (state == LightState::RED) {
        redIntensity = isTransitioning ? (1.0f - transitionProgress) : 1.0f;
    } else {
        redIntensity = isTransitioning ? transitionProgress : 0.0f;
    }
    SDL_SetRenderDrawColor(renderer,
        255,  // Red always max
        static_cast<uint8_t>(50 * redIntensity),  // Slight glow
        static_cast<uint8_t>(50 * redIntensity),  // Slight glow
        255
    );
    SDL_RenderFillRect(renderer, &redLight);

    // Draw GREEN light
    SDL_FRect greenLight = {x, greenY, LIGHT_SIZE, LIGHT_SIZE};
    float greenIntensity;
    if (state == LightState::GREEN) {
        greenIntensity = isTransitioning ? (1.0f - transitionProgress) : 1.0f;
    } else {
        greenIntensity = isTransitioning ? transitionProgress : 0.0f;
    }
    SDL_SetRenderDrawColor(renderer,
        static_cast<uint8_t>(50 * greenIntensity),  // Slight glow
        255,  // Green always max
        static_cast<uint8_t>(50 * greenIntensity),  // Slight glow
        255
    );
    SDL_RenderFillRect(renderer, &greenLight);

    // Draw light borders
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_RenderRect(renderer, &redLight);
    SDL_RenderRect(renderer, &greenLight);

    // Add priority indicator if this is the priority lane
    if (controlledLane == LaneId::AL2_PRIORITY) {
        SDL_FRect priorityIndicator = {
            x + LIGHT_SIZE + 5.0f,
            y,
            10.0f,
            10.0f
        };

        // Pulsing effect for priority mode
        uint8_t alpha = isPriorityMode ?
            static_cast<uint8_t>(128 + 127 * sinf(SDL_GetTicks() / 500.0f)) : 100;

        SDL_SetRenderDrawColor(renderer, 255, 165, 0, alpha);
        SDL_RenderFillRect(renderer, &priorityIndicator);
    }
}