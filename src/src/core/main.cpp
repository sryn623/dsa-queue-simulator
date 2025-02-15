#include "main.h"
#include <SDL3/SDL.h>

namespace GameVariable {
constexpr int WindowHeight = 720;
constexpr int WindowLength = 1280;
constexpr float SpawnInterval = 1000.0f; // Spawn every 1 second for testing
} // namespace GameVariable

App::App()
    : m_window("Traffic Simulator", GameVariable::WindowLength,
               GameVariable::WindowHeight)
    , m_generator(m_window.renderer())
    , m_roadSystem(m_window.renderer())
    , m_lastSpawnTime(0)
    , m_running(true) {

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0) {
        throw std::runtime_error(SDL_GetError());
    }
}

App::~App() {
    for (auto* vehicle : m_vehicles) {
        delete vehicle;
    }
    SDL_Quit();
}

void App::process_event() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_EVENT_QUIT) {
            m_running = false;
        }
    }
}

void App::update() {
    float deltaTime = 1.0f / 60.0f;

    // Spawn vehicles
    Uint64 currentTime = SDL_GetTicks();
    if (currentTime - m_lastSpawnTime >= GameVariable::SpawnInterval) {
        // Spawn a vehicle in each road
        for (int i = 0; i < 4; i++) {  // Spawn one vehicle for each road direction
            Vehicle* vehicle = m_generator.generateVehicle();
            if (vehicle) {
                m_vehicles.push_back(vehicle);
                m_roadSystem.addVehicle(vehicle, vehicle->getCurrentLaneId());
            }
        }
        m_lastSpawnTime = currentTime;
    }

    // Update road system
    m_roadSystem.update(deltaTime);

    // Update and clean up vehicles
    auto it = m_vehicles.begin();
    while (it != m_vehicles.end()) {
        Vehicle* vehicle = *it;

        // Calculate screen bounds with some margin
        bool outOfBounds = false;
        Vector2D pos = vehicle->getPosition();

        switch(vehicle->getFacingDirection()) {
            case Direction::NORTH:
                outOfBounds = pos.y < -100;
                break;
            case Direction::SOUTH:
                outOfBounds = pos.y > GameVariable::WindowHeight + 100;
                break;
            case Direction::EAST:
                outOfBounds = pos.x > GameVariable::WindowLength + 100;
                break;
            case Direction::WEST:
                outOfBounds = pos.x < -100;
                break;
        }

        if (outOfBounds) {
            delete vehicle;
            it = m_vehicles.erase(it);
        } else {
            ++it;
        }
    }
}

void App::render() {
    m_window.clear();

    // Render road system (includes lanes)
    m_roadSystem.render();

    // Render vehicles
    for (const auto* vehicle : m_vehicles) {
        vehicle->render();
    }

    m_window.present();
}

void App::run() {
    while (m_running) {
        process_event();
        update();
        render();
        SDL_Delay(16); // Cap to roughly 60 FPS
    }
}

int main(int argc, const char* argv[]) {
    try {
        App app;
        app.run();
    } catch (const std::exception& e) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error: %s\n", e.what());
    }

    SDL_Quit();
    return 0;
}