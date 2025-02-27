// src/main.cp// src/main.cpp with debug statements
#include <SDL3/SDL.h>
#include "managers/TrafficManager.h"
#include "managers/FileHandler.h"
#include "visualization/Renderer.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

// Debug helper functions
void WriteDebugLog(const std::string& message) {
    // Write to console
    std::cout << "[DEBUG] " << message << std::endl;

    // Also write to file for persistence
    std::ofstream logFile("debug_log.txt", std::ios::app);
    if (logFile.is_open()) {
        logFile << "[DEBUG] " << message << std::endl;
        logFile.close();
    }
}

class Simulator {
private:
    TrafficManager trafficManager;
    Renderer renderer;
    bool running;
    bool debugMode;

    void processInput() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_EVENT_QUIT:
                    running = false;
                    break;
                case SDL_EVENT_KEY_DOWN:
                    handleKeyDown(event.key.scancode);
                    break;
            }
        }
    }

    void handleKeyDown(SDL_Scancode scancode) {
        WriteDebugLog("Key pressed: " + std::to_string(static_cast<int>(scancode)));

        switch(scancode) {
            case SDL_SCANCODE_ESCAPE:
                running = false;
                break;
            case SDL_SCANCODE_D:
                debugMode = !debugMode;
                renderer.setDebugMode(debugMode);
                break;
            case SDL_SCANCODE_G:
                renderer.toggleGridDisplay();
                break;
            case SDL_SCANCODE_P:
                // Manual priority mode toggle for testing
                WriteDebugLog("Manual priority mode toggle requested");
                break;
        }
    }

    void update(float deltaTime) {
        try {
            WriteDebugLog("Updating with deltaTime: " + std::to_string(deltaTime));
            trafficManager.update(deltaTime);
        } catch (const std::exception& e) {
            WriteDebugLog("Exception in update: " + std::string(e.what()));
        } catch (...) {
            WriteDebugLog("Unknown exception in update");
        }
    }

    void render() {
        try {
            WriteDebugLog("Rendering frame");
            renderer.render(trafficManager);
        } catch (const std::exception& e) {
            WriteDebugLog("Exception in render: " + std::string(e.what()));
        } catch (...) {
            WriteDebugLog("Unknown exception in render");
        }
    }

public:
    Simulator() : running(false), debugMode(false) {
        WriteDebugLog("Simulator constructor called");
    }

    bool initialize() {
        WriteDebugLog("Simulator initialization started");

        try {
            // Create the data directory structure if it doesn't exist
            WriteDebugLog("Creating data directories");
            std::filesystem::path dataDir = std::filesystem::current_path() / "data" / "lanes";
            if (!std::filesystem::exists(dataDir)) {
                WriteDebugLog("Creating directory: " + dataDir.string());
                std::filesystem::create_directories(dataDir);
            }

            // Initialize renderer
            WriteDebugLog("Initializing renderer");
            if (!renderer.initialize()) {
                WriteDebugLog("Failed to initialize renderer");
                return false;
            }

            // Clear any existing vehicle data
            WriteDebugLog("Creating FileHandler");
            FileHandler fileHandler;

            WriteDebugLog("Clearing lane files");
            fileHandler.clearLaneFiles();

            // Set initial state
            running = true;
            WriteDebugLog("Setting debug mode");
            renderer.setDebugMode(debugMode);

            WriteDebugLog("Simulator initialization completed successfully");
            return true;
        }
        catch (const std::exception& e) {
            WriteDebugLog("Exception during initialization: " + std::string(e.what()));
            return false;
        }
        catch (...) {
            WriteDebugLog("Unknown exception during initialization");
            return false;
        }
    }

    void run() {
        WriteDebugLog("Simulator run started");
        auto lastUpdateTime = std::chrono::high_resolution_clock::now();

        try {
            while (running) {
                auto currentTime = std::chrono::high_resolution_clock::now();
                float deltaTime = std::chrono::duration<float>(currentTime - lastUpdateTime).count();
                lastUpdateTime = currentTime;

                processInput();
                update(deltaTime);
                render();

                // Cap frame rate at ~60 FPS
                if (deltaTime < SimConstants::UPDATE_INTERVAL) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>((SimConstants::UPDATE_INTERVAL - deltaTime) * 1000)));
                }

                // Only log occasionally to avoid log flood
                static float statTimer = 0.0f;
                statTimer += deltaTime;
                if (statTimer >= 1.0f) {
                    WriteDebugLog("FPS: " + std::to_string(1.0f / deltaTime));
                    statTimer = 0.0f;
                }
            }
        }
        catch (const std::exception& e) {
            WriteDebugLog("Exception in main loop: " + std::string(e.what()));
        }
        catch (...) {
            WriteDebugLog("Unknown exception in main loop");
        }

        WriteDebugLog("Simulator run completed");
    }

    void cleanup() {
        WriteDebugLog("Simulator cleanup started");
        renderer.cleanup();
        WriteDebugLog("Simulator cleanup completed");
    }
};

int main(int argc, char* argv[]) {
    (void)argc; // Suppress unused parameter warning
    (void)argv; // Suppress unused parameter warning

    // Clear previous log file
    std::ofstream logFile("debug_log.txt", std::ios::trunc);
    if (logFile.is_open()) {
        logFile << "[DEBUG] Starting simulator at " << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
        logFile.close();
    }

    WriteDebugLog("Program started");

    try {
        WriteDebugLog("Creating simulator");
        Simulator simulator;

        WriteDebugLog("Initializing simulator");
        if (!simulator.initialize()) {
            WriteDebugLog("Failed to initialize simulator");
            return 1;
        }

        WriteDebugLog("Starting simulator run loop");
        simulator.run();

        WriteDebugLog("Cleaning up simulator");
        simulator.cleanup();
    }
    catch (const std::exception& e) {
        WriteDebugLog("Exception in main: " + std::string(e.what()));
        return 1;
    }
    catch (...) {
        WriteDebugLog("Unknown exception in main");
        return 1;
    }

    WriteDebugLog("Program ending normally");
    return 0;
}