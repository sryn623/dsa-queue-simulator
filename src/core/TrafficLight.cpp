// FILE: src/core/TrafficLight.cpp
#include "core/TrafficLight.h"
#include "utils/DebugLogger.h"
#include <sstream>
#include <cmath>
#include <SDL3/SDL.h>
#include "core/Constants.h"

TrafficLight::TrafficLight()
    : currentState(State::ALL_RED),
      nextState(State::A_GREEN),
      lastStateChangeTime(SDL_GetTicks()),
      isPriorityMode(false),
      shouldResumeNormalMode(false),
      forceAGreen(false),
      priorityModeStartTime(0) {

    DebugLogger::log("TrafficLight initialized");
}

TrafficLight::~TrafficLight() {
    DebugLogger::log("TrafficLight destroyed");
}

void TrafficLight::update(const std::vector<Lane*>& lanes) {
    uint32_t currentTime = SDL_GetTicks();
    uint32_t elapsedTime = currentTime - lastStateChangeTime;

    // CRITICAL: Find priority lane A2 directly
    Lane* al2Lane = nullptr;
    for (auto* lane : lanes) {
        if (lane->getLaneId() == 'A' && lane->getLaneNumber() == 2) {
            al2Lane = lane;
            break;
        }
    }

    // CRITICAL FIX: Direct priority detection and override
    if (al2Lane) {
        int vehicleCount = al2Lane->getVehicleCount();

        // CRITICAL: Force immediate transition to A_GREEN when vehicle count exceeds threshold
        if (vehicleCount > Constants::PRIORITY_THRESHOLD_HIGH) {
            if (!isPriorityMode) {
                isPriorityMode = true;
                forceAGreen = true;
                priorityModeStartTime = currentTime;

                std::ostringstream oss;
                oss << "!!! PRIORITY MODE ACTIVATED: A2 has " << vehicleCount << " vehicles (>10)";
                DebugLogger::log(oss.str(), DebugLogger::LogLevel::ERROR);

                // Force immediate transition to A_GREEN
                if (currentState != State::A_GREEN) {
                    // If not in ALL_RED, go to ALL_RED first
                    if (currentState != State::ALL_RED) {
                        currentState = State::ALL_RED;
                        nextState = State::A_GREEN;
                        lastStateChangeTime = currentTime;
                    } else {
                        // If already in ALL_RED, go directly to A_GREEN
                        currentState = State::A_GREEN;
                        nextState = State::ALL_RED;
                        lastStateChangeTime = currentTime;
                    }
                }
            }
        }
        // Exit priority mode when count drops below threshold
        else if (isPriorityMode && vehicleCount < Constants::PRIORITY_THRESHOLD_LOW) {
            isPriorityMode = false;
            forceAGreen = false;

            std::ostringstream oss;
            oss << "!!! PRIORITY MODE DEACTIVATED: A2 now has " << vehicleCount << " vehicles (<5)";
            DebugLogger::log(oss.str(), DebugLogger::LogLevel::ERROR);
        }

        // CRITICAL: Force-log the priority state for debugging
        if (isPriorityMode && currentTime - priorityModeStartTime > 5000) {
            std::ostringstream oss;
            oss << "Priority mode active: A2 has " << vehicleCount << " vehicles, light state: "
                << (currentState == State::A_GREEN ? "A_GREEN" :
                   (currentState == State::ALL_RED ? "ALL_RED" : "OTHER"));
            DebugLogger::log(oss.str(), DebugLogger::LogLevel::ERROR);
            priorityModeStartTime = currentTime;
        }
    }

    // CRITICAL FIX: If in priority mode, force the light to stay on A_GREEN longer
    if (isPriorityMode && forceAGreen) {
        if (currentState != State::A_GREEN) {
            if (elapsedTime >= allRedDuration) {
                // After ALL_RED, go back to A_GREEN in priority mode
                currentState = State::A_GREEN;
                nextState = State::ALL_RED;
                lastStateChangeTime = currentTime;

                DebugLogger::log("PRIORITY MODE: Forcing A_GREEN", DebugLogger::LogLevel::ERROR);
            }
        } else {
            // Extend the green duration in priority mode
            int priorityGreenDuration = 6000; // 6 seconds of green in priority mode

            if (elapsedTime >= priorityGreenDuration) {
                // Go to ALL_RED briefly before returning to A_GREEN
                currentState = State::ALL_RED;
                nextState = State::A_GREEN;
                lastStateChangeTime = currentTime;

                DebugLogger::log("PRIORITY MODE: Brief ALL_RED before returning to A_GREEN");
            }
        }

        return; // Skip normal light cycling in priority mode
    }

    // NORMAL MODE - Calculate appropriate duration
    int stateDuration;
    if (currentState == State::ALL_RED) {
        stateDuration = allRedDuration; // 2 seconds for ALL_RED
    } else {
        // Calculate average using lane counts
        float averageVehicleCount = calculateAverageVehicleCount(lanes);

        // Set duration using formula: Total time = |V| * t (2 seconds per vehicle)
        stateDuration = static_cast<int>(averageVehicleCount * 2000);

        // Apply minimum and maximum limits for reasonable times
        if (stateDuration < 3000) stateDuration = 3000; // Min 3 seconds
        if (stateDuration > 15000) stateDuration = 15000; // Max 15 seconds

        // Log the calculation
        std::ostringstream oss;
        oss << "Traffic light timing: |V| = " << averageVehicleCount
            << ", Duration = " << stateDuration / 1000.0f << " seconds";
        DebugLogger::log(oss.str());
    }

    // State transition in normal mode
    if (elapsedTime >= stateDuration) {
        // Change to next state
        currentState = nextState;

        // Log the state change clearly
        std::string stateStr;
        switch (currentState) {
            case State::ALL_RED: stateStr = "ALL_RED"; break;
            case State::A_GREEN: stateStr = "A_GREEN"; break;
            case State::B_GREEN: stateStr = "B_GREEN"; break;
            case State::C_GREEN: stateStr = "C_GREEN"; break;
            case State::D_GREEN: stateStr = "D_GREEN"; break;
        }

        DebugLogger::log("Traffic light changed to: " + stateStr);

        // Normal rotation pattern: ALL_RED → A → ALL_RED → B → ALL_RED → C → ALL_RED → D → ...
        if (currentState == State::ALL_RED) {
            // Cycle through green states
            switch (nextState) {
                case State::A_GREEN: nextState = State::B_GREEN; break;
                case State::B_GREEN: nextState = State::C_GREEN; break;
                case State::C_GREEN: nextState = State::D_GREEN; break;
                case State::D_GREEN: nextState = State::A_GREEN; break;
                default: nextState = State::A_GREEN; break;
            }
        } else {
            // Always go to ALL_RED after any green state
            nextState = State::ALL_RED;
        }

        lastStateChangeTime = currentTime;
    }
}

float TrafficLight::calculateAverageVehicleCount(const std::vector<Lane*>& lanes) {
    int normalLaneCount = 0;
    int totalVehicleCount = 0;

    for (auto* lane : lanes) {
        // Only count lane 2 (normal lanes)
        // In priority mode, exclude the priority lane (A2) from calculation
        if (lane->getLaneNumber() == 2 &&
            !(isPriorityMode && lane->getLaneId() == 'A')) {
            normalLaneCount++;
            totalVehicleCount += lane->getVehicleCount();
        }
    }

    // Calculate average: |V| = (1/n) * Σ|Li|
    float average = (normalLaneCount > 0) ?
        static_cast<float>(totalVehicleCount) / normalLaneCount : 0.0f;

    // Return at least 1 to ensure some duration
    return std::max(1.0f, average);
}

void TrafficLight::setNextState(State state) {
    nextState = state;
}

bool TrafficLight::isGreen(char lane) const {
    // CRITICAL: Explicitly check the state machine state
    switch (currentState) {
        case State::A_GREEN: return lane == 'A';
        case State::B_GREEN: return lane == 'B';
        case State::C_GREEN: return lane == 'C';
        case State::D_GREEN: return lane == 'D';
        case State::ALL_RED: return false;
        default: return false;
    }
}

void TrafficLight::render(SDL_Renderer* renderer) {
    // Draw traffic light control box in the corner
    int boxX = 10;
    int boxY = 10;
    int boxWidth = 120;
    int boxHeight = 140;

    // Draw main control box with 3D effect
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255); // Dark gray
    SDL_FRect controlBox = {
        static_cast<float>(boxX),
        static_cast<float>(boxY),
        static_cast<float>(boxWidth),
        static_cast<float>(boxHeight)
    };
    SDL_RenderFillRect(renderer, &controlBox);

    // Top highlight
    SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255); // Light gray
    SDL_FRect topHighlight = {
        static_cast<float>(boxX),
        static_cast<float>(boxY),
        static_cast<float>(boxWidth),
        4.0f
    };
    SDL_RenderFillRect(renderer, &topHighlight);

    // Left highlight
    SDL_FRect leftHighlight = {
        static_cast<float>(boxX),
        static_cast<float>(boxY),
        4.0f,
        static_cast<float>(boxHeight)
    };
    SDL_RenderFillRect(renderer, &leftHighlight);

    // Bottom shadow
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); // Very dark gray
    SDL_FRect bottomShadow = {
        static_cast<float>(boxX),
        static_cast<float>(boxY + boxHeight - 4),
        static_cast<float>(boxWidth),
        4.0f
    };
    SDL_RenderFillRect(renderer, &bottomShadow);

    // Right shadow
    SDL_FRect rightShadow = {
        static_cast<float>(boxX + boxWidth - 4),
        static_cast<float>(boxY),
        4.0f,
        static_cast<float>(boxHeight)
    };
    SDL_RenderFillRect(renderer, &rightShadow);

    // Title bar
    SDL_SetRenderDrawColor(renderer, 50, 50, 150, 255); // Dark blue
    SDL_FRect titleBar = {
        static_cast<float>(boxX + 5),
        static_cast<float>(boxY + 5),
        static_cast<float>(boxWidth - 10),
        20.0f
    };
    SDL_RenderFillRect(renderer, &titleBar);

    // Title text using simplified drawing (T L)
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // White
    // Draw "T"
    SDL_RenderLine(renderer, boxX + 20, boxY + 10, boxX + 35, boxY + 10);
    SDL_RenderLine(renderer, boxX + 27, boxY + 10, boxX + 27, boxY + 20);
    // Draw "L"
    SDL_RenderLine(renderer, boxX + 40, boxY + 10, boxX + 40, boxY + 20);
    SDL_RenderLine(renderer, boxX + 40, boxY + 20, boxX + 55, boxY + 20);

    // Draw light status indicators for each road
    const int LIGHT_SIZE = 20;
    const int LABEL_HEIGHT = 20;
    const int SPACING = 5;
    int startY = boxY + 30;

    // Road A light
    bool isARed = !isGreen('A');
    drawRoadLightIndicator(renderer, boxX + 10, startY, LIGHT_SIZE, LABEL_HEIGHT, 'A', isARed);

    // Road B light
    bool isBRed = !isGreen('B');
    drawRoadLightIndicator(renderer, boxX + 10, startY + LIGHT_SIZE + LABEL_HEIGHT + SPACING,
                          LIGHT_SIZE, LABEL_HEIGHT, 'B', isBRed);

    // Road C light
    bool isCRed = !isGreen('C');
    drawRoadLightIndicator(renderer, boxX + 10, startY + 2 * (LIGHT_SIZE + LABEL_HEIGHT + SPACING),
                          LIGHT_SIZE, LABEL_HEIGHT, 'C', isCRed);

    // Road D light
    bool isDRed = !isGreen('D');
    drawRoadLightIndicator(renderer, boxX + 10, startY + 3 * (LIGHT_SIZE + LABEL_HEIGHT + SPACING),
                          LIGHT_SIZE, LABEL_HEIGHT, 'D', isDRed);

    // Draw priority mode indicator if active
    if (isPriorityMode) {
        // Flash the indicator
        uint32_t time = SDL_GetTicks();
        bool flash = (time / 500) % 2 == 0;

        SDL_SetRenderDrawColor(renderer, flash ? 255 : 200, flash ? 140 : 100, 0, 255);

        SDL_FRect priorityBox = {
            static_cast<float>(boxX + boxWidth + 10),
            static_cast<float>(boxY),
            50.0f,
            50.0f
        };
        SDL_RenderFillRect(renderer, &priorityBox);

        // Draw "P" for priority
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        // Vertical line
        SDL_RenderLine(renderer, boxX + boxWidth + 25, boxY + 10, boxX + boxWidth + 25, boxY + 40);
        // Curved top
        SDL_RenderLine(renderer, boxX + boxWidth + 25, boxY + 10, boxX + boxWidth + 45, boxY + 10);
        SDL_RenderLine(renderer, boxX + boxWidth + 45, boxY + 10, boxX + boxWidth + 45, boxY + 25);
        SDL_RenderLine(renderer, boxX + boxWidth + 45, boxY + 25, boxX + boxWidth + 25, boxY + 25);

        // Draw A2 indicator
        SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
        SDL_RenderLine(renderer, boxX + boxWidth + 30, boxY + 30, boxX + boxWidth + 40, boxY + 30);
        SDL_RenderLine(renderer, boxX + boxWidth + 35, boxY + 30, boxX + boxWidth + 35, boxY + 40);
    }

    // Draw the individual traffic lights at road positions
    drawLightForA(renderer, !isGreen('A'));
    drawLightForB(renderer, !isGreen('B'));
    drawLightForC(renderer, !isGreen('C'));
    drawLightForD(renderer, !isGreen('D'));
}

void TrafficLight::drawRoadLightIndicator(SDL_Renderer* renderer, int x, int y,
                                         int lightSize, int labelHeight,
                                         char roadId, bool isRed) {
    // Draw label background
    SDL_SetRenderDrawColor(renderer, 70, 70, 70, 255);
    SDL_FRect labelBox = {
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(lightSize * 2 + 10),
        static_cast<float>(labelHeight)
    };
    SDL_RenderFillRect(renderer, &labelBox);

    // Draw road identifier letter
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    switch (roadId) {
        case 'A':
            // Draw "A"
            SDL_RenderLine(renderer, x + 5, y + labelHeight - 5, x + 10, y + 5);
            SDL_RenderLine(renderer, x + 10, y + 5, x + 15, y + labelHeight - 5);
            SDL_RenderLine(renderer, x + 7, y + labelHeight/2, x + 13, y + labelHeight/2);
            break;
        case 'B':
            // Draw "B"
            SDL_RenderLine(renderer, x + 5, y + 5, x + 5, y + labelHeight - 5);
            SDL_RenderLine(renderer, x + 5, y + 5, x + 12, y + 5);
            SDL_RenderLine(renderer, x + 12, y + 5, x + 15, y + 8);
            SDL_RenderLine(renderer, x + 15, y + 8, x + 12, y + labelHeight/2);
            SDL_RenderLine(renderer, x + 12, y + labelHeight/2, x + 5, y + labelHeight/2);
            SDL_RenderLine(renderer, x + 5, y + labelHeight/2, x + 12, y + labelHeight/2);
            SDL_RenderLine(renderer, x + 12, y + labelHeight/2, x + 15, y + labelHeight - 8);
            SDL_RenderLine(renderer, x + 15, y + labelHeight - 8, x + 12, y + labelHeight - 5);
            SDL_RenderLine(renderer, x + 12, y + labelHeight - 5, x + 5, y + labelHeight - 5);
            break;
        case 'C':
            // Draw "C"
            SDL_RenderLine(renderer, x + 15, y + 5, x + 5, y + 5);
            SDL_RenderLine(renderer, x + 5, y + 5, x + 5, y + labelHeight - 5);
            SDL_RenderLine(renderer, x + 5, y + labelHeight - 5, x + 15, y + labelHeight - 5);
            break;
        case 'D':
            // Draw "D"
            SDL_RenderLine(renderer, x + 5, y + 5, x + 5, y + labelHeight - 5);
            SDL_RenderLine(renderer, x + 5, y + 5, x + 10, y + 5);
            SDL_RenderLine(renderer, x + 10, y + 5, x + 15, y + labelHeight/2);
            SDL_RenderLine(renderer, x + 15, y + labelHeight/2, x + 10, y + labelHeight - 5);
            SDL_RenderLine(renderer, x + 10, y + labelHeight - 5, x + 5, y + labelHeight - 5);
            break;
    }

    // Draw red light
    SDL_SetRenderDrawColor(renderer, isRed ? 255 : 60, 0, 0, 255);
    SDL_FRect redLight = {
        static_cast<float>(x + 25),
        static_cast<float>(y + labelHeight + 5),
        static_cast<float>(lightSize),
        static_cast<float>(lightSize)
    };
    SDL_RenderFillRect(renderer, &redLight);

    // Add 3D effect to red light
    if (isRed) {
        // Light glow when active
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(renderer, 255, 100, 100, 100);
        SDL_FRect redGlow = {
            redLight.x - 5.0f,
            redLight.y - 5.0f,
            redLight.w + 10.0f,
            redLight.h + 10.0f
        };
        SDL_RenderFillRect(renderer, &redGlow);
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

        // Highlight (top-left edge)
        SDL_SetRenderDrawColor(renderer, 255, 150, 150, 255);
        SDL_FRect redHighlight = {
            redLight.x,
            redLight.y,
            redLight.w/3,
            redLight.h/3
        };
        SDL_RenderFillRect(renderer, &redHighlight);
    }

    // Draw green light
    SDL_SetRenderDrawColor(renderer, 0, isRed ? 60 : 255, 0, 255);
    SDL_FRect greenLight = {
        static_cast<float>(x + 50),
        static_cast<float>(y + labelHeight + 5),
        static_cast<float>(lightSize),
        static_cast<float>(lightSize)
    };
    SDL_RenderFillRect(renderer, &greenLight);

    // Add 3D effect to green light
    if (!isRed) {
        // Light glow when active
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(renderer, 100, 255, 100, 100);
        SDL_FRect greenGlow = {
            greenLight.x - 5.0f,
            greenLight.y - 5.0f,
            greenLight.w + 10.0f,
            greenLight.h + 10.0f
        };
        SDL_RenderFillRect(renderer, &greenGlow);
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

        // Highlight (top-left edge)
        SDL_SetRenderDrawColor(renderer, 150, 255, 150, 255);
        SDL_FRect greenHighlight = {
            greenLight.x,
            greenLight.y,
            greenLight.w/3,
            greenLight.h/3
        };
        SDL_RenderFillRect(renderer, &greenHighlight);
    }

    // Draw light outlines
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &redLight);
    SDL_RenderRect(renderer, &greenLight);
}

void TrafficLight::drawLightForA(SDL_Renderer* renderer, bool isRed) {
    const int LIGHT_SIZE = 20;
    const int LIGHT_BOX_WIDTH = 40;
    const int LIGHT_BOX_HEIGHT = 80;
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 800;
    const int ARROW_SIZE = 10;

    // Position for road A - move slightly for better visibility
    int x = WINDOW_WIDTH/2 + 40;
    int y = WINDOW_HEIGHT/2 - 120;

    // Enhanced traffic light box with 3D effect
    // Shadow
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_FRect shadowBox = {(float)x + 3, (float)y + 3, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &shadowBox);

    // Main box (dark gray)
    SDL_SetRenderDrawColor(renderer, 70, 70, 70, 255);
    SDL_FRect lightBox = {(float)x, (float)y, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &lightBox);

    // Highlight edge
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_FRect highlight = {(float)x, (float)y, LIGHT_BOX_WIDTH, 2.0f};
    SDL_RenderFillRect(renderer, &highlight);
    SDL_FRect highlightSide = {(float)x, (float)y, 2.0f, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &highlightSide);

    // Black border
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &lightBox);

    // Red light with glow effect
    if (isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 150, 0, 0, 100);
        SDL_FRect redGlow = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2 - 3), (float)(y + 10 - 3),
                             LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &redGlow);
    }

    SDL_SetRenderDrawColor(renderer, isRed ? 255 : 50, 0, 0, 255);
    SDL_FRect redLight = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2), (float)(y + 10),
                         LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &redLight);

    // Green light with glow effect
    if (!isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 0, 150, 0, 100);
        SDL_FRect greenGlow = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2 - 3), (float)(y + 40 - 3),
                              LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &greenGlow);
    }

    SDL_SetRenderDrawColor(renderer, 0, isRed ? 50 : 255, 0, 255);
    SDL_FRect greenLight = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2), (float)(y + 40),
                           LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &greenLight);

    // Add straight arrow for green light
    if (!isRed) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Draw arrow shape
        SDL_FRect arrowStem = {(float)(x + LIGHT_BOX_WIDTH/2 - 2), (float)(y + 50 - ARROW_SIZE/2),
                              4.0f, (float)ARROW_SIZE};
        SDL_RenderFillRect(renderer, &arrowStem);

        // Arrow head (triangle)
        SDL_Vertex vertices[3];
        SDL_FColor white = {1.0f, 1.0f, 1.0f, 1.0f};

        vertices[0].position.x = x + LIGHT_BOX_WIDTH/2;
        vertices[0].position.y = y + 50 - ARROW_SIZE/2 - 5;
        vertices[0].color = white;

        vertices[1].position.x = x + LIGHT_BOX_WIDTH/2 - 5;
        vertices[1].position.y = y + 50 - ARROW_SIZE/2;
        vertices[1].color = white;

        vertices[2].position.x = x + LIGHT_BOX_WIDTH/2 + 5;
        vertices[2].position.y = y + 50 - ARROW_SIZE/2;
        vertices[2].color = white;

        SDL_RenderGeometry(renderer, NULL, vertices, 3, NULL, 0);
    }

    // Black borders around lights
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &redLight);
    SDL_RenderRect(renderer, &greenLight);
}

void TrafficLight::drawLightForB(SDL_Renderer* renderer, bool isRed) {
    const int LIGHT_SIZE = 20;
    const int LIGHT_BOX_WIDTH = 80;
    const int LIGHT_BOX_HEIGHT = 40;
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 800;
    const int ARROW_SIZE = 10;

    // Position for road B
    int x = WINDOW_WIDTH/2 - 60;
    int y = WINDOW_HEIGHT/2 + 40;

    // Enhanced traffic light box with 3D effect
    // Shadow
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_FRect shadowBox = {(float)x + 3, (float)y + 3, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &shadowBox);

    // Main box (dark gray)
    SDL_SetRenderDrawColor(renderer, 70, 70, 70, 255);
    SDL_FRect lightBox = {(float)x, (float)y, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &lightBox);

    // Highlight edge
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_FRect highlight = {(float)x, (float)y, LIGHT_BOX_WIDTH, 2.0f};
    SDL_RenderFillRect(renderer, &highlight);
    SDL_FRect highlightSide = {(float)x, (float)y, 2.0f, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &highlightSide);

    // Black border
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &lightBox);

    // Red light with glow effect
    if (isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 150, 0, 0, 100);
        SDL_FRect redGlow = {(float)(x + 10 - 3), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2 - 3),
                            LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &redGlow);
    }

    SDL_SetRenderDrawColor(renderer, isRed ? 255 : 50, 0, 0, 255);
    SDL_FRect redLight = {(float)(x + 10), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2),
                         LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &redLight);

    // Green light with glow effect
    if (!isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 0, 150, 0, 100);
        SDL_FRect greenGlow = {(float)(x + 40 - 3), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2 - 3),
                              LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &greenGlow);
    }

    SDL_SetRenderDrawColor(renderer, 0, isRed ? 50 : 255, 0, 255);
    SDL_FRect greenLight = {(float)(x + 40), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2),
                           LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &greenLight);

    // Add straight arrow for green light
    if (!isRed) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Draw arrow shape (horizontal)
        SDL_FRect arrowStem = {(float)(x + 50 - ARROW_SIZE/2), (float)(y + LIGHT_BOX_HEIGHT/2 - 2),
                              (float)ARROW_SIZE, 4.0f};
        SDL_RenderFillRect(renderer, &arrowStem);

        // Arrow head (triangle)
        SDL_Vertex vertices[3];
        SDL_FColor white = {1.0f, 1.0f, 1.0f, 1.0f};

        vertices[0].position.x = x + 50 - ARROW_SIZE/2 - 5;
        vertices[0].position.y = y + LIGHT_BOX_HEIGHT/2;
        vertices[0].color = white;

        vertices[1].position.x = x + 50 - ARROW_SIZE/2;
        vertices[1].position.y = y + LIGHT_BOX_HEIGHT/2 - 5;
        vertices[1].color = white;

        vertices[2].position.x = x + 50 - ARROW_SIZE/2;
        vertices[2].position.y = y + LIGHT_BOX_HEIGHT/2 + 5;
        vertices[2].color = white;

        SDL_RenderGeometry(renderer, NULL, vertices, 3, NULL, 0);
    }

    // Black borders around lights
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &redLight);
    SDL_RenderRect(renderer, &greenLight);
}

void TrafficLight::drawLightForC(SDL_Renderer* renderer, bool isRed) {
    const int LIGHT_SIZE = 20;
    const int LIGHT_BOX_WIDTH = 40;
    const int LIGHT_BOX_HEIGHT = 80;
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 800;
    const int ARROW_SIZE = 10;

    // Position for road C - move slightly for better visibility
    int x = WINDOW_WIDTH/2 - 80;
    int y = WINDOW_HEIGHT/2 + 40;

    // Enhanced traffic light box with 3D effect
    // Shadow
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_FRect shadowBox = {(float)x + 3, (float)y + 3, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &shadowBox);

    // Main box (dark gray)
    SDL_SetRenderDrawColor(renderer, 70, 70, 70, 255);
    SDL_FRect lightBox = {(float)x, (float)y, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &lightBox);

    // Highlight edge
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_FRect highlight = {(float)x, (float)y, LIGHT_BOX_WIDTH, 2.0f};
    SDL_RenderFillRect(renderer, &highlight);
    SDL_FRect highlightSide = {(float)x, (float)y, 2.0f, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &highlightSide);

    // Black border
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &lightBox);

    // Red light with glow effect
    if (isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 150, 0, 0, 100);
        SDL_FRect redGlow = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2 - 3), (float)(y + 10 - 3),
                             LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &redGlow);
    }

    SDL_SetRenderDrawColor(renderer, isRed ? 255 : 50, 0, 0, 255);
    SDL_FRect redLight = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2), (float)(y + 10),
                         LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &redLight);

    // Green light with glow effect
    if (!isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 0, 150, 0, 100);
        SDL_FRect greenGlow = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2 - 3), (float)(y + 40 - 3),
                              LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &greenGlow);
    }

    SDL_SetRenderDrawColor(renderer, 0, isRed ? 50 : 255, 0, 255);
    SDL_FRect greenLight = {(float)(x + LIGHT_BOX_WIDTH/2 - LIGHT_SIZE/2), (float)(y + 40),
                           LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &greenLight);

    // Add straight arrow for green light (pointing upward)
    if (!isRed) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Draw arrow shape
        SDL_FRect arrowStem = {(float)(x + LIGHT_BOX_WIDTH/2 - 2), (float)(y + 50 - ARROW_SIZE/2),
                              4.0f, (float)ARROW_SIZE};
        SDL_RenderFillRect(renderer, &arrowStem);

        // Arrow head (triangle)
        SDL_Vertex vertices[3];
        SDL_FColor white = {1.0f, 1.0f, 1.0f, 1.0f};

        vertices[0].position.x = x + LIGHT_BOX_WIDTH/2;
        vertices[0].position.y = y + 50 - ARROW_SIZE/2 - 5;
        vertices[0].color = white;

        vertices[1].position.x = x + LIGHT_BOX_WIDTH/2 - 5;
        vertices[1].position.y = y + 50 - ARROW_SIZE/2;
        vertices[1].color = white;

        vertices[2].position.x = x + LIGHT_BOX_WIDTH/2 + 5;
        vertices[2].position.y = y + 50 - ARROW_SIZE/2;
        vertices[2].color = white;

        SDL_RenderGeometry(renderer, NULL, vertices, 3, NULL, 0);
    }

    // Black borders around lights
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &redLight);
    SDL_RenderRect(renderer, &greenLight);
}

void TrafficLight::drawLightForD(SDL_Renderer* renderer, bool isRed) {
    const int LIGHT_SIZE = 20;
    const int LIGHT_BOX_WIDTH = 80;
    const int LIGHT_BOX_HEIGHT = 40;
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 800;
    const int ARROW_SIZE = 10;

    // Position for road D
    int x = WINDOW_WIDTH/2 - 120;
    int y = WINDOW_HEIGHT/2 - 80;

    // Enhanced traffic light box with 3D effect
    // Shadow
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_FRect shadowBox = {(float)x + 3, (float)y + 3, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &shadowBox);

    // Main box (dark gray)
    SDL_SetRenderDrawColor(renderer, 70, 70, 70, 255);
    SDL_FRect lightBox = {(float)x, (float)y, LIGHT_BOX_WIDTH, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &lightBox);

    // Highlight edge
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_FRect highlight = {(float)x, (float)y, LIGHT_BOX_WIDTH, 2.0f};
    SDL_RenderFillRect(renderer, &highlight);
    SDL_FRect highlightSide = {(float)x, (float)y, 2.0f, LIGHT_BOX_HEIGHT};
    SDL_RenderFillRect(renderer, &highlightSide);

    // Black border
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &lightBox);

    // Red light with glow effect
    if (isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 150, 0, 0, 100);
        SDL_FRect redGlow = {(float)(x + 10 - 3), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2 - 3),
                            LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &redGlow);
    }

    SDL_SetRenderDrawColor(renderer, isRed ? 255 : 50, 0, 0, 255);
    SDL_FRect redLight = {(float)(x + 10), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2),
                         LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &redLight);

    // Green light with glow effect
    if (!isRed) {
        // Glow
        SDL_SetRenderDrawColor(renderer, 0, 150, 0, 100);
        SDL_FRect greenGlow = {(float)(x + 40 - 3), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2 - 3),
                              LIGHT_SIZE + 6, LIGHT_SIZE + 6};
        SDL_RenderFillRect(renderer, &greenGlow);
    }

    SDL_SetRenderDrawColor(renderer, 0, isRed ? 50 : 255, 0, 255);
    SDL_FRect greenLight = {(float)(x + 40), (float)(y + LIGHT_BOX_HEIGHT/2 - LIGHT_SIZE/2),
                           LIGHT_SIZE, LIGHT_SIZE};
    SDL_RenderFillRect(renderer, &greenLight);

    // Add straight arrow for green light (horizontal pointing right)
    if (!isRed) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Draw arrow shape
        SDL_FRect arrowStem = {(float)(x + 50 - ARROW_SIZE/2), (float)(y + LIGHT_BOX_HEIGHT/2 - 2),
                              (float)ARROW_SIZE, 4.0f};
        SDL_RenderFillRect(renderer, &arrowStem);

        // Arrow head (triangle)
        SDL_Vertex vertices[3];
        SDL_FColor white = {1.0f, 1.0f, 1.0f, 1.0f};

        vertices[0].position.x = x + 50 + ARROW_SIZE/2 + 5;
        vertices[0].position.y = y + LIGHT_BOX_HEIGHT/2;
        vertices[0].color = white;

        vertices[1].position.x = x + 50 + ARROW_SIZE/2;
        vertices[1].position.y = y + LIGHT_BOX_HEIGHT/2 - 5;
        vertices[1].color = white;

        vertices[2].position.x = x + 50 + ARROW_SIZE/2;
        vertices[2].position.y = y + LIGHT_BOX_HEIGHT/2 + 5;
        vertices[2].color = white;

        SDL_RenderGeometry(renderer, NULL, vertices, 3, NULL, 0);
    }

    // Black borders around lights
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderRect(renderer, &redLight);
    SDL_RenderRect(renderer, &greenLight);
}
