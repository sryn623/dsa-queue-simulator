// src/traffic_generator.cpp
#include "traffic_generator.h"
#include <thread>
#include <fstream>
#include <iostream>
#include <sstream>
#include <ctime>

Generator::Generator()
    : nextVehicleId(1000)
{
    // Seed the random number generator
    std::random_device rd;
    rng = std::mt19937(rd());

    // Initialize lane files system
    initializeLaneFiles();

    // Setup configurations for each lane
    setupLaneConfigs();
}

void Generator::initializeLaneFiles() {
    // Create data directory
    dataDir = std::filesystem::current_path() / "data" / "lanes";
    if (!std::filesystem::exists(dataDir)) {
        std::filesystem::create_directories(dataDir);
    }

    // Initialize the individual lane files
    laneFiles[Constants::LaneId::AL1_INCOMING] = dataDir / "lane_a1.txt";
    laneFiles[Constants::LaneId::AL2_PRIORITY] = dataDir / "lane_a2.txt";
    laneFiles[Constants::LaneId::AL3_FREELANE] = dataDir / "lane_a3.txt";
    laneFiles[Constants::LaneId::BL1_INCOMING] = dataDir / "lane_b1.txt";
    laneFiles[Constants::LaneId::BL2_NORMAL] = dataDir / "lane_b2.txt";
    laneFiles[Constants::LaneId::BL3_FREELANE] = dataDir / "lane_b3.txt";
    laneFiles[Constants::LaneId::CL1_INCOMING] = dataDir / "lane_c1.txt";
    laneFiles[Constants::LaneId::CL2_NORMAL] = dataDir / "lane_c2.txt";
    laneFiles[Constants::LaneId::CL3_FREELANE] = dataDir / "lane_c3.txt";
    laneFiles[Constants::LaneId::DL1_INCOMING] = dataDir / "lane_d1.txt";
    laneFiles[Constants::LaneId::DL2_NORMAL] = dataDir / "lane_d2.txt";
    laneFiles[Constants::LaneId::DL3_FREELANE] = dataDir / "lane_d3.txt";

    // Create/clear all files
    clearAllFiles();

    // Also create a common vehicle data file for compatibility with C implementation
    std::ofstream combinedFile("vehicles.data", std::ios::trunc);
    if (!combinedFile) {
        std::cerr << "Failed to create vehicles.data file" << std::endl;
    }
}

void Generator::setupLaneConfigs() {
    // Set up spawn rates and lane configurations

    // Road A (West) configs
    laneConfigs[Constants::LaneId::AL1_INCOMING] = {0.6, 10, true, false, true};   // A1: Right/Straight
    laneConfigs[Constants::LaneId::AL2_PRIORITY] = {0.7, 15, true, false, false};  // A2: Straight only (priority)
    laneConfigs[Constants::LaneId::AL3_FREELANE] = {0.5, 8, false, true, false};   // A3: Left only

    // Road B (North) configs
    laneConfigs[Constants::LaneId::BL1_INCOMING] = {0.55, 10, true, false, true};  // B1: Right/Straight
    laneConfigs[Constants::LaneId::BL2_NORMAL] = {0.6, 12, true, false, false};    // B2: Straight only
    laneConfigs[Constants::LaneId::BL3_FREELANE] = {0.5, 8, false, true, false};   // B3: Left only

    // Road C (East) configs
    laneConfigs[Constants::LaneId::CL1_INCOMING] = {0.6, 10, true, false, true};   // C1: Right/Straight
    laneConfigs[Constants::LaneId::CL2_NORMAL] = {0.65, 12, true, false, false};   // C2: Straight only
    laneConfigs[Constants::LaneId::CL3_FREELANE] = {0.5, 8, false, true, false};   // C3: Left only

    // Road D (South) configs
    laneConfigs[Constants::LaneId::DL1_INCOMING] = {0.6, 10, true, false, true};   // D1: Right/Straight
    laneConfigs[Constants::LaneId::DL2_NORMAL] = {0.6, 12, true, false, false};    // D2: Straight only
    laneConfigs[Constants::LaneId::DL3_FREELANE] = {0.5, 8, false, true, false};   // D3: Left only
}

void Generator::generateTraffic() {
    // Try to generate traffic for each lane
    for (const auto& [laneId, config] : laneConfigs) {
        // Get current vehicle count
        size_t currentCount = countVehiclesInFile(laneFiles[laneId]);

        // Check if we should generate a new vehicle
        if (shouldGenerateVehicle(config, currentCount)) {
            // Determine direction based on lane rules
            Constants::Direction dir = getRandomDirection(config);

            // Create new vehicle ID and write to file
            uint32_t vehicleId = nextVehicleId++;
            writeVehicleToFile(laneFiles[laneId], vehicleId, dir);

            // Also write to combined file for C code compatibility
            writeVehicleToCombinedFile(laneId, vehicleId);

            // Log the generation
            logGeneration(laneId, vehicleId, dir, currentCount + 1, config.maxVehicles);
        }
    }
}

void Generator::writeVehicleToCombinedFile(Constants::LaneId lane, uint32_t id) {
    std::lock_guard<std::mutex> lock(fileMutex);

    // Determine which road this lane belongs to (A, B, C, D)
    char roadId;
    int laneNum;

    switch (static_cast<int>(lane) / 3) {
        case 0: roadId = 'A'; break;
        case 1: roadId = 'B'; break;
        case 2: roadId = 'C'; break;
        case 3: roadId = 'D'; break;
        default: roadId = 'X'; break;
    }

    // Determine lane number within the road (1, 2, 3)
    laneNum = static_cast<int>(lane) % 3 + 1;

    try {
        std::ofstream file("vehicles.data", std::ios::app);
        if (file) {
            // Format: "VEH1234_L2:A" for vehicle 1234 in lane 2 of road A
            file << "VEH" << id << "_L" << laneNum << ":" << roadId << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error writing to combined file: " << e.what() << std::endl;
    }
}

Constants::Direction Generator::getRandomDirection(const LaneConfig& config) {
    std::vector<Constants::Direction> possibleDirections;

    // Add possible directions based on configuration
    if (config.canGoStraight) possibleDirections.push_back(Constants::Direction::STRAIGHT);
    if (config.canGoLeft) possibleDirections.push_back(Constants::Direction::LEFT);
    if (config.canGoRight) possibleDirections.push_back(Constants::Direction::RIGHT);

    // If no directions allowed (shouldn't happen), default to straight
    if (possibleDirections.empty()) {
        return Constants::Direction::STRAIGHT;
    }

    // Select random direction from allowed set
    std::uniform_int_distribution<> dirDist(0, possibleDirections.size() - 1);
    return possibleDirections[dirDist(rng)];
}

size_t Generator::countVehiclesInFile(const std::filesystem::path& filepath) const {
    try {
        if (!std::filesystem::exists(filepath)) return 0;

        std::ifstream file(filepath);
        if (!file) return 0;

        size_t count = 0;
        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty()) count++;
        }

        return count;
    } catch (const std::exception& e) {
        std::cerr << "Error counting vehicles: " << e.what() << std::endl;
        return 0;
    }
}

void Generator::writeVehicleToFile(const std::filesystem::path& filepath, uint32_t id,
                               Constants::Direction dir) {
    std::lock_guard<std::mutex> lock(fileMutex);

    try {
        std::ofstream file(filepath, std::ios::app);
        if (file) {
            char dirChar;
            switch (dir) {
                case Constants::Direction::STRAIGHT: dirChar = 'S'; break;
                case Constants::Direction::LEFT: dirChar = 'L'; break;
                case Constants::Direction::RIGHT: dirChar = 'R'; break;
                default: dirChar = 'S'; break;
            }

            // Format: "ID,DIRECTION;TIMESTAMP"
            file << id << "," << dirChar << ";" << std::time(nullptr) << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error writing to file: " << e.what() << std::endl;
    }
}

bool Generator::shouldGenerateVehicle(const LaneConfig& config, size_t currentCount) {
    // Check if lane is at capacity
    if (currentCount >= config.maxVehicles) {
        return false;
    }

    // Probability-based generation
    std::uniform_real_distribution<> dist(0.0, 1.0);
    return dist(rng) < config.spawnRate / (currentCount + 1); // Decreases as queue grows
}

void Generator::clearAllFiles() {
    std::lock_guard<std::mutex> lock(fileMutex);

    for (const auto& [_, filepath] : laneFiles) {
        try {
            std::ofstream file(filepath, std::ios::trunc);
        } catch (const std::exception& e) {
            std::cerr << "Error clearing file " << filepath << ": " << e.what() << std::endl;
        }
    }

    // Clear combined file too
    try {
        std::ofstream file("vehicles.data", std::ios::trunc);
    } catch (const std::exception& e) {
        std::cerr << "Error clearing vehicles.data: " << e.what() << std::endl;
    }
}

void Generator::logGeneration(Constants::LaneId lane, uint32_t vehicleId, Constants::Direction dir,
                          size_t currentCount, int maxCount) {
    std::cout << "Generated vehicle " << vehicleId << " in lane "
              << static_cast<int>(lane) << " (";

    // Print road/lane identifier
    char roadId;
    int laneNum = static_cast<int>(lane) % 3 + 1;

    switch (static_cast<int>(lane) / 3) {
        case 0: roadId = 'A'; break;
        case 1: roadId = 'B'; break;
        case 2: roadId = 'C'; break;
        case 3: roadId = 'D'; break;
        default: roadId = 'X'; break;
    }

    std::cout << roadId << laneNum << ") with direction ";

    switch (dir) {
        case Constants::Direction::STRAIGHT: std::cout << "STRAIGHT"; break;
        case Constants::Direction::LEFT: std::cout << "LEFT"; break;
        case Constants::Direction::RIGHT: std::cout << "RIGHT"; break;
    }

    std::cout << " [" << currentCount << "/" << maxCount << "]" << std::endl;
}

void Generator::displayStatus() const {
    // Clear console
    std::cout << "\033[2J\033[1;1H";

    std::cout << "=== Traffic Generator Status ===" << std::endl;
    std::cout << "Next Vehicle ID: " << nextVehicleId << std::endl;
    std::cout << std::endl;

    // Display lane statistics
    std::cout << "Lane Statistics:" << std::endl;
    std::cout << "----------------" << std::endl;

    for (const auto& [laneId, filepath] : laneFiles) {
        size_t count = countVehiclesInFile(filepath);
        const auto& config = laneConfigs.at(laneId);

        // Get lane name
        char roadId;
        int laneNum = static_cast<int>(laneId) % 3 + 1;

        switch (static_cast<int>(laneId) / 3) {
            case 0: roadId = 'A'; break;
            case 1: roadId = 'B'; break;
            case 2: roadId = 'C'; break;
            case 3: roadId = 'D'; break;
            default: roadId = 'X'; break;
        }

        std::cout << "Lane " << roadId << laneNum << ": ";

        // Special indicators
        if (laneNum == 2 && roadId == 'A') std::cout << "[PRIORITY] ";
        if (laneNum == 3) std::cout << "[FREE] ";

        // Display count and capacity
        std::cout << count << "/" << config.maxVehicles;

        // Directions allowed
        std::cout << " Directions: ";
        if (config.canGoStraight) std::cout << "S";
        if (config.canGoLeft) std::cout << "L";
        if (config.canGoRight) std::cout << "R";

        std::cout << std::endl;
    }
}

int main() {
    Generator generator;

    std::cout << "Traffic Generator Started" << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;

    while (true) {
        generator.generateTraffic();
        generator.displayStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}