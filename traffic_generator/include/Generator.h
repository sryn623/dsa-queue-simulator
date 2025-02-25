// Generator.h
#pragma once
#include <string>
#include <random>
#include <map>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include "core/Constants.h"
#include <mutex>

class Generator {
private:
    std::mt19937 rng;  // Random number generator
    std::map<LaneId, std::filesystem::path> laneFiles;  // Mapping of lanes to their file paths
    uint32_t nextVehicleId;  // Counter for vehicle IDs
    std::filesystem::path dataDir;  // Directory for lane data files
    std::chrono::steady_clock::time_point lastGenTime;  // Last generation timestamp
    std::mutex fileMutex;  // Mutex for thread-safe file operations

    // Settings for each lane
    struct LaneConfig {
        double spawnRate;
        int maxVehicles;
        bool canGoStraight;
        bool canGoRight;
    };
    std::map<LaneId, LaneConfig> laneConfigs;

    // Private helper methods
    void initializeLaneFiles();
    void setupLaneConfigs();
    Direction getRandomDirection(const LaneConfig& config);
    size_t countVehiclesInFile(const std::filesystem::path& filepath) const;
    void writeVehicleToFile(const std::filesystem::path& filepath, uint32_t id, Direction dir);
    bool shouldGenerateVehicle(const LaneConfig& config, size_t currentCount);
    void clearAllFiles();
    void logGeneration(LaneId lane, uint32_t vehicleId, Direction dir, size_t currentCount, int maxCount);

public:
    Generator();
    void generateTraffic();
    void displayStatus() const;
    ~Generator() = default;
};