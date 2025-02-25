// Generator.cpp
#include "../include/Generator.h"

Generator::Generator() : nextVehicleId(1) {
    try {
        std::random_device rd;
        rng.seed(rd());
        lastGenTime = std::chrono::steady_clock::now();

        // Set up data directory
        dataDir = (std::filesystem::current_path() / "data" / "lanes").lexically_normal();
        std::cout << "Generator using path: " << dataDir << std::endl;
        std::filesystem::create_directories(dataDir);

        initializeLaneFiles();
        setupLaneConfigs();
        clearAllFiles();

    } catch (const std::exception& e) {
        std::cerr << "Generator initialization failed: " << e.what() << std::endl;
        throw;
    }
}


void Generator::initializeLaneFiles() {
    laneFiles = {
        {LaneId::AL1_INCOMING, (dataDir / "lane_a1.txt").lexically_normal()},
        {LaneId::AL2_PRIORITY, (dataDir / "lane_a2.txt").lexically_normal()},
        {LaneId::BL1_INCOMING, (dataDir / "lane_b1.txt").lexically_normal()},
        {LaneId::BL2_NORMAL,   (dataDir / "lane_b2.txt").lexically_normal()},
        {LaneId::CL1_INCOMING, (dataDir / "lane_c1.txt").lexically_normal()},
        {LaneId::CL2_NORMAL,   (dataDir / "lane_c2.txt").lexically_normal()},
        {LaneId::DL1_INCOMING, (dataDir / "lane_d1.txt").lexically_normal()},
        {LaneId::DL2_NORMAL,   (dataDir / "lane_d2.txt").lexically_normal()}
    };
}

void Generator::setupLaneConfigs() {
    // First lanes (can go straight or right)
    LaneConfig firstLaneConfig;
    firstLaneConfig.spawnRate = 0.12;      // 12% spawn rate
    firstLaneConfig.maxVehicles = 12;      // Max 12 vehicles
    firstLaneConfig.canGoStraight = true;  // Can go straight
    firstLaneConfig.canGoRight = true;     // Can turn right

    // Normal second lanes (straight only)
    LaneConfig normalSecondLaneConfig;
    normalSecondLaneConfig.spawnRate = 0.12;      // 12% spawn rate
    normalSecondLaneConfig.maxVehicles = 12;      // Max 12 vehicles
    normalSecondLaneConfig.canGoStraight = true;  // Can go straight
    normalSecondLaneConfig.canGoRight = false;    // Cannot turn

    // Priority lane config (AL2)
    LaneConfig priorityLaneConfig;
    priorityLaneConfig.spawnRate = 0.15;    // 15% spawn rate
    priorityLaneConfig.maxVehicles = 15;    // Max 15 vehicles
    priorityLaneConfig.canGoStraight = true;// Can go straight
    priorityLaneConfig.canGoRight = false;  // Cannot turn

    // Set configurations for each lane
    laneConfigs[LaneId::AL1_INCOMING] = firstLaneConfig;
    laneConfigs[LaneId::BL1_INCOMING] = firstLaneConfig;
    laneConfigs[LaneId::CL1_INCOMING] = firstLaneConfig;
    laneConfigs[LaneId::DL1_INCOMING] = firstLaneConfig;

    laneConfigs[LaneId::AL2_PRIORITY] = priorityLaneConfig;
    laneConfigs[LaneId::BL2_NORMAL] = normalSecondLaneConfig;
    laneConfigs[LaneId::CL2_NORMAL] = normalSecondLaneConfig;
    laneConfigs[LaneId::DL2_NORMAL] = normalSecondLaneConfig;
}


Direction Generator::getRandomDirection(const LaneConfig& config) {
    if (config.canGoStraight && config.canGoRight) {
        std::uniform_real_distribution<> dist(0.0, 1.0);
        return (dist(rng) < 0.5) ? Direction::STRAIGHT : Direction::RIGHT;
    }
    return Direction::STRAIGHT;
}

void Generator::generateTraffic() {
    try {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - lastGenTime).count();

        if (elapsed < 2000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000 - elapsed));
            return;
        }

        bool anyVehicleGenerated = false;
        std::uniform_real_distribution<> dist(0.0, 1.0);

        for (const auto& [laneId, filepath] : laneFiles) {
            const auto& config = laneConfigs[laneId];
            size_t currentVehicles = countVehiclesInFile(filepath);

            if (currentVehicles < config.maxVehicles && dist(rng) < config.spawnRate) {
                Direction dir = getRandomDirection(config);
                writeVehicleToFile(filepath, nextVehicleId, dir);
                logGeneration(laneId, nextVehicleId, dir, currentVehicles + 1, config.maxVehicles);
                nextVehicleId++;
                anyVehicleGenerated = true;
            }
        }

        lastGenTime = std::chrono::steady_clock::now();

        if (!anyVehicleGenerated) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    } catch (const std::exception& e) {
        std::cerr << "Error in traffic generation: " << e.what() << std::endl;
    }
}

void Generator::writeVehicleToFile(const std::filesystem::path& filepath, uint32_t id, Direction dir) {
    std::lock_guard<std::mutex> lock(fileMutex);
    try {
        std::ofstream file(filepath, std::ios::app);
        if (!file) {
            throw std::runtime_error("Cannot open file: " + filepath.string());
        }

        char dirChar = (dir == Direction::STRAIGHT) ? 'S' :
                       (dir == Direction::LEFT) ? 'L' : 'R';
        file << id << "," << dirChar << ";\n";
        file.flush();

    } catch (const std::exception& e) {
        std::cerr << "Error writing to file " << filepath << ": " << e.what() << std::endl;
    }
}


size_t Generator::countVehiclesInFile(const std::filesystem::path& filepath) const {
    try {
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

void Generator::clearAllFiles() {
    for (const auto& [_, filepath] : laneFiles) {
        try {
            std::ofstream file(filepath, std::ios::trunc);
            if (!file) {
                throw std::runtime_error("Failed to clear file: " + filepath.string());
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error clearing file: " << e.what() << std::endl;
            throw;
        }
    }
}

void Generator::logGeneration(LaneId lane, uint32_t vehicleId, Direction dir,
                            size_t currentCount, int maxCount) {
    std::cout << "Generated vehicle " << vehicleId
              << " in lane " << static_cast<int>(lane)
              << " with direction " << (dir == Direction::STRAIGHT ? "STRAIGHT" : "RIGHT")
              << " (Count: " << currentCount << "/" << maxCount << ")" << std::endl;
}

void Generator::displayStatus() const {
    std::cout << "\nTraffic Generator Status\n";
    std::cout << std::string(50, '=') << std::endl;

    for (const auto& [laneId, filepath] : laneFiles) {
        const auto& config = laneConfigs.at(laneId);
        size_t count = countVehiclesInFile(filepath);

        std::cout << "Lane " << std::setw(2) << static_cast<int>(laneId)
                 << " | Vehicles: " << std::setw(2) << count
                 << "/" << std::setw(2) << config.maxVehicles
                 << " | Spawn Rate: " << std::fixed << std::setprecision(1)
                 << (config.spawnRate * 100) << "% | "
                 << "Directions: "
                 << (config.canGoStraight ? "S" : "")
                 << (config.canGoRight ? "R" : "")
                 << std::endl;
    }
    std::cout << std::string(50, '=') << std::endl;
}