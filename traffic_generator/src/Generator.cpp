
// traffic_generator/src/Generator.cpp
#include "../include/Generator.h"



    static const std::string BASE_PATH;

Generator::Generator() : nextVehicleId(1) {
    try {
        std::random_device rd;
        rng.seed(rd());
        lastGenTime = std::chrono::steady_clock::now();

        // Set up data directory directly
        dataDir = (std::filesystem::current_path() / "data" / "lanes").lexically_normal();
        std::cout << "Generator using absolute path: " << dataDir << std::endl;

        std::filesystem::create_directories(dataDir);

        // Initialize lane files with absolute paths
        laneFiles = {
            {LaneId::AL1_INCOMING, (dataDir / "lane_a1.txt").lexically_normal()},
            {LaneId::AL2_PRIORITY, (dataDir / "lane_a2.txt").lexically_normal()},
            {LaneId::AL3_FREELANE, (dataDir / "lane_a3.txt").lexically_normal()},
            {LaneId::BL1_INCOMING, (dataDir / "lane_b1.txt").lexically_normal()},
            {LaneId::BL2_NORMAL,   (dataDir / "lane_b2.txt").lexically_normal()},
            {LaneId::BL3_FREELANE, (dataDir / "lane_b3.txt").lexically_normal()},
            {LaneId::CL1_INCOMING, (dataDir / "lane_c1.txt").lexically_normal()},
            {LaneId::CL2_NORMAL,   (dataDir / "lane_c2.txt").lexically_normal()},
            {LaneId::CL3_FREELANE, (dataDir / "lane_c3.txt").lexically_normal()},
            {LaneId::DL1_INCOMING, (dataDir / "lane_d1.txt").lexically_normal()},
            {LaneId::DL2_NORMAL,   (dataDir / "lane_d2.txt").lexically_normal()},
            {LaneId::DL3_FREELANE, (dataDir / "lane_d3.txt").lexically_normal()}
        };

        // Clear and initialize files
        clearAllFiles();

        // Verify files were created
        for (const auto& [_, filepath] : laneFiles) {
            std::ofstream testWrite(filepath, std::ios::app);
            if (!testWrite) {
                throw std::runtime_error("Cannot write to " + filepath.string());
            }
            testWrite.close();

            std::ifstream testRead(filepath);
            if (!testRead) {
                throw std::runtime_error("Cannot read from " + filepath.string());
            }
            testRead.close();
        }
    } catch (const std::exception& e) {
        std::cerr << "Generator initialization failed: " << e.what() << std::endl;
        throw;
    }
}


void Generator::clearAllFiles() {
    for (const auto& [_, filepath] : laneFiles) {
        try {
            std::ofstream file(filepath, std::ios::trunc);
            if (!file) {
                throw std::runtime_error("Failed to clear file: " + filepath.string());
            }
            std::cout << "Cleared file: " << filepath << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Error clearing file: " << e.what() << std::endl;
            throw;
        }
    }
}

void Generator::initializeLaneSettings() {
    laneSettings.clear();  // Clear existing settings first

    laneSettings[LaneId::AL1_INCOMING] = {0.12, 12, "A1 (West Incoming)"};
    laneSettings[LaneId::AL2_PRIORITY] = {0.15, 15, "A2 (West Priority)"};
    laneSettings[LaneId::AL3_FREELANE] = {0.10, 8, "A3 (West Free)"};
    laneSettings[LaneId::BL1_INCOMING] = {0.12, 12, "B1 (North Incoming)"};
    laneSettings[LaneId::BL2_NORMAL] = {0.12, 12, "B2 (North Normal)"};
    laneSettings[LaneId::BL3_FREELANE] = {0.10, 8, "B3 (North Free)"};
    laneSettings[LaneId::CL1_INCOMING] = {0.12, 12, "C1 (East Incoming)"};
    laneSettings[LaneId::CL2_NORMAL] = {0.12, 12, "C2 (East Normal)"};
    laneSettings[LaneId::CL3_FREELANE] = {0.10, 8, "C3 (East Free)"};
    laneSettings[LaneId::DL1_INCOMING] = {0.12, 12, "D1 (South Incoming)"};
    laneSettings[LaneId::DL2_NORMAL] = {0.12, 12, "D2 (South Normal)"};
    laneSettings[LaneId::DL3_FREELANE] = {0.10, 8, "D3 (South Free)"};
}

Direction Generator::generateRandomDirection() {
    std::uniform_int_distribution<> dist(0, 100);
    int roll = dist(rng);

    // 60% straight, 20% left, 20% right
    if (roll < 60) return Direction::STRAIGHT;
    if (roll < 80) return Direction::LEFT;
    return Direction::RIGHT;
}


void Generator::writeVehicleToFile(const std::filesystem::path& filepath, uint32_t id, Direction dir) {
    std::lock_guard<std::mutex> lock(fileMutex);
    try {
        std::ofstream file(filepath, std::ios::app);
        if (!file) {
            throw std::runtime_error("Cannot open file for writing: " + filepath.string());
        }

        char dirChar;
        switch (dir) {
            case Direction::STRAIGHT: dirChar = 'S'; break;
            case Direction::LEFT:     dirChar = 'L'; break;
            case Direction::RIGHT:    dirChar = 'R'; break;
            default:                  dirChar = 'S'; break;
        }

        std::string data = std::to_string(id) + "," + dirChar + ";\n";
        file << data;
        file.flush();

        std::cout << "Successfully wrote to " << filepath << ": " << data;
        std::cout << "Current file size: " << std::filesystem::file_size(filepath) << " bytes" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error writing to file " << filepath << ": " << e.what() << std::endl;
    }
}

size_t Generator::countVehiclesInFile(const std::filesystem::path& filepath) const {
    try {
        std::ifstream file(filepath.string());
        if (!file) return 0;

        size_t count = 0;
        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty()) count++;
        }
        return count;
    } catch (const std::exception& e) {
        std::cerr << "Error counting vehicles in file: " << e.what() << std::endl;
        return 0;
    }
}

bool Generator::shouldGenerateVehicle(LaneId laneId, size_t currentCount) {
    if (laneSettings.find(laneId) == laneSettings.end()) return false;

    const auto& settings = laneSettings[laneId];
    if (currentCount >= settings.maxVehicles) return false;

    std::uniform_real_distribution<> dist(0.0, 1.0);
    return dist(rng) < settings.spawnProbability;
}

// In Generator.cpp, update the generateTraffic function:
void Generator::generateTraffic() {
    try {
        // Check timing
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - lastGenTime).count();

        // Increased minimum delay between generations to 2 seconds
        if (elapsed < 2000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000 - elapsed));
            return;
        }

        std::uniform_real_distribution<> dist(0.0, 1.0);
        bool anyVehicleGenerated = false;

        for (const auto& [laneId, filepath] : laneFiles) {
            size_t currentVehicles = countVehiclesInFile(filepath);

            double spawnProbability;
            int maxVehicles;

            // Set spawn probabilities and max vehicles per lane
            switch (laneId) {
                case LaneId::AL2_PRIORITY:
                    spawnProbability = 0.15;  // 15% chance for priority lane
                    maxVehicles = 15;
                    break;
                case LaneId::AL3_FREELANE:
                case LaneId::BL3_FREELANE:
                case LaneId::CL3_FREELANE:
                case LaneId::DL3_FREELANE:
                    spawnProbability = 0.1;   // 10% chance for free lanes
                    maxVehicles = 8;
                    break;
                default:
                    spawnProbability = 0.12;  // 12% chance for normal lanes
                    maxVehicles = 12;
                    break;
            }

            // Check if we should generate a vehicle for this lane
            if (currentVehicles < maxVehicles && dist(rng) < spawnProbability) {
                Direction dir = generateRandomDirection();
                writeVehicleToFile(filepath, nextVehicleId++, dir);
                anyVehicleGenerated = true;

                // Log vehicle generation
                std::cout << "Generated vehicle " << (nextVehicleId-1)
                         << " in lane " << static_cast<int>(laneId)
                         << " with direction " << static_cast<int>(dir)
                         << " (Current vehicles: " << currentVehicles << "/"
                         << maxVehicles << ")" << std::endl;
            }
        }

        lastGenTime = std::chrono::steady_clock::now();

        // Add a small delay if no vehicles were generated
        if (!anyVehicleGenerated) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Display current stats periodically
        static int updateCounter = 0;
        if (++updateCounter % 10 == 0) {
            displayStatus();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in traffic generation: " << e.what() << std::endl;
    }
}



void Generator::displayStatus() const {
    std::cout << "\nCurrent Lane Status:\n";
    std::cout << std::string(50, '-') << std::endl;

    for (const auto& [laneId, filepath] : laneFiles) {
        const auto& settings = laneSettings.at(laneId);
        size_t count = countVehiclesInFile(filepath);

        std::cout << std::setw(20) << settings.name << " | "
                 << std::setw(8) << count << "/"
                 << std::setw(3) << settings.maxVehicles
                 << " vehicles | Spawn rate: "
                 << std::fixed << std::setprecision(1)
                 << (settings.spawnProbability * 100.0) << "%\n";
    }
    std::cout << std::string(50, '-') << std::endl;
}