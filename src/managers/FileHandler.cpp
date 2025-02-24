// FileHandler.cpp
#include "managers/FileHandler.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

const std::string FileHandler::BASE_PATH = "data/lanes";

FileHandler::FileHandler() {
    try {
        initializeFileSystem();
    } catch (const std::exception& e) {
        std::cerr << "FileHandler initialization failed: " << e.what() << std::endl;
        throw;
    }
}

void FileHandler::initializeFileSystem() {
    // Setup data directory path
    dataDir = (std::filesystem::current_path() / BASE_PATH).lexically_normal();
    std::cout << "FileHandler using path: " << dataDir << std::endl;

    // Initialize lane file paths
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

    // Create directory structure
    std::filesystem::create_directories(dataDir);

    // Initialize all files
    for (const auto& [laneId, filepath] : laneFiles) {
        if (!std::filesystem::exists(filepath)) {
            std::ofstream file(filepath);
            if (!file) {
                throw std::runtime_error("Cannot create file: " + filepath.string());
            }
        }
        lastReadPositions[filepath] = 0;
        lastCheckTimes[filepath] = std::chrono::steady_clock::now();
    }
}

// In FileHandler.cpp
std::vector<std::pair<LaneId, std::shared_ptr<Vehicle>>> FileHandler::readNewVehicles() {
    std::vector<std::pair<LaneId, std::shared_ptr<Vehicle>>> newVehicles;
    std::lock_guard<std::mutex> lock(fileMutex);

    for (const auto& [laneId, filepath] : laneFiles) {
        try {
            if (!std::filesystem::exists(filepath)) continue;

            std::ifstream file(filepath);
            if (!file) continue;

            std::string line;
            std::vector<std::string> lines;
            while (std::getline(file, line)) {
                if (!line.empty()) {
                    lines.push_back(line);
                }
            }

            // Clear file after reading
            file.close();
            std::ofstream clearFile(filepath, std::ios::trunc);

            // Process lines
            for (const auto& line : lines) {
                size_t commaPos = line.find(',');
                size_t semicolonPos = line.find(';');

                if (commaPos != std::string::npos && semicolonPos != std::string::npos) {
                    try {
                        uint32_t id = std::stoul(line.substr(0, commaPos));
                        char dirChar = line[commaPos + 1];

                        Direction dir;
                        switch (dirChar) {
                            case 'S': dir = Direction::STRAIGHT; break;
                            case 'L': dir = Direction::LEFT; break;
                            case 'R': dir = Direction::RIGHT; break;
                            default: continue;
                        }

                        auto vehicle = std::make_shared<Vehicle>(id, dir, laneId);
                        newVehicles.emplace_back(laneId, vehicle);

                        std::cout << "Read vehicle " << id << " from " << filepath
                                 << " with direction " << static_cast<int>(dir) << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error reading file " << filepath << ": " << e.what() << std::endl;
        }
    }

    return newVehicles;
}

std::shared_ptr<Vehicle> FileHandler::parseVehicleLine(const std::string& line, LaneId laneId) {
    size_t commaPos = line.find(',');
    size_t semicolonPos = line.find(';');

    if (commaPos == std::string::npos || semicolonPos == std::string::npos) {
        return nullptr;
    }

    try {
        uint32_t id = std::stoul(line.substr(0, commaPos));
        char dirChar = line[commaPos + 1];

        Direction dir;
        switch (dirChar) {
            case 'S': dir = Direction::STRAIGHT; break;
            case 'L': dir = Direction::LEFT; break;
            case 'R': dir = Direction::RIGHT; break;
            default: return nullptr;
        }

        return std::make_shared<Vehicle>(id, dir, laneId);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
        return nullptr;
    }
}

void FileHandler::clearLaneFiles() {
    std::lock_guard<std::mutex> lock(fileMutex);

    for (const auto& [_, filepath] : laneFiles) {
        std::ofstream file(filepath, std::ios::trunc);
        lastReadPositions[filepath] = 0;
    }
}

size_t FileHandler::getVehicleCountInFile(LaneId laneId) const {
    auto it = laneFiles.find(laneId);
    if (it == laneFiles.end()) return 0;

    try {
        std::ifstream file(it->second);
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

void FileHandler::handleFileError(const std::string& operation,
                                const std::filesystem::path& filepath,
                                const std::exception& e) const {
    std::cerr << "File " << operation << " error for " << filepath
              << ": " << e.what() << std::endl;
}