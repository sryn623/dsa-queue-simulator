#include "managers/FileHandler.h"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iostream>



const std::string FileHandler::BASE_PATH = "data/lanes";

// FileHandler Constructor
FileHandler::FileHandler() {
    try {
        // Get absolute path for data directory
        dataDir = (std::filesystem::current_path() / BASE_PATH).lexically_normal();
        std::cout << "FileHandler using absolute path: " << dataDir << std::endl;

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

        // Create data directory if it doesn't exist
        std::filesystem::create_directories(dataDir);

        // Verify all files exist and are readable
        for (const auto& [laneId, filepath] : laneFiles) {
            std::cout << "Checking file for lane " << static_cast<int>(laneId)
                     << ": " << filepath << std::endl;

            // Create file if it doesn't exist
            if (!std::filesystem::exists(filepath)) {
                std::cout << "File does not exist, creating: " << filepath << std::endl;
                std::ofstream createFile(filepath);
                if (!createFile) {
                    throw std::runtime_error("Cannot create file: " + filepath.string());
                }
                createFile.close();
            }

            // Verify read access
            std::ifstream testRead(filepath);
            if (!testRead) {
                throw std::runtime_error("Cannot read from " + filepath.string());
            }
            testRead.close();

            // Initialize read position
            lastReadPositions[filepath] = 0;

            std::cout << "Successfully verified file: " << filepath << std::endl;
        }

        std::cout << "FileHandler successfully initialized" << std::endl;
        std::cout << "Data directory: " << dataDir << std::endl;
        std::cout << "Number of lanes: " << laneFiles.size() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "FileHandler initialization failed: " << e.what() << std::endl;
        throw;
    }
}// src/managers/FileHandler.cpp
std::vector<std::pair<LaneId, std::shared_ptr<Vehicle>>> FileHandler::readNewVehicles() {
    std::vector<std::pair<LaneId, std::shared_ptr<Vehicle>>> newVehicles;

    std::cout << "\n=== Reading Vehicle Files ===" << std::endl;

    for (const auto& [laneId, filepath] : laneFiles) {
        std::cout << "Checking file: " << filepath << std::endl;

        if (!std::filesystem::exists(filepath)) {
            std::cerr << "File does not exist: " << filepath << std::endl;
            continue;
        }

        std::ifstream file(filepath.string(), std::ios::in);
        if (!file) {
            std::cerr << "Cannot open file: " << filepath << std::endl;
            continue;
        }

        // Get file size
        file.seekg(0, std::ios::end);
        int64_t fileSize = file.tellg();
        std::cout << "File size: " << fileSize << " bytes" << std::endl;
        std::cout << "Last read position: " << lastReadPositions[filepath] << std::endl;

        if (fileSize > lastReadPositions[filepath]) {
            file.seekg(lastReadPositions[filepath]);
            std::string line;

            while (std::getline(file, line)) {
                std::cout << "Reading line: " << line << std::endl;

                // Skip empty lines
                if (line.empty()) continue;

                // Parse vehicle data (format: "ID,Direction;")
                size_t commaPos = line.find(',');
                size_t semicolonPos = line.find(';');

                if (commaPos != std::string::npos && semicolonPos != std::string::npos) {
                    try {
                        // Parse ID
                        uint32_t id = std::stoul(line.substr(0, commaPos));

                        // Parse direction
                        char dirChar = line[commaPos + 1];
                        Direction dir;
                        switch (dirChar) {
                            case 'S': dir = Direction::STRAIGHT; break;
                            case 'L': dir = Direction::LEFT; break;
                            case 'R': dir = Direction::RIGHT; break;
                            default:
                                std::cerr << "Invalid direction character: " << dirChar << std::endl;
                                continue;
                        }

                        std::cout << "Creating vehicle: ID=" << id
                                 << ", Direction=" << static_cast<int>(dir)
                                 << ", Lane=" << static_cast<int>(laneId) << std::endl;

                        auto vehicle = std::make_shared<Vehicle>(id, dir, laneId);
                        newVehicles.emplace_back(laneId, vehicle);
                    } catch (const std::exception& e) {
                        std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
                    }
                } else {
                    std::cerr << "Invalid line format: " << line << std::endl;
                }
            }

            lastReadPositions[filepath] = file.tellg();
            std::cout << "Updated read position to: " << lastReadPositions[filepath] << std::endl;
        }
    }

    std::cout << "Found " << newVehicles.size() << " new vehicles" << std::endl;
    return newVehicles;
}


std::vector<std::shared_ptr<Vehicle>> FileHandler::parseVehicleData(
    const std::string& data, LaneId laneId) {
    std::vector<std::shared_ptr<Vehicle>> vehicles;
    std::stringstream ss(data);
    std::string vehicleData;

    while (std::getline(ss, vehicleData, ';')) {
        if (vehicleData.empty()) continue;

        // Expected format: "id,direction"
        std::stringstream vehicleSS(vehicleData);
        std::string idStr, dirStr;

        if (std::getline(vehicleSS, idStr, ',') &&
            std::getline(vehicleSS, dirStr, ',')) {
            try {
                uint32_t id = std::stoul(idStr);
                Direction dir;

                if (dirStr == "S") dir = Direction::STRAIGHT;
                else if (dirStr == "L") dir = Direction::LEFT;
                else if (dirStr == "R") dir = Direction::RIGHT;
                else continue;

                vehicles.push_back(std::make_shared<Vehicle>(id, dir, laneId));
            } catch (...) {
                // Skip invalid data
                continue;
            }
        }
    }

    return vehicles;
}

void FileHandler::clearLaneFiles() {
    for (const auto& [_, filepath] : laneFiles) {
        std::ofstream file(filepath, std::ios::trunc);
        lastReadPositions[filepath] = 0;
    }
}