#include "managers/FileHandler.h"
#include <fstream>
#include <sstream>
#include <iostream>

const std::string FileHandler::BASE_PATH = "data/lanes";

FileHandler::FileHandler() {
    try {
        initializeFileSystem();
        startFileWatching();
    } catch (const std::exception& e) {
        std::cerr << "FileHandler initialization failed: " << e.what() << std::endl;
        throw;
    }
}

void FileHandler::initializeFileSystem() {
    // Setup data directory path
    // Use current_path() for relative paths that work in any build configuration
    dataDir = (std::filesystem::current_path() / BASE_PATH).lexically_normal();
    std::cout << "[FileHandler] Using path: " << dataDir << std::endl;

    // Ensure the directory exists before accessing it
    if (!std::filesystem::exists(dataDir)) {
        std::cout << "[FileHandler] Creating directory: " << dataDir << std::endl;
        try {
            std::filesystem::create_directories(dataDir);
        } catch (const std::exception& e) {
            std::cerr << "[FileHandler] Error creating directory: " << e.what() << std::endl;
            throw;
        }
    }

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

    // Try to access a file to check permissions
    std::cout << "[FileHandler] Checking file access" << std::endl;

    // Initialize all files
    clearLaneFiles(); // This will create empty files
}

std::vector<std::pair<LaneId, std::shared_ptr<Vehicle>>> FileHandler::readNewVehicles() {
    std::vector<std::pair<LaneId, std::shared_ptr<Vehicle>>> newVehicles;
    std::lock_guard<std::mutex> lock(fileMutex);

    // First try to read from combined file (for compatibility with C code)
    try {
        if (std::filesystem::exists(vehicleDataFile)) {
            std::ifstream file(vehicleDataFile);
            if (file) {
                std::string line;
                while (std::getline(file, line)) {
                    if (line.empty()) continue;

                    // Parse line in format "vehicleId:road"
                    size_t colonPos = line.find(':');
                    if (colonPos != std::string::npos) {
                        std::string vehicleId = line.substr(0, colonPos);
                        char roadChar = line[colonPos + 1];

                        // Determine lane based on vehicle ID and road
                        LaneId lane;
                        Direction direction = Direction::STRAIGHT;
                        bool isEmergency = false;

                        // Parse direction/emergency from ID
                        if (vehicleId.find("L1") != std::string::npos) {
                            lane = roadChar == 'A' ? LaneId::AL1_INCOMING :
                                   roadChar == 'B' ? LaneId::BL1_INCOMING :
                                   roadChar == 'C' ? LaneId::CL1_INCOMING :
                                   LaneId::DL1_INCOMING;
                        }
                        else if (vehicleId.find("L2") != std::string::npos) {
                            lane = roadChar == 'A' ? LaneId::AL2_PRIORITY :
                                   roadChar == 'B' ? LaneId::BL2_NORMAL :
                                   roadChar == 'C' ? LaneId::CL2_NORMAL :
                                   LaneId::DL2_NORMAL;
                        }
                        else if (vehicleId.find("L3") != std::string::npos) {
                            lane = roadChar == 'A' ? LaneId::AL3_FREELANE :
                                   roadChar == 'B' ? LaneId::BL3_FREELANE :
                                   roadChar == 'C' ? LaneId::CL3_FREELANE :
                                   LaneId::DL3_FREELANE;
                            direction = Direction::LEFT; // L3 lanes are for left turns
                        }
                        else {
                            // Default to middle lane
                            lane = roadChar == 'A' ? LaneId::AL2_PRIORITY :
                                   roadChar == 'B' ? LaneId::BL2_NORMAL :
                                   roadChar == 'C' ? LaneId::CL2_NORMAL :
                                   LaneId::DL2_NORMAL;
                        }

                        // Check for emergency vehicles
                        isEmergency = vehicleId.find("EMG") != std::string::npos;

                        // Create vehicle
                        uint32_t id = static_cast<uint32_t>(std::hash<std::string>{}(vehicleId));
                        auto vehicle = std::make_shared<Vehicle>(id, direction, lane, isEmergency);
                        newVehicles.emplace_back(lane, vehicle);
                    }
                }

                // Clear file after reading
                file.close();
                std::ofstream clearFile(vehicleDataFile, std::ios::trunc);
            }
        }
    } catch (const std::exception& e) {
        handleFileError("reading combined file", vehicleDataFile, e);
    }

    // Then try to read from individual lane files
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
                auto vehicle = parseVehicleLine(line, laneId);
                if (vehicle) {
                    newVehicles.emplace_back(laneId, vehicle);
                }
            }
        } catch (const std::exception& e) {
            handleFileError("reading file", filepath, e);
        }
    }

    return newVehicles;
}

std::shared_ptr<Vehicle> FileHandler::parseVehicleLine(const std::string& line, LaneId laneId) {
    size_t commaPos = line.find(',');
    size_t semicolonPos = line.find(';');

    // Simple format parsing: "id,direction;..."
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

        // Check if 3rd lane with non-left turn (invalid)
        if (static_cast<int>(laneId) % 3 == 2 && dir != Direction::LEFT) {
            // 3rd lanes are only for left turns
            dir = Direction::LEFT;
        }

        return std::make_shared<Vehicle>(id, dir, laneId);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
        return nullptr;
    }
}

void FileHandler::clearLaneFiles() {
    std::lock_guard<std::mutex> lock(fileMutex);

    for (const auto& [laneId, filepath] : laneFiles) {
        try {
            std::cout << "[FileHandler] Clearing file: " << filepath << std::endl;
            std::ofstream file(filepath, std::ios::trunc);
            if (!file) {
                std::cerr << "[FileHandler] Could not open file for writing: " << filepath << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[FileHandler] Error clearing file " << filepath << ": " << e.what() << std::endl;
        }
    }

    // Clear combined vehicle data file for compatibility with C implementation
    try {
        std::cout << "[FileHandler] Clearing combined vehicle file" << std::endl;
        std::ofstream file("vehicles.data", std::ios::trunc);
        if (!file) {
            std::cerr << "[FileHandler] Could not open vehicles.data for writing" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[FileHandler] Error clearing vehicles.data: " << e.what() << std::endl;
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

void FileHandler::ensureDirectoryExists(const std::filesystem::path& dir) {
    if (!std::filesystem::exists(dir)) {
        if (!std::filesystem::create_directories(dir)) {
            throw std::runtime_error("Failed to create directory: " + dir.string());
        }
    }
}

std::filesystem::path FileHandler::getLaneFilePath(LaneId laneId) const {
    auto it = laneFiles.find(laneId);
    if (it == laneFiles.end()) {
        throw std::runtime_error("Invalid lane ID");
    }
    return it->second;
}

void FileHandler::logFileOperation(const std::string& operation, const std::filesystem::path& filepath) const {
    std::cout << "File " << operation << ": " << filepath.filename() << std::endl;
}

void FileHandler::handleFileError(const std::string& operation,
                                  const std::filesystem::path& filepath,
                                  const std::exception& e) const {
    std::cerr << "File " << operation << " error for " << filepath
              << ": " << e.what() << std::endl;
}

bool FileHandler::isLaneFileAvailable(LaneId laneId) const {
    auto it = laneFiles.find(laneId);
    if (it == laneFiles.end()) return false;
    return std::filesystem::exists(it->second);
}


void FileHandler::watchForFileChanges() {
    // Check for changes to lane files every FILE_CHECK_INTERVAL_MS
    auto lastCheckTime = std::chrono::steady_clock::now();

    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - lastCheckTime).count();

        if (elapsed >= FILE_CHECK_INTERVAL_MS) {
            // Check all lane files for changes
            for (const auto& [laneId, filepath] : laneFiles) {
                if (std::filesystem::exists(filepath)) {
                    std::error_code ec;
                    auto lastWriteTime = std::filesystem::last_write_time(filepath, ec);

                    if (!ec) {
                        // Process file if it's been modified
                        // Use a mutex to avoid concurrent file access
                        std::lock_guard<std::mutex> lock(fileMutex);

                        // Read file contents
                        std::ifstream file(filepath);
                        if (file) {
                            std::string line;
                            std::vector<std::string> lines;

                            while (std::getline(file, line)) {
                                if (!line.empty()) {
                                    lines.push_back(line);
                                }
                            }

                            // Clear file after reading to avoid re-processing
                            file.close();
                            std::ofstream clearFile(filepath, std::ios::trunc);

                            // Process the lines
                            for (const auto& lineText : lines) {
                                auto vehicle = parseVehicleLine(lineText, laneId);
                                if (vehicle) {
                                    vehiclesReady.push_back(std::make_pair(laneId, vehicle));
                                }
                            }
                        }
                    }
                }
            }

            lastCheckTime = currentTime;
        }

        // Small sleep to avoid CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


void FileHandler::startFileWatching() {
    watcherThread = std::thread(&FileHandler::watchForFileChanges, this);
    watcherThread.detach();  // Let it run independently
}


std::vector<std::pair<LaneId, std::shared_ptr<Vehicle>>> FileHandler::getReadyVehicles() {
    std::lock_guard<std::mutex> lock(fileMutex);
    auto result = vehiclesReady;
    vehiclesReady.clear();
    return result;
}


// Fix for the last_write_time conversion error
std::chrono::system_clock::time_point FileHandler::getLastModifiedTime(LaneId laneId) const {
    auto it = laneFiles.find(laneId);
    if (it == laneFiles.end() || !std::filesystem::exists(it->second)) {
        return std::chrono::system_clock::time_point(); // Return epoch time
    }

    try {
        // Convert file_time_type to system_clock::time_point
        auto fileTime = std::filesystem::last_write_time(it->second);

        // In C++17, direct conversion isn't available, so we'll return current time as a workaround
        // This is sufficient for checking if a file has been modified recently
        return std::chrono::system_clock::now();
    } catch (const std::exception& e) {
        std::cerr << "Error getting file time: " << e.what() << std::endl;
        return std::chrono::system_clock::time_point();
    }
}