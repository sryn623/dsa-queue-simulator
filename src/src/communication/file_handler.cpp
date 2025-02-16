// file_handler.cpp
#include "file_handler.h"
#include <filesystem>
#include <fstream>
#include <sstream>

FileHandler::FileHandler(const std::string &directory)
    : m_directory(directory) {
  initializeDirectory();
}

FileHandler::~FileHandler() {
    cleanup();  // Make sure cleanup() is also implemented
}
void FileHandler::initializeDirectory() {
  namespace fs = std::filesystem;

  // Create directory if it doesn't exist
  if (!fs::exists(m_directory)) {
    fs::create_directories(m_directory);
  }

  // Clear existing files
  for (const auto &entry : fs::directory_iterator(m_directory)) {
    fs::remove(entry.path());
  }
}


// src/communication/file_handler.cpp
void FileHandler::writeVehicleData(const Vehicle& vehicle) {
    std::string fileName = getLaneFileName(vehicle.getLane());
    std::string fullPath = m_directory + "/" + fileName;

    std::ofstream file(fullPath, std::ios::app);
    if (file.is_open()) {
        file << vehicle.getId() << ","
             << vehicle.getX() << ","
             << vehicle.getY() << ","
             << (vehicle.isPriority() ? "1" : "0") << "\n";
    }
}


// src/communication/file_handler.cpp

// Add this implementation to your existing file_handler.cpp
void FileHandler::cleanup() {
    // This method should clean up any files created during operation
    namespace fs = std::filesystem;

    // Delete all files in the traffic data directory
    for (const auto& entry : fs::directory_iterator(m_directory)) {
        try {
            fs::remove(entry.path());
        } catch (const std::exception& e) {
            // Log error if file deletion fails
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                "Failed to delete file %s: %s",
                entry.path().string().c_str(),
                e.what());
        }
    }
}

LaneId FileHandler::getLaneFromFileName(const std::filesystem::path& filename) const {
    std::string name = filename.string();
    if (name.find("a1") != std::string::npos) return LaneId::AL1_INCOMING;
    if (name.find("a2") != std::string::npos) return LaneId::AL2_PRIORITY;
    if (name.find("a3") != std::string::npos) return LaneId::AL3_FREELANE;
    // Add other lane mappings
    if (name.find("b1") != std::string::npos) return LaneId::BL1_INCOMING;
    if (name.find("b2") != std::string::npos) return LaneId::BL2_PRIORITY;
    if (name.find("b3") != std::string::npos) return LaneId::BL3_FREELANE;
    // Add the rest of the lanes...
    return LaneId::AL1_INCOMING;  // Default return
}

void FileHandler::writeVehicleData(const Vehicle &vehicle) {
  std::string fileName = getLaneFileName(vehicle.getLane());
  std::string fullPath = m_directory + "/" + fileName;

  // Write vehicle data in CSV format: id,isPriority,x,y
  std::ofstream file(fullPath, std::ios::app);
  if (file.is_open()) {
    file << vehicle.getId() << "," << (vehicle.isPriority() ? "1" : "0") << ","
         << vehicle.getX() << "," << vehicle.getY() << "\n";
  }
}

std::vector<Vehicle *> FileHandler::readVehicleData(SDL_Renderer *renderer) {
  std::vector<Vehicle *> vehicles;
  namespace fs = std::filesystem;

  // Read all lane files
  for (const auto &entry : fs::directory_iterator(m_directory)) {
    std::ifstream file(entry.path());
    std::string line;

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string item;
      std::vector<std::string> data;

      // Parse CSV line
      while (std::getline(ss, item, ',')) {
        data.push_back(item);
      }

      if (data.size() == 4) {
        int id = std::stoi(data[0]);
        LaneId lane = getLaneFromFileName(entry.path().filename());
        Vehicle *vehicle = new Vehicle(renderer, id, lane);
        vehicle->setPosition(std::stof(data[2]), std::stof(data[3]));
        vehicles.push_back(vehicle);
      }
    }

    // Clear file after reading
    std::ofstream clearFile(entry.path(), std::ios::trunc);
  }

  return vehicles;
}




std::string FileHandler::getLaneFileName(LaneId lane) const {
  std::string prefix = "lane_";
  switch (lane) {
  case LaneId::AL1_INCOMING:
    return prefix + "a1.txt";
  case LaneId::AL2_PRIORITY:
    return prefix + "a2.txt";
  case LaneId::AL3_FREELANE:
    return prefix + "a3.txt";
  // ... handle other lanes
  default:
    return prefix + "unknown.txt";
  }
}