#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

// Constants
const std::string DATA_DIR = "data/lanes";

// Basic structure to represent a vehicle
struct SimpleVehicle {
    std::string id;
    char lane;
    int laneNumber;
    bool isEmergency;
    float x, y;
};

// Simple console logging
void log_message(const std::string& msg) {
    std::cout << "[Simulator] " << msg << std::endl;

    // Also log to file
    std::ofstream log("simulator_debug.log", std::ios::app);
    if (log.is_open()) {
        log << "[Simulator] " << msg << std::endl;
        log.close();
    }
}

// Ensure data directories exist
bool ensure_directories() {
    try {
        if (!fs::exists(DATA_DIR)) {
            fs::create_directories(DATA_DIR);
            log_message("Created directory: " + DATA_DIR);
        }
        return true;
    } catch (const std::exception& e) {
        log_message("Error creating directories: " + std::string(e.what()));
        return false;
    }
}

// Read vehicles from lane files
std::vector<SimpleVehicle> read_vehicles() {
    std::vector<SimpleVehicle> vehicles;

    for (char laneId : {'A', 'B', 'C', 'D'}) {
        std::string filePath = DATA_DIR + "/lane" + laneId + ".txt";
        std::ifstream file(filePath);

        if (!file.is_open()) {
            continue;
        }

        std::vector<std::string> lines;
        std::string line;

        while (std::getline(file, line)) {
            if (!line.empty()) {
                lines.push_back(line);
            }
        }
        file.close();

        // Clear the file
        std::ofstream clearFile(filePath, std::ios::trunc);
        clearFile.close();

        // Process lines
        for (const auto& line : lines) {
            size_t pos = line.find(":");
            if (pos == std::string::npos) continue;

            std::string vehicleId = line.substr(0, pos);
            char lane = line[pos + 1];

            // Extract info from ID
            int laneNumber = 2; // Default
            if (vehicleId.find("L1") != std::string::npos) {
                laneNumber = 1;
            } else if (vehicleId.find("L3") != std::string::npos) {
                laneNumber = 3;
            }

            bool isEmergency = (vehicleId.find("EMG") != std::string::npos);

            // Add to vehicles list (position doesn't matter in console)
            SimpleVehicle vehicle = {vehicleId, lane, laneNumber, isEmergency, 0, 0};
            vehicles.push_back(vehicle);

            log_message("Read vehicle: " + vehicleId + " on lane " + lane +
                       (isEmergency ? " (EMERGENCY)" : ""));
        }
    }

    return vehicles;
}

// Simulate traffic flow (just for demonstration)
void update_simulation(std::vector<SimpleVehicle>& vehicles) {
    // Remove some vehicles to simulate them passing through
    if (!vehicles.empty()) {
        // Remove the first vehicle
        log_message("Vehicle " + vehicles[0].id + " passed through the intersection");
        vehicles.erase(vehicles.begin());
    }
}

// Print the current state of the simulation
void print_simulation_state(const std::vector<SimpleVehicle>& vehicles) {
    log_message("Current simulation state:");
    log_message("Total vehicles: " + std::to_string(vehicles.size()));

    // Count vehicles by lane
    int countA = 0, countB = 0, countC = 0, countD = 0;
    for (const auto& v : vehicles) {
        switch (v.lane) {
            case 'A': countA++; break;
            case 'B': countB++; break;
            case 'C': countC++; break;
            case 'D': countD++; break;
        }
    }

    log_message("Vehicles by lane:");
    log_message("  Lane A: " + std::to_string(countA));
    log_message("  Lane B: " + std::to_string(countB));
    log_message("  Lane C: " + std::to_string(countC));
    log_message("  Lane D: " + std::to_string(countD));

    // Print first few vehicles in each lane
    log_message("First few vehicles in each lane:");
    for (char lane : {'A', 'B', 'C', 'D'}) {
        std::string laneVehicles = "  Lane " + std::string(1, lane) + ": ";
        int count = 0;

        for (const auto& v : vehicles) {
            if (v.lane == lane && count < 3) {
                laneVehicles += v.id + " ";
                count++;
            }
        }

        if (count == 0) {
            laneVehicles += "(empty)";
        }

        log_message(laneVehicles);
    }
}

// Main function
int main() {
    try {
        // Initialize
        log_message("Starting console traffic simulator");

        // Ensure data directory exists
        if (!ensure_directories()) {
            log_message("Failed to create data directories");
            return 1;
        }

        // Main simulation loop
        std::vector<SimpleVehicle> vehicles;
        bool running = true;
        int iteration = 0;

        while (running && iteration < 60) { // Run for 60 iterations
            // Read new vehicles
            std::vector<SimpleVehicle> newVehicles = read_vehicles();
            if (!newVehicles.empty()) {
                log_message("Added " + std::to_string(newVehicles.size()) + " new vehicles");
                vehicles.insert(vehicles.end(), newVehicles.begin(), newVehicles.end());
            }

            // Update simulation
            update_simulation(vehicles);

            // Print current state
            print_simulation_state(vehicles);

            // Wait for a second before next iteration
            log_message("Waiting for next iteration...");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            iteration++;
        }

        log_message("Simulation complete. Total iterations: " + std::to_string(iteration));
        return 0;
    }
    catch (const std::exception& e) {
        log_message("Unhandled exception: " + std::string(e.what()));
        return 1;
    }
    catch (...) {
        log_message("Unknown exception");
        return 1;
    }
}
