cmake_minimum_required(VERSION 3.15)
project(TrafficJunctionSimulator VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add SDL3 installation path
# For Mac, check both Homebrew and manually installed locations
list(APPEND CMAKE_PREFIX_PATH 
     "${CMAKE_SOURCE_DIR}/libs/SDL3_install"
     "/usr/local/opt/sdl3"
     "/opt/homebrew/opt/sdl3"
)

# Find SDL3
find_package(SDL3 REQUIRED)

# Define include directories with proper scope
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Define core source files
set(CORE_SOURCES
    src/core/Vehicle.cpp
    src/core/Lane.cpp
    src/core/TrafficLight.cpp
)

# Define manager source files
set(MANAGER_SOURCES
    src/managers/FileHandler.cpp
    src/managers/TrafficManager.cpp
)

# Define visualization source files
set(VISUALIZATION_SOURCES
    src/visualization/Renderer.cpp
)

# Define utility source files
set(UTILITY_SOURCES
    src/utils/DebugLogger.cpp
    # These are header-only, no implementation files
)

# Define simulator sources
set(SIMULATOR_SOURCES
    src/main.cpp
    ${CORE_SOURCES}
    ${MANAGER_SOURCES}
    ${VISUALIZATION_SOURCES}
    ${UTILITY_SOURCES}
)

# Define traffic generator sources
set(GENERATOR_SOURCES
    src/traffic_generator.cpp
)

# Add executables
add_executable(simulator ${SIMULATOR_SOURCES})
add_executable(traffic_generator ${GENERATOR_SOURCES})

# Link SDL libraries
target_link_libraries(simulator PRIVATE SDL3::SDL3)

# Set include directories for each target
target_include_directories(simulator PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)
target_include_directories(traffic_generator PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

# Mac-specific settings
if(APPLE)
    # Enable ARC and other Mac-specific features if needed
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
    
    # Enable application bundle creation
    set_target_properties(simulator PROPERTIES
        MACOSX_BUNDLE TRUE
    )
    
    # If you need to link against macOS frameworks
    find_library(COCOA_LIBRARY Cocoa)
    if(COCOA_LIBRARY)
        target_link_libraries(simulator PRIVATE ${COCOA_LIBRARY})
    endif()
else()
    # Settings for other platforms
    if(MSVC)
        # MSVC-specific compiler settings
        target_compile_definitions(simulator PRIVATE
            _USE_MATH_DEFINES
            _CRT_SECURE_NO_WARNINGS
        )
        target_compile_definitions(traffic_generator PRIVATE
            _USE_MATH_DEFINES
            _CRT_SECURE_NO_WARNINGS
        )
        # Disable specific warnings
        add_compile_options(
            /wd4100  # Unreferenced formal parameter
            /wd4189  # Local variable is initialized but not referenced
            /wd4244  # Conversion from 'double' to 'float', possible loss of data
            /wd4305  # Truncation from 'double' to 'float'
        )
    else()
        # GCC/Clang settings
        target_compile_options(simulator PRIVATE -Wall -Wextra)
        target_compile_options(traffic_generator PRIVATE -Wall -Wextra)
    endif()
endif()

# Create data directory in build directory
add_custom_command(
    TARGET simulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_BINARY_DIR}/bin/data/lanes
)
add_custom_command(
    TARGET traffic_generator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_BINARY_DIR}/bin/data/lanes
)

# Print configuration summary
message(STATUS "Build configuration:")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "  Platform: ${CMAKE_SYSTEM_NAME}")