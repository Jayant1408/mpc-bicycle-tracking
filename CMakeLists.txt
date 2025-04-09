cmake_minimum_required(VERSION 3.10)
project(MPC_Bicycle_Tracking)

set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(include)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Source files
file(GLOB SOURCES "src/*.cpp")

# Executable
add_executable(sim ${SOURCES})

# Link libraries (AFTER defining the executable)
target_link_libraries(sim
    Eigen3::Eigen
    yaml-cpp
)
