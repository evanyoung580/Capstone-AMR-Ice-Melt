cmake_minimum_required(VERSION 3.10)
project(robot_ctrl VERSION 1.0 LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Directories
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(SRC_DIR     ${CMAKE_SOURCE_DIR}/src/cpp)

# Find system dependencies
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# Try pkg-config for libzmq
pkg_check_modules(ZMQ libzmq)

# Find JSON and YAML
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
    ${INCLUDE_DIR}
    ${ZMQ_INCLUDE_DIRS}
)

# Library directories
link_directories(
    ${ZMQ_LIBRARY_DIRS}
)

# Sources
file(GLOB_RECURSE ROBOT_SOURCES
    ${SRC_DIR}/robot_ctrl/*.cpp
)
file(GLOB EXTRA_SOURCES
    ${SRC_DIR}/*.cpp
)

add_executable(${PROJECT_NAME}
    ${ROBOT_SOURCES}
    ${EXTRA_SOURCES}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Threads::Threads
        ${ZMQ_LIBRARIES}
        nlohmann_json::nlohmann_json
        yaml-cpp
)

# Installation
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
