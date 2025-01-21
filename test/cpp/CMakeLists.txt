cmake_minimum_required(VERSION 3.10)
project(local_msgmgr VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find ZeroMQ using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# nlohmann::json is header-only; no linking is needed
find_package(nlohmann_json REQUIRED)

# Add the executable
add_executable(local_msgmgr local_msgmgr.cpp)

# Include directories
target_include_directories(local_msgmgr PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(local_msgmgr PRIVATE ${ZMQ_LIBRARIES} nlohmann_json::nlohmann_json)

# Add compiler definitions and flags
target_compile_definitions(local_msgmgr PRIVATE ${ZMQ_CFLAGS_OTHER})
