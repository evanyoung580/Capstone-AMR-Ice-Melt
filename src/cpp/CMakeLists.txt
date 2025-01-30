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
add_executable(cmd_manager cmd_manager.cpp)
add_executable(telem_manager telem_manager.cpp)

# Include directories
target_include_directories(cmd_manager PRIVATE ${ZMQ_INCLUDE_DIRS})
target_include_directories(telem_manager PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(cmd_manager PRIVATE ${ZMQ_LIBRARIES} nlohmann_json::nlohmann_json)
target_link_libraries(telem_manager PRIVATE ${ZMQ_LIBRARIES} nlohmann_json::nlohmann_json)

# Add compiler definitions and flags
target_compile_definitions(cmd_manager PRIVATE ${ZMQ_CFLAGS_OTHER})
target_compile_definitions(telem_manager PRIVATE ${ZMQ_CFLAGS_OTHER})
