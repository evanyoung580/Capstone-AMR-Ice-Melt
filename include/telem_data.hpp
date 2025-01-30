#pragma once

#include <array>
#include <string>
#include <nlohmann/json.hpp>

// Enum for positions
enum Position { LEFT = 0, CENTER, RIGHT, POSITION_COUNT };

// Typedef for multidimensional arrays
using PositionArray = std::array<std::array<float, 2>, POSITION_COUNT>;

// Struct for vision data
struct VisionData {
    bool detected;
    PositionArray positions;
    float area;
};

// Function prototypes
VisionData parse_color_data(const nlohmann::json& json_data);
VisionData parse_line_data(const nlohmann::json& json_data);
