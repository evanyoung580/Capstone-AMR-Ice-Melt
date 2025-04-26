#pragma once
#include <atomic>

struct VisionData {
    bool color_detected = false;
    double color_x = 0.0, color_y = 0.0;
    bool line_detected = false;
    double line_x = 0.0, line_y = 0.0;
};

enum class ManageState { STOP, RUN };
