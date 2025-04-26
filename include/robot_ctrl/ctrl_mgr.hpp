#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include "robot_ctrl/types.hpp"
#include "robot_ctrl/serial_port.hpp"

class ControlManager
{
public:
    ControlManager(const std::string &mcu_dev,
                   int baud,
                   double kp, double ki, double kd,
                   double target_center,
                   double translation_velocity,
                   double disk_velocity,
                   int period_ms,
                   VisionData &vd,
                   std::mutex &m,
                   std::atomic<ManageState> &cs);

    void start();
    void join();

private:
    void run();

    VisionData &vision_data_;
    std::mutex &data_mutex_;
    std::atomic<ManageState> &control_state_;
    SerialPort mcu_serial_;
    double kp_, ki_, kd_;
    double target_, velocity_, disk_velocity_;
    int control_period_ms_;
    std::thread thr_;
};