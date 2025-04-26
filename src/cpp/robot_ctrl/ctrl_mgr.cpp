#include "robot_ctrl/ctrl_mgr.hpp"
#include <chrono>
#include <cstdio>
using namespace std::chrono_literals;

ControlManager::ControlManager(const std::string &mcu_dev,
                               int baud,
                               double kp, double ki, double kd,
                               double target_center,
                               double translation_velocity,
                               double disk_velocity,
                               int period_ms,
                               VisionData &vd,
                               std::mutex &m,
                               std::atomic<ManageState> &cs)
    : vision_data_(vd),
      data_mutex_(m),
      control_state_(cs),
      mcu_serial_(mcu_dev, baud),
      kp_(kp), ki_(ki), kd_(kd),
      target_(target_center),
      disk_velocity_(disk_velocity),
      velocity_(translation_velocity),
      control_period_ms_(period_ms)
{}

void ControlManager::start() {
    thr_ = std::thread(&ControlManager::run, this);
}

void ControlManager::join() {
    thr_.join();
}

void ControlManager::run() {
    double integral = 0.0, prev_error = 0.0;

    while (true) {
        ManageState st = control_state_.load();
        double v = 0.0, w = 0.0, d = 0.0;

        if (st == ManageState::RUN) {
            double x = 0.0;
            int count = 0;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (vision_data_.color_detected) {
                    x += vision_data_.color_x;
                    count++;
                }
                if (vision_data_.line_detected) {
                    x += vision_data_.line_x;
                    count++;
                }
            }
            if (count > 0) {
                x /= count;
                double error      = target_ - x;
                integral         += error * (control_period_ms_ / 1000.0);
                double derivative = (error - prev_error) / (control_period_ms_ / 1000.0);
                w = kp_ * error + ki_ * integral + kd_ * derivative;
                v = velocity_;
                d = disk_velocity_;
                prev_error = error;
            }
        }
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "v:%f,w:%f,d:%fX", v, w, d);
        mcu_serial_.writeString(std::string(buf, n));

        std::this_thread::sleep_for(std::chrono::milliseconds(control_period_ms_));
    }
}
