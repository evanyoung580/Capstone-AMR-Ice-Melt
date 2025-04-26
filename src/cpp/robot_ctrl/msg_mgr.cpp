#include "robot_ctrl/msg_mgr.hpp"
#include <chrono>
#include <cstring>
using namespace std::chrono_literals;
using json = nlohmann::json;

MessageManager::MessageManager(const std::string &radio_dev,
                               const std::string &mcu_dev,
                               int baud,
                               const std::string &zmq_sub_addr,
                               const std::string &zmq_pub_addr,
                               VisionData &vd,
                               std::mutex &m,
                               std::atomic<ManageState> &cs)
    : ctx_(1),
      vision_sub_(ctx_, ZMQ_SUB),
      vision_pub_(ctx_, ZMQ_PUB),
      radio_serial_(radio_dev, baud),
      mcu_serial_(mcu_dev, baud),
      vision_data_(vd),
      data_mutex_(m),
      control_state_(cs)
{
    vision_sub_.connect(zmq_sub_addr);
    vision_sub_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    vision_pub_.bind(zmq_pub_addr);
}

void MessageManager::start() {
    thr_ = std::thread(&MessageManager::run, this);
}

void MessageManager::join() {
    thr_.join();
}

void MessageManager::run() {
    while (true) {
        // 1) Vision telemetry (ZMQ SUB)
        zmq::message_t msg;
        if (vision_sub_.recv(&msg, ZMQ_DONTWAIT)) {
            auto s = std::string(static_cast<char*>(msg.data()), msg.size());
            try {
                auto js = json::parse(s);
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (js["status"] == "detecting") {
                    if (js.contains("color_det") && js["color_det"]["sw_detected"] == "True") {
                        vision_data_.color_detected = true;
                        vision_data_.color_x = js["color_det"]["sw_pos"]["center"]["x"];
                        vision_data_.color_y = js["color_det"]["sw_pos"]["center"]["y"];
                    } else {
                        vision_data_.color_detected = false;
                    }
                    if (js.contains("line_det") && js["line_det"]["sw_detected"] == "True") {
                        vision_data_.line_detected = true;
                        vision_data_.line_x = js["line_det"]["sw_pos"]["center"]["x"];
                        vision_data_.line_y = js["line_det"]["sw_pos"]["center"]["y"];
                    } else {
                        vision_data_.line_detected = false;
                    }
                } else {
                    vision_data_.color_detected = false;
                    vision_data_.line_detected  = false;
                }
            } catch (...) { /* ignore malformed */ }
        }

        // 2) Radio commands (UART)
        std::string line;
        if (radio_serial_.readLine(line)) {
            try {
                auto js = json::parse(line);
                std::string manage = js.value("manage", "");

                if (manage == "vision") {
                    // Forward to vision program
                    auto out = js.dump();
                    zmq::message_t omsg(out.size());
                    memcpy(omsg.data(), out.data(), out.size());
                    vision_pub_.send(omsg);

                } else if (manage == "control") {
                    // Toggle PID run/stop
                    control_state_ = (js.value("state", "") == "run")
                                         ? ManageState::RUN
                                         : ManageState::STOP;
                    if (control_state_ == ManageState::STOP) {
                        // Immediately send zero velocities
                        mcu_serial_.writeString("v:0.0,w:0.0,d:0.0X");
                    }

                } else if (manage == "set") {
                    // Forward arbitrary UART command to MCU
                    std::string cmd = js.value("state", "");
                    if (!cmd.empty()) {
                        mcu_serial_.writeString(cmd);
                    }
                }
            } catch (...) {
                // ignore parse errors
            }
        }

        // 3) MCU telemetry forwarding
        if (mcu_serial_.readLine(line)) {
            radio_serial_.writeString(line + "\n");
        }

        std::this_thread::sleep_for(10ms);
    }
}
