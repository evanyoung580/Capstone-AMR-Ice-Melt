#pragma once
#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include "robot_ctrl/types.hpp"
#include "robot_ctrl/serial_port.hpp"

class MessageManager {
public:
    MessageManager(const std::string &radio_dev,
                   const std::string &mcu_dev,
                   int baud,
                   const std::string &zmq_sub_addr,
                   const std::string &zmq_pub_addr,
                   VisionData &vd,
                   std::mutex &m,
                   std::atomic<ManageState> &cs);

    void start();
    void join();

private:
    void run();

    zmq::context_t        ctx_;
    zmq::socket_t         vision_sub_, vision_pub_;
    SerialPort            radio_serial_, mcu_serial_;
    VisionData           &vision_data_;
    std::mutex           &data_mutex_;
    std::atomic<ManageState> &control_state_;
    std::thread           thr_;
};