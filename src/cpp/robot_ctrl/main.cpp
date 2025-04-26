#include <iostream>
#include <mutex>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include "robot_ctrl/ctrl_mgr.hpp"
#include "robot_ctrl/msg_mgr.hpp"

int main() {
    // Load config
    YAML::Node cfg = YAML::LoadFile("/home/amrmgr/amr/config/robot_config.yaml");

    // Serial settings
    auto serial = cfg["serial"];
    std::string mcu_port   = serial["mcu_port"].as<std::string>();
    std::string radio_port = serial["radio_port"].as<std::string>();
    int baud               = serial["baud"].as<int>();

    // ZMQ endpoints
    auto zmq_cfg         = cfg["zmq"];
    std::string sub_addr = zmq_cfg["telemetry_sub"].as<std::string>();
    std::string pub_addr = zmq_cfg["commands_pub"].as<std::string>();

    // PID gains
    auto pid_cfg = cfg["pid"];
    double kp     = pid_cfg["kp"].as<double>();
    double ki     = pid_cfg["ki"].as<double>();
    double kd     = pid_cfg["kd"].as<double>();

    // Control params
    auto ctl_cfg           = cfg["control"];
    double target_center   = ctl_cfg["target_center"].as<double>();
    double translation_vel = ctl_cfg["translation_velocity"].as<double>();
    double disk_vel        = ctl_cfg["disk_velocity"].as<double>();
    int period_ms          = ctl_cfg["period_ms"].as<int>();

    // Shared state
    VisionData vision_data;
    std::mutex data_mutex;
    std::atomic<ManageState> control_state{ManageState::STOP};

    // Instantiate managers
    MessageManager msg_mgr(
        radio_port,
        mcu_port,
        baud,
        sub_addr,
        pub_addr,
        vision_data,
        data_mutex,
        control_state
    );

    ControlManager ctrl_mgr(
        mcu_port,
        baud,
        kp, ki, kd,
        target_center,
        translation_vel,
        disk_vel,
        period_ms,
        vision_data,
        data_mutex,
        control_state
    );

    // Start threads
    msg_mgr.start();
    ctrl_mgr.start();

    // Wait forever
    msg_mgr.join();
    ctrl_mgr.join();
    return 0;
}