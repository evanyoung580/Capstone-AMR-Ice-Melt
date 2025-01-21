#include <iostream>
#include <fstream>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <thread>

void send_commands(zmq::socket_t &vision_socket)
{
    while (true)
    {
        std::string ctrl, cmd;
        std::cout << "Enter ctrl cmd: ";
        std::cin >> ctrl >> cmd;
        nlohmann::json out_msg;

        if (ctrl == "vision")
        {
            if (cmd == "calibrate")
            {
                out_msg = {
                    {"manage", "calibrate"},
                };
                std::cout << out_msg.dump(4) << "\n\n";
            }
            else if (cmd == "detect")
            {
                out_msg = {
                    {"manage", "detect"},
                };
                std::cout << out_msg.dump(4) << "\n\n";
            }
            else if (cmd == "idle")
            {
                nlohmann::json out_msg = {
                    {"manage", "idle"},
                };
                std::cout << out_msg.dump(4) << "\n\n";
            }
            else
            {
                std::cout << "cmd not valid" << "\n\n";
            }

            zmq::message_t message(out_msg.dump());
            vision_socket.send(message, zmq::send_flags::none);
        }
        else
        {
            std::cout << "ctrl not valid" << "\n\n";
        }
    }
}

void manage_vision_data(zmq::socket_t &vision_socket)
{
    std::ofstream pipe("data_monitor");
    while (true)
    {
        zmq::message_t message;
        auto result = vision_socket.recv(message, zmq::recv_flags::none);
        std::string msg = std::string(static_cast<char *>(message.data()), message.size());
        nlohmann::json in_msg = nlohmann::json::parse(msg);
        pipe << msg << "\n";
        std::cout << in_msg.dump(4) << std::endl;  // Print the received message
    }
}

int main()
{
    zmq::context_t cmd_context(1);
    zmq::context_t telem_context(1);
    zmq::socket_t vis_cmd_socket(cmd_context, ZMQ_PUSH);
    vis_cmd_socket.bind("ipc:///home/amrmgr/amr/tmp/vision_cmd");
    zmq::socket_t vis_telem_socket(telem_context, ZMQ_PULL);
    vis_telem_socket.connect("ipc:///home/amrmgr/amr/tmp/vision_telem");

    std::thread send_com_thread(send_commands, std::ref(vis_cmd_socket));
    std::thread vis_data_thread(manage_vision_data, std::ref(vis_telem_socket));

    send_com_thread.join();
    vis_data_thread.join();

    return 0;
}
