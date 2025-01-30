#include <iostream>
#include <fstream>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <thread>
#include <cstdlib>

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
                    {"manage", {{"state", "calibrate"}}}};
                std::cout << out_msg.dump(4) << "\n\n";
            }
            else if (cmd == "detect")
            {
                out_msg = {
                    {"manage", {{"state", "detect"}}}};

                std::cout << out_msg.dump(4) << "\n\n";
            }
            else if (cmd == "idle")
            {
                out_msg = {
                    {"manage", {{"state", "idle"}}}};
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
    while (true)
    {
        zmq::message_t message;
        auto result = vision_socket.recv(message, zmq::recv_flags::none);
        std::string msg = std::string(static_cast<char *>(message.data()), message.size());
        nlohmann::json in_msg = nlohmann::json::parse(msg);

        system("clear");
        {
            using namespace std;

            cout << "Status: " << in_msg["status"] << "\n";
            if (in_msg["status"] == "detecting")
            {
                // Output color_det section
                cout << left << setw(6) << "\nColor" << "Detection: ";
                if (in_msg["color_det"]["sw_detected"] == "True")
                {
                    cout << "True\n";
                }
                else
                {
                    cout << "False\n";
                }
                cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                cout << left << setw(12) << "Position" << "|" << setw(10) << "x" << "|" << setw(10) << "y" << "\n";
                cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                if (in_msg["color_det"]["sw_detected"] == "True")
                {
                    cout << left << setw(12) << "Left" << "|" << setw(10) << fixed << setprecision(3) 
                        << static_cast<float>(in_msg["color_det"]["sw_pos"]["left"]["x"]) << "|" 
                        << setw(10) << static_cast<float>(in_msg["color_det"]["sw_pos"]["left"]["y"]) << "\n";
                    cout << left << setw(12) << "Center" << "|" << setw(10) 
                        << static_cast<float>(in_msg["color_det"]["sw_pos"]["center"]["x"]) << "|" 
                        << setw(10) << static_cast<float>(in_msg["color_det"]["sw_pos"]["center"]["y"]) << "\n";
                    cout << left << setw(12) << "Right" << "|" << setw(10) 
                        << static_cast<float>(in_msg["color_det"]["sw_pos"]["right"]["x"]) << "|" 
                        << setw(10) << static_cast<float>(in_msg["color_det"]["sw_pos"]["right"]["y"]) << "\n";
                    cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                    cout << "Area: " << fixed << setprecision(3) << in_msg["color_det"]["sw_pos"]["area"] << "\n";
                }
                else
                {
                    cout << left << setw(12) << "Left" << "|" << setw(10) << "N/A" << "|" << setw(10) << "N/A" << "\n";
                    cout << left << setw(12) << "Center" << "|" << setw(10) << "N/A" << "|" << setw(10) << "N/A" << "\n";
                    cout << left << setw(12) << "Right" << "|" << setw(10) << "N/A" << "|" << setw(10) << "N/A" << "\n";
                    cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                }

                // Output line_det section
                cout << left << setw(6) << "\nLine" << "Detection: ";
                if (in_msg["line_det"]["sw_detected"] == "True")
                {
                    cout << "True\n";
                }
                else
                {
                    cout << "False\n";
                }
                cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                cout << left << setw(12) << "Position" << "|" << setw(10) << "x" << "|" << setw(10) << "y" << "\n";
                cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                if (in_msg["line_det"]["sw_detected"] == "True")
                {
                    cout << left << setw(12) << "Left" << "|" << setw(10) << fixed << setprecision(3) 
                        << static_cast<float>(in_msg["line_det"]["sw_pos"]["left"]["x"]) << "|" 
                        << setw(10) << static_cast<float>(in_msg["line_det"]["sw_pos"]["left"]["y"]) << "\n";
                    cout << left << setw(12) << "Center" << "|" << setw(10) 
                        << static_cast<float>(in_msg["line_det"]["sw_pos"]["center"]["x"]) << "|" 
                        << setw(10) << static_cast<float>(in_msg["line_det"]["sw_pos"]["center"]["y"]) << "\n";
                    cout << left << setw(12) << "Right" << "|" << setw(10) 
                        << static_cast<float>(in_msg["line_det"]["sw_pos"]["right"]["x"]) << "|" 
                        << setw(10) << static_cast<float>(in_msg["line_det"]["sw_pos"]["right"]["y"]) << "\n";
                    cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                }
                else
                {
                    cout << left << setw(12) << "Left" << "|" << setw(10) << "N/A" << "|" << setw(10) << "N/A" << "\n";
                    cout << left << setw(12) << "Center" << "|" << setw(10) << "N/A" << "|" << setw(10) << "N/A" << "\n";
                    cout << left << setw(12) << "Right" << "|" << setw(10) << "N/A" << "|" << setw(10) << "N/A" << "\n";
                    cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                }
            }
        }
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
