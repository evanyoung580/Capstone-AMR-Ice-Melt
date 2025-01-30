#include <iostream>
#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <iomanip>
#include <array>

// Enums for accessing positions
enum Position
{
    LEFT = 0,
    CENTER,
    RIGHT,
    POSITION_COUNT // Keeps track of the number of positions
};

// Typedef for multidimensional arrays
using PositionArray = std::array<std::array<float, 2>, POSITION_COUNT>;

void manage_vision_data(zmq::socket_t &vision_socket)
{
    while (true)
    {
        zmq::message_t message;
        auto result = vision_socket.recv(message, zmq::recv_flags::none);
        std::string msg = std::string(static_cast<char *>(message.data()), message.size());
        nlohmann::json in_msg = nlohmann::json::parse(msg);

        // Extract general status
        std::string status = in_msg["status"];

        // Initialize arrays to store position data
        PositionArray color_positions = {};
        PositionArray line_positions = {};

        // Flags for detection
        bool color_detected = false;
        bool line_detected = false;

        // Area for color detection
        float color_area = 0.0f;

        // Parse color detection data
        if (in_msg["status"] == "detecting" && in_msg.contains("color_det"))
        {
            color_detected = in_msg["color_det"]["sw_detected"] == "True";

            if (color_detected)
            {
                color_positions[LEFT] = {
                    static_cast<float>(in_msg["color_det"]["sw_pos"]["left"]["x"]),
                    static_cast<float>(in_msg["color_det"]["sw_pos"]["left"]["y"])};

                color_positions[CENTER] = {
                    static_cast<float>(in_msg["color_det"]["sw_pos"]["center"]["x"]),
                    static_cast<float>(in_msg["color_det"]["sw_pos"]["center"]["y"])};

                color_positions[RIGHT] = {
                    static_cast<float>(in_msg["color_det"]["sw_pos"]["right"]["x"]),
                    static_cast<float>(in_msg["color_det"]["sw_pos"]["right"]["y"])};

                color_area = static_cast<float>(in_msg["color_det"]["sw_pos"]["area"]);
            }
        }

        // Parse line detection data
        if (in_msg["status"] == "detecting" && in_msg.contains("line_det"))
        {
            line_detected = in_msg["line_det"]["sw_detected"] == "True";

            if (line_detected)
            {
                line_positions[LEFT] = {
                    static_cast<float>(in_msg["line_det"]["sw_pos"]["left"]["x"]),
                    static_cast<float>(in_msg["line_det"]["sw_pos"]["left"]["y"])};

                line_positions[CENTER] = {
                    static_cast<float>(in_msg["line_det"]["sw_pos"]["center"]["x"]),
                    static_cast<float>(in_msg["line_det"]["sw_pos"]["center"]["y"])};

                line_positions[RIGHT] = {
                    static_cast<float>(in_msg["line_det"]["sw_pos"]["right"]["x"]),
                    static_cast<float>(in_msg["line_det"]["sw_pos"]["right"]["y"])};
            }
        }

        // Display data
        system("clear");
        {
            using namespace std;

            cout << "Status: " << status << "\n";
            if (status == "detecting")
            {
                std::array<std::string, 2> det_types = {"Color", "Line"};
                std::array<bool, 2> det_flags = {color_detected, line_detected};
                std::array<PositionArray, 2> det_positions = {color_positions, line_positions};
                std::array<float, 2> det_areas = {color_area, 0.0f}; // Line detection has no area

                for (size_t i = 0; i < det_types.size(); ++i)
                {
                    cout << "\n" << left << setw(6) << det_types[i] << "Detection: ";
                    if (det_flags[i])
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
                    
                    if (det_flags[i])
                    {
                        for (size_t pos = 0; pos < POSITION_COUNT; ++pos)
                        {
                            const char* positions[] = {"Left", "Center", "Right"};
                            cout << left << setw(12) << positions[pos] << "|" 
                                 << setw(10) << fixed << setprecision(3) << det_positions[i][pos][0] << "|" 
                                 << setw(10) << det_positions[i][pos][1] << "\n";
                        }
                        if (i == 0) // Only Color detection has area
                        {
                            cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                            cout << "Area: " << fixed << setprecision(0) << det_areas[i] << "\n";
                        }
                    }
                    else
                    {
                        for (size_t pos = 0; pos < POSITION_COUNT; ++pos)
                        {
                            cout << left << setw(12) << "N/A" << "|" << setw(10) << "N/A" << "|" << setw(10) << "N/A" << "\n";
                        }
                        if (i == 0) // Only Color detection has area
                        {
                            cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                            cout << "N/A\n";
                        }
                    }
                    cout << setfill('-') << setw(40) << "" << "\n" << setfill(' ');
                }
            }
        }
    }
}

int main()
{
    zmq::context_t context(1);
    zmq::socket_t vis_telem_socket(context, ZMQ_PULL);
    vis_telem_socket.connect("ipc:///home/amrmgr/amr/tmp/vision_telem");

    manage_vision_data(vis_telem_socket);

    return 0;
}
