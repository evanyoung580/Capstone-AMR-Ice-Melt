// #include <iostream>
// #include <nlohmann/json.hpp>
// #include <zmq.hpp>

// void send_commands(zmq::socket_t &vision_socket)
// {
//     while (true)
//     {
//         std::string ctrl, cmd;
//         std::cout << "Enter ctrl cmd: ";
//         std::cin >> ctrl >> cmd;
//         nlohmann::json out_msg;

//         if (ctrl == "vision")
//         {
//             if (cmd == "calibrate")
//                 out_msg = {{"manage", "vision"}, {"state", "calibrate"}};
//             else if (cmd == "detect")
//                 out_msg = {{"manage", "vision"}, {"state", "detect"}};
//             else if (cmd == "idle")
//                 out_msg = {{"manage", "vision"}, {"state", "idle"}};
//             else
//             {
//                 std::cout << "cmd not valid\n\n";
//                 continue;
//             }
//             zmq::message_t message(out_msg.dump());
//             vision_socket.send(message, zmq::send_flags::none);
//         }
//         else
//         {
//             std::cout << "ctrl not valid\n\n";
//         }
//     }
// }

// int main()
// {
//     zmq::context_t context(1);
//     zmq::socket_t vis_cmd_socket(context, ZMQ_PUSH);
//     vis_cmd_socket.bind("ipc:///home/amrmgr/amr/tmp/ctrl_cmd");

//     send_commands(vis_cmd_socket);

//     return 0;
// }
