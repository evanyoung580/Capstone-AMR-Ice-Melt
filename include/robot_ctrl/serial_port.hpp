#pragma once
#include <string>

class SerialPort {
public:
    SerialPort(const std::string &device, int baud);
    ~SerialPort();
    bool readLine(std::string &out);
    void writeString(const std::string &s);

private:
    int fd_{-1};
};
