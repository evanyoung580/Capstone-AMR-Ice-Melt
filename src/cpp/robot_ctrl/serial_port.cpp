#include "robot_ctrl/serial_port.hpp"
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

SerialPort::SerialPort(const std::string &device, int baud) {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) throw std::runtime_error("Unable to open " + device);

    struct termios tty{};
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                    // disable break processing
    tty.c_lflag = 0;                           // no echo, no signals
    tty.c_oflag = 0;                           // no remap, no delays
    tty.c_cc[VMIN]  = 0;                       // non-blocking read
    tty.c_cc[VTIME] = 5;                       // 0.5s read timeout
    tty.c_cflag |= (CLOCAL | CREAD);           // ignore modem lines, enable read
    tty.c_cflag &= ~(PARENB | PARODD);         // no parity
    tty.c_cflag &= ~CSTOPB;                    // one stop bit
    tty.c_cflag &= ~CRTSCTS;                   // no flow control

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        throw std::runtime_error("Error configuring " + device);
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) close(fd_);
}

bool SerialPort::readLine(std::string &out) {
    char buf;
    out.clear();
    while (true) {
        int n = ::read(fd_, &buf, 1);
        if (n <= 0) return false;
        if (buf == '\n') break;
        out.push_back(buf);
    }
    return true;
}

void SerialPort::writeString(const std::string &s) {
    ::write(fd_, s.c_str(), s.size());
}