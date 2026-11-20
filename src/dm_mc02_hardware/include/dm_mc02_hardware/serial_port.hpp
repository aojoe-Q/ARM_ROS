#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace dm_mc02 {

// Non-blocking serial port for Linux (termios)
class SerialPort {
public:
    SerialPort() : fd_(-1) {}
    ~SerialPort() { close(); }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud = 115200);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // Non-blocking read: returns bytes actually read (0 = EAGAIN)
    size_t read(uint8_t* buf, size_t max_len);

    // Non-blocking write with a short bounded drain; returns bytes actually written
    size_t write(const uint8_t* data, size_t len);
    size_t write(const std::vector<uint8_t>& data) {
        return write(data.data(), data.size());
    }

private:
    int fd_;
};

}  // namespace dm_mc02

