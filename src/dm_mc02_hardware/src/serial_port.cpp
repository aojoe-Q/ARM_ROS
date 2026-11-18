#include "dm_mc02_hardware/serial_port.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>

namespace dm_mc02 {

bool SerialPort::open(const std::string& device, int baud) {
    close();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_); fd_ = -1; return false;
    }
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    speed_t speed = B115200;
    if (baud == 9600) speed = B9600;
    else if (baud == 19200) speed = B19200;
    else if (baud == 38400) speed = B38400;
    else if (baud == 57600) speed = B57600;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_); fd_ = -1; return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

size_t SerialPort::read(uint8_t* buf, size_t max_len) {
    if (fd_ < 0) return 0;
    const ssize_t n = ::read(fd_, buf, max_len);
    if (n <= 0) return 0;
    return static_cast<size_t>(n);
}

size_t SerialPort::write(const uint8_t* data, size_t len) {
    if (fd_ < 0 || data == nullptr) return 0;
    size_t total = 0;
    while (total < len) {
        const ssize_t n = ::write(fd_, data + total, len - total);
        if (n > 0) {
            total += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd{fd_, POLLOUT, 0};
            if (::poll(&pfd, 1, 2) > 0) continue;
        }
        break;
    }
    return total;
}

}  // namespace dm_mc02

