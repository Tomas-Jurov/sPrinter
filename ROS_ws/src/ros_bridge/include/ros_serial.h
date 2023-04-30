#ifndef SPRINTER_ROS_SERIAL_H
#define SPRINTER_ROS_SERIAL_H

#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "utility.h"

namespace sprinter{
  class SerialPort{
  public:
    SerialPort() = delete;
    SerialPort(const std::string& port_name, const uint32_t baud_rate);
    ~SerialPort() = default;

    void open();

    int close();

    ssize_t read(bytePtr buffer, size_t size);

    ssize_t write(bytePtr buffer, size_t size);

  private:
    std::string port_name_;
    uint32_t baud_rate_;
    static constexpr int timeout_micro_s_ = 50000;
    int file_desc_;

  private:
    void configureTermios();
    speed_t getBaudrate() const;
    int getElapsedTime(struct timeval *start, struct timeval *end);

  };
}

#endif //SPRINTER_ROS_SERIAL_H