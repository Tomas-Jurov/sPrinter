#include "../include/ros_serial.h"
#include <iostream>

sprinter::SerialPort::SerialPort(const std::string& port_name, const uint32_t baud_rate)
: port_name_(port_name)
, baud_rate_(baud_rate) 
{ 
}

void sprinter::SerialPort::open()
{
    file_desc_ = ::open(port_name_.c_str(), O_RDWR | O_EXCL | O_ASYNC);
    this->configureTermios();
}

int sprinter::SerialPort::close()
{
  if (file_desc_ != -1)
  {
    auto ret_val = ::close(file_desc_);
    if (ret_val != 0)
    {
      std::cerr << "Tried to close serial port but close() failed." << std::endl;
    }
    else
    {
      return ret_val;
    }
    file_desc_ = -1;
  }
  return file_desc_;
}

ssize_t sprinter::SerialPort::read(bytePtr buffer, size_t size)
{
  if (file_desc_ < 0)
  {
      return -1;
  }

  ssize_t read_bytes = 0;
  if (!buffer || size <= 0)
  {
      return read_bytes;
  }

  struct timeval start, end;
  gettimeofday(&start, NULL);

  memcpy(&end, &start, sizeof(start));

  while (getElapsedTime(&start, &end) < timeout_micro_s_ && read_bytes < (ssize_t)size)
  {
      ssize_t read_bytes_iteration = ::read(file_desc_, buffer + read_bytes, size - read_bytes);
      if (read_bytes_iteration < 0)
      {
          return -1;
      }

      read_bytes += read_bytes_iteration;

      gettimeofday(&end, NULL);
  }

  return read_bytes;
}

ssize_t sprinter::SerialPort::write(bytePtr buffer, size_t size)
{
  if (file_desc_ < 0)
  {
      return -1;
  }

  if (!buffer || size <= 0)
  {
      return 0;
  }

  return ::write(file_desc_, buffer, size);
}

void sprinter::SerialPort::configureTermios()
{
    struct termios tty;

    if (tcgetattr(file_desc_, &tty) != 0)
    {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));  
      close();
    }

    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = IGNPAR;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    speed_t system_baudrate = getBaudrate();

    // Set in/out baud rate
    cfsetispeed(&tty, system_baudrate);
    cfsetospeed(&tty, system_baudrate);

    // Save tty settings, also checking for error
    if (tcsetattr(file_desc_, TCSANOW, &tty) < 0)
    {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      close();
    }
}

speed_t sprinter::SerialPort::getBaudrate() const
{
    switch (baud_rate_) {
      case 230400:	    return B230400;
      case 115200:	    return B115200;
      case 57600:	      return B57600;
      case 38400:	      return B38400;
      case 19200:	      return B19200;
      case 9600:	      return B9600;
      case 4800:	      return B4800;
      case 2400:	      return B2400;
      case 1800:	      return B1800;
      case 1200:	      return B1200;
      case 600:	        return B600;
      case 300:	        return B300;
      case 200:	        return B200;
      case 150:	        return B150;
      case 134:	        return B134;
      case 110:	        return B110;
      case 75:	        return B75;
      case 50:	        return B50;
      #ifdef B460800
      case 460800:	    return B460800;
      #endif
      #ifdef B500000
      case 500000:      return B500000;
      #endif
      #ifdef B576000
      case 576000:	    return B576000;
      #endif
      #ifdef B921600
      case 921600:	    return B921600;
      #endif
      #ifdef B1000000
      case 1000000:	    return B1000000;
      #endif
      #ifdef B1152000
      case 1152000:	    return B1152000;
      #endif
      #ifdef B1500000
      case 1500000:	    return B1500000;
      #endif
      #ifdef B2000000
      case 2000000:	    return B2000000;
      #endif
      #ifdef B2500000
      case 2500000:	    return B2500000;
      #endif
      #ifdef B3000000
      case 3000000:	    return B3000000;
      #endif
      #ifdef B3500000
      case 3500000:	    return B3500000;
      #endif
      #ifdef B4000000
      case 4000000:	    return B4000000;
      #endif
      #ifdef B7200
      case 7200:	      return B7200;
      #endif
      #ifdef B14400
      case 14400:	      return B14400;
      #endif
      #ifdef B28800
      case 28800:	      return B28800;
      #endif
      #ifdef B76800
      case 76800:	      return B76800;
      #endif
      default:          return -1;
	} 
}

int sprinter::SerialPort::getElapsedTime(struct timeval *start, struct timeval *end)
{
    if (!start || !end)
    {
        return -1;
    }

    return ((end->tv_sec - start->tv_sec) * 1000000) + (end->tv_usec - start->tv_usec);
}