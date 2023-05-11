#include "../include/sprinter.h"
#include <iostream>

ROSbridge::Sprinter::Sprinter(const std::string& port_name, const uint32_t baud_rate)
  : port_name_(port_name), baud_rate_(baud_rate)
{
}

bool ROSbridge::Sprinter::connect()
{
  if (port_name_.empty())
  {
    return 1;
  }

  if (baud_rate_ != 230400 && baud_rate_ != 1000000)
  {
    return 1;
  }

  serial_port_ = std::make_unique<SerialPort>(port_name_, baud_rate_);
  serial_port_->open();
  return 0;
}

bool ROSbridge::Sprinter::disconnect()
{
  return serial_port_->close();
}

bool ROSbridge::Sprinter::readReturns(ROSbridge::Returns* data)
{
  return readStateOfSprinter(data);
}

bool ROSbridge::Sprinter::setVelocityOfWheels(const ROSbridge::VelocityOfWheels& velocity_of_wheels)
{
  return writeParameters(SET_VELOCITY_OF_WHEELS, (constBytePtr)std::addressof(velocity_of_wheels), sizeof(VelocityOfWheels));
}

<<<<<<< HEAD
bool ROSbridge::Sprinter::setVelocityOfLinearActuator(int8_t velocity_of_actuator)
{
  return writeParameters(SET_VELOCITY_OF_LIN_ACTUATOR, (constBytePtr)&velocity_of_actuator, sizeof(int8_t));
}

bool ROSbridge::Sprinter::setAngleOfServo1(uint16_t angle)
{
  return writeParameters(SET_SERVO1_TARG_ANGLE, (constBytePtr)&angle, sizeof(uint16_t));
}

bool ROSbridge::Sprinter::setAngleOfServo2(uint16_t angle)
{
  return writeParameters(SET_SERVO2_TARG_ANGLE, (constBytePtr)&angle, sizeof(uint16_t));
}

bool ROSbridge::Sprinter::stepper1SetTargetSteps(int32_t steps)
{
  return writeParameters(SET_STEPPER1_TARG_STEPS, (constBytePtr)&steps, sizeof(int32_t));
}

bool ROSbridge::Sprinter::stepper2SetTargetSteps(int32_t steps)
{
  return writeParameters(SET_STEPPER2_TARG_STEPS, (constBytePtr)&steps, sizeof(int32_t));
}

bool ROSbridge::Sprinter::setSpeedOfStepper1(uint16_t speed)
{
  return writeParameters(SET_STEPPER1_SPEED, (constBytePtr)&speed, sizeof(uint16_t));
}

bool ROSbridge::Sprinter::setSpeedOfStepper2(uint16_t speed)
{
  return writeParameters(SET_STEPPER2_SPEED, (constBytePtr)&speed, sizeof(uint16_t));
}

bool ROSbridge::Sprinter::runSunTracking()
{
  return writeParameters(START_SUNTRACKING, nullptr, 0);
}

bool ROSbridge::Sprinter::readStateOfSprinter(ROSbridge::Returns* data)
=======
bool sprinter::Sprinter::setSpeedOfLinearActuator(int8_t speed_of_actuator)
{
  return writeParameters(SET_SPEED_OF_LIN_ACTUATOR, (constBytePtr)speed_of_actuator, sizeof(int8_t));
}

bool sprinter::Sprinter::readStateOfSprinter(sprinter::Returns* data)
>>>>>>> Linear actuator callback
{
  DataPacket data_packet;

  size_t header_size = sizeof(data_packet.header);
  size_t crc_size = sizeof(uint32_t);

  ssize_t read_header_bytes = serial_port_->read(data_packet.messsage, header_size);

  if (read_header_bytes != static_cast<ssize_t>(header_size))
  {
    return 1;
  }

  size_t data_size = static_cast<size_t>(data_packet.header.data_len);
  size_t expected_data_size = sizeof(Returns);
  size_t read_data_bytes = serial_port_->read(data_packet.messsage + header_size, data_size + crc_size);

  if (read_data_bytes < (data_size + crc_size) || expected_data_size != data_size ||
      (read_data_bytes - crc_size) != expected_data_size)
  {
    return 1;
  }

  if (data_packet.header.header_func != READ_RETURNS)
  {
    return 1;
  }

  uint32_t crc = crc32(data_packet.messsage, header_size + data_size);

  uint32_t data_packet_crc;
  memcpy(&data_packet_crc, data_packet.messsage + header_size + data_size, crc_size);
  if (crc != data_packet_crc)
  {
    return 1;
  }

  memcpy((bytePtr)data, data_packet.messsage + header_size, data_size);

  return 0;
}

bool ROSbridge::Sprinter::writeParameters(uint8_t command, constBytePtr data, size_t data_size)
{
  DataPacket data_packet;

  size_t header_size = sizeof(data_packet.header);
  size_t crc_size = sizeof(uint32_t);
  size_t data_packet_size = header_size + data_size + crc_size;

  data_packet.header.header_func = WRITE_PARAMS;
  data_packet.header.data_len = (uint16_t)data_size;
  data_packet.header.command = command;

  memcpy(data_packet.messsage + header_size, data, data_size);

  uint32_t crc = crc32(data_packet.messsage, header_size + data_size);

  memcpy(data_packet.messsage + header_size + data_size, &crc, crc_size);

  if (serial_port_->write(data_packet.messsage, data_packet_size) != static_cast<ssize_t>(data_packet_size))
  {
    return 1;
  }

  return 0;
}

uint32_t ROSbridge::Sprinter::crc32(const bytePtr data, size_t length)
{
  uint32_t crc = 0xFFFFFFFF;

  for (size_t i = 0; i < length; i++)
  {
    uint8_t byte = data[i];
    crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF];
  }

  return crc ^ 0xFFFFFFFF;
}