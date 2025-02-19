#include "pi_servo/pi_servo.hpp"

namespace pi_servo
{
hardware_interface::return_type PiServo::configure(const hardware_interface::HardwareInfo & info)
{
  // ...existing code...
  position_ = 0.0;
  command_ = 0.0;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> PiServo::export_state_interfaces()
{
  // ...existing code...
  return {hardware_interface::StateInterface("pi_servo", "position", &position_)};
}

std::vector<hardware_interface::CommandInterface> PiServo::export_command_interfaces()
{
  // ...existing code...
  return {hardware_interface::CommandInterface("pi_servo", "position", &command_)};
}

hardware_interface::return_type PiServo::start()
{
  // ...existing code...
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiServo::stop()
{
  // ...existing code...
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiServo::read()
{
  // ...existing code...
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiServo::write()
{
  // ...existing code...
  return hardware_interface::return_type::OK;
}
}  // namespace pi_servo
