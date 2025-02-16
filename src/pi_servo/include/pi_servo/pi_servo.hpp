#ifndef PI_SERVO_HPP_
#define PI_SERVO_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pi_servo
{
class PiServo : public hardware_interface::ActuatorInterface
{
public:
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  double position_;
  double command_;
};
}  // namespace pi_servo

#endif  // PI_SERVO_HPP_
