
#ifndef CURIO_ONE__CurioBot_SYSTEM_HPP_
#define CURIO_ONE__CurioBot_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/sensor.hpp"
#include "rclcpp/node.hpp"

#include "curio_one/visibility_control.h"
#include "curio_one/arduino_comms.hpp"
#include "curio_one/wheel.hpp"
#include "curio_one/sensor.hpp"
#include "rclcpp/publisher.hpp" 
#include "sensor_msgs/msg/range.hpp"
// #include "curio_one/sensor_pub.hpp"



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace curio_one
{
class CurioBotSystemHardware : public hardware_interface::SystemInterface
{
  struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";

  

  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;

  std::string sensor_name = "";
  std::string sensor1_name = "";
  std::string sensor2_name = "";
  double reading;
};
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CurioBotSystemHardware)
  CurioBotSystemHardware() = default;
  CurioBotSystemHardware(const rclcpp::Node::SharedPtr& node) : node_(node) {}
  CURIO_ONE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CURIO_ONE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CURIO_ONE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CURIO_ONE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CURIO_ONE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CURIO_ONE_PUBLIC
  hardware_interface::return_type read() override;

  CURIO_ONE_PUBLIC
  hardware_interface::return_type write( ) override;

private:
  // Parameters for the CurioBot simulation
  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  Sensors sensor_n_;
  Sensors sensor_l_;
  Sensors sensor_r_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_;
  rclcpp::Node::SharedPtr node_;

};

}  // namespace CURIO_ONE

#endif  // CURIO_ONE__CurioBOT_SYSTEM_HPP_
