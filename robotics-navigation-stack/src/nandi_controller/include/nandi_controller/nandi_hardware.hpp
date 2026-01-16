#pragma once

#include <atomic>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/int64.hpp"

namespace nandi_controller {

class NandiHardwareSystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NandiHardwareSystem)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void encoder_right_callback(const std_msgs::msg::Int64::SharedPtr msg);
  void encoder_left_callback(const std_msgs::msg::Int64::SharedPtr msg);

  // Parameters
  std::string right_encoder_topic_;
  std::string left_encoder_topic_;
  std::string right_motor_topic_;
  std::string left_motor_topic_;
  double ticks_per_rev_{4000.0};
  double gear_ratio_{35.0};
  bool encoder_on_motor_shaft_{true};
  int right_encoder_sign_{-1};
  int left_encoder_sign_{-1};

  // ROS 2 node and comms
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_right_enc_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_left_enc_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_right_motor_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_left_motor_;

  // Joint state/command storage (two wheel joints)
  std::vector<double> joint_position_{0.0, 0.0};
  std::vector<double> joint_velocity_{0.0, 0.0};
  std::vector<double> joint_effort_{0.0, 0.0};
  std::vector<double> joint_velocity_command_{0.0, 0.0};

  // Encoder counts and last readings
  std::atomic<long long> right_encoder_count_{0};
  std::atomic<long long> left_encoder_count_{0};
  long long right_encoder_last_{0};
  long long left_encoder_last_{0};

  // Utility
  double ticks_per_wheel_rev_() const;

  // Cached wheel joint indices (set in on_init)
  int right_joint_index_{-1};
  int left_joint_index_{-1};
};

} // namespace nandi_controller
