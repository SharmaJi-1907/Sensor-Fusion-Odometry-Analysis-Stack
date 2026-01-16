#include "nandi_controller/nandi_hardware.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nandi_controller {

hardware_interface::CallbackReturn NandiHardwareSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create node for comms
  node_ = rclcpp::Node::make_shared("nandi_hardware");

  // Parameters from URDF <ros2_control><hardware><param>
  auto get_param = [&](const std::string &name, std::string &out,
                       const std::string &def) {
    auto it = info_.hardware_parameters.find(name);
    out = it == info_.hardware_parameters.end() ? def : it->second;
  };
  auto get_param_double = [&](const std::string &name, double &out,
                              double def) {
    auto it = info_.hardware_parameters.find(name);
    out = it == info_.hardware_parameters.end() ? def : std::stod(it->second);
  };
  auto get_param_int = [&](const std::string &name, int &out, int def) {
    auto it = info_.hardware_parameters.find(name);
    out = it == info_.hardware_parameters.end() ? def : std::stoi(it->second);
  };
  auto get_param_bool = [&](const std::string &name, bool &out, bool def) {
    auto it = info_.hardware_parameters.find(name);
    if (it == info_.hardware_parameters.end()) {
      out = def;
    } else {
      const std::string &v = it->second;
      out = (v == "true" || v == "1");
    }
  };

  get_param("right_encoder_topic", right_encoder_topic_,
            std::string("right_encoder_counts"));
  get_param("left_encoder_topic", left_encoder_topic_,
            std::string("left_encoder_counts"));
  get_param("right_motor_topic", right_motor_topic_,
            std::string("right_motor_speed_cmd"));
  get_param("left_motor_topic", left_motor_topic_,
            std::string("left_motor_speed_cmd"));
  get_param_double("ticks_per_rev", ticks_per_rev_, 4000.0);
  get_param_double("gear_ratio", gear_ratio_, 35.0);
  get_param_bool("encoder_on_motor_shaft", encoder_on_motor_shaft_, true);
  get_param_int("right_encoder_sign", right_encoder_sign_, -1);
  get_param_int("left_encoder_sign", left_encoder_sign_, -1);

  // Init comms
  sub_right_enc_ = node_->create_subscription<std_msgs::msg::Int64>(
      right_encoder_topic_, 10,
      std::bind(&NandiHardwareSystem::encoder_right_callback, this,
                std::placeholders::_1));
  sub_left_enc_ = node_->create_subscription<std_msgs::msg::Int64>(
      left_encoder_topic_, 10,
      std::bind(&NandiHardwareSystem::encoder_left_callback, this,
                std::placeholders::_1));

  pub_right_motor_ =
      node_->create_publisher<std_msgs::msg::Int64>(right_motor_topic_, 1);
  pub_left_motor_ =
      node_->create_publisher<std_msgs::msg::Int64>(left_motor_topic_, 1);

  // Initialize last counts
  right_encoder_last_ = right_encoder_count_.load();
  left_encoder_last_ = left_encoder_count_.load();

  // Initialize sizes according to joints
  const std::size_t n_joints = info_.joints.size();
  joint_position_.assign(n_joints, 0.0);
  joint_velocity_.assign(n_joints, 0.0);
  joint_effort_.assign(n_joints, 0.0);
  joint_velocity_command_.assign(n_joints, 0.0);

  // Cache wheel joint indices once
  right_joint_index_ = -1;
  left_joint_index_ = -1;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].name.find("right") != std::string::npos)
      right_joint_index_ = static_cast<int>(i);
    if (info_.joints[i].name.find("left") != std::string::npos)
      left_joint_index_ = static_cast<int>(i);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
NandiHardwareSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
NandiHardwareSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_command_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
NandiHardwareSystem::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(node_->get_logger(), "NandiHardwareSystem activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NandiHardwareSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(node_->get_logger(), "NandiHardwareSystem deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

double NandiHardwareSystem::ticks_per_wheel_rev_() const {
  return ticks_per_rev_ * (encoder_on_motor_shaft_ ? gear_ratio_ : 1.0);
}

void NandiHardwareSystem::encoder_right_callback(
    const std_msgs::msg::Int64::SharedPtr msg) {
  right_encoder_count_.store(
      static_cast<long long>(right_encoder_sign_ * msg->data));
}

void NandiHardwareSystem::encoder_left_callback(
    const std_msgs::msg::Int64::SharedPtr msg) {
  left_encoder_count_.store(
      static_cast<long long>(left_encoder_sign_ * msg->data));
}

hardware_interface::return_type
NandiHardwareSystem::read(const rclcpp::Time &,
                          const rclcpp::Duration &period) {
  // Process pending encoder callbacks
  rclcpp::spin_some(node_);

  const double dt = period.seconds();
  const double ticks_per_wheel = ticks_per_wheel_rev_();

  // Use cached indices
  const int right_index = right_joint_index_;
  const int left_index = left_joint_index_;

  const long long right_now = right_encoder_count_.load();
  const long long left_now = left_encoder_count_.load();

  // Position in radians: 2*pi * revolutions
  if (right_index >= 0) {
    joint_position_[right_index] =
        2.0 * M_PI * (static_cast<double>(right_now) / ticks_per_wheel);
  }
  if (left_index >= 0) {
    joint_position_[left_index] =
        2.0 * M_PI * (static_cast<double>(left_now) / ticks_per_wheel);
  }

  // Velocity from delta counts
  const long long right_delta = right_now - right_encoder_last_;
  const long long left_delta = left_now - left_encoder_last_;

  right_encoder_last_ = right_now;
  left_encoder_last_ = left_now;

  if (dt > 0.0) {
    const double right_rev_per_sec =
        static_cast<double>(right_delta) / ticks_per_wheel / dt;
    const double left_rev_per_sec =
        static_cast<double>(left_delta) / ticks_per_wheel / dt;
    if (right_index >= 0) {
      joint_velocity_[right_index] = right_rev_per_sec * 2.0 * M_PI; // rad/s
    }
    if (left_index >= 0) {
      joint_velocity_[left_index] = left_rev_per_sec * 2.0 * M_PI; // rad/s
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
NandiHardwareSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  // Convert wheel angular velocity command [rad/s] to motor RPM if encoder is
  // on motor shaft rpm = rad/s * 30/pi; multiply by gear ratio to get motor rpm
  const double rad_per_sec_to_rpm = 30.0 / M_PI;

  const int right_index = right_joint_index_;
  const int left_index = left_joint_index_;

  long long right_rpm_cmd = 0;
  long long left_rpm_cmd = 0;

  if (right_index >= 0) {
    const double rpm =
        joint_velocity_command_[right_index] * rad_per_sec_to_rpm * gear_ratio_;
    right_rpm_cmd = static_cast<long long>(std::llround(rpm));
  }
  if (left_index >= 0) {
    const double rpm =
        joint_velocity_command_[left_index] * rad_per_sec_to_rpm * gear_ratio_;
    left_rpm_cmd = static_cast<long long>(std::llround(rpm));
  }

  std_msgs::msg::Int64 msg_right;
  msg_right.data = right_rpm_cmd;
  std_msgs::msg::Int64 msg_left;
  msg_left.data = left_rpm_cmd;

  pub_right_motor_->publish(msg_right);
  pub_left_motor_->publish(msg_left);

  return hardware_interface::return_type::OK;
}

} // namespace nandi_controller

PLUGINLIB_EXPORT_CLASS(nandi_controller::NandiHardwareSystem,
                       hardware_interface::SystemInterface)
