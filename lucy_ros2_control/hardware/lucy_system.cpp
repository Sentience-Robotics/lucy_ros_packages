// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/lucy_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"

namespace ros2_control_demo_example_2
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double rad_to_deg(double rad)
{
  return rad * 180.0 / kPi;
}

double deg_to_rad(double deg)
{
  return deg * kPi / 180.0;
}

double parse_required_double(
  const hardware_interface::ComponentInfo & joint,
  const char * key)
{
  const auto it = joint.parameters.find(key);
  if (it == joint.parameters.end() || it->second.empty()) {
    throw std::runtime_error(
            "missing required parameter '" + std::string(key) + "' for joint '" + joint.name + "'");
  }
  return std::stod(it->second);
}
}  // namespace

hardware_interface::CallbackReturn LucySystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger((info_.name).c_str()));

  // resizing command and state vectors
  hw_positions_.resize(info_.joints.size(), 0);
  // hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN()); // no velocities for our servos
  hw_commands_.resize(info_.joints.size(), 0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto it_topic = info_.hardware_parameters.find("publisher_topic");
  if (it_topic == info_.hardware_parameters.end() || it_topic->second.empty()) {
    RCLCPP_FATAL(get_logger(), "Hardware parameter 'publisher_topic' is missing or empty.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string publisher_topic = it_topic->second;

  std::string node_name = "lucy_hardware_interface";
  auto it_node = info_.hardware_parameters.find("node_name");
  if (it_node != info_.hardware_parameters.end() && !it_node->second.empty()) {
    node_name = it_node->second;
  }
  node_ = std::make_shared<rclcpp::Node>(node_name);

  // RELIABLE to match micro-ROS rclc_subscription_init_default (RELIABLE). A BEST_EFFORT publisher
  // does not match a RELIABLE subscription in ROS 2, so the Pico would receive no commands.
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(publisher_topic, qos);

  RCLCPP_INFO(
    get_logger(), "Publishing joint state on topic '%s' (RELIABLE for micro-ROS default subscriber)",
    publisher_topic.c_str());

  mappings_.clear();
  mappings_.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    const auto it_vpin = joint.parameters.find("virtual_pin");
    if (it_vpin == joint.parameters.end() || it_vpin->second.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has no virtual_pin; treated as passive/unmapped for actuator output.",
        joint.name.c_str());
      continue;
    }

    ActuatedJointMapping m{};
    try {
      m.joint_index = i;
      m.virtual_pin = std::stoi(it_vpin->second, nullptr, 10);
      m.offset_deg = parse_required_double(joint, "offset_deg");
      m.direction = parse_required_double(joint, "direction");
      m.scale = parse_required_double(joint, "scale");
      m.servo_min_deg = parse_required_double(joint, "servo_min_deg");
      m.servo_max_deg = parse_required_double(joint, "servo_max_deg");
      m.servo_default_deg = parse_required_double(joint, "servo_default_deg");
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has invalid parameters: %s",
        joint.name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (m.virtual_pin < 0) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has invalid virtual_pin %d (must be >= 0).",
        joint.name.c_str(),
        m.virtual_pin);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (m.direction == 0.0 || m.scale == 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has invalid direction/scale (must be non-zero).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (m.servo_min_deg > m.servo_max_deg) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has servo_min_deg > servo_max_deg.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    mappings_.push_back(m);

    const double default_joint_rad =
      deg_to_rad((m.servo_default_deg - m.offset_deg) * m.direction * m.scale);
    hw_commands_[i] = default_joint_rad;
    hw_positions_[i] = default_joint_rad;
  }

  std::sort(
    mappings_.begin(),
    mappings_.end(),
    [](const ActuatedJointMapping & a, const ActuatedJointMapping & b) {
      return a.virtual_pin < b.virtual_pin;
    });
  for (std::size_t i = 1; i < mappings_.size(); ++i) {
    if (mappings_[i].virtual_pin == mappings_[i - 1].virtual_pin) {
      RCLCPP_FATAL(
        get_logger(), "Duplicate virtual_pin %d in hardware '%s'.", mappings_[i].virtual_pin,
        info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> LucySystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LucySystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

/*
This function should be used to initialize/activate the hardware.
In our case, the hardware is already ready to receive informations from the Jetson ?
*/
hardware_interface::CallbackReturn LucySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LucySystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Updating the position of each joint from the command
  for (std::size_t i = 0; i < hw_commands_.size(); i++) {
    // No encoder for our servos, we assume that the position is always reached
    hw_positions_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_2::LucySystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Firmware reads inputs->position.data[joint->config.virtual_pin] per configured joint.
  // Build sparse position array by configured virtual_pin.
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.name.clear();

  if (mappings_.empty()) {
    joint_publisher_->publish(msg);
    return hardware_interface::return_type::OK;
  }

  int max_vp = mappings_.back().virtual_pin;
  msg.position.assign(static_cast<size_t>(max_vp) + 1U, 0.0);

  for (const auto & m : mappings_) {
    const double joint_deg = rad_to_deg(hw_commands_[m.joint_index]);
    const double servo_deg = (joint_deg / (m.direction * m.scale)) + m.offset_deg;
    const double clamped_deg = std::min(std::max(servo_deg, m.servo_min_deg), m.servo_max_deg);
    msg.position[static_cast<size_t>(m.virtual_pin)] = deg_to_rad(clamped_deg);
  }

  joint_publisher_->publish(msg);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::LucySystemHardware, hardware_interface::SystemInterface)
