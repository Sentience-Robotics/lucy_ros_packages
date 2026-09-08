// Copyright 2025 Sentience Robotics Team
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "include/lucy_system.hpp"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "include/joint_config.hpp"
#include "include/position_limit_clamp.hpp"
#include "rclcpp/qos.hpp"

namespace lucy_ros2_control
{
namespace
{
/// POSIX shm name for one hardware component: "/lucy." + sanitised name.
///
/// shm_open() takes a name, not a path: a leading '/' and no other slash.
/// Darwin also caps it at PSHMNAMLEN (31) and fails with ENAMETOOLONG beyond
/// that, so an over-long name keeps its tail -- Lucy's components share the
/// "LucyHardware" prefix and differ only in the suffix.
std::string shared_registers_name_for(const std::string & component)
{
  constexpr std::size_t kMaxShmName = 31;
  constexpr const char * kPrefix = "/lucy.";

  std::string sanitised;
  sanitised.reserve(component.size());
  for (const char c : component) {
    const bool keep = std::isalnum(static_cast<unsigned char>(c)) != 0 ||
      c == '_' || c == '-' || c == '.';
    sanitised.push_back(keep ? c : '_');
  }

  const std::size_t budget = kMaxShmName - std::strlen(kPrefix);
  if (sanitised.size() > budget) {
    sanitised.erase(0, sanitised.size() - budget);
  }
  return kPrefix + sanitised;
}
}  // namespace

LucySystemHardware::~LucySystemHardware()
{
  release_registers();
}

void LucySystemHardware::release_registers()
{
  if (shared_registers_ != nullptr) {
    munmap(shared_registers_, sizeof(SharedRegisters));
    shared_registers_ = nullptr;
  }
  if (!shared_registers_name_.empty()) {
    // Drop the name too: a segment outlives its creator, and Darwin rejects
    // ftruncate() on one that already has a size, so leaving it behind makes
    // the next run fail to start.
    shm_unlink(shared_registers_name_.c_str());
    shared_registers_name_.clear();
  }
}

hardware_interface::CallbackReturn LucySystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
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

  if (validate_joints() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (configure_publisher() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (init_joint_limits() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (init_actuator_mappings() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (init_registers() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::validate_joints()
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    const std::string error = validate_joint_interfaces(joint);
    if (!error.empty()) {
      RCLCPP_FATAL(get_logger(), "%s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::configure_publisher()
{
  auto it_publish = info_.hardware_parameters.find("publish_actuators");
  if (it_publish != info_.hardware_parameters.end()) {
    const std::string & v = it_publish->second;
    publish_actuators_ = !(v == "false" || v == "0" || v == "False");
  }

  std::string publisher_topic;
  auto it_topic = info_.hardware_parameters.find("publisher_topic");
  if (publish_actuators_) {
    if (it_topic == info_.hardware_parameters.end() || it_topic->second.empty()) {
      RCLCPP_FATAL(get_logger(), "Hardware parameter 'publisher_topic' is missing or empty.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    publisher_topic = it_topic->second;
  }

  std::string node_name = "lucy_hardware_interface";
  auto it_node = info_.hardware_parameters.find("node_name");
  if (it_node != info_.hardware_parameters.end() && !it_node->second.empty()) {
    node_name = it_node->second;
  }
  node_ = std::make_shared<rclcpp::Node>(node_name);

  // RELIABLE to match micro-ROS rclc_subscription_init_default (RELIABLE). A BEST_EFFORT publisher
  // does not match a RELIABLE subscription in ROS 2, so the Pico would receive no commands.
  if (publish_actuators_) {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(publisher_topic, qos);
    RCLCPP_INFO(
      get_logger(),
      "Publishing joint state on topic '%s' (RELIABLE for micro-ROS default subscriber)",
      publisher_topic.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "publish_actuators=false: URDF limits enforced in-process only (no actuator topics).");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::init_joint_limits()
{
  joint_min_rad_.assign(info_.joints.size(), -std::numeric_limits<double>::infinity());
  joint_max_rad_.assign(info_.joints.size(), std::numeric_limits<double>::infinity());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    try {
      const JointLimits limits = parse_joint_limits(info_.joints[i]);
      joint_min_rad_[i] = limits.min_rad;
      joint_max_rad_[i] = limits.max_rad;
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has invalid command_interface limits: %s",
        info_.joints[i].name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::init_actuator_mappings()
{
  mappings_.clear();
  mappings_.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];

    std::optional<ActuatedJointMapping> mapping;
    try {
      mapping = build_actuated_joint_mapping(joint, i, joint_min_rad_[i], joint_max_rad_[i]);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has invalid parameters: %s", joint.name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!mapping.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has no virtual_pin; treated as passive/unmapped for actuator output.",
        joint.name.c_str());
      continue;
    }

    const ActuatedJointMapping & m = mapping.value();
    if (!std::isfinite(m.min_rad) || !std::isfinite(m.max_rad)) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has no finite URDF position limits on command interface; "
        "only servo_min/max_deg will bound output.",
        joint.name.c_str());
    }

    mappings_.push_back(m);
    hw_commands_[i] = default_joint_position_rad(m);
    hw_positions_[i] = hw_commands_[i];
  }

  const std::optional<int> duplicate = sort_and_find_duplicate_virtual_pin(mappings_);
  if (duplicate.has_value()) {
    RCLCPP_FATAL(
      get_logger(), "Duplicate virtual_pin %d in hardware '%s'.", duplicate.value(),
      info_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::init_registers() {
  release_registers();
  const std::string name = shared_registers_name_for(info_.name);

  // O_EXCL so an existing segment is a fact to act on rather than one silently
  // adopted: it can only be a leak from a run that died before release_registers(),
  // and Darwin rejects ftruncate() on an already-sized segment (EINVAL).
  int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  if (fd == -1 && errno == EEXIST) {
    RCLCPP_WARN(
      get_logger(), "Reclaiming shared memory segment '%s' left by an earlier run.",
      name.c_str());
    shm_unlink(name.c_str());
    fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  }
  if (fd == -1) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s': shm_open('%s') failed: %s",
      info_.name.c_str(), name.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Named from here on, so every later failure unlinks it back out.
  shared_registers_name_ = name;

  if (ftruncate(fd, sizeof(SharedRegisters)) == -1) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s': ftruncate('%s') failed: %s",
      info_.name.c_str(), name.c_str(), std::strerror(errno));
    close(fd);
    release_registers();
    return hardware_interface::CallbackReturn::ERROR;
  }

  void* addr = mmap(nullptr, sizeof(SharedRegisters),
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd); // fd not needed after mmap

  if (addr == MAP_FAILED) {
    RCLCPP_FATAL(
      get_logger(), "Hardware '%s': mmap('%s') failed: %s",
      info_.name.c_str(), name.c_str(), std::strerror(errno));
    release_registers();
    return hardware_interface::CallbackReturn::ERROR;
  }

  shared_registers_ = static_cast<SharedRegisters*>(addr);
  RCLCPP_INFO(
    get_logger(), "Register table mapped on shared memory segment '%s'.", name.c_str());
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
In our case, the hardware is already ready to receive informations
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

hardware_interface::return_type lucy_ros2_control::LucySystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    const double cmd_rad = lucy_ros2_control::clamp_position_command(
      hw_commands_[i], joint_min_rad_[i], joint_max_rad_[i]);
    hw_commands_[i] = cmd_rad;
    hw_positions_[i] = cmd_rad;
  }

  if (!publish_actuators_ || !joint_publisher_) {
    return hardware_interface::return_type::OK;
  }

  // Firmware reads inputs->position.data[joint->config.virtual_pin] per configured joint.
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
    shared_registers_->register_table[m.virtual_pin] = hw_commands_[m.joint_index];
    msg.position[static_cast<size_t>(m.virtual_pin)] =
      actuator_command_to_servo_rad(m, hw_commands_[m.joint_index]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace lucy_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  lucy_ros2_control::LucySystemHardware, hardware_interface::SystemInterface)
