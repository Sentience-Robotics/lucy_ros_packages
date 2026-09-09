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

#include <cmath>
#include <format>
#include <cstddef>
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

  {
    auto it = info_.hardware_parameters.find("node_name");
    if (it != info_.hardware_parameters.end()) {
      node_name_ = it->second;
    } else {
      node_name_ = "lucy_hardware_interface";
    }


  }
    // resizing command and state vectors
  hw_positions_.resize(info_.joints.size(), 0);
  // hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN()); // no velocities for our servos
  hw_commands_.resize(info_.joints.size(), 0);
  hw_old_commands_.resize(info_.joints.size(), 0);

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

  node_ = std::make_shared<rclcpp::Node>(node_name_);

  // RELIABLE QoS kept for optional JointState debug publishers.
  if (publish_actuators_) {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(publisher_topic, qos);
    RCLCPP_INFO(
      get_logger(),
      "Publishing joint state on topic '%s' (debug/legacy path; actuation uses SHM+Modbus)",
      publisher_topic.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "publish_actuators=false: URDF limits enforced in-process only (SHM register path).");
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
  { // REGISTER
    int fd = shm_open(std::format("/{}.lucy_reg_table", node_name_).c_str(), O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create shared memory space for register table (shm_open() failed).",
        info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (ftruncate(fd, sizeof(SharedRegisters)) == -1) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create shared memory space for register table (ftruncate() failed).",
        info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    void* addr = mmap(nullptr, sizeof(SharedRegisters),
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd); // fd not needed after mmap
  
    if (addr == MAP_FAILED) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create shared memory space for register table (mmap() failed).",
        info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    shared_registers_ = static_cast<SharedRegisters*>(addr);
  }

  { // REGISTER TABLE
    int fd = shm_open(std::format("/{}.lucy_reg_header", node_name_).c_str(), O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create shared memory space for register table (shm_open() failed).",
        info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (ftruncate(fd, sizeof(RegisterHeader)) == -1) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create shared memory space for register table (ftruncate() failed).",
        info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    void* addr = mmap(nullptr, sizeof(RegisterHeader),
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd); // fd not needed after mmap
  
    if (addr == MAP_FAILED) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create shared memory space for register table (mmap() failed).", info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    register_header_ = static_cast<RegisterHeader*>(addr);
  }

  sem_ = sem_open(std::format("/{}", node_name_).c_str(), O_CREAT, 0644, 1);
  if (sem_ == SEM_FAILED) {
    RCLCPP_FATAL(
        get_logger(), "Failed to create named sem.", info_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
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

  if (sem_ != SEM_FAILED && sem_ != nullptr) {
    sem_close(sem_);
    sem_unlink(std::format("/{}", node_name_).c_str());
    sem_ = SEM_FAILED;
  }

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
  // Encode servo angle into Modbus holding registers (milliradians = rad * 1000):
  //   reg[virtual_pin * 2]     = cmd (1 = move)
  //   reg[virtual_pin * 2 + 1] = angle_millirad
  for (const auto & m : mappings_) {
    const double cmd_rad = hw_commands_[m.joint_index];
    if (std::abs(cmd_rad - hw_old_commands_[m.joint_index]) <= 0.001) {
      continue;
    }
    hw_old_commands_[m.joint_index] = cmd_rad;
    hw_positions_[m.joint_index] = cmd_rad;

    const double servo_rad = actuator_command_to_servo_rad(m, cmd_rad);
    const uint16_t angle_millirad = static_cast<uint16_t>(
      std::lround(servo_rad * 1000.0));
    const uint16_t base = static_cast<uint16_t>(m.virtual_pin * 2);

    sem_wait(sem_);
    shared_registers_->register_table[base] = 1;
    shared_registers_->register_table[base + 1] = angle_millirad;
    register_header_->set_dirty(base);
    register_header_->set_dirty(static_cast<uint16_t>(base + 1));
    sem_post(sem_);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace lucy_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  lucy_ros2_control::LucySystemHardware, hardware_interface::SystemInterface)
