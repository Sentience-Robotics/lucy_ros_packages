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

/// @file lucy_system.hpp
/// @brief ros2_control ``SystemInterface`` plugin for Lucy-compatible robots.
///
/// Maps URDF joint-space commands to servo-space output and clamps them to the
/// ros2_control command_interface ``<param name="min/max">`` envelope (real +
/// mock paths) via ``position_limit_clamp.hpp``. Gazebo uses stock
/// ``gz_ros2_control`` and does not load this plugin.

#ifndef LUCY_ROS2_CONTROL__LUCY_SYSTEM_HPP_
#define LUCY_ROS2_CONTROL__LUCY_SYSTEM_HPP_

#include <memory>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <array>

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include "joint_config.hpp"

namespace lucy_ros2_control
{

// Byte-for-byte mirror of the bridge's `#[repr(C)] RegisterHeader`
// (lucy_embedded_firmware/firmwares/sim/src/main.rs): one dirty bit per
// register, MSB-first inside each byte. A wider element type here puts every
// register from 8 upwards on a different byte than the bridge reads.
struct RegisterHeader {
    uint8_t header[32];
    uint16_t iterator;

    bool get_register_status(uint16_t reg) {
        uint16_t index = reg / 8;
        uint16_t index2 = reg % 8;
        return ((header[index] >> (7 - index2)) & 0b1) != 0;
    }

    void switch_register_status(uint16_t reg) {
        uint16_t index = reg / 8;
        uint16_t index2 = reg % 8;
        header[index] = static_cast<uint8_t>(header[index] ^ (1u << (7 - index2)));
    }

    void set_dirty(uint16_t reg) {

        if (get_register_status(reg)) {
            return;
        }
        switch_register_status(reg);
    }

    void set_clean(uint16_t reg) {
        if (!get_register_status(reg)) {
            return;
        }
        switch_register_status(reg);
    }
};

struct SharedRegisters {
  uint16_t register_table[256];
};

// Bus-servo register block layout. The bridge forwards dirty registers in
// ascending index order and the firmware consumes `cmd` and clears it inside
// the same tick, so the opcode must sit above its operands.
constexpr int kBusServoIdOffset = 0;
constexpr int kBusServoAngleOffset = 1;
constexpr int kBusServoCmdOffset = 2;
constexpr int kBusServoRegisterCount = 3;

// Firmware bus-servo opcodes (BusServoModbusAdapter::tick).
constexpr uint16_t kBusServoCmdMove = 1;
constexpr uint16_t kBusServoCmdEnableTorque = 3;


class LucySystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LucySystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const {return *logger_;}
  // rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  /// Validate every joint's command/state interfaces (see on_init step 1).
  hardware_interface::CallbackReturn validate_joints();

  /// Read publish/topic/node params and create the actuator publisher + node.
  hardware_interface::CallbackReturn configure_publisher();

  /// Fill joint_min_rad_ / joint_max_rad_ from command_interface min/max.
  hardware_interface::CallbackReturn init_joint_limits();

  /// Build mappings_, seed default positions, sort and reject duplicate pins.
  hardware_interface::CallbackReturn init_actuator_mappings();

  /// Initialising sensors / actuators registers in shared memory
  hardware_interface::CallbackReturn init_registers();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Node::SharedPtr node_;

  std::string node_name_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  // rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_old_commands_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;

  // Table containing the register values of every sensor and actuator
  RegisterHeader* register_header_ = nullptr;
  SharedRegisters* shared_registers_ = nullptr;
  sem_t *sem_ = nullptr;
  std::string shared_registers_filename_ = "";

  // std::vector<double> hw_velocities_; // We have no velocity for our servos

  /** Per-joint URDF limits from command_interface min/max (rad); ±inf when unset. */
  std::vector<double> joint_min_rad_;
  std::vector<double> joint_max_rad_;

  bool publish_actuators_{true};

  /// When non-zero, only this bus id is driven; set from LUCY_BUS_SERVO_ID.
  /// The rp2040 firmware exposes a single bus-servo adapter at register 0, so
  /// every bus joint currently shares one register block.
  int active_bus_id_{0};

  std::vector<ActuatedJointMapping> mappings_;
};

}  // namespace lucy_ros2_control

#endif  // LUCY_ROS2_CONTROL__LUCY_SYSTEM_HPP_
