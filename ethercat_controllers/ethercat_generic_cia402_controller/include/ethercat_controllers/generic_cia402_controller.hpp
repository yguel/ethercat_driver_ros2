// Copyright 2023, ICube Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_CONTROLLERS__GENERIC_CIA402_CONTROLLER_HPP_
#define ETHERCAT_CONTROLLERS__GENERIC_CIA402_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ethercat_controllers/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "ethercat_msgs/msg/cia402_drive_states.hpp"

namespace ethercat_controllers
{
using DriveStateMsgType = ethercat_msgs::msg::Cia402DriveStates;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CiA402Controller : public controller_interface::ControllerInterface
{
public:
  CIA402_CONTROLLER_PUBLIC
  CiA402Controller();

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  CIA402_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CIA402_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CIA402_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CIA402_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> dof_names_;
  std::vector<double> mode_ops_;
  std::vector<double> status_words_;


  using DriveStatePublisher = realtime_tools::RealtimePublisher<DriveStateMsgType>;
  rclcpp::Publisher<DriveStateMsgType>::SharedPtr drive_state_publisher_;
  std::unique_ptr<DriveStatePublisher> rt_drive_state_publisher_;

  std::string logger_name_;

  std::string device_state_str(uint16_t status_word);
  std::string mode_of_operation_str(double mode_of_operation);
};

}  // namespace ethercat_controllers

#endif  // ETHERCAT_CONTROLLERS__GENERIC_CIA402_CONTROLLER_HPP_
