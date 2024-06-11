// Copyright 2024 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_DRIVER__ETHERCAT_SAFETY_DRIVER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_SAFETY_DRIVER_HPP_

#include <vector>

#include "ethercat_driver/ethercat_driver.hpp"
#include "ethercat_interface/ec_safety.hpp"

namespace ethercat_driver
{

class EthercatSafetyDriver : public EthercatDriver
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatSafetyDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  /** Indexes of modules inside ec_modules_ vector that are safety masters */
  std::vector<size_t> ec_safety_masters_;
  /** Indexes of modules inside ec_modules_ vector that are safety slaves only */
  std::vector<size_t> ec_safety_slaves_;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_SAFETY_DRIVER_HPP_
