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
#include <unordered_map>
#include <memory>
#include <string>

#include "yaml-cpp/yaml.h"

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
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // ETHERCAT_DRIVER_PUBLIC
  // CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  /** @brief Load FailSafe Over EtherCAT Safety config YAML file
   * @param[out] node YAML node containing the safety configuration root
   * @param[in] path Path to the YAML file, if empty, the file is loaded from the *fsoe_config* of the YAML document
   */
  void loadFsoeConfigYamlFile(YAML::Node & node, const std::string & path = "");

  /** @brief Get safety module parameters from YAML filer
   * @param[in] config YAML node containing the safety configuration root
   * @return Vector of maps containing safety module parameters, each map corresponds to a safety module
   */
  std::vector<std::unordered_map<std::string, std::string>> getEcSafetyModuleParam(
    const YAML::Node & config);

  /** @brief Get safety nets from YAML file
   * @param[in] config YAML node containing the safety configuration root
   * @return Vector of safety nets
   */
  std::vector<ethercat_interface::EcSafetyNet> getEcSafetyNets(const YAML::Node & config);

  CallbackReturn setupMaster() override;

  /** @brief Configure the safety network
   */
  void configSafetyNetwork();

protected:
  /** Master hable to manage a safety network */
  std::shared_ptr<ethercat_interface::EcSafety> safety_;

  /** Safety nets */
  std::vector<ethercat_interface::EcSafetyNet> ec_safety_nets_;

  /** Indexes of modules inside ec_modules_ vector that are safety masters */
  std::vector<size_t> ec_safety_masters_;
  /** Indexes of modules inside ec_modules_ vector that are safety slaves only */
  std::vector<size_t> ec_safety_slaves_;

  /** Empty interfaces */
  std::vector<double> empty_interface_;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_SAFETY_DRIVER_HPP_
