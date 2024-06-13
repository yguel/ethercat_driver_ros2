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
//
// Author: Manuel YGUEL (yguel.robotics@gmail.com)

#include "ethercat_driver/ethercat_safety_driver.hpp"

#include <tinyxml2.h>
#include <string>
#include <regex>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef CLASSM
#define CLASSM  EthercatSafetyDriver
#else
#error alias CLASSM already defined!
#endif  //< CLASSM

namespace ethercat_driver
{

std::vector<std::unordered_map<std::string, std::string>> CLASSM::getEcSafetyModuleParam(
  const std::string & urdf, const std::string & component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  while (ros2_control_it) {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it) {
      const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");
      while (ec_module_it) {
        module_param.clear();
        module_param["name"] = ec_module_it->Attribute("name");
        auto type = ec_module_it->FindAttribute("type");
        if (type) {
          module_param["type"] = type->Value();
        } else {
          module_param["type"] = "safety_slave";
        }
        const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
        if (NULL != plugin_it) {
          module_param["plugin"] = plugin_it->GetText();
        }
        const auto * param_it = ec_module_it->FirstChildElement("param");
        while (param_it) {
          module_param[param_it->Attribute("name")] = param_it->GetText();
          param_it = param_it->NextSiblingElement("param");
        }
        module_params.push_back(module_param);
        ec_module_it = ec_module_it->NextSiblingElement("ec_module");
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }
  return module_params;
}

unsigned int uint_from_string(const std::string & str)
{
  // Strip leading and trailing whitespaces
  std::string s = std::regex_replace(str, std::regex("^ +| +$|( ) +"), "$1");
  // Test if the number is in hexadecimal format
  if (s.find("0x") == 0) {
    return std::stoul(s, nullptr, 16);
  }
  return std::stoul(s);
}

std::vector<ethercat_interface::EcSafetyNet> CLASSM::getEcSafetyNets(
  const std::string & urdf, const std::string & component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<ethercat_interface::EcSafetyNet> safety_nets;
  ethercat_interface::EcSafetyNet safety_net;

  while (ros2_control_it) {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it) {
      const auto * net_it = ros2_control_child_it->FirstChildElement("net");
      while (net_it) {
        safety_net.reset(net_it->Attribute("name"));

        // Master name
        const auto * master_it = net_it->FirstChildElement("ec_safety_master");
        safety_net.master.name = master_it->GetText();

        // Transfers
        const auto * transfer_it = net_it->FirstChildElement("transfer");
        while (transfer_it) {
          ethercat_interface::EcSafetyTransfer transfer;
          const auto * in = transfer_it->FirstChildElement("in");
          const auto * out = transfer_it->FirstChildElement("out");
          const auto * size = transfer_it->FirstChildElement("size");

          transfer.input.module_name = in->Attribute("ec_module");
          transfer.input.index = uint_from_string(in->GetText());
          transfer.output.module_name = out->Attribute("ec_module");
          transfer.output.index = uint_from_string(out->GetText());
          transfer.size = uint_from_string(size->GetText());

          safety_net.transfers.push_back(transfer);

          transfer_it = transfer_it->NextSiblingElement("transfer");
        }
        safety_nets.push_back(safety_net);
        net_it = net_it->NextSiblingElement("net");
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }
  return safety_nets;
}

CallbackReturn CLASSM::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto res = this->EthercatDriver::on_init(info);
  if (res != CallbackReturn::SUCCESS) {
    return res;
  }

  // Prevent the same module from being activated while being configured
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  // Parse safety modules from the safety tag in the URDF
  auto safety_module_params = getEcSafetyModuleParam(info_.original_xml, "safety");
  ec_module_parameters

  return CallbackReturn::SUCCESS;
}


CallbackReturn CLASSM::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return this->EthercatDriver::on_activate(previous_state);
}

}  // namespace ethercat_driver

#undef CLASSM

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatSafetyDriver, hardware_interface::SystemInterface)
