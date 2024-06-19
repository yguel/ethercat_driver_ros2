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

#include <tinyxml2.h>
#include <string>
#include <regex>

#include "yaml-cpp/yaml.h"
#include "ethercat_driver/ethercat_safety_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef CLASSM
#define CLASSM EthercatSafetyDriver
#else
#error alias CLASSM already defined!
#endif  //< CLASSM

namespace ethercat_driver
{

void loadFsoeConfigYamlFile(YAML::Node & node)
{
  // Get the fsoe_config parameter of the ethercat_driver hardware plugin
  if (info_.hardware_parameters.find("fsoe_config") == info_.hardware_parameters.end()) {
    std::string msg("fsoe_config parameter is missing!");
    // Safety fsoe config file was not provided
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str());
    throw std::runtime_error(msg);
  } else {
    try {
      node = YAML::LoadFile(file_path);
    } catch (const YAML::ParserException & ex) {
      std::string msg =
        std::string(
        "EthercatSafetyDriver : failed to load fsoe configuration "
        "(YAML file is incorrect): ") + std::string(ex.what());
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str() );
      throw std::runtime_error(msg);
    } catch (const YAML::BadFile & ex) {
      std::string msg =
        std::string(
        "EthercatSafetyDriver : failed to load fsoe configuration "
        "(file path is incorrect or file is damaged): " + std::string(ex.what()));
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str() );
      throw std::runtime_error(msg);
    } catch (std::exception & e) {
      std::string msg =
        std::string(
        "EthercatSafetyDriver : error while loading fsoe configuration: ") + std::string(e.what());
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str() );
      throw std::runtime_error(msg);
    }
  }
}

std::vector<std::unordered_map<std::string, std::string>> CLASSM::getEcSafetyModuleParam(
  const std::string & urdf, const std::string & component_type)
{
  YAML::Node config;
  // Load the fsoe_config file
  loadFsoeConfigYamlFile(config);

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

void getTransferMemoryInfo(
  const tinyxml2::XMLElement * element,
  ethercat_interface::EcMemoryEntry & entry,
  const std::string & safety_net_name)
{
  auto name = element->FindAttribute("ec_module");
  if (!name) {
    std::string msg = "Transfer definition without ec_module attribute in tag, net: " +
      safety_net_name + " direction: " + std::string(element->Name());
    throw std::runtime_error(msg);
  }
  auto index = element->FindAttribute("index");
  if (!index) {
    std::string msg = "Transfer definition without index attribute in tag, net: " +
      safety_net_name + " direction: " + std::string(element->Name());
    throw std::runtime_error(msg);
  }
  auto subindex = element->FindAttribute("subindex");
  if (!subindex) {
    std::string msg = "Transfer definition without subindex attribute in tag, net: " +
      safety_net_name + " direction: " + std::string(element->Name());
    throw std::runtime_error(msg);
  }

  entry.module_name = name->Value();
  entry.index = uint_from_string(index->Value());
  entry.subindex = uint_from_string(subindex->Value());
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
    throw std::runtime_error("invalid URDF passed to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed to robot parser");
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
        auto master_name = net_it->FindAttribute("safety_master");
        if (master_name) {
          safety_net.master = master_name->Value();
        } else {
          std::string msg = "Net definition without safety_master attribute, net: " +
            safety_net.name;
          throw std::runtime_error(msg);
        }

        // Transfers
        const auto * transfer_it = net_it->FirstChildElement("transfer");
        while (transfer_it) {
          ethercat_interface::EcTransferEntry transfer;

          // Get transfer size
          auto transfer_size = transfer_it->FindAttribute("size");
          if (!transfer_size) {
            std::string msg = "Transfer definition without size attribute, net: " +
              safety_net.name;
            throw std::runtime_error(msg);
          }
          transfer.size = uint_from_string(transfer_size->Value());

          // Get transfer in and out
          const auto * in = transfer_it->FirstChildElement("in");
          const auto * out = transfer_it->FirstChildElement("out");
          if (!in || !out) {
            std::string msg = "Transfer definition without in or out tag, net: " +
              safety_net.name;
            throw std::runtime_error(msg);
          }

          getTransferMemoryInfo(in, transfer.input, safety_net.name);
          getTransferMemoryInfo(out, transfer.output, safety_net.name);

          // Record transfer
          safety_net.transfers.push_back(transfer);

          // Iterate
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

void throwErrorIfModuleParametersNotFound(
  const ethercat_interface::EcTransferEntry & transfer,
  const std::string & module_name,
  const std::string & safety_net_name,
  const std::string & direction)
{
  std::string msg = "In safety net: " + safety_net_name + ", for transfer " +
    transfer.to_simple_string() + ", the module name of the " + direction + "( " + module_name +
    ") among all the recorded modules.";
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "EthercatSafetyDriver"), msg.c_str());
  throw std::runtime_error(msg);
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

  // Parse safety modules from the safety yaml file defined
  // in fsoe_config attribute of a param tag of the the URDF
  auto safety_module_params = getEcSafetyModuleParam(info_.original_xml, "fsoe_config");

  // Append the safety modules parameters to the list of modules parameters
  size_t idx_1st = ec_module_parameters_.size();
  ec_module_parameters_.insert(
    ec_module_parameters_.end(), safety_module_params.begin(), safety_module_params.end());
  for (size_t i = 0; i < safety_module_params.size(); i++) {
    ec_safety_slaves_.push_back(idx_1st + i);
  }

  // Parse safety nets from the safety yaml file defined
  // in fsoe_config attribute of a param tag of the the URDF
  ec_safety_nets_ = getEcSafetyNets(info_.original_xml, "fsoe_config");

  // Append the safety modules to the list of modules and load them
  for (const auto & safety_module_param : safety_module_params) {
    try {
      auto ec_module = ec_loader_.createSharedInstance(safety_module_param.at("plugin"));
      if (!ec_module->setupSlave(
          safety_module_param, &empty_interface_, &empty_interface_))
      {
        const std::string & module_name = safety_module_param.at("name");
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Setup of safety only module %s FAILED.", module_name.c_str() );
        return CallbackReturn::ERROR;
      }

      auto idx = ec_modules_.size();
      ec_module->setAliasAndPosition(
        getAliasOrDefaultAlias(safety_module_param),
        std::stoul(safety_module_param.at("position")));
      ec_modules_.push_back(ec_module);
      ec_safety_slaves_.push_back(idx);
    } catch (const pluginlib::PluginlibException & ex) {
      const std::string & module_name = safety_module_param.at("name");
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "EthercatSafetyDriver"),
        "The plugin failed to load for safety module %s. Error: %s\n",
        module_name.c_str(), ex.what());
    }
  }

  // Find all masters from the nets
  {
    std::vector<std::string> master_names;
    for (const auto & net : ec_safety_nets_) {
      master_names.push_back(net.master);
    }
    for (size_t i = 0; i < ec_module_parameters_.size(); i++) {
      if (std::find(
          master_names.begin(), master_names.end(),
          ec_module_parameters_[i].at("name")) !=
        master_names.end())
      {
        ec_safety_masters_.push_back(i);
      }
    }
  }

  // Identify (alias,position) all the modules participating in transfers
  for (auto & net : ec_safety_nets_) {
    for (auto & transfer : net.transfers) {
      // Update each EcMemoryEntry with the alias and position of the module
      auto it_in = std::find_if(
        ec_module_parameters_.begin(), ec_module_parameters_.end(),
        [&transfer](const std::unordered_map<std::string, std::string> & module_param)
        {
          return module_param.at("name") == transfer.input.module_name;
        });
      auto it_out = std::find_if(
        ec_module_parameters_.begin(), ec_module_parameters_.end(),
        [&transfer](const std::unordered_map<std::string, std::string> & module_param)
        {
          return module_param.at("name") == transfer.output.module_name;
        });
      if (it_in == ec_module_parameters_.end()) {
        throwErrorIfModuleParametersNotFound(
          transfer, transfer.input.module_name, net.name, "input");
      }
      if (it_out == ec_module_parameters_.end()) {
        throwErrorIfModuleParametersNotFound(
          transfer, transfer.output.module_name, net.name, "output");
      }

      auto in_idx = std::distance(ec_module_parameters_.begin(), it_in);
      auto out_idx = std::distance(ec_module_parameters_.begin(), it_out);
      const auto & input_module = ec_modules_[in_idx];
      const auto & output_module = ec_modules_[out_idx];

      transfer.input.alias = input_module->alias_;
      transfer.input.position = input_module->position_;
      transfer.output.alias = output_module->alias_;
      transfer.output.position = output_module->position_;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CLASSM::setupMaster()
{
  unsigned int master_id = 666;
  // Get master id
  if (info_.hardware_parameters.find("master_id") == info_.hardware_parameters.end()) {
    // Master id was not provided, default to 0
    master_id = 0;
  } else {
    try {
      master_id = std::stoul(info_.hardware_parameters["master_id"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid master id (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  // Safety master
  safety_ = std::make_shared<ethercat_interface::EcSafety>(master_id);
  master_ = safety_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CLASSM::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ...please wait...");

  // Setup master
  setupMaster();

  // Standard Network configuration
  configNetwork();

  // Safety Network configuration
  safety_->registerTransferInDomain(ec_safety_nets_);

  if (!master_->activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_->update();
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

    // check if operational
    bool isAllInit = true;
    for (auto & module : ec_modules_) {
      isAllInit = isAllInit && module->initialized();
    }
    if (isAllInit) {
      running = false;
    }
    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += master_->getInterval();
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  activated_ = true;

  return CallbackReturn::SUCCESS;
}

}  // namespace ethercat_driver

#undef CLASSM

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatSafetyDriver, hardware_interface::SystemInterface)
