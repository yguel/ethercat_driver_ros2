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

#ifndef ETHERCAT_INTERFACE__EC_SAFETY_HPP_
#define ETHERCAT_INTERFACE__EC_SAFETY_HPP_

#include <ecrt.h>

#include <string>
#include <vector>

#include "ethercat_interface/ec_master.hpp"

namespace ethercat_interface
{

class EcSafetyModule
{
public:
  std::string name;  //< Module name (ec_module tag)
  ec_slave_info_t * module_info;  //< Module info
};

class EcSafetyEntry
{
public:
  std::string module_name;
  uint16_t index;  //< PDO entry index.
};

class EcSafetyTransfer
{
public:
  EcSafetyEntry input;
  EcSafetyEntry output;
  size_t size;  //< Size of the exchange data.
};

class EcSafetyNet
{
public:
  EcSafetyModule master;  //< safety master
  std::vector<EcSafetyModule> slaves;  //< safety slaves
  std::vector<EcSafetyTransfer> transfers;  //< safety data transfers
};

class EcSafetyTransferInfo
{
public:
  size_t in_offset;
  size_t out_offset;
  size_t size;
};


class EcSafetyDomainInfo
{
public:
  uint8_t * domain_address;
  std::vector<EcSafetyTransferInfo> transfers;
};

class EcSafety : public EcMaster
{
public:
  explicitly EcSafety(const unsigned int master = 0);
  ~EcSafety();

  void addSafetyModule(const std::string & name);
};


}  // namespace ethercat_interface

#endif  // ETHERCAT_INTERFACE__EC_SAFETY_HPP_
