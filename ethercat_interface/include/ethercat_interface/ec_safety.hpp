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

#ifndef ETHERCAT_INTERFACE__EC_SAFETY_HPP_
#define ETHERCAT_INTERFACE__EC_SAFETY_HPP_

#include <ecrt.h>

#include <string>
#include <vector>

#include "ethercat_interface/ec_master.hpp"

namespace ethercat_interface
{

class EcSafetyEntry
{
public:
  std::string module_name;   //< Module.
  uint16_t index;            //< PDO entry index.
};

class EcSafetyTransfer
{
public:
  EcSafetyEntry input;
  EcSafetyEntry output;
  size_t size;   //< Size of the exchange data.
};

class EcSafetyNet
{
public:
  std::string name;                          //< safety net name
  EcSafetyModule master;                     //< safety master
  std::vector<EcSafetyTransfer> transfers;   //< safety data transfers

public:
  void reset(const std::string & new_name)
  {
    name = new_name;
    master.name = "";
    master.module_info = nullptr;
    transfers.clear();
  }
};

class EcTransferInfo
{
public:
  uint32_t domain_index;
  size_t in_offset;
  size_t out_offset;
  size_t size;
};

class EcTransferDomainInfo
{
public:
  uint8_t * domain_address;
  std::vector<EcTransferInfo> transfers;
};

class EcSafety : public EcMaster
{
public:

protected:
  std::vector<EcTransferDomainInfo> transfer_domains_;
};

} // namespace ethercat_interface

#endif // ETHERCAT_INTERFACE__EC_SAFETY_HPP_
