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

#include "ethercat_interface/ec_safety.hpp"
#include "ethercat_interface/ec_slave.hpp"

namespace ethercat_interface
{

void EcSafety::checkDomainInfoValidity(
  const DomainInfo & domain_info,
  const ec_pdo_entry_reg_t & pdo_entry_reg)
{
  if (nullptr == domain_info.domain_pd) {
    throw std::runtime_error("Domain process data pointer not set.");
  }
  if (nullptr == pdo_entry_reg.offset) {
    throw std::runtime_error("Offset not set in pdo_entry_reg.");
  }
}

void EcSafety::registerTransferInDomain(const std::vector<EcSafetyNet> & safety_nets)
{
  // Fill in the EcTransferInfo structures

  // For each transfer of each net,
  for (auto & net : safety_nets) {
    for (auto & transfer : net.transfers) {
      EcTransferInfo transfer_info;
      transfer_info.size = transfer.size;
      // For the input and the output of the transfer find
      //  1. the process domain data pointer
      //  2. the offset in the process domain data
      // By iterating over the existing DomainInfo and domain_regs vector to find the ec_pdo_entry_reg_t whose alias, position, index and subindex match the transfer input and output memory entries
      for (const auto & key_val : domain_info_) {
        const DomainInfo & domain = *(key_val.second);
        for (auto & domain_reg : domain.domain_regs) {
          // Find match for input
          if (domain_reg.alias == transfer.input.alias &&
            domain_reg.position == transfer.input.position &&
            domain_reg.index == transfer.input.index &&
            domain_reg.subindex == transfer.input.subindex)
          {

            transfer_info.input_domain = &domain;
            // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.in_ptr = domain.domain_pd + *(domain_reg.offset);
          }
          // Find match for output
          if (domain_reg.alias == transfer.output.alias &&
            domain_reg.position == transfer.output.position &&
            domain_reg.index == transfer.output.index &&
            domain_reg.subindex == transfer.output.subindex)
          {
            transfer_info.output_domain = &domain;
            // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.out_ptr = domain.domain_pd + *(domain_reg.offset);
          }
        }
      }

      // Record the transfer
      transfers_.push_back(transfer_info);
    }
  }
}

EcSafety::EcSafety(const unsigned int master)
: EcMaster(master)
{
}

EcSafety::~EcSafety()
{
}

}  // namespace ethercat_interface
