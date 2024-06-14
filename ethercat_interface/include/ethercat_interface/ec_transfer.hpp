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

#ifndef ETHERCAT_INTERFACE__EC_TRANSFER_HPP_
#define ETHERCAT_INTERFACE__EC_TRANSFER_HPP_

#include <cstdint>

namespace ethercat_interface
{

struct EcTransfer
{
  uint32_t domain_index;
  size_t in_offset;
  size_t out_offset;
  size_t size;
};

} // namespace ethercat_interface

#endif // ETHERCAT_INTERFACE__EC_TRANSFER_HPP_
