// Copyright (c) 2020 by Robert Bosch GmbH. All rights reserved.
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

#ifndef IOX_DDS_DDS_DDS_TYPES_HPP
#define IOX_DDS_DDS_DDS_TYPES_HPP

#ifdef USE_CYCLONE_DDS
#include "eth/eth_writer.hpp"
#include "eth/eth_reader.hpp"
#else
#error "A eth implementation must be provided."
#endif

namespace iox
{
namespace eth
{
// Fixed strings
using IdString = iox::cxx::string<100u>;

// DDS implementation defined with compiler flags
#ifdef USE_CYCLONE_DDS
using data_reader_t = iox::eth::ethDataReader;
using data_writer_t = iox::eth::ethDataWriter;
#else
#error "A DDS implementation must be set."
#endif
} // namespace ipcf
} // namespace iox

#endif // IOX_DDS_DDS_DDS_TYPES_HPP