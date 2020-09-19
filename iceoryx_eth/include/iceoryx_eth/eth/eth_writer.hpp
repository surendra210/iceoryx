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

#ifndef IOX_eth_eth_CYCLONE_DATA_WRITER_HPP
#define IOX_eth_eth_CYCLONE_DATA_WRITER_HPP

#include "eth/data_writer.hpp"
#include "iceoryx_eth/eth/eth_gatewayconf.hpp"

#include "eth/eth_context.hpp"
#include "internal/log/logging.hpp"

#include <string>

/* Tcp headers */
#include <string>
#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#define PORT 8080 

namespace iox
{
namespace eth
{
///
/// @brief Implementation of the DataWriter abstraction using the eth implementation.
///
class ethDataWriter : public iox::eth::DataWriter
{
  public:
    ethDataWriter() = delete;
    ethDataWriter(const IdString serviceId, const IdString instanceId, const IdString eventId);
    virtual ~ethDataWriter();
    ethDataWriter(const ethDataWriter&) = delete;
    uint8_t setUniqueCode(const iox::capro::ServiceDescription&);
    int SetSocketChannelID(int);
    void connect() noexcept override;
    void write(const uint8_t* const bytes, const uint64_t size) noexcept override;
    IdString getServiceId() const noexcept override;
    IdString getInstanceId() const noexcept override;
    IdString getEventId() const noexcept override;

  private:
    IdString m_serviceId{""};
    IdString m_instanceId{""};
    IdString m_eventId{""};
    int client_handle{-1};
    union uniCode
    {
     uint64_t unique_code{0};
     uint8_t  u8Array[8];
    }ServiceHash;  
};

} // namespace eth
} // namespace iox

#endif // IOX_eth_eth_CYCLONE_DATA_WRITER_HPP
