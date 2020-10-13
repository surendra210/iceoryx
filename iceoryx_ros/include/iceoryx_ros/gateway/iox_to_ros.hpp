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

#ifndef IOX_ros_GATEWAY_IOX_TO_DDS_HPP
#define IOX_ros_GATEWAY_IOX_TO_DDS_HPP

#include "ros/ros_types.hpp"
#include "iceoryx_posh/gateway/channel.hpp"
#include "iceoryx_posh/gateway/gateway_generic.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"

/* Tcp headers */
#include <string>
#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 

namespace iox
{
namespace ros
{
///
/// @brief  Gateway implementation for the iceoryx to ros direction.
///
template <typename channel_t = iox::gw::Channel<iox::popo::Subscriber, iox::ros::data_writer_t>,
          typename gateway_t = iox::gw::GatewayGeneric<channel_t>>
class Iceoryx2rosGateway : public gateway_t
{
  public:
    Iceoryx2rosGateway() noexcept;
    void loadConfiguration(const iox::config::GatewayConfig& config) noexcept;
    void discover(const iox::capro::CaproMessage& msg) noexcept;
    void forward(const channel_t& channel) noexcept;
    int GetSocketChannelID();
   ~Iceoryx2rosGateway() noexcept;
  private:
    iox::cxx::expected<channel_t, iox::gw::GatewayError>
    setupChannel(const iox::capro::ServiceDescription& service) noexcept;

};

} // namespace ros
} // namespace iox

#include "internal/gateway/iox_to_ros.inl"

#endif // IOX_ros_GATEWAY_IOX_TO_DDS_HPP
