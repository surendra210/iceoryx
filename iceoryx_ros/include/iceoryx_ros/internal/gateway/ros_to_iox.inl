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

#ifndef IOX_ros_INTERNAL_GATEWAY_ros_TO_IOX_INL
#define IOX_ros_INTERNAL_GATEWAY_ros_TO_IOX_INL

#include "iceoryx_ros/ros/ros_config.hpp"
#include "iceoryx_ros/internal/log/logging.hpp"
#include "iceoryx_posh/capro/service_description.hpp"
#include "iceoryx_utils/cxx/string.hpp"

#include "gateway/ros_to_iox.hpp"

/* TCP headers */
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <vector>


namespace iox
{
namespace ros
{

template <typename channel_t, typename gateway_t>
inline ros2IceoryxGateway<channel_t, gateway_t>::ros2IceoryxGateway() noexcept
    : Node("ROS_NODE"),gateway_t(iox::capro::Interfaces::DDS, DISCOVERY_PERIOD, FORWARDING_PERIOD)
{
    this->subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "ROS_TOPIC", 200, std::bind(&ros2IceoryxGateway::forwardLocal, this, _1));
    printf("Subscription created!\n");
}

template <typename channel_t, typename gateway_t>
inline ros2IceoryxGateway<channel_t, gateway_t>::~ros2IceoryxGateway() noexcept
{
}

template <typename channel_t, typename gateway_t>
inline void ros2IceoryxGateway<channel_t, gateway_t>::loadConfiguration(const iox::config::GatewayConfig& config) noexcept
{
    iox::LogDebug() << "[ros2IceoryxGateway] Configuring gateway...";
    for (const auto& service : config.m_configuredServices)
    {
        if (!this->findChannel(service.m_serviceDescription).has_value())
        {
            auto serviceDescription =  service.m_serviceDescription;
            iox::LogDebug() << "[ros2IceoryxGateway] Setting up channel for service: {"
                            << serviceDescription.getServiceIDString() << ", "
                            << serviceDescription.getInstanceIDString() << ", "
                            << serviceDescription.getEventIDString() << "}";
            setupChannel(serviceDescription);
        }
    }
}

template <typename channel_t, typename gateway_t>
inline void
ros2IceoryxGateway<channel_t, gateway_t>::discover([[gnu::unused]] const iox::capro::CaproMessage& msg) noexcept
{
    /// @note not implemented - requires ros discovery which is currently not implemented in the used ros stack.
}

template <typename channel_t, typename gateway_t>
inline void ros2IceoryxGateway<channel_t, gateway_t>::forward(const channel_t& channel) noexcept
{
    
}

template <typename channel_t, typename gateway_t>
inline void ros2IceoryxGateway<channel_t, gateway_t>::forwardLocal(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) const
{
    

    uint64_t hashcode{};
    uint64_t payloadSize{msg->data.size() - sizeof(hashcode)}; // size of data packet = size of hashcode (8 bytes) + size of payload

    // RCLCPP_INFO(this->get_logger(), "I have %u bytes of payload\n",payloadSize);

    memcpy((void*) &hashcode, (void *)&(msg->data.at(0)), sizeof(hashcode));

    // RCLCPP_INFO(this->get_logger(),"hashcode : %lu",hashcode);
    
    this->forEachChannel([this,&hashcode, &payloadSize, &msg](channel_t channel) { 
        auto reader = channel.getExternalTerminal();
        printf("Inside forwardlocal %lu : %lu\n",reader->getUniqueCode(), hashcode);
        if(reader->getUniqueCode() == hashcode){
            /* it's a match for publisher */
            auto publisher = channel.getIceoryxTerminal();
            auto publisherData = publisher->allocateChunk(payloadSize);
            memcpy((void*)publisherData, (void *)&(msg->data.at(sizeof(hashcode))), payloadSize);
            publisher->sendChunk(publisherData);
        }
    });
}

// ======================================== Private ======================================== //
template <typename channel_t, typename gateway_t>
iox::cxx::expected<channel_t, iox::gw::GatewayError>
ros2IceoryxGateway<channel_t, gateway_t>::setupChannel(const iox::capro::ServiceDescription& service) noexcept
{
    
    return this->addChannel(service).and_then([&service](channel_t channel) {
        auto publisher = channel.getIceoryxTerminal();
        auto reader = channel.getExternalTerminal();
        publisher->offer();
        reader->setUniqueCode(service);
        reader->connect();   
        iox::LogDebug() << "[ros2IceoryxGateway] Setup channel for service: {" << service.getServiceIDString() << ", "
                        << service.getInstanceIDString() << ", " << service.getEventIDString() << "}";
    });
}

} // namespace ros
} // namespace iox

#endif
