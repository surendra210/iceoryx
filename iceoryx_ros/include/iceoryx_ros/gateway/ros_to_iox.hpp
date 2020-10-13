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

#ifndef IOX_ros_GATEWAY_ros_TO_IOX_HPP
#define IOX_ros_GATEWAY_ros_TO_IOX_HPP
#include "iceoryx_ros/ros/ros_types.hpp"
#include "iceoryx_posh/gateway/channel.hpp"
#include "iceoryx_posh/gateway/gateway_generic.hpp"
#include "iceoryx_posh/gateway/gateway_config.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
using std::placeholders::_1;


namespace iox
{
namespace ros
{

///
/// @brief ros Gateway implementation for the ros to iceoryx direction.
///
template <typename channel_t = iox::gw::Channel<iox::popo::Publisher, iox::ros::data_reader_t>,
          typename gateway_t = iox::gw::GatewayGeneric<channel_t>>
class ros2IceoryxGateway : public rclcpp::Node, public gateway_t
{
    using ChannelFactory = std::function<channel_t(const iox::capro::ServiceDescription)>;

  public:
    ros2IceoryxGateway() noexcept;
    ros2IceoryxGateway(ChannelFactory channelFactory) noexcept;
    void loadConfiguration(const iox::config::GatewayConfig& config) noexcept;
    void discover(const iox::capro::CaproMessage& msg) noexcept;
    void forward(const channel_t& channel) noexcept;
    void forwardLocal(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) const;
    ~ros2IceoryxGateway() noexcept;
  private:
    void* m_reservedChunk = nullptr;
    iox::cxx::expected<channel_t, iox::gw::GatewayError>
    setupChannel(const iox::capro::ServiceDescription& service) noexcept;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
};

} // namespace ros
} // namespace iox

#include "iceoryx_ros/internal/gateway/ros_to_iox.inl"

#endif // IOX_ros_GATEWAY_eth_TO_IOX_HPP
