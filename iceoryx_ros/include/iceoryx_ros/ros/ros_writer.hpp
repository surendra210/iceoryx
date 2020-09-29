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

#ifndef IOX_ros_ros_CYCLONE_DATA_WRITER_HPP
#define IOX_ros_ros_CYCLONE_DATA_WRITER_HPP

#include "ros/data_writer.hpp"
#include "ros/ros_context.hpp"
#include "internal/log/logging.hpp"

#include "iceoryx_posh/capro/service_description.hpp"
#include "iceoryx_posh/iceoryx_posh_types.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace iox
{
namespace ros
{
///
/// @brief Implementation of the DataWriter abstraction using the ros implementation.
///
class rosDataWriter : public iox::ros::DataWriter, public rclcpp::Node
{
  public:
    rosDataWriter() = delete;
    rosDataWriter(const IdString serviceId, const IdString instanceId, const IdString eventId);
    virtual ~rosDataWriter();
    rosDataWriter(const rosDataWriter&) = delete;
    void setUniqueCode(const iox::capro::ServiceDescription&);
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
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_{nullptr};
    union uniCode
    {
     uint64_t unique_code{0};
     uint8_t  u8Array[8];
    }ServiceHash;  
};

} // namespace ros
} // namespace iox

#endif // IOX_ros_ros_CYCLONE_DATA_WRITER_HPP
