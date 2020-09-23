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

#ifndef IOX_ros_ros_CYCLONE_DATA_READER_HPP
#define IOX_ros_ros_CYCLONE_DATA_READER_HPP

#include "ros/data_reader.hpp"

#include "iceoryx_posh/capro/service_description.hpp"
#include "iceoryx_posh/iceoryx_posh_types.hpp"

namespace iox
{
namespace ros
{
///
/// @brief Implementation of the DataReader abstraction using the ros implementation.
///
class rosDataReader : public DataReader
{
  public:
    rosDataReader() = delete;
    rosDataReader(IdString serviceId, IdString instanceId, IdString eventId) noexcept;
    virtual ~rosDataReader();

    rosDataReader(const rosDataReader&) = delete;
    rosDataReader& operator=(const rosDataReader&) = delete;
    rosDataReader(rosDataReader&&) = delete;
    rosDataReader& operator=(rosDataReader&&) = delete;

    void connect() noexcept override;

    void setUniqueCode(const iox::capro::ServiceDescription&);
    uint64_t getUniqueCode();

    iox::cxx::optional<uint64_t> peekNextSize() override;

    iox::cxx::expected<DataReaderError> takeNext(uint8_t* const buffer, const uint64_t& bufferSize) override;

    iox::cxx::expected<uint64_t, DataReaderError>
    take(uint8_t* const buffer, const uint64_t& bufferSize, const iox::cxx::optional<uint64_t>& maxSamples) override;

    IdString getServiceId() const noexcept override;
    IdString getInstanceId() const noexcept override;
    IdString getEventId() const noexcept override;

  private:
    IdString m_serviceId{""};
    IdString m_instanceId{""};
    IdString m_eventId{""};
    union uniCode
    {
     uint64_t unique_code{0};
     uint8_t  u8Array[8];
    }ServiceHash;  
};

} // namespace ros
} // namespace iox

#endif // IOX_ros_ros_CYCLONE_DATA_READER_HPP
