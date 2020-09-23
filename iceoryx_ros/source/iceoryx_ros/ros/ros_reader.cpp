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

#include "ros/ros_reader.hpp"
#include "ros/ros_context.hpp"
#include "internal/log/logging.hpp"

iox::ros::rosDataReader::rosDataReader(IdString serviceId, IdString instanceId, IdString eventId) noexcept
    : m_serviceId(serviceId)
    , m_instanceId(instanceId)
    , m_eventId(eventId)
{
    LogDebug() << "[rosDataReader] Created rosDataReader.";
}

iox::ros::rosDataReader::~rosDataReader()
{
    LogDebug() << "[rosDataReader] Destroyed rosDataReader.";
}

uint64_t iox::ros::rosDataReader::getUniqueCode(){ return ServiceHash.unique_code; };

void iox::ros::rosDataReader::setUniqueCode(const iox::capro::ServiceDescription& service){
    auto ServiceString     = "/" + service.getServiceIDString()+ "/"+service.getInstanceIDString()+"/"+service.getEventIDString();
    std::size_t hashcode   = std::hash<std::string>{}(ServiceString);
    ServiceHash.unique_code=hashcode;
}

void iox::ros::rosDataReader::connect() noexcept
{
}

iox::cxx::optional<uint64_t> iox::ros::rosDataReader::peekNextSize()
{

    return iox::cxx::nullopt_t();
}

iox::cxx::expected<iox::ros::DataReaderError> iox::ros::rosDataReader::takeNext(uint8_t* const buffer,
                                                                                    const uint64_t& bufferSize)
{

    return iox::cxx::success<>();
}

iox::cxx::expected<uint64_t, iox::ros::DataReaderError> iox::ros::rosDataReader::take(
    uint8_t* const buffer, const uint64_t& bufferSize, const iox::cxx::optional<uint64_t>& maxSamples)
{

}

iox::ros::IdString iox::ros::rosDataReader::getServiceId() const noexcept
{
    return m_serviceId;
}

iox::ros::IdString iox::ros::rosDataReader::getInstanceId() const noexcept
{
    return m_instanceId;
}

iox::ros::IdString iox::ros::rosDataReader::getEventId() const noexcept
{
    return m_eventId;
}
