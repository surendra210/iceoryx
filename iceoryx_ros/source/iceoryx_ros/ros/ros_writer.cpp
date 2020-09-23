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

#include "ros/ros_writer.hpp"
#include "ros/ros_context.hpp"
#include "internal/log/logging.hpp"

//#include <Mempool_DCPS.hpp>
#include <string>
#include <vector>

iox::ros::rosDataWriter::rosDataWriter(IdString serviceId, IdString instanceId, IdString eventId)
    : m_serviceId(serviceId)
    , m_instanceId(instanceId)
    , m_eventId(eventId)
{
    LogDebug() << "[rosDataWriter] Created rosDataWriter.";
}

iox::ros::rosDataWriter::~rosDataWriter()
{

}

void iox::ros::rosDataWriter::setUniqueCode(const iox::capro::ServiceDescription& service){

    auto ServiceString      = "/" + service.getServiceIDString()+ "/"+service.getInstanceIDString()+"/"+service.getEventIDString();
    std::size_t hashcode    = std::hash<std::string>{}(ServiceString);
    ServiceHash.unique_code =hashcode;
}

int iox::ros::rosDataWriter::SetSocketChannelID(int SocketChannelID)
{
    int ret=-1;
    if(SocketChannelID!=-1){
     client_handle = SocketChannelID;
     ret=0;
    }
   return ret; 
}

void iox::ros::rosDataWriter::connect() noexcept
{
   std::cout << "Testing the connect" << std::endl ;  
}

void iox::ros::rosDataWriter::write(const uint8_t* const bytes, const uint64_t size) noexcept
{
   
}
iox::ros::IdString iox::ros::rosDataWriter::getServiceId() const noexcept
{
    return m_serviceId;
}

iox::ros::IdString iox::ros::rosDataWriter::getInstanceId() const noexcept
{
    return m_instanceId;
}

iox::ros::IdString iox::ros::rosDataWriter::getEventId() const noexcept
{
    return m_eventId;
}

