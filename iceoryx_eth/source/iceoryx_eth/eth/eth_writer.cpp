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

#include "eth/eth_writer.hpp"
#include "eth/eth_context.hpp"
#include "internal/log/logging.hpp"

//#include <Mempool_DCPS.hpp>
#include <string>
extern "C"{
    #include "eth/ipc-shm.h"
}
#include <vector>

iox::eth::ethDataWriter::ethDataWriter(IdString serviceId, IdString instanceId, IdString eventId)
    : m_serviceId(serviceId)
    , m_instanceId(instanceId)
    , m_eventId(eventId)
{
    LogDebug() << "[ethDataWriter] Created ethDataWriter.";
}

iox::eth::ethDataWriter::~ethDataWriter()
{

}

void iox::eth::ethDataWriter::setUniqueCode(const iox::capro::ServiceDescription& service){

    auto ServiceString      = "/" + service.getServiceIDString()+ "/"+service.getInstanceIDString()+"/"+service.getEventIDString();
    std::size_t hashcode    = std::hash<std::string>{}(ServiceString);
    ServiceHash.unique_code =hashcode;
}

int iox::eth::ethDataWriter::SetSocketChannelID(int SocketChannelID)
{
    int ret=-1;
    if(SocketChannelID!=-1){
     client_handle = SocketChannelID;
     ret=0;
    }
   return ret; 
}

void iox::eth::ethDataWriter::connect() noexcept
{
   std::cout << "Testing the connect" << std::endl ;  
}

void iox::eth::ethDataWriter::write(const uint8_t* const bytes, const uint64_t size) noexcept
{
    uint8_t size_array[sizeof(size)/sizeof(uint8_t)];
    std::vector<uint8_t> gatewayWrapper;

    //std::cout << "Testing the write Byte Address Received" <<bytes << "Size received"<< size << std::endl ;     
    if(client_handle >= 0){
        std::cout << "client handle : " << client_handle << std::endl;

        // GatewayWrapper is the packet that will be sent over Eth, (Header(ServiceHash,SizeOfPublisherData))

        //gatewayWrapper.push_back(unique_code);
        gatewayWrapper.insert(gatewayWrapper.end(),&ServiceHash.u8Array[0],&ServiceHash.u8Array[sizeof(uint64_t)]);
        
        (void) memcpy(size_array, &size, sizeof(size));
        //Size of Publish Data
        gatewayWrapper.insert(gatewayWrapper.end(),&size_array[0],&size_array[sizeof(size_array)]);

        if(-1 != send(client_handle, (std::vector<uint8_t>*)&gatewayWrapper[0], gatewayWrapper.size() , 0 ))
        {
            if(-1 != send(client_handle, bytes,size, 0 )){
                std::cout << "Sent payload!" << std::endl;
            }
            else{
                std::cout << "Send data failed!" << std::endl;
            }
        }
        else{
            std::cout << "Send data header failed!" << std::endl;
        }        
    }
    else{
        std::cout << "client handle is -1 !!" << std::endl;
    }

}
iox::eth::IdString iox::eth::ethDataWriter::getServiceId() const noexcept
{
    return m_serviceId;
}

iox::eth::IdString iox::eth::ethDataWriter::getInstanceId() const noexcept
{
    return m_instanceId;
}

iox::eth::IdString iox::eth::ethDataWriter::getEventId() const noexcept
{
    return m_eventId;
}

