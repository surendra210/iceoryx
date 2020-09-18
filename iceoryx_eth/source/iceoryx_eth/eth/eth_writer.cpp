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

uint8_t iox::eth::ethDataWriter::setUniqueCode(const iox::capro::ServiceDescription& service){

    uint8_t ret = 1u;
    for(uint8_t idx = 0u; idx < pMap.size(); ++idx){
        if(service == pMap[idx].m_serviceDescription){
            unique_code = pMap[idx].unique_id;
            ret = 0; //success - found
            break;
        }
    }
    if(ret == 1)
        std::cout << "Unknown service!" << std::endl;
    return ret;
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

    //std::cout << "Testing the write Byte Address Received" <<bytes << "Size received"<< size << std::endl ;     
    if(client_handle >= 0){
        std::cout << "client handle : " << client_handle << std::endl;

        // GatewayWrapper is the packet that will be sent over Eth, (Header + Payload)
        std::vector<uint8_t> gatewayWrapper;

        gatewayWrapper.push_back(unique_code);

        uint8_t size_array[sizeof(size)/sizeof(uint8_t)];
        (void) memcpy(size_array, &size, sizeof(size));
        //Size of Publish Data
        gatewayWrapper.insert(gatewayWrapper.end(),&size_array[0],&size_array[sizeof(size_array)]);

        if(-1 != send(client_handle, (std::vector<uint8_t>*)&gatewayWrapper[0], gatewayWrapper.size() , 0 ))
        {
            if(-1 != send(client_handle, bytes,size, 0 )){
                std::cout << "Send data payload failed!" << std::endl;
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

