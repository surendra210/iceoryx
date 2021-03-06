// Copyright (c) 2019 by Robert Bosch GmbH. All rights reserved.
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

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "topic_data.hpp"

#include <chrono>
#include <csignal>
#include <iostream>

bool killswitch = false;

static void sigHandler(int f_sig[[gnu::unused]])
{
    // caught SIGINT, now exit gracefully
    killswitch = true;
}
uint16_t delay_factor{1000};
uint16_t delay_factor2{100};

void sending()
{
    // Create the runtime for registering with the RouDi daemon
    iox::runtime::PoshRuntime::getInstance("/iox-ex-publisher-bare-metal");

    // Create a publisher
    iox::popo::Publisher myPublisher({"RADAR", "Video", "Some1"});

    // With offer() the publisher gets visible to potential subscribers
    myPublisher.offer();

    uint32_t ct = 0;
        
    while (!killswitch)
    {
        // Allocate a memory chunk for the sample to be sent
        auto sample = static_cast<PoshPub1*>(myPublisher.allocateChunk(sizeof(PoshPub1)));

        std::this_thread::sleep_for(std::chrono::milliseconds(delay_factor));

        // Write sample data
        sample->doubleValue = ct* (-21.0);
        sample->integer = ct;
        sample->floating_pt = ct * 5.28;
        strcpy(sample->word, "This is a sample string");


        std::cout << "Sending: " << sample->doubleValue << " : " << sample->integer << " : " << sample->floating_pt << " : "
        << sample->word << std::endl;

        // Send the sample
        myPublisher.sendChunk(sample);
        delay_factor = delay_factor2;
        ct++;

         // Sleep some time to avoid flooding the system with messages as there's basically no delay in transfer
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        
    }

    // with stopOffer we disconnect all subscribers and the publisher is no more visible
    myPublisher.stopOffer();
}

int main(int argc, char* argv[])
{
    if(argc == 3){
        delay_factor = atoi(argv[1]);
        delay_factor2 = atoi(argv[2]);
    }
    
    // Register sigHandler for SIGINT
    printf("Delay : %hu, Delay2 : %hu\n", delay_factor, delay_factor2);
    signal(SIGINT, sigHandler);

    std::thread tx(sending);
    tx.join();

    return (EXIT_SUCCESS);
}
