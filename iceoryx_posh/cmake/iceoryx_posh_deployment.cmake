# configure deployment
if(NOT IOX_MAX_PUBLISHERS)
    set(IOX_MAX_PUBLISHERS 512)
endif()

if(NOT IOX_MAX_SUBSCRIBERS)
    set(IOX_MAX_SUBSCRIBERS 1024)
endif()

if(NOT IOX_MAX_INTERFACE_NUMBER)
    set(IOX_MAX_INTERFACE_NUMBER 4)
endif()

if(NOT IOX_MAX_SUBSCRIBERS_PER_PUBLISHER)
    set(IOX_MAX_SUBSCRIBERS_PER_PUBLISHER 256)
endif()

if(NOT IOX_MAX_CHUNKS_ALLOCATED_PER_PUBLISHER_SIMULTANEOUSLY)
    set(IOX_MAX_CHUNKS_ALLOCATED_PER_PUBLISHER_SIMULTANEOUSLY 8)
endif()

if(NOT IOX_MAX_PUBLISHER_HISTORY)
    set(IOX_MAX_PUBLISHER_HISTORY 16)
endif()

if(NOT IOX_MAX_CHUNKS_HELD_PER_SUBSCRIBER_SIMULTANEOUSLY)
    set(IOX_MAX_CHUNKS_HELD_PER_SUBSCRIBER_SIMULTANEOUSLY 256)
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/iceoryx_posh_deployment.hpp.in"
  "${CMAKE_BINARY_DIR}/generated/iceoryx/include/iceoryx_posh/iceoryx_posh_deployment.hpp" @ONLY)
