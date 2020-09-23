# Copyright (c) 2020 by Robert Bosch GmbH. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.7)

file (STRINGS "../VERSION" iceoryx_ros_VERSION)
project(iceoryx_ros VERSION ${iceoryx_ros_VERSION})

option(test "build test binaries")

find_package(cpptoml REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_utils REQUIRED)

include(IceoryxPackageHelper)
include(IceoryxPlatformDetection)

#
########## set variables for export ##########
#
setup_package_name_and_create_files(
    NAME ${PROJECT_NAME}
    NAMESPACE iceoryx_ros
    PROJECT_PREFIX ${PREFIX}
)

#
########## build building-block library ##########
#


###################################################################################
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/iceoryx_ros
)

add_library(iceoryx_ros
    STATIC
    source/iceoryx_ros/log/logging.cpp
)

target_compile_options(iceoryx_ros PUBLIC -std=c++11)

target_include_directories(iceoryx_ros
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/dependencies/install/include>
    $<INSTALL_INTERFACE:include/${PREFIX}>
)

target_link_libraries(iceoryx_ros
    PRIVATE
    iceoryx_posh::iceoryx_posh
    iceoryx_utils::iceoryx_utils
    iceoryx_posh::iceoryx_posh_gateway
    iceoryx_posh::iceoryx_posh_config
)

################################################

    target_sources(iceoryx_ros
        PRIVATE
        source/iceoryx_ros/ros/ros_context.cpp
        source/iceoryx_ros/ros/ros_writer.cpp
        source/iceoryx_ros/ros/ros_reader.cpp
    )

#
########## build gateway apps ##########
#
add_executable(iox-gw-iceoryx2ros
    source/iceoryx2ros_app/main.cpp
)

target_link_libraries(iox-gw-iceoryx2ros
    PRIVATE
    iceoryx_posh::iceoryx_posh
    iceoryx_ros
)

target_compile_options(iox-gw-iceoryx2ros PRIVATE ${ICEORYX_WARNINGS})



add_executable(iox-gw-ros2iceoryx
    source/ros2iceoryx_app/main.cpp
)

target_link_libraries(iox-gw-ros2iceoryx
    PRIVATE
    iceoryx_posh::iceoryx_posh
    iceoryx_ros
)

target_compile_options(iox-gw-iceoryx2ros PRIVATE ${ICEORYX_WARNINGS})
#
########## build test executables ##########
#
if(test)
    add_subdirectory(test)
endif(test)


#
########## export library ##########
#
setup_install_directories_and_export_package(
    TARGETS  iox-gw-iceoryx2ros iox-gw-ros2iceoryx
    INCLUDE_DIRECTORY include/
)