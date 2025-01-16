# Copyright (C) 2015-2017 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

#This file is part of the VSOMEIP example code, originally retrieved from:
#https://github.com/COVESA/vsomeip/tree/master/examples

cmake_minimum_required(VERSION 3.25)

project(examples)

include(CMakePackageConfigHelpers)

set(EXAMPLE_CONFIG_FILES
    "../Config/vsomeip-udp-client.json"
)

# Find required packages
find_package(vsomeip3 REQUIRED)
find_package(Boost REQUIRED)

# Examples
add_executable(request-sample request-sample.cpp ${EXAMPLE_CONFIG_FILES})
target_link_libraries(request-sample vsomeip3 ${Boost_LIBRARIES} ${DL_LIBRARY})

# Create a custom target named 'examples' to group the sample executables
add_custom_target(examples DEPENDS request-sample)

install(
    TARGETS request-sample RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
)
