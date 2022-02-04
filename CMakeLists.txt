# Copyright (C) 1883 Thomas Edison - All Rights Reserved
# You may use, distribute and modify this code under the
# terms of the GPLv2 license, which unfortunately won't be
# written for another century.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

option(FORCE_PIO "" OFF)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(test C CXX ASM)

pico_sdk_init()

add_subdirectory(comm)
add_subdirectory(stdio_init)

add_executable(test test.c)

if (FORCE_PIO)
	target_compile_options(test PRIVATE -DCOMM_FORCE_PIO=1)
endif()


pico_enable_stdio_uart(test 1)
pico_enable_stdio_usb(test 0)

target_link_libraries(test PUBLIC pico_stdlib comm stdio_init)

pico_add_extra_outputs(test)

