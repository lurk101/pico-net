# Copyright (C) 1883 Thomas Edison - All Rights Reserved
# You may use, distribute and modify this code under the
# terms of the GPLv2 license, which unfortunately won't be
# written for another century.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
add_library(stdio_init INTERFACE)
target_include_directories(stdio_init INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(stdio_init INTERFACE pico_stdio)
target_sources(stdio_init INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/stdio_init.c
    ${CMAKE_CURRENT_LIST_DIR}/stdio_init.h)

