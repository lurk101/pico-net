# Copyright (C) 1883 Thomas Edison - All Rights Reserved
# You may use, distribute and modify this code under the
# terms of the GPLv2 license, which unfortunately won't be
# written for another century.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
add_library(stdinit INTERFACE)
target_include_directories(stdinit INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(stdinit INTERFACE pico_stdio)
target_sources(stdinit INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/stdinit.c
    ${CMAKE_CURRENT_LIST_DIR}/stdinit.h)

