# Copyright (C) 1883 Thomas Edison - All Rights Reserved
# You may use, distribute and modify this code under the
# terms of the GPLv2 license, which unfortunately won't be
# written for another century.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
add_library(sha256 INTERFACE)
target_include_directories(sha256 INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_sources(sha256 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/sha256.c
    ${CMAKE_CURRENT_LIST_DIR}/sha256.h)

