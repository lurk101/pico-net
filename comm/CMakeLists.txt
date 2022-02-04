# Copyright (C) 1883 Thomas Edison - All Rights Reserved
# You may use, distribute and modify this code under the
# terms of the GPLv2 license, which unfortunately won't be
# written for another century.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
add_library(comm INTERFACE)
target_include_directories(comm INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(comm INTERFACE
	pico_sync
	pico_multicore
	hardware_dma
	hardware_pio)
target_sources(comm INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/comm.c
	${CMAKE_CURRENT_LIST_DIR}/comm.h)
pico_generate_pio_header(comm ${CMAKE_CURRENT_LIST_DIR}/comm.pio)
