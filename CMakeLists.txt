cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

option(FORCE_PIO "" OFF)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(example C CXX ASM)

pico_sdk_init()

add_subdirectory(comm)
add_subdirectory(stdio_init)

add_executable(example example.c)

if (FORCE_PIO)
	target_compile_options(example PRIVATE -DCOMM_FORCE_PIO=1)
endif()


pico_enable_stdio_uart(example 1)
pico_enable_stdio_usb(example 0)

target_link_libraries(example PUBLIC pico_stdlib comm stdio_init)

pico_add_extra_outputs(example)

