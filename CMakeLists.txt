cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

pico_sdk_init()

project(example C CXX ASM)

add_subdirectory(comm)

add_executable(example example.c)

pico_enable_stdio_uart(example 1)
pico_enable_stdio_usb(example 0)

target_link_libraries(example PUBLIC pico_stdlib comm)

pico_add_extra_outputs(example)
