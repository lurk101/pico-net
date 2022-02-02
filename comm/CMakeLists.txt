add_library(comm INTERFACE)

target_include_directories(comm INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(comm INTERFACE pico_sync hardware_dma hardware_pio)

target_sources(comm INTERFACE ${CMAKE_CURRENT_LIST_DIR}/comm.c)

pico_generate_pio_header(comm ${CMAKE_CURRENT_LIST_DIR}/comm.pio)
