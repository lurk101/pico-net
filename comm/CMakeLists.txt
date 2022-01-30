add_library(comm INTERFACE)

target_include_directories(comm INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(comm INTERFACE pico_sync hardware_dma)

target_sources(comm INTERFACE ${CMAKE_CURRENT_LIST_DIR}/comm.c)
