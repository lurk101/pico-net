add_library(mylib INTERFACE)

target_include_directories(mylib INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(mylib INTERFACE pico_stdlib)

target_sources(mylib INTERFACE ${CMAKE_CURRENT_LIST_DIR}/mylib.c)
