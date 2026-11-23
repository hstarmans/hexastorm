# Create an INTERFACE library for our C module.
add_library(usermod_vector_core INTERFACE)

target_sources(usermod_vector_core INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/vector_core.c
    ${CMAKE_CURRENT_LIST_DIR}/modvector_core.c
)

target_include_directories(usermod_vector_core INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_vector_core)
