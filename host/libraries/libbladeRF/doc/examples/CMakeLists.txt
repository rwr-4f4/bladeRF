cmake_minimum_required(VERSION 2.8)
project(libbladeRF_doc_snippets C)

set(INCLUDES 
    ${libbladeRF_SOURCE_DIR}/include
    ${BLADERF_HOST_COMMON_SOURCE_DIR}/include
)
set(LIBS libbladerf_shared)

add_executable(libbladeRF_doc_example_sync_rx sync_rx.c example_common.c)
target_link_libraries(libbladeRF_doc_example_sync_rx ${LIBS})
