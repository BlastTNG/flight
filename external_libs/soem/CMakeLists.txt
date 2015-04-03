project (ethercat)
cmake_minimum_required (VERSION 2.8)

# Set global compile flags
set(CMAKE_C_FLAGS "-std=gnu99 -ffunction-sections -Wall -Wextra -Wno-unused-parameter -fomit-frame-pointer -fno-strict-aliasing")
set(CMAKE_LD_FLAGS ${CMAKE_LD_FLAGS} "-Wl,--gc-sections")

# Find external libs
find_package(PkgConfig)

# Find Threading lib
find_package (Threads)

# add the binary tree to the search path for include files so that we will find mcp_config.h
include_directories ("${PROJECT_BINARY_DIR}")

set(OSAL_DIR "${PROJECT_SOURCE_DIR}/osal")
set(OSHW_DIR "${PROJECT_SOURCE_DIR}/oshw")
set(SOEM_DIR "${PROJECT_SOURCE_DIR}/soem")

include_directories (
    "${OSAL_DIR}"
    "${OSHW_DIR}"
    "${SOEM_DIR}"
    )

# add subobjects
add_subdirectory(${OSAL_DIR})
add_subdirectory(${OSHW_DIR})
add_subdirectory(${SOEM_DIR})

add_library (osal OBJECT ${OSAL_SRCS} ${OSAL_HEADERS})
add_library (oshw OBJECT ${OSHW_SRCS} ${OSHW_HEADERS})
add_library (soem OBJECT ${SOEM_SRCS} ${SOEM_HEADERS})
            
add_library(ethercat STATIC 
    $<TARGET_OBJECTS:osal>
    $<TARGET_OBJECTS:oshw>
    $<TARGET_OBJECTS:soem>
    )

target_link_libraries(ethercat ${CMAKE_THREAD_LIBS_INIT} rt)

install(TARGETS ethercat DESTINATION lib)