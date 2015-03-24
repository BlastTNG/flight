cmake_minimum_required (VERSION 3.0)
project (blast_mcp)

# Set global compile flags
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-std=gnu99")

# Find external libs
find_package(PkgConfig)

# Find Threading lib
find_package (Threads)

# Find gsl    
pkg_check_modules(PC_GSL QUIET gsl)
set(GSL_DEFINITIONS ${PC_GSL_CFLAGS_OTHER})

find_path(GSL_INCLUDE_DIR gsl/gsl_blas.h
          HINTS ${PC_GSL_INCLUDEDIR} ${PC_GSL_INCLUDE_DIRS}
          PATH_SUFFIXES gsl )

find_library(GSL_LIBRARY NAMES gsl libgsl
            HINTS ${PC_GSL_LIBDIR} ${PC_GSL_LIBRARY_DIRS} )
find_library(GSL_CBLAS_LIBRARY NAMES gslcblas libgslcblas
            HINTS ${PC_GSL_LIBDIR} ${PC_GSL_LIBRARY_DIRS} )
include_directories(${GSL_INCLUDE_DIR})

# Find glib    
pkg_check_modules(PC_GLIB  glib-2.0)
set(GLIB_DEFINITIONS ${PC_GLIB_CFLAGS_OTHER})

find_path(GLIB_INCLUDE_DIR NAMES glib.h glibconfig.h
          HINTS ${PC_GLIB_INCLUDEDIR} ${PC_GLIB_INCLUDE_DIRS}
          PATH_SUFFIXES glib-2.0 )
find_path(GLIBCONFIG_INCLUDE_DIR NAMES glibconfig.h
          HINTS ${PC_GLIB_INCLUDEDIR} ${PC_GLIB_INCLUDE_DIRS}
          PATH_SUFFIXES glib-2.0 )
find_library(GLIB_LIBRARY NAMES glib-2.0 libglib-2.0
            HINTS ${PC_GLIB_LIBDIR} ${PC_GLIB_LIBRARY_DIRS} )
include_directories(${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

# configure a header file to pass some of the CMake settings to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/mcp_config.h.in"
  "${PROJECT_BINARY_DIR}/include/mcp_config.h"
  )

# add the binary tree to the search path for include files so that we will find mcp_config.h
include_directories ("${PROJECT_BINARY_DIR}")

set(CONFIG_DIR "${PROJECT_SOURCE_DIR}/../blast_config")
set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")

include_directories (
    "${PROJECT_SOURCE_DIR}/include"
    "${CONFIG_DIR}/include"
    "${COMMON_DIR}/include"
    )

# add commanding objects
add_subdirectory(${CONFIG_DIR} "${PROJECT_BINARY_DIR}/blast_config")
add_subdirectory("${PROJECT_SOURCE_DIR}/commanding")

# add pointing objects
add_subdirectory("${PROJECT_SOURCE_DIR}/pointing")

# add motors objects
add_subdirectory("${PROJECT_SOURCE_DIR}/motors")

# add motors objects
add_subdirectory("${PROJECT_SOURCE_DIR}/sensors")

# add common objects
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/blast_common")

add_library (pointing OBJECT ${POINTING_SRCS} ${POINTING_HEADERS})
add_library (motors OBJECT ${MOTOR_SRCS} ${MOTOR_HEADERS})
add_library (sensors OBJECT ${SENSOR_SRCS} ${SENSOR_HEADERS})
add_library (commanding OBJECT ${CMD_SRCS} ${CMD_HEADERS})
add_library (blast_common OBJECT ${MCP_COMMON_SRCS} ${MCP_COMMON_HEADERS})

# add mcp objects
add_library(mcp_obj OBJECT 
    mcp.c 
    tx.c
    ${PROJECT_SOURCE_DIR}/include/mcp.h
    ${PROJECT_SOURCE_DIR}/include/tx.h
    )
            
add_executable (mcp 
    $<TARGET_OBJECTS:commanding>
    $<TARGET_OBJECTS:pointing>
    $<TARGET_OBJECTS:motors>
    $<TARGET_OBJECTS:sensors>
    $<TARGET_OBJECTS:blast_common>
    $<TARGET_OBJECTS:mcp_obj>
    )

target_link_libraries(mcp ${GSL_LIBRARY} ${GSL_CBLAS_LIBRARY} ${GLIB_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} m)

install(TARGETS mcp DESTINATION bin)