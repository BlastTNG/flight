cmake_minimum_required (VERSION 2.8)

option(USE_CLANG "build mcp using clang" OFF)

if (USE_CLANG)
    SET (CMAKE_C_COMPILER               "/usr/bin/clang")
    SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++")
endif(USE_CLANG)

SET (CMAKE_C_FLAGS                  "-Wall -std=gnu99")
SET (CMAKE_C_FLAGS_DEBUG            "-g")
SET (CMAKE_C_FLAGS_MINSIZEREL       "-Os -DNDEBUG")
SET (CMAKE_C_FLAGS_RELEASE          "-O3 -DNDEBUG")
SET (CMAKE_C_FLAGS_RELWITHDEBINFO   "-O2 -g")

SET (CMAKE_CXX_FLAGS                "-Wall")
SET (CMAKE_CXX_FLAGS_DEBUG          "-g")
SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")    

project (blast_mcp)
# Set global definitions
add_definitions(-D_GNU_SOURCE)

# Find external libs
find_package(PkgConfig)

# Find Threading lib
find_package (Threads)

# Find ExternalProject module
include (ExternalProject)

# Used to check contents of header files
include (CheckLibraryExists)

# Find gsl    
pkg_check_modules(PC_GSL QUIET gsl)
set(GSL_DEFINITIONS ${PC_GSL_CFLAGS_OTHER})

find_path(GSL_INCLUDE_DIR gsl/gsl_blas.h
          HINTS ${PC_GSL_INCLUDEDIR} ${PC_GSL_INCLUDE_DIRS}
          PATH_SUFFIXES gsl REQUIRED)

find_library(GSL_LIBRARY NAMES gsl libgsl
            HINTS ${PC_GSL_LIBDIR} ${PC_GSL_LIBRARY_DIRS} REQUIRED)
find_library(GSL_CBLAS_LIBRARY NAMES gslcblas libgslcblas
            HINTS ${PC_GSL_LIBDIR} ${PC_GSL_LIBRARY_DIRS} REQUIRED)
IF(GSL_INCLUDE_DIR_NOTFOUND)
    MESSAGE(FATAL_ERROR "Did not find gsl_blas.h.  Is libgsl0-dev installed?")
ENDIF()
IF(GSL_LIBRARY_NOTFOUND)  
    MESSAGE(FATAL_ERROR "Did not find libgsl.  Is libgsl0ldb installed?")
ENDIF()
include_directories(${GSL_INCLUDE_DIR})

# Find glib    
pkg_check_modules(PC_GLIB  glib-2.0)
set(GLIB_DEFINITIONS ${PC_GLIB_CFLAGS_OTHER})

find_path(GLIB_INCLUDE_DIR NAMES glib.h
          HINTS ${PC_GLIB_INCLUDEDIR} ${PC_GLIB_INCLUDE_DIRS}
          PATH_SUFFIXES glib-2.0 REQUIRED)
find_path(GLIBCONFIG_INCLUDE_DIR NAMES glibconfig.h
          HINTS ${PC_GLIB_INCLUDEDIR} ${PC_GLIB_INCLUDE_DIRS}
          PATH_SUFFIXES glib-2.0 REQUIRED)
find_library(GLIB_LIBRARY NAMES glib-2.0 libglib-2.0
            HINTS ${PC_GLIB_LIBDIR} ${PC_GLIB_LIBRARY_DIRS} REQUIRED)
IF(GLIB_INCLUDE_DIR_NOTFOUND)
    MESSAGE(FATAL_ERROR "Did not find glib.h.  Is libglib2.0-dev installed?")
ENDIF()
IF( GLIB_LIBRARY_NOTFOUND)  
    MESSAGE(FATAL_ERROR "Did not find libglib-2.0.  Is libglib2.0-0 installed?")
ENDIF()
include_directories(${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

# Find mosquitto    
ExternalProject_add(MOSQ
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libmosquitto-1.4.1
    URL ${PROJECT_SOURCE_DIR}/../external_libs/mosquitto
    CMAKE_ARGS -DWITH_TLS:BOOL=OFF -DWITH_SOCKS:BOOL=OFF -DWITH_SRV:BOOL=OFF
    )

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

# add soem library
set(ECAT_LIBRARY ${PROJECT_BINARY_DIR}/soem/libethercat.a)
ExternalProject_Add(soem 
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/soem 
    BINARY_DIR ${PROJECT_BINARY_DIR}/soem 
    URL ${PROJECT_SOURCE_DIR}/../external_libs/soem
    INSTALL_COMMAND "")

include_directories(${PROJECT_SOURCE_DIR}/../external_libs/soem/soem 
    ${PROJECT_SOURCE_DIR}/../external_libs/soem/osal
    ${PROJECT_SOURCE_DIR}/../external_libs/soem/oshw)

# add commanding objects
add_subdirectory(${CONFIG_DIR} "${PROJECT_BINARY_DIR}/blast_config")
add_subdirectory("${PROJECT_SOURCE_DIR}/commanding")

# add pointing objects
add_subdirectory("${PROJECT_SOURCE_DIR}/pointing")

# add motors objects
add_subdirectory("${PROJECT_SOURCE_DIR}/motors")

# add sensors objects
add_subdirectory("${PROJECT_SOURCE_DIR}/sensors")

# add housekeeping objects
add_subdirectory("${PROJECT_SOURCE_DIR}/housekeeping")

# add data frame objects
add_subdirectory("${PROJECT_SOURCE_DIR}/framing")

# add common objects
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/blast_common")

add_library (pointing OBJECT ${POINTING_SRCS} ${POINTING_HEADERS})
add_library (motors OBJECT ${MOTOR_SRCS} ${MOTOR_HEADERS})
add_library (sensors OBJECT ${SENSOR_SRCS} ${SENSOR_HEADERS})
add_library (commanding OBJECT ${CMD_SRCS} ${CMD_HEADERS})
add_library (housekeeping OBJECT ${HOUSEKEEPING_SRCS} ${HOUSEKEEPING_HEADERS})
add_library (framing OBJECT ${FRAME_SRCS} ${FRAME_HEADERS})
add_library (blast_common OBJECT ${MCP_COMMON_SRCS} ${MCP_COMMON_HEADERS})

# add mcp objects
add_library(mcp_obj OBJECT 
    mcp.c 
    tx.c
    ${PROJECT_SOURCE_DIR}/include/mcp.h
    ${PROJECT_SOURCE_DIR}/include/tx.h
    )
            
add_executable (mcp 
    $<TARGET_OBJECTS:housekeeping>
    $<TARGET_OBJECTS:commanding>
    $<TARGET_OBJECTS:pointing>
    $<TARGET_OBJECTS:motors>
    $<TARGET_OBJECTS:sensors>
    $<TARGET_OBJECTS:framing>
    $<TARGET_OBJECTS:blast_common>
    $<TARGET_OBJECTS:mcp_obj>
    ) 

add_dependencies(mcp soem)

target_link_libraries(mcp 
    ${GSL_LIBRARY} ${GSL_CBLAS_LIBRARY} 
    ${GLIB_LIBRARY} 
    ${ECAT_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT} 
    mosquitto m)

install(TARGETS mcp DESTINATION bin)