
project (defricher)

cmake_minimum_required (VERSION 2.8)

option(USE_CLANG "build defricher using clang" OFF)

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

# Set global definitions
add_definitions(-D_GNU_SOURCE)

# Find external libs
find_package(PkgConfig)

# Find Threading lib
find_package (Threads)

# Find ExternalProject module
include (ExternalProject)

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

# Find getdata    
pkg_check_modules(PC_GD QUIET getdata)

find_path(GD_INCLUDE_DIR NAMES getdata.h
          HINTS ${PC_GD_INCLUDEDIR} ${PC_GD_INCLUDE_DIRS} REQUIRED)
find_library(GD_LIBRARY NAMES getdata
            HINTS ${PC_GD_LIBDIR} ${PC_GD_LIBRARY_DIRS} REQUIRED)
IF(GD_INCLUDE_DIR_NOTFOUND)
    MESSAGE(FATAL_ERROR "Did not find getdata.h.  Is libgetdata-dev installed?")
ENDIF()
IF(GD_LIBRARY_NOTFOUND)  
    MESSAGE(FATAL_ERROR "Did not find libgetdata.  Is libgetdata4 installed?")
ENDIF()
include_directories(${GD_INCLUDE_DIR})

# configure a header file to pass some of the CMake settings to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/defricher_config.h.in"
  "${PROJECT_BINARY_DIR}/include/defricher_config.h"
  )

# add the binary tree to the search path for include files so that we will find mcp_config.h
include_directories ("${PROJECT_BINARY_DIR}/include")

set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")

include_directories (
    "${PROJECT_SOURCE_DIR}/include"
    "${COMMON_DIR}/include"
    )

# add common objects
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/blast_common")

add_library (blast_common OBJECT ${DEFRICHER_COMMON_SRCS} ${DEFRICHER_COMMON_HEADERS})

            
add_executable (defricher 
    $<TARGET_OBJECTS:blast_common>
    ${PROJECT_SOURCE_DIR}/defricher.c
    ${PROJECT_SOURCE_DIR}/defricher.h
    ${PROJECT_SOURCE_DIR}/defricher_utils.c
    ${PROJECT_SOURCE_DIR}/defricher_utils.h
    ${PROJECT_SOURCE_DIR}/defricher_netreader.c
    ${PROJECT_SOURCE_DIR}/defricher_netreader.h
    ${PROJECT_SOURCE_DIR}/defricher_writer.c
    ${PROJECT_SOURCE_DIR}/defricher_writer.h
    ) 

target_link_libraries(defricher  
    ${GLIB_LIBRARY}  
    ${GD_LIBRARY} 
    ${CMAKE_THREAD_LIBS_INIT} 
    mosquitto m)

install(TARGETS defricher DESTINATION bin)