# Helper function to add preprocesor definition of FILE_BASENAME
# to pass the filename without directory path for debugging use.
#
# Example:
#
#   define_file_basename_for_sources(my_target)
#
# Will add -DFILE_BASENAME="filename" for each source file depended on
# by my_target, where filename is the name of the file.
#
function(define_file_basename_for_sources targetname)
    get_target_property(source_files "${targetname}" SOURCES)
    foreach(sourcefile ${source_files})
        # Get source file's current list of compile definitions.
        get_property(defs SOURCE "${sourcefile}"
            PROPERTY COMPILE_DEFINITIONS)
        # Add the FILE_BASENAME=filename compile definition to the list.
        get_filename_component(basename "${sourcefile}" NAME)
        list(APPEND defs "__FILENAME__=\"${basename}\"")
        # Set the updated compile definitions on the source file.
        set_property(
            SOURCE "${sourcefile}"
            PROPERTY COMPILE_DEFINITIONS ${defs})
    endforeach()
endfunction()

project (groundhog)

cmake_minimum_required (VERSION 2.8.8)

option(USE_CLANG "build groundhog using clang" OFF)

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
IF(NOT GLIB_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find glib.h.  Is libglib2.0-dev installed?")
ENDIF()
IF(NOT GLIB_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libglib-2.0.  Is libglib2.0-0 installed?")
ENDIF()
include_directories(${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

# Find mosquitto    
pkg_check_modules(PC_MOSQ QUIET mosquitto)
set(MOSQ_DEFINITIONS ${PC_MOSQ_CFLAGS_OTHER})

find_path(MOSQ_INCLUDE_DIR NAMES mosquitto.h
          HINTS ${PC_MOSQ_INCLUDEDIR} ${PC_MOSQ_INCLUDE_DIRS}
          PATH_SUFFIXES mosquitto REQUIRED)
find_library(MOSQ_LIBRARY NAMES mosquitto libmosquitto
            HINTS ${PC_MOSQ_LIBDIR} ${PC_MOSQ_LIBRARY_DIRS} REQUIRED)
IF(NOT MOSQ_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find mosquitto.h.  Is libmosquitto-dev installed?")
ENDIF()
IF(NOT MOSQ_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libmosquitto.  Is libmosquitto1 installed?")
ENDIF()
include_directories(${MOSQ_INCLUDE_DIR})

# Find getdata    
pkg_check_modules(PC_GD QUIET getdata)

find_path(GD_INCLUDE_DIR NAMES getdata.h
          HINTS ${PC_GD_INCLUDEDIR} ${PC_GD_INCLUDE_DIRS} REQUIRED)
find_library(GD_LIBRARY NAMES getdata
            HINTS ${PC_GD_LIBDIR} ${PC_GD_LIBRARY_DIRS} REQUIRED)
IF(NOT GD_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find getdata.h.  Is libgetdata-dev installed?")
ENDIF()
IF(NOT GD_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libgetdata.  Is libgetdata4 installed?")
ENDIF()
include_directories(${GD_INCLUDE_DIR})

# configure a header file to pass some of the CMake settings to the source code
#configure_file (
#  "${PROJECT_SOURCE_DIR}/groundhog_config.h.in"
#  "${PROJECT_BINARY_DIR}/include/groundhog_config.h"
#  )

# add machine directory for OSX builds (needs endian.h)
include_directories ("/usr/include/machine")

# add the binary tree to the search path for include files so that we will find decomd_config.h
include_directories ("${PROJECT_BINARY_DIR}/include")

set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")
set(LINKLIST_DIR "${PROJECT_SOURCE_DIR}/../common/linklist")
set(CONFIG_DIR "${PROJECT_SOURCE_DIR}/../blast_config")

include_directories (
    "${COMMON_DIR}/include"
    "${MCP_DIR}/include"
    "${CONFIG_DIR}/include"
    "${LINKLIST_DIR}"
    )

# add common objects
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/blast_common")
add_subdirectory(${CONFIG_DIR} "${PROJECT_BINARY_DIR}/blast_config")
add_subdirectory(${LINKLIST_DIR} "${PROJECT_BINARY_DIR}/linklist")

add_library (blast_common OBJECT ${GROUNDHOG_COMMON_SRCS} ${GROUNDHOG_COMMON_HEADERS})
define_file_basename_for_sources(blast_common)

add_library (linklist OBJECT ${LINKLIST_SRCS} ${LINKLIST_HEADERS})
define_file_basename_for_sources(linklist)
            
add_executable (groundhog 
    $<TARGET_OBJECTS:blast_common>
    $<TARGET_OBJECTS:linklist>
    ${PROJECT_SOURCE_DIR}/groundhog.c
    ${PROJECT_SOURCE_DIR}/decom_pci.h
    ${PROJECT_SOURCE_DIR}/groundhog_framing.h
    ${PROJECT_SOURCE_DIR}/groundhog_framing.c
    ${PROJECT_SOURCE_DIR}/groundhog_biphase.c
    ${PROJECT_SOURCE_DIR}/groundhog_pilot.c
    ) 

define_file_basename_for_sources(groundhog)

target_link_libraries(groundhog
    ${GLIB_LIBRARY}  
    ${GD_LIBRARY}
    ${MOSQ_LIBRARY} 
    ${CMAKE_THREAD_LIBS_INIT} 
    )

set(check_files ${FRAME_SRCS} ${GROUNDHOG_COMMON_SRCS} 
		groundhog.c groundhog_biphase.c groundhog_pilot.c 
                ${FRAME_HEADERS} ${GROUNDHOG_COMMON_HEADERS}
   )


install(TARGETS groundhog DESTINATION bin)
