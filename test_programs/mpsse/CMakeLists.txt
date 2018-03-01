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

project (test_biphase)
project (test_libusb)

cmake_minimum_required (VERSION 2.8.8)

SET (CMAKE_C_FLAGS                  "-Wall -std=gnu99 -mcx16")
SET (CMAKE_C_FLAGS_DEBUG            "-O0 -fno-omit-frame-pointer -ggdb")
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
find_package (Threads REQUIRED)
find_package(OpenSSL REQUIRED)

# Find ExternalProject module
include (ExternalProject)

# Used to check contents of header files
include (CheckLibraryExists)


# Find libusb    
pkg_check_modules(PC_USB QUIET libusb-1.0)
set(USB_DEFINITIONS ${PC_USB_CFLAGS_OTHER})

find_path(USB_INCLUDE_DIR libusb-1.0/libusb.h
          HINTS ${PC_USB_INCLUDEDIR} ${PC_USB_INCLUDE_DIRS}
          REQUIRED)

find_library(USB_LIBRARY NAMES usb-1.0 libusb-1.0
            HINTS ${PC_USB_LIBDIR} ${PC_USB_LIBRARY_DIRS} REQUIRED)
IF(NOT USB_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find libusb.h.  Is libusb-1.0-0-dev installed?")
ENDIF()
IF(NOT USB_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libusb-1.0.  Please install this using apt-get.")
ENDIF()
include_directories(${USB_INCLUDE_DIR})

# configure a header file to pass some of the CMake settings to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/test_biphase_config.h.in"
  "${PROJECT_BINARY_DIR}/include/test_biphase_config.h"
  )

# add the binary tree to the search path for include files so that we will find test_biphase_config.h
include_directories ("${PROJECT_BINARY_DIR}")

set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../../common")
set(MPSSE_DIR "${PROJECT_SOURCE_DIR}/../../external_libs/mpsse")

include_directories (
    "${PROJECT_SOURCE_DIR}/include"
    "${COMMON_DIR}/include"
    "${MPSSE_DIR}"
    )


# add MPSSE Files
add_subdirectory(${MPSSE_DIR} "${PROJECT_BINARY_DIR}/mpsse")

add_library (mpsse OBJECT ${MPSSE_SRCS} ${MPSSE_HEADERS})
define_file_basename_for_sources(mpsse)

# add blast objects
add_library(blast_obj OBJECT 
    ${COMMON_DIR}/blast.c 
    ${COMMON_DIR}/include/blast.h
    )
define_file_basename_for_sources(blast_obj)

# add crc objects
add_library(crc_obj OBJECT 
    ${COMMON_DIR}/crc.c 
    ${COMMON_DIR}/include/crc.h
    )
define_file_basename_for_sources(crc_obj)
	
add_executable (test_biphase 
    test_biphase.c
    $<TARGET_OBJECTS:mpsse>
    $<TARGET_OBJECTS:blast_obj>
    $<TARGET_OBJECTS:crc_obj>
    ) 

add_executable (test_libusb 
    test_libusb.c
    $<TARGET_OBJECTS:mpsse>
    $<TARGET_OBJECTS:blast_obj>
    $<TARGET_OBJECTS:crc_obj>
    ) 

define_file_basename_for_sources(test_biphase)
define_file_basename_for_sources(test_libusb)

target_link_libraries(test_biphase 
    ${USB_LIBRARY}
    ssl crypto 
    m)

target_link_libraries(test_libusb 
    ${USB_LIBRARY}
    ssl crypto 
    m)

#set(check_files ${POINTING_SRCS} ${MOTOR_SRCS} 
#		${SENSOR_SRCS} ${CMD_SRCS} ${MCP_COMMS_SRCS} ${HOUSEKEEPING_SRCS} 
#		${FRAME_SRCS} ${MCP_COMMON_SRCS} ${COMMS_SRCS} ${ROACH_SRCS} 
#		mcp.c tx.c
#		${POINTING_HEADERS} ${MOTOR_HEADERS} ${SENSOR_HEADERS} ${CMD_HEADERS} ${MCP_COMMS_HEADERS} 
#		${HOUSEKEEPING_HEADERS} ${FRAME_HEADERS} ${MCP_COMMON_HEADERS} ${COMMS_HEADERS}
#		${ROACH_HEADERS}
#	    ${PROJECT_SOURCE_DIR}/include/mcp.h
#	    ${PROJECT_SOURCE_DIR}/include/tx.h
#	)
#list(REMOVE_ITEM check_files ${PROJECT_SOURCE_DIR}/sensors/geomag2015.c ${PROJECT_SOURCE_DIR}/sensors/../include/geomag2015.h ${PROJECT_SOURCE_DIR}/sensors/../include/EGM9615.h)
#string(REGEX REPLACE "[ ]*[a-zA-Z/]*PMurHash.[ch][ ]*" " " check_files "${check_files}")

#add_custom_command(TARGET test_biphase PRE_BUILD
#	COMMAND /bin/echo ${check_files}
#	#WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
#	COMMENT "Checking Code Style"
#	VERBATIM
#	)

