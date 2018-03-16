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

project (blast_mcp)

cmake_minimum_required (VERSION 2.8.8)

option(USE_CLANG "build mcp using clang" OFF)

if (USE_CLANG)
    SET (CMAKE_C_COMPILER               "/usr/bin/clang")
    SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++")
endif(USE_CLANG)

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
IF(NOT GSL_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find gsl_blas.h.  Is libgsl0-dev installed?")
ENDIF()
IF(NOT GSL_LIBRARY)  
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
IF(NOT GLIB_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find glib.h.  Is libglib2.0-dev installed?")
ENDIF()
IF(NOT GLIB_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libglib-2.0.  Is libglib2.0-0 installed?")
ENDIF()
include_directories(${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

# Find glib    
pkg_check_modules(PC_SENSORS QUIET sensors)
set(SENSORS_DEFINITIONS ${PC_SENSORS_CFLAGS_OTHER})

find_path(SENSORS_INCLUDE_DIR NAMES sensors.h
          HINTS ${PC_SENSORS_INCLUDEDIR} ${PC_SENSORS_INCLUDE_DIRS}
          PATH_SUFFIXES sensors REQUIRED)
find_library(SENSORS_LIBRARY NAMES sensors libsensors
            HINTS ${PC_SENSORS_LIBDIR} ${PC_SENSORS_LIBRARY_DIRS} REQUIRED)
IF(NOT SENSORS_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find sensors.h.  Is libsensors4-dev installed?")
ENDIF()
IF(NOT SENSORS_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libsensors.  Is libsensors4 installed?")
ENDIF()
include_directories(${SENSORS_INCLUDE_DIR})

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

# Find Concurrency Kit    
pkg_check_modules(PC_CK QUIET ck)
set(CK_DEFINITIONS ${PC_CK_CFLAGS_OTHER})

find_path(CK_INCLUDE_DIR NAMES ck_cc.h
          HINTS ${PC_CK_INCLUDEDIR} ${PC_CK_INCLUDE_DIRS})
find_library(CK_LIBRARY NAMES ck libck
            HINTS ${PC_CK_LIBDIR} ${PC_CK_LIBRARY_DIRS} REQUIRED)
IF(NOT CK_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find ck_cc.  Is libck-dev installed?")
ENDIF()
IF(NOT CK_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libck.  Is the libck package installed?")
ENDIF()
include_directories(${CK_INCLUDE_DIR})


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

# add our version of libphenom
ExternalProject_Add(phenomproj
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/phenom
  UPDATE_COMMAND git pull
  PATCH_COMMAND ./autogen.sh
  CONFIGURE_COMMAND ./configure -q --disable-shared --enable-silent-rules
  GIT_REPOSITORY https://github.com/sethhillbrand/libphenom.git
  BUILD_IN_SOURCE 1
  BUILD_COMMAND make libphenom.la
  INSTALL_COMMAND "")

set(PH_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/phenom/src/phenomproj/.libs/libphenom.a)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/phenom/src/phenomproj/include")


# Find alsa
pkg_check_modules(PC_ALSA QUIET alsa)
set(ALSA_DEFINITIONS ${PC_ALSA_CFLAGS_OTHER})

find_path(ALSA_INCLUDE_DIR asoundlib.h
          HINTS ${PC_ALSA_INCLUDEDIR} ${PC_ALSA_INCLUDE_DIRS}
          REQUIRED)

find_library(ALSA_LIBRARY NAMES asound
            HINTS ${PC_ALSA_LIBDIR} ${PC_ALSA_LIBRARY_DIRS} REQUIRED)
IF(NOT ALSA_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find asoundlib.h.  Is libasound2-dev installed?")
ENDIF()
IF(NOT ALSA_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libasound2.  Please install this using apt-get.")
ENDIF()
include_directories(${ALSA_INCLUDE_DIR})

# Find libmodbux
pkg_check_modules(PC_MODBUS QUIET libmodbus)
set(MODBUS_DEFINITIONS ${PC_MODBUS_CFLAGS_OTHER})

find_path(MODBUS_INCLUDE_DIR modbus.h
          HINTS ${PC_MODBUS_INCLUDEDIR} ${PC_MODBUS_INCLUDE_DIRS}
          REQUIRED)

find_library(MODBUS_LIBRARY NAMES libmodbus modbus
            HINTS ${PC_MODBUS_LIBDIR} ${PC_MODBUS_LIBRARY_DIRS} REQUIRED)
IF(NOT MODBUS_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find modbus.h.  Is libmodbus-dev installed?")
ENDIF()
IF(NOT MODBUS_LIBRARY)  
    MESSAGE(FATAL_ERROR "Did not find libmodbus.  Please install this using apt-get.")
ENDIF()
include_directories(${MODBUS_INCLUDE_DIR})


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
  "${PROJECT_SOURCE_DIR}/mcp_config.h.in"
  "${PROJECT_BINARY_DIR}/include/mcp_config.h"
  )

# add the binary tree to the search path for include files so that we will find mcp_config.h
include_directories ("${PROJECT_BINARY_DIR}")

set(CONFIG_DIR "${PROJECT_SOURCE_DIR}/../blast_config")
set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")
set(COMMS_DIR "${PROJECT_SOURCE_DIR}/../blast_comms")
set(MPSSE_DIR "${PROJECT_SOURCE_DIR}/../external_libs/mpsse")
set(SYNCLINK_DIR "${PROJECT_SOURCE_DIR}/../external_libs/synclink")
set(LINKLIST_DIR "${PROJECT_SOURCE_DIR}/../common/linklist")
set(COMMUNICATIONS_DIR "${PROJECT_SOURCE_DIR}/communications/")

set(ETC_DIR "${PROJECT_SOURCE_DIR}/../blast_etc")
FILE(GLOB etc_files "${ETC_DIR}/*.txt" "${ETC_DIR}/*.sch" "${ETC_DIR}/*.library" "${ETC_DIR}/*.lut" "${ETC_DIR}/WMM.COF")

include_directories (
    "${PROJECT_SOURCE_DIR}/include"
    "${CONFIG_DIR}/include"
    "${COMMON_DIR}/include"
    "${COMMS_DIR}/include"
    "${COMMUNICATIONS_DIR}/include"
    "${MPSSE_DIR}"
    "${SYNCLINK_DIR}"
    "${LINKLIST_DIR}"
    )

# add commanding objects
add_subdirectory(${CONFIG_DIR} "${PROJECT_BINARY_DIR}/blast_config")
add_subdirectory("${PROJECT_SOURCE_DIR}/commanding")

# add mcp communication objects
add_subdirectory("${PROJECT_SOURCE_DIR}/communications")

# add pointing objects
add_subdirectory("${PROJECT_SOURCE_DIR}/pointing")

# add motors objects
add_subdirectory("${PROJECT_SOURCE_DIR}/motors")

# add sensors objects
add_subdirectory("${PROJECT_SOURCE_DIR}/sensors")

# add housekeeping objects
add_subdirectory("${PROJECT_SOURCE_DIR}/housekeeping")

# add diskmanager objects
add_subdirectory("${PROJECT_SOURCE_DIR}/diskmanager")

# add data frame objects
add_subdirectory("${PROJECT_SOURCE_DIR}/framing")

# add roach objects
add_subdirectory("${PROJECT_SOURCE_DIR}/roach")

# add common objects
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/blast_common")

# add asynchronous communication objects
add_subdirectory(${COMMS_DIR} "${PROJECT_BINARY_DIR}/blast_comms")

# add MPSSE Files
add_subdirectory(${MPSSE_DIR} "${PROJECT_BINARY_DIR}/mpsse")

#add linklists files
add_subdirectory(${LINKLIST_DIR} "${PROJECT_BINARY_DIR}/linklists")

add_library (pointing OBJECT ${POINTING_SRCS} ${POINTING_HEADERS})
define_file_basename_for_sources(pointing)
add_library (motors OBJECT ${MOTOR_SRCS} ${MOTOR_HEADERS})
define_file_basename_for_sources(motors)
add_library (sensors OBJECT ${SENSOR_SRCS} ${SENSOR_HEADERS})
define_file_basename_for_sources(sensors)
add_library (commanding OBJECT ${CMD_SRCS} ${CMD_HEADERS})
define_file_basename_for_sources(commanding)
add_library (communications OBJECT ${MCP_COMMUNICATIONS_SRCS} ${MCP_COMMUNICATIONS_HEADERS})
define_file_basename_for_sources(communications)
add_library (housekeeping OBJECT ${HOUSEKEEPING_SRCS} ${HOUSEKEEPING_HEADERS})
define_file_basename_for_sources(housekeeping)
add_library (diskmanager OBJECT ${DISKMANAGER_SRCS} ${DISKMANAGER_HEADERS})
define_file_basename_for_sources(diskmanager)
add_library (framing OBJECT ${FRAME_SRCS} ${FRAME_HEADERS})
define_file_basename_for_sources(framing)
add_library (blast_common OBJECT ${MCP_COMMON_SRCS} ${MCP_COMMON_HEADERS})
define_file_basename_for_sources(blast_common)
add_dependencies(blast_common phenomproj)
add_library (blast_comms OBJECT ${COMMS_SRCS} ${COMMS_HEADERS})
define_file_basename_for_sources(blast_comms)
add_library (mpsse OBJECT ${MPSSE_SRCS} ${MPSSE_HEADERS})
define_file_basename_for_sources(mpsse)
add_library (linklist OBJECT ${LINKLIST_SRCS} ${LINKLIST_HEADERS})
define_file_basename_for_sources(linklist)

add_library (roach OBJECT ${ROACH_SRCS} ${ROACH_HEADERS})
define_file_basename_for_sources(roach)

# add mcp objects
add_library(mcp_obj OBJECT 
    mcp.c 
    tx.c
    ${PROJECT_SOURCE_DIR}/include/mcp.h
    ${PROJECT_SOURCE_DIR}/include/tx.h
    )
define_file_basename_for_sources(mcp_obj)
	
add_executable (mcp 
    $<TARGET_OBJECTS:housekeeping>
    $<TARGET_OBJECTS:diskmanager>
    $<TARGET_OBJECTS:commanding>
    $<TARGET_OBJECTS:communications>
    $<TARGET_OBJECTS:pointing>
    $<TARGET_OBJECTS:motors>
    $<TARGET_OBJECTS:sensors>
    $<TARGET_OBJECTS:framing>
    $<TARGET_OBJECTS:blast_common>
    $<TARGET_OBJECTS:blast_comms>
    $<TARGET_OBJECTS:mpsse>
    $<TARGET_OBJECTS:roach>
    $<TARGET_OBJECTS:mcp_obj>
    $<TARGET_OBJECTS:linklist>
    ) 

add_dependencies(mcp soem phenomproj)

define_file_basename_for_sources(mcp)

target_link_libraries(mcp 
	${ALSA_LIBRARY}
	${MODBUS_LIBRARY}
    ${GSL_LIBRARY} ${GSL_CBLAS_LIBRARY}
    ${MOSQ_LIBRARY}
    ${GLIB_LIBRARY}
    ${ECAT_LIBRARY}
    ${CK_LIBRARY}
    ${PH_LIBRARY}
    ${SENSORS_LIBRARY}
    ${USB_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ssl crypto 
    m)

set(check_files ${POINTING_SRCS} ${MOTOR_SRCS} 
		${SENSOR_SRCS} ${CMD_SRCS} ${MCP_COMMS_SRCS} ${HOUSEKEEPING_SRCS} 
		${FRAME_SRCS} ${MCP_COMMON_SRCS} ${COMMS_SRCS} ${ROACH_SRCS} ${DISKMANAGER_SRCS} 
		mcp.c tx.c ${PROJECT_SOURCE_DIR}/../common/linklist/bitserver.c
		${POINTING_HEADERS} ${MOTOR_HEADERS} ${SENSOR_HEADERS} ${CMD_HEADERS} ${MCP_COMMS_HEADERS} 
		${HOUSEKEEPING_HEADERS} ${FRAME_HEADERS} ${MCP_COMMON_HEADERS} ${COMMS_HEADERS} ${DISKMANAGER_HEADERS} 
		${ROACH_HEADERS} 
	    ${PROJECT_SOURCE_DIR}/include/mcp.h
	    ${PROJECT_SOURCE_DIR}/include/tx.h
	)
list(REMOVE_ITEM check_files ${PROJECT_SOURCE_DIR}/sensors/geomag2015.c ${PROJECT_SOURCE_DIR}/sensors/../include/geomag2015.h ${PROJECT_SOURCE_DIR}/sensors/../include/EGM9615.h)
string(REGEX REPLACE "[ ]*[a-zA-Z/]*PMurHash.[ch][ ]*" " " check_files "${check_files}")

add_custom_command(TARGET mcp PRE_BUILD
	#COMMAND /bin/echo ${check_files}
	#COMMAND ${PROJECT_SOURCE_DIR}/testing/clint.sh ${check_files}
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	COMMENT "Checking Code Style"
	VERBATIM
	)

install(TARGETS mcp DESTINATION bin)
install(DIRECTORY DESTINATION "/data/etc")
install(FILES ${etc_files} DESTINATION "/data/etc/blast")
