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

find_path(BLAST_CONFIG NAMES tx_struct_tng.c
          HINTS ${PROJECT_SOURCE_DIR}/../blast_config REQUIRED)
IF(BLAST_CONFIG)
		message("Have blast_config")
    set(MAKE_BLAST True)
    include(${PROJECT_SOURCE_DIR}/CMakeLists.blast)
ENDIF()

find_path(BIT_CONFIG NAMES telemlist.bit
          HINTS ${PROJECT_SOURCE_DIR}/../bit_config REQUIRED)
IF(BIT_CONFIG)
		message("Have bit_config")
    set(MAKE_BIT True)
    include(${PROJECT_SOURCE_DIR}/CMakeLists.bit)
ENDIF()


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

# add machine directory for OSX builds (needs endian.h)
include_directories ("/usr/include/machine")
# add the binary tree to the search path for include files so that we will find decomd_config.h
include_directories (
      "${PROJECT_BINARY_DIR}/include"
      )

IF(NOT MAKE_BLAST AND NOT MAKE_BIT)
		message("No _config directories found. No executable compiled.")
ENDIF()

install(DIRECTORY DESTINATION "/data/rawdir"
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE)
install(DIRECTORY DESTINATION "/data/groundhog"
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE)
install(DIRECTORY DESTINATION "/data/etc"
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE)
