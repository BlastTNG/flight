# Find glib    
set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")

include_directories (
    "${COMMON_DIR}"
)

# add common objects 
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/bit_common")

add_library (bit_common OBJECT ${GROUNDHOG_COMMON_SRCS} ${GROUNDHOG_COMMON_HEADERS})
define_file_basename_for_sources(bit_common)

define_file_basename_for_sources(groundhog-bit)

add_executable (groundhog-bit 
    $<TARGET_OBJECTS:liblinklist>
    ${PROJECT_SOURCE_DIR}/groundhog_funcs.c
    ${PROJECT_SOURCE_DIR}/groundhog_funcs.h

    $<TARGET_OBJECTS:bit_common>
    ${PROJECT_SOURCE_DIR}/bit/main.c
    ) 

target_link_libraries(groundhog-bit
    ${GD_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT} 
    ssl crypto
    m)

install(TARGETS groundhog-bit DESTINATION bin)
