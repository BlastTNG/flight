project(linklist)
cmake_minimum_required(VERSION 2.8.8)
include(GNUInstallDirs)

set(SOURCES
    linklist.c
    linklist_compress.c
    linklist_writer.c
    linklist_connect.c
)

set(HEADERS
    linklist.h
    linklist_compress.h
    linklist_writer.h
    linklist_connect.h
)

add_library(linklist STATIC
    ${SOURCES}
    ${HEADERS}
)

target_include_directories(linklist PRIVATE .)

install(TARGETS linklist DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/linklist)
