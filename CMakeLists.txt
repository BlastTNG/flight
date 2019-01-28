set(LL_SOURCES
    ${LL_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist_compress.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist_writer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist_connect.c
)

set(LL_HEADERS
    ${LL_HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist.h
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist_compress.h
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist_writer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/linklist_connect.h
)

IF(BUILD_LIBRARY)
		project(linklist)
		cmake_minimum_required(VERSION 2.8.8)
		include(GNUInstallDirs)

		add_library(linklist STATIC
				${LL_SOURCES}
				${LL_HEADERS}
		)

		target_include_directories(linklist PRIVATE .)

		install(TARGETS linklist DESTINATION ${CMAKE_INSTALL_LIBDIR})
		install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/linklist)
ELSE()
		set(LL_HEADERS
				${LL_HEADERS}
        PARENT_SCOPE)
		set(LL_SOURCES
				${LL_SOURCES}
        PARENT_SCOPE)
ENDIF()
