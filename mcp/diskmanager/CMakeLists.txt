set(DISKMANAGER_SRCS
    ${DISKMANAGER_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/store_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/diskmanager_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/file_buffer_tng.c
    PARENT_SCOPE
)
set(DISKMANAGER_HEADERS
    ${DISKMANAGER_HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/store_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/diskmanager_tng.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/file_buffer_tng.h
   PARENT_SCOPE
)
