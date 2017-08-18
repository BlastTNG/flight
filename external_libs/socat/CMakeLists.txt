project (libsocat)
cmake_minimum_required (VERSION 2.8)

set(SOCAT_SRCS
    ${SOCAT_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/socat.c
    PARENT_SCOPE
)
set(SOCAT_HEADERS
    ${MPSSE_HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/mpsse.h
    ${CMAKE_CURRENT_SOURCE_DIR}/compat.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dalan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/error.h
    ${CMAKE_CURRENT_SOURCE_DIR}/filan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/hostan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mytypes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/nestlex.h
    ${CMAKE_CURRENT_SOURCE_DIR}/procan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/snprinterr.h
    ${CMAKE_CURRENT_SOURCE_DIR}/sslcls.h
    ${CMAKE_CURRENT_SOURCE_DIR}/sycls.h
    ${CMAKE_CURRENT_SOURCE_DIR}/sysincludes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/sysutils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/vsnprintf_r.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-ascii.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xioconfig.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-creat.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xiodiag.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-exec.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-ext2.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-fd.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-fdnum.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-file.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-gopen.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xiohelp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-ip4.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-ip6.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-ipapp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-ip.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xiolayer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-listen.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xiolockfile.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xiomodes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-named.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xioopen.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-openssl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xioopts.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-pipe.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-process.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-progcall.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-proxy.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-pty.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-rawip.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-readline.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-sctp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-socket.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-socks.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-stdio.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-streams.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xiosysincludes.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-system.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-tcp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-tcpwrap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-termios.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-tun.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-udp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xio-unix.h
    PARENT_SCOPE
)

#add_library(socat STATIC 
#	${SOCAT_SRC}
#	${SOCAT_HDR}
#    )

include_directories ("${PROJECT_SOURCE_DIR}")

#target_link_libraries(socat)
