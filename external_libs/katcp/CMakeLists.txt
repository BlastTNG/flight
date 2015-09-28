project (libkatcp)
cmake_minimum_required (VERSION 2.8)

# Set global compile flags
set(CMAKE_C_FLAGS "-Wall -fPIC")
# amount of debugging logic to enable
set(CMAKE_C_FLAGS_DEBUG  "-DDEBUG=4 -ggdb")

set(KATCP_SRC arb.c avltree.c bytebit.c client.c dbase.c dispatch.c dpx-cmds.c dpx-core.c dpx-forward.c dpx-info.c dpx-katcp.c dpx-listen.c dpx-mgmt.c dpx-misc.c dpx-sensor.c dpx-vrbl.c endpoint.c event.c fork-parent.c generic-queue.c job.c ktype.c kurl.c line.c log.c loop.c map.c misc.c netc.c nonsense.c notice.c parse-queue.c parse.c queue.c rpc.c server.c services.c shared.c spointer.c stack.c time.c ts.c version.c )
set(KATCP_HDR katcp.h katcl.h katpriv.h fork-parent.h avltree.h netc.h)

# where the C implementation has diverged from the katcp standard,
# force standard behaviour. Currently only one case: log-level affects
# all connections immediately in standard, otherwise local connection only
add_definitions(-DKATCP_STRICT_CONFORMANCE)

# how many messages to hold before forcing a flush, only useful 
# on memory constrained systems. When this value is unset the 
# system buffers as much as needed
#add_definitions(-DKATCP_FLUSH_THRESHOLD=4)

# enable floating point support (floating point sensor type)
# unless you don't want to use your fpu (or don't have one) enable this
add_definitions(-DKATCP_USE_FLOATS)

# perform redundant checks, abort if things don't look alright
# enabling this reduces performance and makes the system less
# resistant to failures, but will also notice failures sooner
add_definitions(-DKATCP_CONSISTENCY_CHECKS)

# log selected noncritical failures to stderr 
add_definitions(-DKATCP_STDERR_ERRORS)

# implement the katcp -T option
#add_definitions(-DKATCP_ENABLE_TAGS)

# log client messages 
add_definitions(-DKATCP_LOG_REQUESTS)

# enable the ability to manage katcp subprocesses
add_definitions(-DKATCP_SUBPROCESS)

# enable newer, broken or nonfunctional code
add_definitions(-DKATCP_EXPERIMENTAL)

# keep older code 
add_definitions(-DKATCP_DEPRECATED)

# respond to trap TERM signal in main loop
add_definitions(-DKATCP_TRAP_TERM)

# inhibit sensor-{value,list} prefix matching, unless it is on a . boundary
#add_definitions(-DKATCP_STRICT_SENSOR_MATCH)

# pick a protocol reversion from the list. Making up
# your own major values doesn't work unless you also implement what 
# the protocol requires
#add_definitions(-DKATCP_PROTOCOL_MAJOR_VERSION=5 -DKATCP_PROTOCOL_MINOR_VERSION=0)
add_definitions(-DKATCP_PROTOCOL_MAJOR_VERSION=4 -DKATCP_PROTOCOL_MINOR_VERSION=9)

add_library(libkatcp STATIC 
	${KATCP_SRC}
	${KATCP_HDR}
    )

include_directories ("${PROJECT_SOURCE_DIR}")

target_link_libraries(libkatcp)

install(TARGETS libkatcp DESTINATION lib)