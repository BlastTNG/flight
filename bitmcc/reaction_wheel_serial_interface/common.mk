# This is an automatically generated record.
# The area between QNX Internal Start and QNX Internal End is controlled by
# the QNX IDE properties.

ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

#===== USEFILE - the file containing the usage message for the application. 
USEFILE=

#===== EXTRA_SRCVPATH - a space-separated list of directories to search for source files.
EXTRA_SRCVPATH+=$(PROJECT_ROOT_reaction_wheel_controller)/src  \
	$(PROJECT_ROOT_serial_board)/src  \
	$(PROJECT_ROOT_IO_board)/src

#===== EXTRA_INCVPATH - a space-separated list of directories to search for include files.
EXTRA_INCVPATH+=$(PROJECT_ROOT_serial_board)/include  \
	$(PROJECT_ROOT_reaction_wheel_controller)/include  \
	$(PROJECT_ROOT_IO_board)/include  \
	/mnt/qnxtarget/opt/UD-6.02

#===== LDFLAGS - add the flags to the linker command line.
LDFLAGS+=-lm

#===== EXTRA_LIBVPATH - a space-separated list of directories to search for library files.
EXTRA_LIBVPATH+=/mnt/qnxtarget/opt/UD-6.02

#===== LIBS - a space-separated list of library items to be included in the link.
LIBS+=-Bstatic UD-6.02 -Bdynamic

include $(MKFILES_ROOT)/qmacros.mk
ifndef QNX_INTERNAL
QNX_INTERNAL=$(PROJECT_ROOT)/.qnx_internal.mk
endif
include $(QNX_INTERNAL)

include $(MKFILES_ROOT)/qtargets.mk

OPTIMIZE_TYPE_g=none
OPTIMIZE_TYPE=$(OPTIMIZE_TYPE_$(filter g, $(VARIANTS)))

