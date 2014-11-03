# This is an automatically generated record.
# The area between QNX Internal Start and QNX Internal End is controlled by
# the QNX IDE properties.

ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

#===== USEFILE - the file containing the usage message for the application. 
USEFILE=

#===== EXTRA_INCVPATH - a space-separated list of directories to search for include files.
EXTRA_INCVPATH+=/mnt/qnxtarget/opt/UD-6.02  \
	$(PROJECT_ROOT_PWM_board)/include  \
	$(PROJECT_ROOT_SUSI_API_test)/include  \
	$(PROJECT_ROOT_serial_board)/include  \
	$(PROJECT_ROOT_IO_board)/include $(PROJECT_ROOT)/include  \
	$(PROJECT_ROOT_pivot_motor_test)/include  \
	$(PROJECT_ROOT_reaction_wheel_controller)/include  \
	$(PROJECT_ROOT_bitcmd_receive)/include  \
	/home/javier/ide-4.7-workspace/bitcmd_code/trunk/bit_config  \
	/home/javier/Dropbox/ide-4.7-workspace/bitcmd_code/trunk/common  \
	$(PROJECT_ROOT_magnetometer)/include

#===== EXTRA_LIBVPATH - a space-separated list of directories to search for library files.
EXTRA_LIBVPATH+=/mnt/qnxtarget/opt/UD-6.02  \
	/mnt/qnxtarget/lib /mnt/qnxtarget/usr/local/lib

#===== LIBS - a space-separated list of library items to be included in the link.
LIBS+=-Bstatic UD-6.02 -Bdynamic SUSI-3.02 gsl gslcblas

#===== LDFLAGS - add the flags to the linker command line.
LDFLAGS+=-lm -lsocket

#===== EXTRA_SRCVPATH - a space-separated list of directories to search for source files.
EXTRA_SRCVPATH+=/mnt/qnxtarget/usr/local/include  \
	$(PROJECT_ROOT_PWM_board)/src  \
	$(PROJECT_ROOT_SUSI_API_test)/src  \
	$(PROJECT_ROOT_serial_board)/src  \
	$(PROJECT_ROOT_IO_board)/src $(PROJECT_ROOT)/src  \
	$(PROJECT_ROOT_pivot_motor_test)/src  \
	$(PROJECT_ROOT_reaction_wheel_controller)/src  \
	$(PROJECT_ROOT_bitcmd_receive)/src  \
	/home/javier/Dropbox/ide-4.7-workspace/bitcmd_code/trunk/bit_config  \
	$(PROJECT_ROOT_magnetometer)/src

include $(MKFILES_ROOT)/qmacros.mk
ifndef QNX_INTERNAL
QNX_INTERNAL=$(PROJECT_ROOT)/.qnx_internal.mk
endif
include $(QNX_INTERNAL)

include $(MKFILES_ROOT)/qtargets.mk

OPTIMIZE_TYPE_g=none
OPTIMIZE_TYPE=$(OPTIMIZE_TYPE_$(filter g, $(VARIANTS)))

