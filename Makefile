#
#  Dyplo driver makefile (based on lib makefile template)
#

LIBNAME=libdyplodriver.a        # xxx- your library names goes here
LIB=${ARCH}/${LIBNAME}

# C and C++ source names, if any, go here -- minus the .c or .cc
C_PIECES=dyplo_driver dyplo_core
C_FILES=$(C_PIECES:%=%.c)
C_O_FILES=$(C_PIECES:%=${ARCH}/%.o)

CC_PIECES=
CC_FILES=$(CC_PIECES:%=%.cc)
CC_O_FILES=$(CC_PIECES:%=${ARCH}/%.o)

H_FILES=dyplo_driver.h dyplo.h dyplo_core.h print_helpers.h dyplo-ioctl.h dyploconfig.h

# Assembly source names, if any, go here -- minus the .S
S_PIECES=
S_FILES=$(S_PIECES:%=%.S)
S_O_FILES=$(S_FILES:%.S=${ARCH}/%.o)

SRCS=$(C_FILES) $(CC_FILES) $(H_FILES) $(S_FILES)
OBJS=$(C_O_FILES) $(CC_O_FILES) $(S_O_FILES) $(H_FILES)

include $(RTEMS_MAKEFILE_PATH)/Makefile.inc

include $(RTEMS_CUSTOM)
include lib.cfg

#
# Add local stuff here using +=
#

DEFINES  +=
CPPFLAGS +=
CFLAGS   += -std=c99 -Wall -Werror

#
# Add your list of files to delete here.  The config files
#  already know how to delete some stuff, so you may want
#  to just run 'make clean' first to see what gets missed.
#  'make clobber' already includes 'make clean'
#

CLEAN_ADDITIONS += 
CLOBBER_ADDITIONS +=

all: $(LIB)

$(OBJS): $(ARCH)

$(LIB): $(OBJS)
	$(make-library)

# Install the library, appending _g or _p as appropriate.
# for include files, just use $(INSTALL_CHANGE)

# Don't install libraries for now:
#install:  all
#	$(INSTALL_VARIANT) -m 644 ${LIB} ${PROJECT_RELEASE}/lib

