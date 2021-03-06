# Application name
APPL ?= he

# Optimization Level
# Please Refer to toolchain_xxx.mk for this option
OLEVEL ?= O2
BD_VER ?= 11
CUR_CORE ?= arcem4
TOOLCHAIN ?= gnu
##
# select the operation cpu core
##


##
# select debugging jtag
##
JTAG ?= usb

#
# root dir of embARC
#
EMBARC_ROOT = ../../..

MID_SEL = common u8glib

# application source dirs
APPL_CSRC_DIR = .
APPL_ASMSRC_DIR = .

# application include dirs
APPL_INC_DIR = .

# include current project makefile
COMMON_COMPILE_PREREQUISITES += makefile

### Options above must be added before include options.mk ###
# include key embARC build system makefile
override EMBARC_ROOT := $(strip $(subst \,/,$(EMBARC_ROOT)))
include $(EMBARC_ROOT)/options/options.mk
