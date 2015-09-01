# 
# Makefile template for out of tree kernel modules
#

# PetaLinux-related stuff
ifndef PETALINUX
$(error You must source the petalinux/settings.sh script before working with PetaLinux)
endif

-include modules.common.mk

KERNEL_BUILD:=$(PROOT)/build/$(LINUX_KERNEL)

LOCALPWD=$(shell pwd)
obj-m += axidmatest.o

all: build modules install

build:modules

.PHONY: build clean modules

clean:
	make INSTANCE=$(LINUX_KERNEL) -C $(KERNEL_BUILD) M=$(LOCALPWD) clean

modules:
	make INSTANCE=$(LINUX_KERNEL) -C $(KERNEL_BUILD) M=$(LOCALPWD) modules_only; \

install: $(addprefix $(DIR),$(subst .o,.ko,$(obj-m)))
	make INSTANCE=$(LINUX_KERNEL) -C $(KERNEL_BUILD) M=$(LOCALPWD) INSTALL_MOD_PATH=$(TARGETDIR) modules_install_only




