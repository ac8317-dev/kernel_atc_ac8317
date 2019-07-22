#
# Makefile for the linux kernel.
#
export NOHWSEMAPHORE

ifdef NOHWSEMAPHORE
KBUILD_CFLAGS	+= -DNOHWSEMAPHORE
endif

EXTRA_CFLAGS := -I$(KERNELDIR)/../autochips/platform/kernel/drivers/inc -I$(KERNELDIR)/../autochips/device/atc/ac8317/common
KBUILD_CFLAGS	+= -I$(KERNELDIR)/include/generated

# Object file lists.

obj-y		:=
obj-m			:=
obj-n			:=
obj-			:=

# Power Management
obj-$(CONFIG_PM)		+= pm.o
obj-$(CONFIG_PM)		+= sleep.o
# CPU-specific support
obj-$(CONFIG_ARCH_AC83XX)	+= ac83xx-device.o ac83xx_bim.o pll.o mtk_sym.o ic_version.o err_log.o libieee754.o bpabi.o libgcc2.o bim_if.o ac83xx_system.o ac83xx_64b_timer.o  spm.o
obj-$(CONFIG_SMP) += ac83xx-smp.o ac83xx-headsmp.o

# AC83XX board-specific support
obj-$(CONFIG_MACH_AC83XXFPGA)	+=

ifeq ($(AC83XX_BOOT_DEVICE),nand)
ccflags-y                       := -DLOAD_ATC_NAND_DRV
endif

ifeq ($(AC83XX_BOOT_DEVICE),mmc)
ccflags-y  := -DCONFIG_BOOT_MMC
endif
