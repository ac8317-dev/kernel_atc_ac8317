#
# Makefile for MMC/SD card drivers
#

obj-$(CONFIG_MMC_BLOCK)		+= mmc_block.o
mmc_block-objs			:= block.o queue.o
obj-$(CONFIG_MMC_TEST)		+= mmc_test.o

obj-$(CONFIG_SDIO_UART)		+= sdio_uart.o

ifeq ($(AC83XX_BOOT_DEVICE),mmc)
	ccflags-y  := -DCONFIG_BOOT_MMC
else 
	ifeq ($(AC83XX_BOOT_DEVICE),sd2)
		ccflags-y  := -DCONFIG_BOOT_SD2
	endif
endif
