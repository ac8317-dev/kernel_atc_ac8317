#
# Makefile for the linux kernel.
#
export NOHWSEMAPHORE

ifdef NOHWSEMAPHORE
KBUILD_CFLAGS	+= -DNOHWSEMAPHORE
endif

KBUILD_CFLAGS	+= -I$(KERNELDIR)/include/generated

# Object file lists.

#obj-y			:= core.o pll.o ic_version.o cache_operation.o mt85xx_64b_timer.o irq_priority.o libieee754.o bpabi.o libgcc2.o bim_if.o mt85xx_isr_watch.o mt85xx_system.o err_log.o mt85xx_CS_watch.o mt85xx_gpio_pinmux.o

obj-y			:= core.o pll.o mt33xx_system.o ic_version.o bim_if.o irq_priority.o  libieee754.o bpabi.o libgcc2.o mt33xx_isr_watch.o err_log.o mt33xx_CS_watch.o mt33xx_gpio_pinmux.o mtk_sym.o #pmem.o

obj-m			:=
obj-n			:=
obj-			:=

# Power Management
obj-$(CONFIG_PM)		+= pm.o
obj-$(CONFIG_PM)		+= sleep.o
