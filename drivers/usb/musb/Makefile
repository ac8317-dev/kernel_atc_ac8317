#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#

#CFLAGS += -O0
KBUILD_CFLAGS	+= -O0
KBUILD_CFLAGS   += -DDEBUG
EXTRA_CFLAGS		+= -DDEBUG

EXTRA_CONFIG=-DCONFIG_USB_GADGET_MUSB_HDRC
#EXTRA_CONFIG=-DCONFIG_USB_MUSB_HDRC_HCD
EXTRA_CONFIG+=-DCONFIG_USB_INVENTRA_DMA
#EXTRA_CONFIG+=-DCONFIG_MUSB_PIO_ONLY
EXTRA_CFLAGS+=$(EXTRA_CONFIG) -I$(shell pwd)/include -Idrivers/usb/core
EXTRA_CFLAGS += -O0

musb_hdrc-objs := musb_core.o

obj-$(CONFIG_USB_MUSB_HDRC)	+= musb_hdrc.o

musb_hdrc-objs+= musb_hdrc.o

musb_hdrc-objs+= mt7119.o

musb_hdrc-objs+= mt7109.o

musb_hdrc-objs		+= musb_gadget_ep0.o musb_gadget.o

#musb_hdrc-objs		+= musb_virthub.o musb_host.o

musb_hdrc-objs		+= musbhsdma.o

