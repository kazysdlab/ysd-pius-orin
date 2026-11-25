# Out-of-tree build of the mainline CH341/CH340 USB serial driver for the Jetson (L4T) kernel,
# which ships with CONFIG_USB_SERIAL_CH341 disabled.
obj-m := ch341.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
