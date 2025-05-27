obj-m += ch432.o  # Replace 'mymodule' with your module's name

KDIR := ../6.1_linux/linux-rpi-6.12.y/
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
