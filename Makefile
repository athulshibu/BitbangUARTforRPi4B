obj-m := device_setup.o platform_driver.o

ARCH = arm
ARCH_RASP = arm64
CROSS_COMPILE = arm-linux-gnueabihf-
CROSS_COMPILE_RASP = aarch64-linux-gnu-
KERN_DIR = /home/athul/Desktop/Athul/Workspace/ldd/source/raspberry_linux/
HOST_KERN_DIR = /lib/modules/$(shell uname -r)/build/

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=$(PWD) modules

clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=$(PWD) clean

help:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=$(PWD) help

host:
	make -C $(HOST_KERN_DIR) M=$(PWD) modules

raspberry_host:
	make ARCH=$(ARCH_RASP) CROSS_COMPILE=$(CROSS_COMPILE_RASP) -C $(KERN_DIR) M=$(PWD) modules

send_to_raspberry:
	scp *.ko pi@192.128.10.65:/home/pi/drivers

everything:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=$(PWD) clean
	make ARCH=$(ARCH_RASP) CROSS_COMPILE=$(CROSS_COMPILE_RASP) -C $(KERN_DIR) M=$(PWD) modules
	scp *.ko pi@192.128.10.65:/home/pi/drivers
