#include <linux/module.h>
#include <linux/platform_device.h>

#include "platform.h"

#undef pr_fmt
#define pr_fmt(fmt) "%s : " fmt,__func__

void pcdev_release(struct device *dev) {
	pr_info("Device released \n");
}

// 1. Create 4 platform data
struct pcdev_platform_data pcdev_pdata[1] = {
	[0] = {.baudrate = 19200, .Rx = 23, .Tx = 24, .databuffer = "PCDEVABC111", .size = 256, .perm = RDWR}
};

// 2. Create two platform devices

struct platform_device platform_pcdev_1 = {
	.name = "tty666",
	.id = 0,
	.dev = {
		.platform_data = &pcdev_pdata[0],
		.release = pcdev_release
	}
};

struct platform_device *platform_pcdevs[] = {
	&platform_pcdev_1
};

static int __init pcdev_platform_init(void) {
	// Register platform devices
	// platform_device_register(&platform_pcdev_1);
	platform_add_devices(platform_pcdevs,ARRAY_SIZE(platform_pcdevs));

	pr_info("Device setup module loaded\n");

	return 0;
}

static void __exit pcdev_platform_exit(void) {
	platform_device_unregister(&platform_pcdev_1);

	pr_info("Device setup module unloaded\n");
}

module_init(pcdev_platform_init);
module_exit(pcdev_platform_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Athul");
MODULE_DESCRIPTION("Module which registers platform devicess");
