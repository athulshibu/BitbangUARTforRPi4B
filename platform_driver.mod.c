#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif


static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x92997ed8, "_printk" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0xdcb764ad, "memset" },
	{ 0x3c5d543a, "hrtimer_start_range_ns" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x3854774b, "kstrtoll" },
	{ 0xc8dcc62a, "krealloc" },
	{ 0xe914e41e, "strcpy" },
	{ 0x54a4f7cf, "devm_kmalloc" },
	{ 0xa1ba346f, "cdev_init" },
	{ 0xb807d7b7, "cdev_add" },
	{ 0xf07ec351, "device_create" },
	{ 0xab87e1a8, "cdev_del" },
	{ 0x516b3608, "sysfs_create_group" },
	{ 0x41b99c46, "device_destroy" },
	{ 0xaf56600a, "arm64_use_ng_mappings" },
	{ 0x40863ba1, "ioremap_prot" },
	{ 0x2d0684a9, "hrtimer_init" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x46a4b118, "hrtimer_cancel" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xae9c36e7, "__class_create" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x5c6615fa, "__platform_driver_register" },
	{ 0xed22970c, "platform_driver_unregister" },
	{ 0x11c62b2, "class_destroy" },
	{ 0x828e22f4, "hrtimer_forward" },
	{ 0x98cf60b3, "strlen" },
	{ 0x87b40f76, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "90624BC2466C8FA6D3FEE7E");
