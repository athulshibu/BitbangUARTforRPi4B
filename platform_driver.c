#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <asm/io.h>
#include <linux/ktime.h>

#include "platform.h"

#undef pr_fmt
#define pr_fmt(fmt) "%s : " fmt, __func__

unsigned long __stack_chk_guard;

void __stack_chk_guard_setup(void) {
	__stack_chk_guard = 0xBAAAAAAD; //provide some magic numbers
}

void __stack_chk_fail(void) { // will be called when guard variable is corrupted 
	/* Error message */                                 
}

#define MAX_DEVICES 10
#define SIZE_OF_DATAFRAME 8

#define MEM_SIZE_MAX_PCDEV1 1024
#define MEM_SIZE_MAX_PCDEV2 1024
#define MEM_SIZE_MAX_PCDEV3 1024
#define MEM_SIZE_MAX_PCDEV4 1024

// Pseudo Device's memory
char device_buffer_pcdev1[MEM_SIZE_MAX_PCDEV1];
char device_buffer_pcdev2[MEM_SIZE_MAX_PCDEV2];
char device_buffer_pcdev3[MEM_SIZE_MAX_PCDEV3];
char device_buffer_pcdev4[MEM_SIZE_MAX_PCDEV4];

#define GPIO_BASE_ADDRESS 0xfe200000
#define GPIO_MEM_SIZE 0xf0

#define FSEL_ADDR(p, b) b + ((p / 10) * 4)
#define FSEL_SET(v, p) v << ((p % 10) * 3)

#define GPIO_SET_ADDR(p, b) p > 31 ? 0x20 + b : 0x1c + b
#define GPIO_CLR_ADDR(p, b) p > 31 ? 0x2c + b : 0x28 + b
#define GPIO_LVL_ADDR(p, b)  p > 31 ? 0x38 + b : 0x34 + b
#define GPIO_SET(v, p) v << p

volatile void __iomem *gpio_remapped_base;

// Driver private data structure
struct pcdrv_private_data {
	int total_devices;
	dev_t device_num_base;
	struct class *class_pcd;
	struct device *device_pcd;
};

// Device private data structure
struct pcdev_private_data {
	struct pcdev_platform_data pdata;
	char *readbuffer;
	char *writebuffer;
	dev_t dev_num;
	struct cdev cdev;
	struct mutex pcdev_lock;
	struct hrtimer read_hrtimer;
	struct hrtimer write_hrtimer;
};

int gpio_avail(int gpio_pin) {
	if(gpio_pin >= 2 && gpio_pin <= 27 && gpio_pin != 14 && gpio_pin != 15)
		return 1;
	return 0;
}

int baudrate_avail(int baudrate) {
	int baudrate_avails[10] = {9600, 19200, 38400, 115200};
	for(int i=0 ; i<sizeof(baudrate_avails) ; i++)
		if(baudrate == baudrate_avails[i])
			return 1;
	return 0;
}

int check_permission(int dev_perm, int acc_mode) {
	if(dev_perm == RDWR)
		return 0;
	if ( (dev_perm == RDONLY) && ((acc_mode & FMODE_READ) && !(acc_mode & FMODE_WRITE)) )
		return 0;
	if ( (dev_perm == WRONLY) && ((acc_mode & FMODE_WRITE) && !(acc_mode & FMODE_READ)) )
		return 0;
	
	return -EPERM;
}

struct pcdrv_private_data pcdrv_data;

loff_t pcd_lseek (struct file *filp, loff_t offset, int whence) {
	struct pcdev_private_data *pcdev_data = (struct pcdev_private_data*)filp->private_data;
	int max_size = pcdev_data->pdata.size;
	
	loff_t temp;

	pr_info("lseek requested\n");
	pr_info("Current file position = %lld\n",filp->f_pos);

	switch(whence) {
		case SEEK_SET:
			if( (offset > max_size) || (offset < 0) )
				return -EINVAL;
			filp->f_pos = offset;
			break;
		case SEEK_CUR:
			temp = filp->f_pos + offset;
			if( (temp > max_size) || (temp < 0) )
				return -EINVAL;
			filp->f_pos = temp;
			break;
		case SEEK_END:
			temp = max_size + offset;
			if( (temp > max_size) || (temp < 0) )
				return -EINVAL;
			filp->f_pos = temp;
			break;
		default:
			return -EINVAL;
	}

	pr_info("Updated file position = %lld\n",filp->f_pos);
	return 0;
}

int CUR_READ_BIT_POINTER = SIZE_OF_DATAFRAME;
int READ_FLAG = 0;
int current_char = 0;

int append(char*s, size_t size, char c) {
	int len = strlen(s);
	if(strlen(s) + 1 >= size) {
        return 0;
    }
    s[len] = c;
    s[len+1] = '\0';
    return 1;
}

enum hrtimer_restart read_hrtimer_handler(struct hrtimer *timer) {
	struct pcdev_private_data *pcdev_data = container_of(timer, struct pcdev_private_data, read_hrtimer);
	
	// int CUR_BIT = ((readl(GPIO_LVL_ADDR(pcdev_data->pdata.Rx, gpio_remapped_base)) >> (pcdev_data->pdata.Rx)) & 0x1);
	int CUR_BIT = (readl(GPIO_LVL_ADDR(pcdev_data->pdata.Rx, gpio_remapped_base)) >> (pcdev_data->pdata.Rx)) & 0x1;
	
	if (READ_FLAG) {
		current_char = (current_char << 1) + CUR_BIT;
		// pr_info("%d read\n", CUR_BIT);

		if(CUR_READ_BIT_POINTER == 0) {
			pr_info("End of character\n");
			pr_info("......................................%c",current_char);
			if(!append(pcdev_data->readbuffer, pcdev_data->pdata.size, current_char))
				return -EINVAL;
			current_char = 0;
			READ_FLAG = 0;
			// pr_info("................................... %s\n", buffers);
		}

		CUR_READ_BIT_POINTER -= 1;
	}
	else {
		if(!CUR_BIT) {
			pr_info("%d (Read started)\n", CUR_BIT);
			CUR_READ_BIT_POINTER -= 1;
			READ_FLAG = 1;
		}
		else {
			// pr_info("%d (Read ended)\n", CUR_BIT);
			CUR_READ_BIT_POINTER = SIZE_OF_DATAFRAME;
		}
	}

	hrtimer_forward_now(timer, ktime_set(0, 1000000000 / (long)pcdev_data->pdata.baudrate));
	return HRTIMER_RESTART;
}

ssize_t pcd_read (struct file *filp, char __user *buff, size_t count, loff_t *f_pos) {
	struct pcdev_private_data *pcdev_data = (struct pcdev_private_data*)filp->private_data;
	int max_size = pcdev_data->pdata.size;
	
	pr_info("read requested for %zu bytes\n",count);
	pr_info("Current file position = %lld\n",*f_pos);

	mutex_lock(&pcdev_data->pcdev_lock);

	// Adjust the "count"
	if((*f_pos + count) > max_size)
		count = max_size - *f_pos;

	// Copy to User
	if(copy_to_user(buff,pcdev_data->readbuffer+(*f_pos),count))
		return -EFAULT;

	mutex_unlock(&pcdev_data->pcdev_lock);

	// Update current file position
	*f_pos += count;

	pr_info("%zu bytes successfully read\n",count);
	pr_info("Updated file position = %lld\n",*f_pos);

	return count;
}

int CUR_POINTER = 0;
int CUR_BIT_POINTER = SIZE_OF_DATAFRAME;

enum hrtimer_restart write_hrtimer_handler(struct hrtimer *timer) {
	struct pcdev_private_data *pcdev_data = container_of(timer, struct pcdev_private_data, write_hrtimer);
	
	if(pcdev_data->writebuffer[CUR_POINTER] == 10 && CUR_BIT_POINTER == -1) {
		// pr_info("1 (End of Message)\n");
		writel(GPIO_SET(1, pcdev_data->pdata.Tx), GPIO_SET_ADDR(pcdev_data->pdata.Tx, gpio_remapped_base));
		CUR_POINTER = 0;
		hrtimer_forward_now(timer, ktime_set(0, 1000000000 / (long)pcdev_data->pdata.baudrate));
		return HRTIMER_NORESTART;
	}
	
	if(CUR_BIT_POINTER == -1) {
		// pr_info("1 (End of Frame)\n");
		CUR_POINTER += 1;
		CUR_BIT_POINTER = SIZE_OF_DATAFRAME;
		writel(GPIO_SET(1, pcdev_data->pdata.Tx), GPIO_SET_ADDR(pcdev_data->pdata.Tx, gpio_remapped_base));
		hrtimer_forward_now(timer, ktime_set(0, 1000000000 / (long)pcdev_data->pdata.baudrate));
		return HRTIMER_RESTART;	
	}

	if(CUR_BIT_POINTER == SIZE_OF_DATAFRAME)
		pr_info("Current letter's ASCII = %d \n",pcdev_data->writebuffer[CUR_POINTER]);
	
	// pr_info("%d", (pcdev_data->writebuffer[CUR_POINTER] >> CUR_BIT_POINTER) % 2);

	if((pcdev_data->writebuffer[CUR_POINTER] >> CUR_BIT_POINTER) % 2)
		writel(GPIO_SET(1, pcdev_data->pdata.Tx), GPIO_SET_ADDR(pcdev_data->pdata.Tx, gpio_remapped_base));
	else
		writel(GPIO_SET(1, pcdev_data->pdata.Tx), GPIO_CLR_ADDR(pcdev_data->pdata.Tx, gpio_remapped_base));
	
	CUR_BIT_POINTER -= 1;

	hrtimer_forward_now(timer, ktime_set(0, 1000000000 / (long)pcdev_data->pdata.baudrate));

	return HRTIMER_RESTART;
}

ssize_t pcd_write (struct file *filp, const char __user *buff, size_t count, loff_t *f_pos) {
	struct pcdev_private_data *pcdev_data = (struct pcdev_private_data*)filp->private_data;
	int max_size = pcdev_data->pdata.size;
	
	pr_info("Write requested for %zu bytes\n",count);
	pr_info("Current file position = %lld\n",*f_pos);

	mutex_lock(&pcdev_data->pcdev_lock);

	// Adjust the "count"
	if((*f_pos + count) > max_size)
		count = max_size - *f_pos;

	if(!count) {
		pr_err("No space left on device\n");
		return -ENOMEM;
	}

	// Copy from User
	if(copy_from_user(pcdev_data->writebuffer+(*f_pos),buff,count))
		return -EFAULT;

	// Update current file position
	*f_pos += count;

	pr_info("%zu bytes successfully written\n",count);
	pr_info("Updated file position = %lld\n",*f_pos);

	hrtimer_start(&pcdev_data->write_hrtimer, ns_to_ktime((u64)1000000000/pcdev_data->pdata.baudrate), HRTIMER_MODE_REL);

	mutex_unlock(&pcdev_data->pcdev_lock);
	
	return count;
}

int pcd_open (struct inode *inode, struct file *filp) {
	int ret;

	int minor_n;
	
	struct pcdev_private_data *pcdev_data;
	
	// Find out which device file is being opened
	minor_n = MINOR(inode->i_rdev);
	pr_info("Accessing %d\n",minor_n);
	
	// Get the device's private data to other methods of the driver
	pcdev_data = container_of(inode->i_cdev, struct pcdev_private_data,cdev);
	
	// Supply private data to other methods of the driver
	filp->private_data = pcdev_data;
	
	// Check permission
	pr_info("File Pointer Permission = %d", filp->f_mode);
	pr_info("Device Permission = %d", pcdev_data->pdata.perm);
	ret = check_permission(pcdev_data->pdata.perm,filp->f_mode);
	pr_info("Check Permission returns %d", ret);

	(!ret)?pr_info("Open was successful\n"):pr_info("Open was unsuccessful\n");
	return ret;
}

int pcd_release (struct inode *inode, struct file *filp) {
	pr_info("Release was successful\n");
	return 0;
}

struct file_operations pcd_fops = {
	.open = pcd_open,
	.write = pcd_write,
	.read = pcd_read,
	.llseek = pcd_lseek,
	.release = pcd_release,
	.owner = THIS_MODULE
};

ssize_t show_max_size(struct device *dev, struct device_attribute *attr, char *buf) {
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);
	int ret;

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = sprintf(buf, "%d\n", dev_data->pdata.size);

	mutex_unlock(&dev_data->pcdev_lock);

	return ret;
}

ssize_t show_baudrate(struct device *dev, struct device_attribute *attr, char *buf) {
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);
	int ret;

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = sprintf(buf, "%d\n", dev_data->pdata.baudrate);

	mutex_unlock(&dev_data->pcdev_lock);

	return ret;
}

ssize_t store_baudrate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	long result;
	int ret;
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = kstrtol(buf, 10, &result);
	if(ret) 
		return ret;

	if(!baudrate_avail(result))
		return -EINVAL;
	
	dev_data->pdata.baudrate = result;
	dev_data->writebuffer = krealloc(dev_data->writebuffer, dev_data->pdata.baudrate, GFP_KERNEL);

	mutex_unlock(&dev_data->pcdev_lock);
	
	return count;
}

ssize_t show_Rx(struct device *dev, struct device_attribute *attr, char *buf) {
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);
	int ret;

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = sprintf(buf, "%d\n", dev_data->pdata.Rx);

	mutex_unlock(&dev_data->pcdev_lock);

	return ret;
}

ssize_t store_Rx(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	long result;
	int ret;
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = kstrtol(buf, 10, &result);
	if(ret) 
		return ret;

	if (!gpio_avail(dev_data->pdata.Rx) || dev_data->pdata.Rx == dev_data->pdata.Tx) {
		pr_err("Pin not available");
		return -EINVAL;
	}
	
	writel(GPIO_SET(1, dev_data->pdata.Rx), GPIO_CLR_ADDR(dev_data->pdata.Rx, gpio_remapped_base));
	
	dev_data->pdata.Rx = result;
	dev_data->writebuffer = krealloc(dev_data->writebuffer, dev_data->pdata.Rx, GFP_KERNEL);

	writel(FSEL_SET(0, dev_data->pdata.Rx), FSEL_ADDR(dev_data->pdata.Rx, gpio_remapped_base));
	writel(GPIO_SET(1, dev_data->pdata.Rx), GPIO_SET_ADDR(dev_data->pdata.Rx, gpio_remapped_base));

	mutex_unlock(&dev_data->pcdev_lock);
	
	return count;
}

ssize_t show_Tx(struct device *dev, struct device_attribute *attr, char *buf) {
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);
	int ret;

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = sprintf(buf, "%d\n", dev_data->pdata.Tx);

	mutex_unlock(&dev_data->pcdev_lock);

	return ret;
}

ssize_t store_Tx(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	long result;
	int ret;
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = kstrtol(buf, 10, &result);
	if(ret) 
		return ret;

	if (!gpio_avail(dev_data->pdata.Tx) || dev_data->pdata.Tx == dev_data->pdata.Rx) {
		pr_err("Pin not available");
		return -EINVAL;
	}
	
	writel(GPIO_SET(1, dev_data->pdata.Tx), GPIO_CLR_ADDR(dev_data->pdata.Tx, gpio_remapped_base));
	
	dev_data->pdata.Tx = result;
	dev_data->writebuffer = krealloc(dev_data->writebuffer, dev_data->pdata.Tx, GFP_KERNEL);

	writel(FSEL_SET(1, dev_data->pdata.Tx), FSEL_ADDR(dev_data->pdata.Tx, gpio_remapped_base));

	mutex_unlock(&dev_data->pcdev_lock);
	
	return count;
}

ssize_t show_databuffer(struct device *dev, struct device_attribute *attr, char *buf) {
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);
	int ret;

	mutex_lock(&dev_data->pcdev_lock);
	
	ret = sprintf(buf, "%s\n", dev_data->pdata.databuffer);

	mutex_unlock(&dev_data->pcdev_lock);

	return ret;
}

ssize_t store_databuffer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct pcdev_private_data *dev_data = dev_get_drvdata(dev->parent);

	mutex_lock(&dev_data->pcdev_lock);
	
	strcpy(dev_data->pdata.databuffer, buf);
	dev_data->writebuffer = krealloc(dev_data->writebuffer, (size_t)dev_data->pdata.databuffer, GFP_KERNEL);

	mutex_unlock(&dev_data->pcdev_lock);
	
	return count;
}

static DEVICE_ATTR(max_size, S_IRUGO, show_max_size, NULL);
static DEVICE_ATTR(baudrate, S_IRUGO|S_IWUSR, show_baudrate, store_baudrate);
static DEVICE_ATTR(Rx, S_IRUGO|S_IWUSR, show_Rx, store_Rx);
static DEVICE_ATTR(Tx, S_IRUGO|S_IWUSR, show_Tx, store_Tx);
static DEVICE_ATTR(databuffer, S_IRUGO|S_IWUSR, show_databuffer, store_databuffer);

struct attribute *pcd_attrs[] = {
	&dev_attr_max_size.attr,
	&dev_attr_baudrate.attr,
	&dev_attr_Rx.attr,
	&dev_attr_Tx.attr,
	&dev_attr_databuffer.attr,
	NULL
};

struct attribute_group pcd_attr_group = {
	.attrs = pcd_attrs
};

int pcd_sysfs_create_files(struct device *pcd_dev) {
	return sysfs_create_group(&pcd_dev->kobj, &pcd_attr_group);
}

// Gets called when a matching platform device is found
int pcd_platform_driver_probe(struct platform_device *pdev) {
	int ret;
	struct pcdev_private_data *dev_data;
	struct pcdev_platform_data *pdata;

	struct device *dev = &pdev->dev;
	
	pr_info("A device is inserted\n");
	
	// 1. Get the platform data
	pdata = (struct pcdev_platform_data*)dev_get_platdata(&pdev->dev);
	if(!pdata) {
		pr_info("No platform data available\n");
		return -EINVAL;
	}
	
	// 2. Dynamically allocate memory for the device private data
	dev_data = devm_kzalloc(&pdev->dev,sizeof(*dev_data),GFP_KERNEL);
	if(!dev_data) {
		pr_err("Cannot allocate memory for device private data\n");
		pr_info("Device probe failed\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev,dev_data); // Save device driver pointer in platform device structure
	
	dev_data->pdata.baudrate = pdata->baudrate;
	dev_data->pdata.Rx = pdata->Rx;
	dev_data->pdata.Tx = pdata->Tx;
	dev_data->pdata.perm = pdata->perm;
	dev_data->pdata.size = pdata->size;
	dev_data->pdata.databuffer = devm_kzalloc(&pdev->dev,dev_data->pdata.size,GFP_KERNEL);
	strcpy(dev_data->pdata.databuffer, pdata->databuffer);
	
	pr_info("Baudrate = %d\n",dev_data->pdata.baudrate);
	pr_info("Rx Pin = %d\n",dev_data->pdata.Rx);
	pr_info("Tx Pin = %d\n",dev_data->pdata.Tx);
	pr_info("Device Permission = %d\n", dev_data->pdata.perm);
	pr_info("Data buffer maximum size = %d\n", dev_data->pdata.size);
	pr_info("Data Buffer = %s\n",dev_data->pdata.databuffer);
	
	// 3. Dynamically allocate memory for the device buffers using size information from the platform data
	dev_data->readbuffer = devm_kzalloc(&pdev->dev,pdata->size,GFP_KERNEL);
	dev_data->writebuffer = devm_kzalloc(&pdev->dev,pdata->size,GFP_KERNEL);
	if(!dev_data) {
		pr_err("Cannot allocate device buffer memory\n");
		pr_info("Device probe failed\n");
		return -ENOMEM;
	}
	
	// 4. Get the device Number
	dev_data->dev_num = pcdrv_data.device_num_base + pdev->id;
	
	// 5. Do cdev init and cdev add
	cdev_init(&dev_data->cdev,&pcd_fops);
	dev_data->cdev.owner = THIS_MODULE;
	ret = cdev_add(&dev_data->cdev,dev_data->dev_num,1);
	if(ret < 0) {
		pr_err("cdev add failed\n");
		pr_info("Device probe failed\n");
		return ret;
	}

	// 6. Create device file for the detected platform device
	pcdrv_data.device_pcd = device_create(pcdrv_data.class_pcd,dev,dev_data->dev_num,NULL,pdev->name);
	if(IS_ERR(pcdrv_data.device_pcd)) {
		pr_err("Device creation failed\n");
		cdev_del(&dev_data->cdev);
		pr_info("Device probe failed\n");
		return PTR_ERR(pcdrv_data.device_pcd);
	}

	pcdrv_data.total_devices++;

	ret = pcd_sysfs_create_files(pcdrv_data.device_pcd);
	if(ret) {
		pr_info("Sysfs attribute creation failed, Creation returned %d\n", ret);
		device_destroy(pcdrv_data.class_pcd, dev_data->dev_num);
		return ret;
	}

	// 7. Remap GPIO pins and set the correct values
	gpio_remapped_base = ioremap(GPIO_BASE_ADDRESS, GPIO_MEM_SIZE);
	if (gpio_remapped_base == NULL) {
        printk(KERN_INFO "ioremap() failed\n");
        return -EBUSY;
    }
	else {
		if(gpio_avail(dev_data->pdata.Rx))
			writel(FSEL_SET(0, dev_data->pdata.Rx), FSEL_ADDR(dev_data->pdata.Rx, gpio_remapped_base));
		if(gpio_avail(dev_data->pdata.Tx) && dev_data->pdata.Tx != dev_data->pdata.Rx)
			writel(FSEL_SET(1, dev_data->pdata.Tx), FSEL_ADDR(dev_data->pdata.Tx, gpio_remapped_base));
	}

	hrtimer_init(&dev_data->read_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev_data->read_hrtimer.function = &read_hrtimer_handler;

	hrtimer_init(&dev_data->write_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev_data->write_hrtimer.function = &write_hrtimer_handler;

	writel(GPIO_SET(1, dev_data->pdata.Tx), GPIO_SET_ADDR(dev_data->pdata.Tx, gpio_remapped_base));

	hrtimer_start(&dev_data->read_hrtimer, ns_to_ktime((u64)1000000000/dev_data->pdata.baudrate), HRTIMER_MODE_REL);

	mutex_init(&dev_data->pcdev_lock); // Initialize locks to handle possible race conditions
	
	pr_info("Probe was successful\n");
	return 0;
}

// Gets called when a device is removed
int pcd_platform_driver_remove(struct platform_device *pdev) {
	struct pcdev_private_data *dev_data = dev_get_drvdata(&pdev->dev);

	writel(GPIO_SET(1, dev_data->pdata.Rx), GPIO_CLR_ADDR(dev_data->pdata.Rx, gpio_remapped_base));
	writel(GPIO_SET(1, dev_data->pdata.Tx), GPIO_CLR_ADDR(dev_data->pdata.Tx, gpio_remapped_base));

	hrtimer_cancel(&dev_data->write_hrtimer);
	
	// 1. Remove a device that was created with device_create()
	device_destroy(pcdrv_data.class_pcd,dev_data->dev_num);
	
	// 2. Remove cdev entry from system
	cdev_del(&dev_data->cdev);

	hrtimer_cancel(&dev_data->read_hrtimer);
	
	pcdrv_data.total_devices--;

	pr_info("Device is removed\n");
	return 0;
}

struct platform_device_id pcdev_ids[] = {
	[0] = {.name = "tty666"}
};

struct platform_driver pcd_platform_driver = {
	.probe = pcd_platform_driver_probe,
	.remove = pcd_platform_driver_remove,
	.id_table = pcdev_ids,
	.driver = {
			.name = "pseudo-char-device"
	}
};

static int __init pcd_driver_init(void) {
	int ret;
	
	// 1. Dynamicaly allocate a device number for MAX_DEVICES
	ret = alloc_chrdev_region(&pcdrv_data.device_num_base,0,MAX_DEVICES,"pcdevs");
	if(ret < 0)
		pr_err("Alloc chardev failed!\n");
	
	// 2. Create a device class under /sys/class
	pcdrv_data.class_pcd = class_create(THIS_MODULE,"pcd_class");
	if(IS_ERR(pcdrv_data.class_pcd)) {
		pr_info("Class creation failed\n");
		ret = PTR_ERR(pcdrv_data.class_pcd);
		unregister_chrdev_region(pcdrv_data.device_num_base,MAX_DEVICES);
		return ret;
	}
	
	// 3. Register the platform driver	
	platform_driver_register(&pcd_platform_driver);

	pr_info("Platform driver loaded Successfully\n");
	
	return 0;
}

static void __exit pcd_driver_cleanup(void) {
	// 1. Register the platform driver
	platform_driver_unregister(&pcd_platform_driver);
	
	// 2. Class destroy
	class_destroy(pcdrv_data.class_pcd);
	
	// 3. Unregister device numbers for MAX_DEVICES
	unregister_chrdev_region(pcdrv_data.device_num_base,MAX_DEVICES);
	
	pr_info("Platform driver unloaded Successfully\n");
}

module_init(pcd_driver_init);
module_exit(pcd_driver_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ATHUL");
MODULE_DESCRIPTION("A Pseudo character driver");
