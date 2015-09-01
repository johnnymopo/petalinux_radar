#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "radar_ioctl.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("John M Mower");
MODULE_DESCRIPTION("radar controller driver");

#define DRIVER_NAME "radar_controller"

static dev_t f_dev;
static struct cdev c_dev;
static struct class *cl;
static int is_open;
static struct radar_params *params;
static struct timespec *ts;
static void *regs;

inline void dev_write_io(u32 data, void *addr)
{
    iowrite32(data, addr);
}

inline u32 dev_read_io(void *addr)
{
    return ioread32(addr);
}

void dev_write_params()
{
    u32 control_word = 0;

    dev_write_io(params->time, regs+RADAR_EPOCH_OFFSET);
    dev_write_io(params->trigger_dwell, regs+RADAR_TRIGGER_OFFSET);
    dev_write_io(params->sys_clock_frequency, regs+RADAR_CLOCK_OFFSET);

    if (params->use_external_pps)
	control_word = control_word | 1<<RADAR_PPS_EXTERN_BIT;
    if (params->enable_trigger)
	control_word = control_word | 1<<RADAR_TRIGGER_ENABLE_BIT;
    if (params->reset)
	control_word = control_word | 1<<RADAR_RST_BIT;
    control_word = control_word | params->leds<<RADAR_LED_BITS | 1<<RADAR_WRITE_REG_BIT;

    dev_write_io(control_word, regs+RADAR_CONTROL_OFFSET);

    udelay(100);

    control_word = control_word & ~(1<<RADAR_WRITE_REG_BIT);

    dev_write_io(control_word, regs+RADAR_CONTROL_OFFSET);
}

void dev_read_params()
{
    u32 control_word = dev_read_io(regs+RADAR_CONTROL_OFFSET);
    params->use_external_pps = (control_word & 1<<RADAR_PPS_EXTERN_BIT) >> RADAR_PPS_EXTERN_BIT;
    params->enable_trigger = (control_word & 1<<RADAR_TRIGGER_ENABLE_BIT) >> RADAR_TRIGGER_ENABLE_BIT;
    params->reset = (control_word & 1<<RADAR_RST_BIT) >> RADAR_RST_BIT;
    params->leds = control_word >> RADAR_LED_BITS;
    params->time = dev_read_io(regs+RADAR_EPOCH_OFFSET);
    params->trigger_dwell = dev_read_io(regs+RADAR_TRIGGER_OFFSET);
    params->sys_clock_frequency = dev_read_io(regs+RADAR_CLOCK_OFFSET);	
}

void dev_set_defaults()
{
    getnstimeofday(ts);    

    params->use_external_pps = RADAR_DEFAULT_EXTERNAL_PPS;
    params->enable_trigger = RADAR_DEFAULT_TRIGGER_ENABLE;;
    params->reset = 1;
    params->leds = RADAR_DEFAULT_LEDS;
    params->time = 0;
    params->trigger_dwell = RADAR_DEFAULT_TRIGGER_DWELL;
    params->sys_clock_frequency = RADAR_DEFAULT_FREQUENCY;
    dev_write_params();
    udelay(100);
    params->reset = 0;
    params->time = (u32)ts->tv_sec;
    dev_write_params();
}

static long dev_ioctl(struct file* F, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
    case RADAR_SET_REGS:
	if(copy_from_user(params, (struct radar_params *)arg, sizeof(struct radar_params))){
	    printk(KERN_ALERT "ioctl copy_from_user failed\n");
	    return -EACCES;
	}
        dev_write_params();
	break;
    case RADAR_GET_REGS:
	dev_read_params();
	if (copy_to_user((struct radar_params *)arg, params, sizeof(struct radar_params))){
	    printk(KERN_ALERT "ioctl copy_to_user failed\n");
	    return -EACCES;
	}
	break;
    case RADAR_SET_DEFAULTS:
	dev_set_defaults();
	dev_write_params();
	break;
    default:
	return -EINVAL;
    }
    return 0;
}

static ssize_t dev_read(struct file* F, char *buf, size_t count, loff_t *f_pos)
{    
    return -EPERM;
}
 
static ssize_t dev_write(struct file* F, const char *buf, size_t count, loff_t *f_pos)
{
    return -EPERM;
}
 
static int dev_open(struct inode *inode, struct file *file)
{
    if (is_open > 0) {
	return -EMFILE;
    }	
    else {
	is_open++;
	return 0;
    }
}
 
static int dev_close(struct inode *inode, struct file *file)
{
    is_open--;
    return 0;
}


/***********************************************/
/* platform device land */

static struct of_device_id radar_controller_driver_of_match[] = {
    { .compatible = "mowerj,radar_controller",},
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, radar_controller_driver_of_match);

static int dev_probe(struct platform_device *pdev)
{
    printk(KERN_INFO "Device Tree Probing\n");

    const struct of_device_id *match;
    struct resource *res;
    
    match = of_match_device(radar_controller_driver_of_match, &pdev->dev);
    if (!match) {
	printk(KERN_ALERT "no match in device tree: %s\n", DRIVER_NAME);
	return -EINVAL;
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    printk(KERN_INFO "requesting memory size %x of %d\n",res->start, resource_size(res));
    if (!request_mem_region(res->start, resource_size(res), DRIVER_NAME)) {
	printk(KERN_ALERT "no mem region available\n");
	return -EBUSY;
    }

    regs = (void *)ioremap_nocache(res->start, resource_size(res));

    if (!regs) {
	printk(KERN_ALERT "iomap failed\n");
	release_mem_region(res->start, resource_size(res));
	return -ENOMEM;
    }

    printk(KERN_INFO "%s driver: reg %x remapped to %x\n", DRIVER_NAME, res->start, regs);

    return 0;
}

static int dev_remove(struct platform_device *pdev)
{
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    iounmap(regs);
    release_mem_region(res->start, resource_size(res));
    return 0;
}

static struct platform_driver radar_controller_driver = {
    .driver = {
	.name = DRIVER_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(radar_controller_driver_of_match),
    },
    .probe = dev_probe,
    .remove = dev_remove,
};

/* END: platform device land */
/***********************************************/

static struct file_operations FileOps =
{
    .owner = THIS_MODULE,
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_close,
    .unlocked_ioctl = dev_ioctl,
};

static int __init dev_init(void)
{
    is_open = 0;
    int ret = 0;
    
    printk(KERN_INFO "loading %s module\n", DRIVER_NAME);
    
    ret = platform_driver_probe(&radar_controller_driver, &dev_probe);
    if (ret < 0) {
	printk(KERN_ALERT "%s register failed\n", DRIVER_NAME);
	return ret;
    }
    
    ret = alloc_chrdev_region(&f_dev, 0, 1, DRIVER_NAME);
    if (ret < 0) {
	printk(KERN_ALERT "%s alloc chrdev failed\n", DRIVER_NAME);
	goto ERR0;
    }

    cl = class_create(THIS_MODULE, "chardev");
    if (cl == NULL) {
	printk(KERN_ALERT "%s class failed\n", DRIVER_NAME);
	ret = -ENODEV;
	goto ERR1;
    }

    if (device_create(cl, NULL, f_dev, NULL, DRIVER_NAME) == NULL) {
	printk(KERN_ALERT "%s device failed\n", DRIVER_NAME);
	ret = -ENODEV;
	goto ERR2;
    }

    cdev_init(&c_dev, &FileOps);

    ret = cdev_add(&c_dev, f_dev, 1);
    if (ret < 0) {
	printk(KERN_ALERT "%s device additon failed\n", DRIVER_NAME);
	goto ERR2;
    }

    params = (struct radar_params *)kmalloc(sizeof(struct radar_params), GFP_KERNEL);
    if (!params) {
	ret = -ENOMEM;
	printk(KERN_ALERT "%s failed to alloc internal memory\n", DRIVER_NAME);
	goto ERR3;
    }


    ts = (struct timespec *)kmalloc(sizeof(struct timespec), GFP_KERNEL);
    if (!params) {
	ret = -ENOMEM;
	printk(KERN_ALERT "%s failed to alloc internal memory\n", DRIVER_NAME);
	goto ERR4;
    }

    dev_set_defaults();
    dev_write_params();

    return 0;

ERR4:
    kfree(params);
ERR3:
    cdev_del(&c_dev);
    device_destroy(cl, f_dev);
ERR2:
    class_destroy(cl);
ERR1:
    unregister_chrdev_region(f_dev, 1);
ERR0:
    platform_driver_unregister(&radar_controller_driver);
    return ret;

}

static int __exit dev_exit(void) 
{
    dev_set_defaults();
    params->leds = 0;
    dev_write_params();

    printk(KERN_INFO "unloading %s module\n", DRIVER_NAME);
    kfree(params);
    kfree(ts);
    cdev_del(&c_dev);
    device_destroy(cl, f_dev);
    class_destroy(cl);
    unregister_chrdev_region(f_dev, 1);
    platform_driver_unregister(&radar_controller_driver);
    return 0;
}

module_init(dev_init);
module_exit(dev_exit);

