
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/amba/xilinx_dma.h>

//#include <linux/dma-mapping.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("John M Mower <mowerj@apl.washington.edu>");
MODULE_DESCRIPTION
    ("radar_dma - loadable module with self probe");

#define DRIVER_NAME "radar_dma"

static dev_t f_dev;
static struct cdev c_dev;
static struct class *cl;
static bool is_open;
static struct dma_chan *tx_chan;
static struct dma_chan *rx_chan;

static int radar_dma_open(struct inode *inode, struct file *file)
{
    if (is_open) 
	return -EMFILE;
    else
	is_open = true;
    return 0;
}

static int radar_dma_close(struct inode *inode, struct file *file)
{
    is_open = false;
    return 0;
}

static struct file_operations FileOps = 
{
    .owner = THIS_MODULE,
    .open = radar_dma_open,
    .release = radar_dma_close,
};

static bool radar_dma_filter(struct dma_chan *chan, void *param)
{
    if (*((int *)chan->private) == *(int *)param)
	return true;
    return false;
}

static void radar_dma_probe(void)
{
    printk(KERN_INFO "entering dma_probe\n");

    dma_cap_mask_t mask;
    
    u32 tx_match;
    u32 rx_match;
    enum dma_data_direction direction;

    dma_cap_zero(mask);
    dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

    direction = DMA_MEM_TO_DEV;
    tx_match = (direction & 0xFF) | XILINX_DMA_IP_DMA;
    printk("dmatest: match is %x\n", tx_match);

    tx_chan = dma_request_channel(mask, radar_dma_filter, (void *)&tx_match);

    if (tx_chan)
	printk("dmatest: Found tx device\n");
    else
	printk("dmatest: Did not find tx device\n");

    direction = DMA_DEV_TO_MEM;
    rx_match = (direction & 0xFF) | XILINX_DMA_IP_DMA;
    printk("dmatest: match is %x\n", rx_match);
    rx_chan = dma_request_channel(mask, radar_dma_filter, (void *)&rx_match);

    if (rx_chan)
	printk("dmatest: Found rx device\n");
    else
	printk("dmatest: Did not find rx device\n");

}

static void radar_dma_release(void)
{
    if (tx_chan)
	dma_release_channel(tx_chan);
    if (rx_chan)
	dma_release_channel(rx_chan);
}

static int __init radar_dma_init(void)
{
    int ret;
    is_open = false;

    printk(KERN_INFO "loading %s module\n", DRIVER_NAME);

    ret = alloc_chrdev_region(&f_dev, 0, 1, DRIVER_NAME);
    if (ret < 0) {
	printk(KERN_ALERT "%s alloc chrdev failed\n", DRIVER_NAME);
	return ret;
    }

    cl = class_create(THIS_MODULE, "chardev");
    if (cl == NULL) {
	printk(KERN_ALERT "%s class create failed\n", DRIVER_NAME);
	ret = -ENODEV;
	goto ERR1;
    }

    if (device_create(cl, NULL, f_dev, NULL, DRIVER_NAME) == NULL) {
	printk(KERN_ALERT "%s device create failed\n", DRIVER_NAME);
	ret = -ENODEV;
	goto ERR2;
    }

    cdev_init(&c_dev, &FileOps);

    ret = cdev_add(&c_dev, f_dev, 1);
    if (ret < 0) {
	printk(KERN_ALERT "%s c_dev add failed\n", DRIVER_NAME);
	goto ERR3;
    }

    radar_dma_probe();

    return 0;

ERR3:
    cdev_del(&c_dev);
    device_destroy(cl, f_dev);
ERR2:
    class_destroy(cl);
ERR1:
    unregister_chrdev_region(f_dev, 1);
    return ret;
}

static int __exit radar_dma_exit(void)
{
    radar_dma_release();
    cdev_del(&c_dev);
    device_destroy(cl, f_dev);
    class_destroy(cl);
    unregister_chrdev_region(f_dev, 1);
    return 0;
}

module_init(radar_dma_init);
module_exit(radar_dma_exit);

