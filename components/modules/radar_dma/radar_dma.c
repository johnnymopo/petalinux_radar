 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/amba/xilinx_dma.h>
#include <xen/page.h>


/* this code based on:
   - bmartini/xdma - https://github.com/bmartini/zynq-xdma
   - axidmatest.c
   - drivers-session4-dma-4public.pdf
*/


MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("John M Mower <mowerj@apl.washington.edu>");
MODULE_DESCRIPTION
    ("radar_dma - loadable module with self probe");

#define DRIVER_NAME "radar_dma"

static unsigned int DMA_MAX_MEM = 1*1024*1024;
module_param(DMA_MAX_MEM, uint, S_IRUGO);
MODULE_PARM_DESC(DMA_MAX_MEM, "size of maximum allocated dma memory");

static dev_t f_dev;
static struct cdev c_dev;
static struct class *cl;
static bool is_open;
static bool is_mapped;

struct radar_dma_dev {
    u32 device_id;
    struct dma_chan *chan;    
    struct completion *cmp;
};
static struct radar_dma_dev *dma_dev;

static char *radar_addr;
static dma_addr_t radar_handle;
static unsigned long radar_dma_size;
static int low_high_pos;
static int msec_timeout;

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

static int radar_dma_mmap(struct file *filp, struct vm_area_struct *vma)
{
    unsigned long requested_size;
    requested_size = vma->vm_end - vma->vm_start;
    radar_dma_size = requested_size;
    printk(KERN_INFO "%s: reserved: %d, mmap size requested: %lu\n",
	   DRIVER_NAME, DMA_MAX_MEM, requested_size);

    if (requested_size > DMA_MAX_MEM) 
	return -EAGAIN;
    
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    if(remap_pfn_range(vma, vma->vm_start, 
		       virt_to_pfn(radar_addr), requested_size, vma->vm_page_prot)) {
	printk(KERN_ERR "%s Error: in calling remap_pfn_range\n",DRIVER_NAME);
	return -EAGAIN;
    }

    is_mapped = true;

    return 0;
}

static void radar_sync_callback(void *completion)
{
    complete(completion);
}

static ssize_t radar_dma_read(struct file* F, char *buf, size_t count, loff_t *f_pos)
{
    dma_addr_t buffer;
    struct dma_async_tx_descriptor *chan_desc;
    dma_cookie_t cookie;
    unsigned long tmo;
    enum dma_status status;

    if (!is_mapped) {
	printk(KERN_ERR "%s: memory is not mapped\n", DRIVER_NAME);
	return -1;
    }

    buffer = radar_handle;

    chan_desc = dmaengine_prep_slave_single(dma_dev->chan, buffer, (size_t)radar_dma_size, 
					    DMA_DEV_TO_MEM, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
    if (!chan_desc) {
	printk(KERN_ERR "%s: dmaengine_prep_slave_single error\n", DRIVER_NAME);
	return -1;
    }

    chan_desc->callback = radar_sync_callback;
    chan_desc->callback_param = dma_dev->cmp;

    cookie = chan_desc->tx_submit(chan_desc);
    if (dma_submit_error(cookie)) {
	printk(KERN_ERR "%s: tx_submit error\n", DRIVER_NAME);
	return -1;
    }

    init_completion(dma_dev->cmp);
    dma_async_issue_pending(dma_dev->chan);
    tmo = wait_for_completion_timeout(dma_dev->cmp, msecs_to_jiffies(msec_timeout));
    status = dma_async_is_tx_complete(dma_dev->chan, cookie, NULL, NULL);

    if (tmo == 0) {
	printk(KERN_ERR "%s: transfer timed out\n", DRIVER_NAME);
	return -1;
    }

    if (status != DMA_SUCCESS) {
	printk(KERN_DEBUG "%s: transfer, returned %s\n", DRIVER_NAME,
	       status == DMA_ERROR ? "error" : "in progress");
	return -1;
    }

    return 0;
}

static struct file_operations FileOps = 
{
    .owner = THIS_MODULE,
    .open = radar_dma_open,
    .release = radar_dma_close,
    .mmap = radar_dma_mmap,
    .read = radar_dma_read,
};

static bool radar_dma_filter(struct dma_chan *chan, void *param)
{
    if (*((int *)chan->private) == *(int *)param)
	return true;
    return false;
}

static int radar_dma_probe(void)
{
    dma_cap_mask_t mask;
    u32 match;
    enum dma_data_direction direction;

    dma_dev = (struct radar_dma_dev *)kzalloc(sizeof(struct radar_dma_dev), GFP_KERNEL);
    if (!dma_dev) 
	return -ENOMEM;

    // assume device_id is zero
    dma_dev->device_id = 0;

    dma_cap_zero(mask);
    dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

    direction = DMA_DEV_TO_MEM;
    match = (direction & 0xFF) | XILINX_DMA_IP_DMA | dma_dev->device_id << 28; 
    dma_dev->chan = dma_request_channel(mask, radar_dma_filter, (void *)&match);

    if (!dma_dev->chan) {
	kfree(dma_dev);
	return -ENODEV;
    }
    
    dma_dev->cmp = (struct completion *)kzalloc(sizeof(struct completion), GFP_KERNEL);
    if (!dma_dev->cmp) {
	dma_release_channel(dma_dev->chan);
	kfree(dma_dev);
	return -ENODEV;
    }

    
    return 0;
}

static void radar_dma_release(void)
{
    dma_release_channel(dma_dev->chan);
    kfree(dma_dev->cmp);
    kfree(dma_dev);
}

static int __init radar_dma_init(void)
{
    int ret;
    is_open = false;
    is_mapped = false;
    low_high_pos = 0;
    msec_timeout = 1000;
    
    printk(KERN_INFO "loading %s module\n", DRIVER_NAME);

    ret = alloc_chrdev_region(&f_dev, 0, 1, DRIVER_NAME);
    if (ret < 0) {
	printk(KERN_ERR "%s alloc chrdev failed\n", DRIVER_NAME);
	return ret;
    }

    cl = class_create(THIS_MODULE, "radar_dma_dev");
    if (cl == NULL) {
	printk(KERN_ERR "%s class create failed\n", DRIVER_NAME);
	ret = -ENODEV;
	goto ERR1;
    }

    if (device_create(cl, NULL, f_dev, NULL, DRIVER_NAME) == NULL) {
	printk(KERN_ERR "%s device create failed\n", DRIVER_NAME);
	ret = -ENODEV;
	goto ERR2;
    }

    cdev_init(&c_dev, &FileOps);

    ret = cdev_add(&c_dev, f_dev, 1);
    if (ret < 0) {
	printk(KERN_ERR "%s c_dev add failed\n", DRIVER_NAME);
	goto ERR3;
    }

    radar_addr = dma_zalloc_coherent(NULL, DMA_MAX_MEM, &radar_handle, GFP_KERNEL);
    if (!radar_addr) {
	printk(KERN_ERR "dma alloc failed\n");
	ret = -ENOMEM;
	goto ERR4;
    }

    ret = radar_dma_probe();
    if (ret < 0) {
	printk(KERN_ERR "%s: Did not find rx device\n",DRIVER_NAME);
	ret = -ENODEV;
	goto ERR5;
    }

    return 0;

ERR5:
    dma_free_coherent(NULL, DMA_MAX_MEM, radar_addr, radar_handle);
ERR4:
    cdev_del(&c_dev);
ERR3:
    device_destroy(cl, f_dev);
ERR2:
    class_destroy(cl);
ERR1:
    unregister_chrdev_region(f_dev, 1);
    return ret;
}

static int __exit radar_dma_exit(void)
{
    dma_free_coherent(NULL, DMA_MAX_MEM, radar_addr, radar_handle);
    radar_dma_release();
    cdev_del(&c_dev);
    device_destroy(cl, f_dev);
    class_destroy(cl);
    unregister_chrdev_region(f_dev, 1);

    return 0;
}

module_init(radar_dma_init);
module_exit(radar_dma_exit);

