/*  radar_dma.c - The simplest kernel module.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

/* Standard module information, edit as appropriate */
MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("John Williams - PetaLogix Qld Pty Ltd <john.williams@petalogix.com>");
MODULE_DESCRIPTION
    ("radar_dma - loadable module template generated by petalinux-new-module");

#define DRIVER_NAME "radar_dma"

/* Simple example of how to receive command line parameters to your module.
   Delete if you don't need them */
unsigned myint = 0xdeadbeef;
char *mystr = "default";

module_param(myint, int, S_IRUGO);
module_param(mystr, charp, S_IRUGO);

struct radar_dma_local {
	int irq;
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
};

static irqreturn_t radar_dma_irq(int irq, void *lp)
{
	printk("radar_dma interrupt\n");
	return IRQ_HANDLED;
}

static int radar_dma_probe(struct platform_device *pdev)
{
	struct resource *r_irq; /* Interrupt resources */
	struct resource *r_mem; /* IO mem resources */
	struct device *dev = &pdev->dev;
	struct radar_dma_local *lp = NULL;

	int rc = 0;
	
	dev_info(dev, "Device Tree Probing\n");

	/* Get iospace for the device */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(dev, "invalid address\n");
		return -ENODEV;
	}
	
	lp = (struct radar_dma_local *) kmalloc(sizeof(struct radar_dma_local), GFP_KERNEL);
	if (!lp) {
		dev_err(dev, "Cound not allocate radar_dma device\n");
		return -ENOMEM;
	}
	
	dev_set_drvdata(dev, lp);
	
	lp->mem_start = r_mem->start;
	lp->mem_end = r_mem->end;

	if (!request_mem_region(lp->mem_start,
				lp->mem_end - lp->mem_start + 1,
				DRIVER_NAME)) {
		dev_err(dev, "Couldn't lock memory region at %p\n",
			(void *)lp->mem_start);
		rc = -EBUSY;
		goto error1;
	}

	lp->base_addr = ioremap(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	if (!lp->base_addr) {
		dev_err(dev, "radar_dma: Could not allocate iomem\n");
		rc = -EIO;
		goto error2;
	}

	/* Get IRQ for the device */
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r_irq) {
		dev_info(dev, "no IRQ found\n");
		dev_info(dev, "radar_dma at 0x%08x mapped to 0x%08x\n",
			(unsigned int __force)lp->mem_start,
			(unsigned int __force)lp->base_addr);
		return 0;
	} 
	lp->irq = r_irq->start;
	
	rc = request_irq(lp->irq, &radar_dma_irq, 0, DRIVER_NAME, lp);
	if (rc) {
		dev_err(dev, "testmodule: Could not allocate interrupt %d.\n",
			lp->irq);
		goto error3;
	}

	dev_info(dev,"radar_dma at 0x%08x mapped to 0x%08x, irq=%d\n",
		(unsigned int __force)lp->mem_start,
		(unsigned int __force)lp->base_addr,
		lp->irq);
	return 0;
error3:
	free_irq(lp->irq, lp);
error2:
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
error1:
	kfree(lp);
	dev_set_drvdata(dev, NULL);
	return rc;
}

static int radar_dma_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct radar_dma_local *lp = dev_get_drvdata(dev);
	free_irq(lp->irq, lp);
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	kfree(lp);
	dev_set_drvdata(dev, NULL);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id radar_dma_of_match[] = {
	{ .compatible = "petalogix,radar_dma", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, radar_dma_of_match);
#else
# define radar_dma_of_match
#endif


static struct platform_driver radar_dma_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= radar_dma_of_match,
	},
	.probe		= radar_dma_probe,
	.remove		= radar_dma_remove,
};

static int __init radar_dma_init(void)
{
	printk("<1>Hello module world.\n");
	printk("<1>Module parameters were (0x%08x) and \"%s\"\n", myint,
	       mystr);

	return platform_driver_register(&radar_dma_driver);
}


static void __exit radar_dma_exit(void)
{
	platform_driver_unregister(&radar_dma_driver);
	printk(KERN_ALERT "Goodbye module world.\n");
}

module_init(radar_dma_init);
module_exit(radar_dma_exit);
