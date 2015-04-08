/*
 * rx1_rfio RF switch
 *
 * Copyright 2015 GR UPM
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitrev.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/io.h>

struct rx1_rfio_platform_data{
	unsigned long *base_addr;   
	struct resource *res;      
	unsigned long remap_size;   
};

struct rx1_rfio_platform_data pdata;

struct rx1_rfio_state {
	unsigned int selected;
	unsigned int rfen;
	unsigned int picrst;
	struct rx1_rfio_platform_data* pdata;
};

static ssize_t rx1_rfio_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct rx1_rfio_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	int ret=0;
	unsigned int regval=0;
	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)private) {
	case 0:
		if(readin < 0 || readin > 4) ret = -EINVAL;
		else st->selected = readin;
		break;
	case 1:
		st->rfen = readin?0:1;	
		break;
	case 2:
		st->picrst =readin?1:0;	
		break;
	default:
		ret = -EINVAL;
	}

	if (!ret){
		regval|=st->rfen;
		regval|=st->selected?(st->selected)<<1:0xE;
		regval|=st->picrst<<7;
		iowrite32(regval, st->pdata->base_addr);
	}
	mutex_unlock(&indio_dev->mlock);
	return ret ? ret : len;

}


static ssize_t rx1_rfio_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
struct rx1_rfio_state *st = iio_priv(indio_dev);
	int ret=0;
	mutex_lock(&indio_dev->mlock);
	switch ((u32)private) {
	case 0:
		switch(st->selected){
			case 0:
				ret=sprintf(buf,"%s","Disconnected");
				break;
			case 1:
				ret=sprintf(buf,"%s","Test");
				break;
			case 2:
				ret=sprintf(buf,"%s","1.2GHz-3GHz");
				break;
			case 3:	
				ret=sprintf(buf,"%s","50ohm load");
				break;	
			case 4:
				ret=sprintf(buf,"%s","20MHz-1.2GHz");
				break;
			}
		break;
	case 1:
		ret=sprintf(buf,"%s",st->rfen?"RF disabled":"RF enabled");	
		break;
	case 2:
		ret=sprintf(buf,"%s",st->picrst?"PIC held in reset":"PIC running");	
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}



#define _RX1_RFIO_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = rx1_rfio_read, \
	.write = rx1_rfio_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info rx1_rfio_ext_info[] = {
	_RX1_RFIO_EXT_INFO("branch", 0),
	_RX1_RFIO_EXT_INFO("rfen", 1),
	_RX1_RFIO_EXT_INFO("picrst", 2),
	{ },
};

static const struct iio_chan_spec rx1_rfio_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.ext_info = rx1_rfio_ext_info,
};

static const struct iio_info rx1_rfio_info = {
	.driver_module = THIS_MODULE,
};


static int rx1_rfio_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct rx1_rfio_state *st;
     	int ret = 0;
 
     	pdata.res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
     	if (!pdata.res) {
         	dev_err(&pdev->dev, "No memory resource\n");
         	return -ENODEV;
     	}
 
     	pdata.remap_size = pdata.res->end - pdata.res->start + 1;
     	if (!request_mem_region(pdata.res->start, pdata.remap_size, pdev->name)) {
         	dev_err(&pdev->dev, "Cannot request IO\n");
         	return -ENXIO;
    	}
 
     	pdata.base_addr = ioremap(pdata.res->start, pdata.remap_size);
     	if (pdata.base_addr == NULL) {
         	dev_err(&pdev->dev, "Couldn't ioremap memory at 0x%08lx\n",
             		(unsigned long)pdata.res->start);
         	ret = -ENOMEM;
        	goto err_release_region;
     	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret =  -ENOMEM;
		dev_err(&pdev->dev, "failed to allocate indio_dev\n");
		goto err_alloc_indio;
	}
	st = iio_priv(indio_dev);
	platform_set_drvdata(pdev,indio_dev);
	st->pdata = &pdata;

	indio_dev->dev.parent = NULL;
	indio_dev->name = "RX1SW";

	indio_dev->info = &rx1_rfio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &rx1_rfio_chan;
	indio_dev->num_channels = 1;

	ret = iio_device_register(indio_dev);
	iowrite32(0x8F, pdata.base_addr);
	udelay(1000);
	iowrite32(0x0F, pdata.base_addr);
     	return 0;
 	err_alloc_indio:
		iounmap(pdata.base_addr);
  	err_release_region:
     		release_mem_region(pdata.res->start, pdata.remap_size);
     	return ret;
}

static int rx1_rfio_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
     	iounmap(pdata.base_addr);
	release_mem_region(pdata.res->start, pdata.remap_size);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rx1_rfio_dt_match[] = {
	{ .compatible = "gr,rx1_rfio" },
	{},
};
MODULE_DEVICE_TABLE(of, rx1_rfio_dt_match);
#endif

static struct platform_driver rx1_rfio_driver = {
	.driver = {
		.name	= "rx1_rfio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rx1_rfio_dt_match),
	},
	.probe		= rx1_rfio_probe,
	.remove		= rx1_rfio_remove,
};



module_platform_driver(rx1_rfio_driver);

MODULE_AUTHOR("David Marcos <dmarcosgon@gr.ssr.upm.es>");
MODULE_DESCRIPTION("MACOM rx1_rfio RF SWITCH");
MODULE_LICENSE("GPL v2");

