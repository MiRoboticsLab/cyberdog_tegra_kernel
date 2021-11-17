/******************************************************************************
 * Copyright (C) 2015 Broadcom Corporation
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/clk.h>

#include <linux/tty_flip.h>
#include <linux/sysrq.h>
#include <linux/regulator/consumer.h>

#include <linux/serial_core.h>
#include <linux/of.h>

//TODO: this is temporary

#ifdef CONFIG_SENSORS_SSP_BBD
extern void bbd_parse_asic_data(unsigned char *pucData, unsigned short usLen, void (*to_gpsd)(unsigned char *packet, unsigned short len, void* priv), void* priv);
#endif

//--------------------------------------------------------------
//
//               Structs
//
//--------------------------------------------------------------
struct bcm_tty_priv
{
	int nstandby;
        int gps_1v8;
        int wifi_gpio;
        int wlan_en_gpio;
        int bt_en_gpio;
};

static struct bcm_tty_priv *g_bcm_gps;
static ssize_t bcm_4775_nstandby_show(struct device *dev,
	   struct device_attribute *attr,
	   char *buf)
{
	int value = 0;
        pr_err("[SSPBBD} bcm_4775_nstandby, read is begion ");
	//struct platform_device *spi = to_platform_device(dev);
	//struct bcm_tty_priv *priv = (struct bcm_tty_priv *)platform_get_drvdata(spi);

	//value = gpio_get_value(g_bcm_gps->nstandby);
        value = gpio_get_value_cansleep(g_bcm_gps->nstandby);
        pr_err("[SSPBBD} bcm_4775_nstandby, value is %d\n ",value);
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t bcm_4775_nstandby_store(struct device *dev,
	   struct device_attribute *attr,
	   const char *buf, size_t count)
{
	//struct platform_device *spi = to_platform_device(dev);
	//struct bcm_tty_priv *priv = (struct bcm_tty_priv *)platform_get_drvdata(spi);

	pr_err("[SSPBBD} bcm_4775_nstandby, gpio is %d\n", g_bcm_gps->nstandby);

        if (gpio_is_valid(g_bcm_gps->nstandby)) {
            pr_err("!!! SSPBBDbcm gpio nstandby is valid");
        }

	if (!strncmp("0", buf, 1))
		//gpio_set_value(g_bcm_gps->nstandby, 0);
                gpio_set_value_cansleep(g_bcm_gps->nstandby, 0);
	else
		//gpio_set_value(g_bcm_gps->nstandby, 1);
                gpio_set_value_cansleep(g_bcm_gps->nstandby, 1);
	return count;
}

static DEVICE_ATTR(nstandby, 0660,bcm_4775_nstandby_show, bcm_4775_nstandby_store);

/*static ssize_t bcm_4775_1v8_show(struct device *dev,
	   struct device_attribute *attr,
	   char *buf)
{
	int value = 0;
        pr_err("[SSPBBD} bcm_4775_nstandby, read is begion ");
	//struct platform_device *spi = to_platform_device(dev);
	//struct bcm_tty_priv *priv = (struct bcm_tty_priv *)platform_get_drvdata(spi);

	//value = gpio_get_value(g_bcm_gps->nstandby);
        value = gpio_get_value_cansleep(g_bcm_gps->gps_1v8);
        pr_err("[SSPBBD} bcm_4775_gps_1v8, value is %d\n ",value);
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t bcm_4775_1v8_store(struct device *dev,
	   struct device_attribute *attr,
	   const char *buf, size_t count)
{
	//struct platform_device *spi = to_platform_device(dev);
	//struct bcm_tty_priv *priv = (struct bcm_tty_priv *)platform_get_drvdata(spi);

	pr_err("[SSPBBD} bcm_4775_gps_1v8, gpio is %d\n", g_bcm_gps->gps_1v8);

        if (gpio_is_valid(g_bcm_gps->gps_1v8)) {
            pr_err("!!! SSPBBDbcm gpio nstandby is valid");
        }

	if (!strncmp("0", buf, 1))
		//gpio_set_value(g_bcm_gps->nstandby, 0);
                gpio_set_value_cansleep(g_bcm_gps->gps_1v8, 0);
	else
		//gpio_set_value(g_bcm_gps->nstandby, 1);
                gpio_set_value_cansleep(g_bcm_gps->gps_1v8, 1);
	return count;
}
static DEVICE_ATTR(gps_1v8, 0660,bcm_4775_1v8_show, bcm_4775_1v8_store);

static ssize_t bcm_4775_wifi_gpio_show(struct device *dev,
	   struct device_attribute *attr,
	   char *buf)
{
	int value = 0;
        pr_err("[SSPBBD} bcm_4775_wifi_gpio, read is begion ");
	//struct platform_device *spi = to_platform_device(dev);
	//struct bcm_tty_priv *priv = (struct bcm_tty_priv *)platform_get_drvdata(spi);

	//value = gpio_get_value(g_bcm_gps->nstandby);
        value = gpio_get_value_cansleep(g_bcm_gps->wifi_gpio);
        pr_err("[SSPBBD} bcm_4775_wifi_gpio, value is %d\n ",value);
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t bcm_4775_wifi_gpio_store(struct device *dev,
	   struct device_attribute *attr,
	   const char *buf, size_t count)
{
	//struct platform_device *spi = to_platform_device(dev);
	//struct bcm_tty_priv *priv = (struct bcm_tty_priv *)platform_get_drvdata(spi);

	pr_err("[SSPBBD} bcm_4775_wifi_gpio, gpio is %d\n", g_bcm_gps->wifi_gpio);

        if (gpio_is_valid(g_bcm_gps->wifi_gpio)) {
            pr_err("!!! SSPBBDbcm gpio nstandby is valid");
        }

	if (!strncmp("0", buf, 1))
		//gpio_set_value(g_bcm_gps->nstandby, 0);
                gpio_set_value_cansleep(g_bcm_gps->wifi_gpio, 0);
	else
		//gpio_set_value(g_bcm_gps->nstandby, 1);
                gpio_set_value_cansleep(g_bcm_gps->wifi_gpio, 1);
	return count;
}

static DEVICE_ATTR(wifi_gpio, 0660,bcm_4775_wifi_gpio_show, bcm_4775_wifi_gpio_store);*/


//--------------------------------------------------------------
//
//               Module init/exit
//
//--------------------------------------------------------------
static int xiaomi_uart_probe(struct platform_device *pdev)
{
	struct bcm_tty_priv *priv;
	//int ret, irq;
        int ret;
        int nstandby = 0;
        struct regulator *reg;
	/* Check GPIO# */
        printk(KERN_ERR "KERN_ERR [SSPBBD] gps probe \n");
	dev_err(&pdev->dev,"[SSPBBD]: Check platform_data for bcm device\n");


	/*===================================================
	  We need folowing OF node in dts

	  bcm477x-gpio {
		  ssp-mcu-req = <some_gpio_number>
		  ssp-mcu-resp = <some_gpio_number>
		  ssp-host-req = <some_gpio_number>
	  }
	  ===================================================== */
	//struct device_node *np = of_find_node_by_name(NULL, "bcm477x-gpio");
        if (!pdev->dev.of_node) {
		pr_err("[SSPBBD]: Failed to find of_node\n");
		goto err_exit;
	}
        nstandby = of_get_named_gpio(pdev->dev.of_node, "nstandby-gpio", 0);
        
        //nstandby = of_get_named_gpio(pdev->dev.of_node, "nstandby-gpio", 0);
	if (nstandby < 0 ) {
                printk(KERN_ERR "KERN_ERR [SSPBBD] nstandby fail result is %d \n",nstandby);
		printk(KERN_ERR "KERN_ERR [SSPBBD] fail to find OF node nstandby-gpio\n");
		return  -EPROBE_DEFER;
	}
	printk(KERN_ERR "[SSPBBD] nstandby=%d\n", nstandby);


        /*gps_1v8 = of_get_named_gpio(pdev->dev.of_node, "gps-1v8-gpio", 0);
	if (gps_1v8 < 0 ) {
                printk(KERN_ERR "KERN_ERR [SSPBBD] gps_1v8 fail result is %d \n",gps_1v8);
		pr_err(KERN_ERR "[SSPBBD] fail to find OF node gps-1v8-gpio\n");
		return  -EPROBE_DEFER;
	}


	//of_property_read_u32(np, "nstandby-gpio", &nstandby);



	printk(KERN_ERR "[SSPBBD] nstandby=%d\n", nstandby);
        printk(KERN_ERR "[SSPBBD] gps_1v8=%d\n", gps_1v8);


	
	

	
	ret = gpio_request(gps_1v8, "gps_1v8-gpio");
	if (ret) {
		printk(KERN_ERR "SSPBBD request GPS gps_1v8 fail %d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(gps_1v8, 1);
	if (ret) {
		printk(KERN_ERR "SSPBBD set GPS gps_1v8 as out mode fail %d", ret);
		goto err_exit;
	}*/
        reg = devm_regulator_get(&pdev->dev, "vdd");
        if (IS_ERR(reg)) {
            ret = PTR_ERR(reg);
            if (ret != -EPROBE_DEFER) 
                 dev_err(&pdev->dev, "SSPBBDreg get err: %d\n", ret);
            return ret;
        }
        ret = regulator_enable(reg);
        if (ret) {
             dev_err(&pdev->dev, "SSPBBDreg en err: %d\n", ret);
             return ret;
        }

	ret = gpio_request(nstandby, "nstandby-gpio");
	if (ret) {
		printk(KERN_ERR "SSPBBD request GPS NSTANDBY fail %d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(nstandby, 1);
	if (ret) {
		printk(KERN_ERR "SSPBBD set GPS NSTANDBY as out mode fail %d", ret);
		goto err_exit;
	}
       msleep(30);
        reg = devm_regulator_get(&pdev->dev, "vddgps");
        if (IS_ERR(reg)) {
            ret = PTR_ERR(reg);
            if (ret != -EPROBE_DEFER) 
                 dev_err(&pdev->dev, "SSPBBDreg get err: %d\n", ret);
            return ret;
        }
        ret = regulator_enable(reg);
        if (ret) {
             dev_err(&pdev->dev, "SSPBBDreg en err: %d\n", ret);
             return ret;
        }


        /*wifi_gpio = of_get_named_gpio(pdev->dev.of_node, "gps-bt-3v3-gpio", 0);
        if (wifi_gpio < 0) {
            dev_err(&pdev->dev, "%s:  ssp bbdno wifi gpio provided, will not HW wifi device\n", __func__);
            return -1;
        } else {
            dev_info(&pdev->dev, "%s: ssp bbd wifi gpio provided ok, wifi:%d\n", __func__, wifi_gpio);
        }
	ret = gpio_request(wifi_gpio, "gps-bt-3v3-gpio");
	if (ret) {
		printk(KERN_ERR "SSPBBD request GPS NSTANDBY fail %d", ret);
		goto err_exit;
	}
        gpio_direction_output(wifi_gpio, 1);*/

	/* Alloc */
	priv = (struct bcm_tty_priv*) kmalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		printk(KERN_ERR "!!!!SSPBBDkmalloc  fail sspbbd gps");
		goto err_exit;
	}

	memset(priv, 0, sizeof(*priv));

	/* Init - gpios */

	priv->nstandby = nstandby;
        //priv->gps_1v8 = gps_1v8;
        //priv->wifi_gpio = wifi_gpio;

        g_bcm_gps = priv;

	if (device_create_file(&pdev->dev, &dev_attr_nstandby))
		pr_err("!!! SSPBBDbcm Unable to create sysfs 4775 nstandby entry");

	/*if (device_create_file(&pdev->dev, &dev_attr_gps_1v8))
		pr_err("!!! SSPBBDbcm Unable to create sysfs 4775 nstandby entry");

	if (device_create_file(&pdev->dev, &dev_attr_wifi_gpio))
		pr_err("!!! SSPBBDbcm Unable to create sysfs 4775 nstandby entry");*/

	return 0;


err_exit:
	return -ENODEV;
}


static int xiaomi_uart_remove(struct platform_device *pdev)
{
        device_remove_file(&pdev->dev, &dev_attr_nstandby);
        g_bcm_gps = NULL;
	return 0;
}


static const struct of_device_id match_table[] = {
	{ .compatible = "bcm4775",},
	{},
};


/*
 * platform driver stuff
 */
static struct platform_driver xiaomi_uart_platform_driver = {
	.probe	= xiaomi_uart_probe,
	.remove	= xiaomi_uart_remove,
	.driver	= {
		.name  = "bcm4775",
		.of_match_table = match_table,

	},
};

static int __init xiaomi_tty_init(void)
{
	int ret;
        printk(KERN_ERR "!!! bcm_tty_init  to go");
	ret = platform_driver_register(&xiaomi_uart_platform_driver);
        printk(KERN_ERR "!!! platform_driver_register  sspbbd bcm_gps_tty misc_register ret is %d",ret);

	return ret;
}

static void __exit xiaomi_tty_exit(void)
{
    platform_driver_unregister(&xiaomi_uart_platform_driver);
}

//module_platform_driver(xiaomi_uart_platform_driver)

//late_initcall(xiaomi_tty_init);
late_initcall_sync(xiaomi_tty_init);
//module_init(xiaomi_tty_init);
module_exit(xiaomi_tty_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM TTY Driver");

