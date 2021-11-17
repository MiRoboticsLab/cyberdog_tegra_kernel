// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 InvenSense, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/gfp.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>

#include "chx01_data.h"
#include "chx01_client.h"
#include "src/ch101_reg.h"
#include "src/system.h"

// Device tree entery for CH-101 on Asus Tinker (Debian 9)
//
// &i2c4 { 
// 	status = "okay";
// 	clock-frequency = <400000>;
	
// 	ch101: ch101@45 { 
// 		compatible = "invn,ch101";                
// 		reg = <0x45>;       
// 		pinctrl-names = "default";		
// 		pinctrl-0 = <&chirp_int &chirp_rst &chirp_prg &chirp_cal>;  
// 		/*pinctrl-0 = <&chirp_rst>, <&chirp_prg>, <&chirp_cal>;*/
		
// 		rst-gpios = <&gpio8 3 GPIO_ACTIVE_HIGH>;     		
// 		prg-gpios = <&gpio8 8 GPIO_ACTIVE_HIGH>;  
// 		int-gpios = <&gpio7 22 GPIO_ACTIVE_HIGH>; 	 
// 		cal-gpios = <&gpio8 6 GPIO_ACTIVE_HIGH>; 
		
// 		interrupt-parent = <&gpio7>;
// 		interrupts = <22 IRQ_TYPE_EDGE_FALLING>;  
// 	};
// };
// &pinctrl {
// 	ch101 { 
// 		chirp_int: chirp-int{
// 			rockchip,pins = <RK_GPIO7 22 RK_FUNC_GPIO &pcfg_pull_none>;
// 		};
// 		chirp_rst: chirp-rst{
// 			rockchip,pins = <RK_GPIO8 3 RK_FUNC_GPIO &pcfg_pull_none>;
// 		};		
// 		chirp_prg: chirp-prg{
// 			rockchip,pins = <RK_GPIO8 8 RK_FUNC_GPIO &pcfg_pull_none>;
// 		};
// 		chirp_cal: chirp-cal{
// 			rockchip,pins = <RK_GPIO8 6 RK_FUNC_GPIO &pcfg_pull_none>;
// 		};		
// 	};
// };

static struct chx01_callbacks *cbk;

static const struct regmap_config chx01_i2c_regmap_config = {
	.reg_bits = 8, .val_bits = 8,
};

static int read_reg(void *client, u16 i2c_addr, u8 reg, u16 length, u8 *data)
{
	int res;
	int i;

	struct i2c_msg msg[] = {
		{ .addr = i2c_addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = i2c_addr, .flags = I2C_M_RD,
	  .len = length, .buf = (char *)data }
	};

	struct i2c_client *i2c_client = (struct i2c_client *)client;
	struct i2c_adapter *i2c_adapter = i2c_client->adapter;

	dbg_info("<<chx01>>%s -> i2c_addr: 0x%x, reg: 0x%x, length: %d\n", __func__, i2c_addr, reg, length);

	res = i2c_transfer(i2c_adapter, msg, 2);
	if(length <= 4)
		for(i=0; i< length; i++) dbg_info("<<chx01>>%s -> data[%d] = 0x%x\n", __func__, i, data[i]); //debug--yd

	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else {
		return 0;
	}
}

static int write_reg(void *client, u16 i2c_addr, u8 reg, u16 length, u8 *data)
{
	int res;
	int i;
	struct i2c_msg msg[] = {
		{ .addr = i2c_addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = i2c_addr, .flags = 0, .len = length, .buf = (char *)data }
	};

	struct i2c_client *i2c_client = (struct i2c_client *)client;
	struct i2c_adapter *i2c_adapter = i2c_client->adapter;

	dbg_info("<<chx01>>%s -> i2c_addr: 0x%x, reg: 0x%x, length: %d\n", __func__, i2c_addr, reg, length);
	if(length <= 4)
		for(i=0; i< length; i++) dbg_info("<<chx01>>%s -> data[%d] = 0x%x\n", __func__, i, data[i]); //debug--yd

	res = i2c_transfer(i2c_adapter, msg, 2);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else {
		return 0;
	}
}

static int chx01_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct device *dev;
	int ret = 0;

	dev = &client->dev;

	dev_info(dev, "%s: Start", __func__);

	if (!client)
		return -ENOMEM;

	if (!i2c_check_functionality(client->adapter,
	I2C_FUNC_SMBUS_BYTE_DATA |
	I2C_FUNC_SMBUS_WORD_DATA |
	I2C_FUNC_SMBUS_I2C_BLOCK)) {
		return -EPERM;
	}

	cbk = devm_kmalloc(dev, sizeof(struct chx01_callbacks), GFP_KERNEL);
	cbk->read = read_reg;
	cbk->write = write_reg;

	regmap = devm_regmap_init_i2c(client, &chx01_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Error initializing i2c regmap: %ld\n",
				PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	ret = chx01_core_probe(client, regmap, cbk, id ? id->name : NULL);

	dev_info(dev, "%s: End\n", __func__);

	return ret;
}

static int chx01_i2c_remove(struct i2c_client *client)
{
	struct device *dev;
	int ret = 0;

	dev = &client->dev;
	devm_kfree(dev, cbk);
	ret = chx01_core_remove(client);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int chx01_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct chx01_data *data = iio_priv(indio_dev);
	int ret;

	// mutex_lock(&data->lock);
	mutex_lock(&indio_dev->mlock);
	enable_irq_wake(data->irq);

	ret = regmap_write(data->regmap, CH_PROG_REG_CPU, 0x11);

	// mutex_unlock(&data->lock);
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int chx01_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct chx01_data *data = iio_priv(indio_dev);
	int ret;

	// mutex_lock(&data->lock);
	mutex_lock(&indio_dev->mlock);
	disable_irq_wake(data->irq);

	ret = regmap_write(data->regmap, CH_PROG_REG_CPU, 0x02);
	// mutex_unlock(&data->lock);
	mutex_unlock(&indio_dev->mlock);
	

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops chx01_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(chx01_suspend, chx01_resume)
};

static const struct i2c_device_id chx01_i2c_id[] = {
	{ "chx01", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, chx01_i2c_id);

static const struct of_device_id chx01_of_match[] = {
{ .compatible = "invn,chx01" },
{ },
};

MODULE_DEVICE_TABLE(of, chx01_of_match);

static struct i2c_driver chx01_i2c_driver = {
	.driver = {
		.name = "chx01_i2c",
		.of_match_table = chx01_of_match,
		.pm = &chx01_pm_ops,
	},
	.probe = chx01_i2c_probe,
	.remove = chx01_i2c_remove,
	.id_table = chx01_i2c_id,
};

module_i2c_driver(chx01_i2c_driver);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense CH101 I2C device driver");
MODULE_LICENSE("GPL");
