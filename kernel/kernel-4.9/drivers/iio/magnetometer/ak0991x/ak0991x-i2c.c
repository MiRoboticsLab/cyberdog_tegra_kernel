/* drivers/iio/magnetometer/ak0991x-i2c.c
 *
 * Copyright (C) 2014 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/iio/iio.h>
#include "ak0991x.h"
#include "ak0991x_iio.h"

/***** I2C Tx/Rx operation ******************************************/
static int aki2c_rxdata(struct device *dev, unsigned char *rxdata,
						int length)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxdata,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(dev, "%s: transfer failed(size error).",
			__func__);
		return -ENXIO;
	}

	dev_vdbg(dev, "RxData: len=%02x, addr=%02x  data=%02x",
		length, rxdata[0], rxdata[1]);
	return 0;
}

static int aki2c_txdata(struct device *dev, unsigned char *txdata,
						int length)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(dev, "%s: transfer failed(size error).",
			__func__);
		return -ENXIO;
	}

	dev_vdbg(dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txdata[0], txdata[1]);
	return 0;
}


static const struct ak0991x_bus_ops ak0991x_i2c_bops = {
	.rxdata  = aki2c_rxdata,
	.txdata  = aki2c_txdata,
};

/***** Probe/Remove function ****************************************/
static int ak0991x_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"%s: check_functionality failed.", __func__);
		return -ENODEV;
	}

	indio_dev = ak0991x_probe(&client->dev, client->irq, &ak0991x_i2c_bops);
	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);

	/* Success */
	i2c_set_clientdata(client, indio_dev);
	return 0;
}

static int ak0991x_i2c_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	return ak0991x_remove(indio_dev);
}

/***** Power management *********************************************/
static int ak0991x_i2c_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(i2c);
	struct ak0991x_data *akm = iio_priv(indio_dev);

	return ak0991x_suspend(akm);
}

static int ak0991x_i2c_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(i2c);
	struct ak0991x_data *akm = iio_priv(indio_dev);

	return ak0991x_resume(akm);
}

static const struct dev_pm_ops ak0991x_i2c_pops = {
	.suspend	= ak0991x_i2c_suspend,
	.resume		= ak0991x_i2c_resume,
};

/***** I2C interface ***********************************************/
static const struct i2c_device_id ak0991x_id[] = {
	{ AKM_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak0991x_id);

static const struct of_device_id ak0991x_of_match[] = {
	{ .compatible = "akm,ak09911", },
	{ .compatible = "akm,ak09912", },
	{ .compatible = "akm,ak09913", },
	{ .compatible = "akm,ak09915", },
	{ .compatible = "akm,ak09915d", },
	{ .compatible = "akm,ak09916", },
	{ .compatible = "akm,ak09916d", },
	{ .compatible = "akm,ak09918", },
	{ },
};
MODULE_DEVICE_TABLE(of, ak0991x_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id ak0991x_acpi_match[] = {
	{ AKM_DRIVER_NAME, 0 },
	{ },
};
#endif

static struct i2c_driver ak0991x_i2c_driver = {
	.probe		= ak0991x_i2c_probe,
	.remove		= ak0991x_i2c_remove,
	.id_table	= ak0991x_id,
	.driver = {
		.name = AKM_DRIVER_NAME,
		.bus = &i2c_bus_type,
		.owner = THIS_MODULE,
		.pm = &ak0991x_i2c_pops,
		.of_match_table = ak0991x_of_match,
		.acpi_match_table = ACPI_PTR(ak0991x_acpi_match),
	},
};

module_i2c_driver(ak0991x_i2c_driver)

MODULE_AUTHOR("Asahi Kasei Microdevices Corp. <multi-s@om.asahi-kasei.co.jp>");
MODULE_DESCRIPTION("AK0991X I2C compass driver");
MODULE_LICENSE("GPL");
