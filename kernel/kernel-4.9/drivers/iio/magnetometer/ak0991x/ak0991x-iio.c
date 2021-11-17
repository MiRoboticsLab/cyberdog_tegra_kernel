/* drivers/staging/iio/magnetometer/ak0991x.c
 *
 * Copyright (C) 2014 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * Authors: Rikita Yamada <yamada.rj (at) om.asahi-kasei.co.jp>
 *          Takashi Shiina <takashi.shiina (at) sonymobile.com>
 *          Masashi Shimizu <Masashi.X.Shimizu (at) sonymobile.com>
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

#define DEBUG
#define VERBOSE_DEBUG

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "ak0991x.h"
#include "ak0991x_iio.h"

/* this is 3-axis  sensor */
#define NUM_OF_AXIS		3

/* Setting for CNTL2 */
#define AK0991X_CNTL_PDN	0x00
#define AK0991X_CNTL_SNG	0x01
#define AK0991X_CNTL_CONT	0x02
#define AK0991X_CNTL_TEST	0x10
#define AK0991X_CNTL_FUSE	0x1F
/* setting for CNTL3 */
#define AK0991X_CNTL_RESET	0x01

/* other parameters */
#define AK0991X_COMPANY_ID	0x48
#define AK09911_DEVICE_ID	0x05
#define AK09912_DEVICE_ID	0x04
#define AK09913_DEVICE_ID	0x08
#define AK09915_DEVICE_ID	0x10
#define AK09915D_INFO_ID	0x02
#define AK09916_DEVICE_ID	0x09
#define AK09916D_DEVICE_ID	0x0B
#define AK09918_DEVICE_ID	0x0C
#define AK0991X_DATA_READY	0x01

/* register address and parameters */
#define AK0991X_ADDR_WIA	0x00
#define AK0991X_WIA_SIZE	4
#define AK0991X_ADDR_ST1	0x10
#define AK0991X_ADDR_HXL	0x11
#define AK0991X_ADDR_HXH	0x12
#define AK0991X_ADDR_HYL	0x13
#define AK0991X_ADDR_HYH	0x14
#define AK0991X_ADDR_HZL	0x15
#define AK0991X_ADDR_HZH	0x16
#define AK0991X_ADDR_TEMP	0x17
#define AK0991X_ADDR_ST2	0x18
#define AK0991X_DATA_SIZE	9
#define AK0991X_ADDR_FUSE	0x60
#define AK0991X_FUSE_SIZE	NUM_OF_AXIS
#define AK0991X_FUSE_DUMMY	(128)
#define AK09911_FUSE_COEF	(39322) /*0.6  in Q16 format */
#define AK09912_FUSE_COEF	(9830)	/*0.15	in Q16 format */
#define AK09913_FUSE_COEF	(9830)	/*0.15	in Q16 format */
#define AK09915_FUSE_COEF	(9830)	/*0.15	in Q16 format */
#define AK09916_FUSE_COEF	(9830)	/*0.15	in Q16 format */
#define AK09918_FUSE_COEF	(9830)	/*0.15	in Q16 format */
#define AK0991X_ADDR_CNTL1	0x30
#define AK0991X_ADDR_CNTL2	0x31
#define AK0991X_ADDR_CNTL3	0x32

/* Bit mask */
#define AK0991X_DRDY_BIT_MASK	 0x01
#define AK0991X_INV_BIT_MASK	 0x04
#define AK0991X_FIFO_EN_BIT_MASK 0x80

#define AK0991X_DEFAULT_NSF	0
#define AK0991X_DEFAULT_WM	1

/* AK09911 selftest threshold */
#define AK09911_TEST_LOLIM_X	(-30)
#define AK09911_TEST_HILIM_X	(30)
#define AK09911_TEST_LOLIM_Y	(-30)
#define AK09911_TEST_HILIM_Y	(30)
#define AK09911_TEST_LOLIM_Z	(-400)
#define AK09911_TEST_HILIM_Z	(-50)
/* AK09912 selftest threshold */
#define AK09912_TEST_LOLIM_X	(-200)
#define AK09912_TEST_HILIM_X	(200)
#define AK09912_TEST_LOLIM_Y	(-200)
#define AK09912_TEST_HILIM_Y	(200)
#define AK09912_TEST_LOLIM_Z	(-1600)
#define AK09912_TEST_HILIM_Z	(-400)
/* AK09913 selftest threshold */
#define AK09913_TEST_LOLIM_X	(-32767)
#define AK09913_TEST_HILIM_X	(32767)
#define AK09913_TEST_LOLIM_Y	(-32767)
#define AK09913_TEST_HILIM_Y	(32767)
#define AK09913_TEST_LOLIM_Z	(-32767)
#define AK09913_TEST_HILIM_Z	(32767)
/* AK09915 selftest threshold */
#define AK09915_TEST_LOLIM_X	(-200)
#define AK09915_TEST_HILIM_X	(200)
#define AK09915_TEST_LOLIM_Y	(-200)
#define AK09915_TEST_HILIM_Y	(200)
#define AK09915_TEST_LOLIM_Z	(-800)
#define AK09915_TEST_HILIM_Z	(-200)
/* AK09916 selftest threshold */
#define AK09916_TEST_LOLIM_X	(-200)
#define AK09916_TEST_HILIM_X	(200)
#define AK09916_TEST_LOLIM_Y	(-200)
#define AK09916_TEST_HILIM_Y	(200)
#define AK09916_TEST_LOLIM_Z	(-1000)
#define AK09916_TEST_HILIM_Z	(-200)
/* AK09918 selftest threshold (TBD) */
#define AK09918_TEST_LOLIM_X	(-200)
#define AK09918_TEST_HILIM_X	(200)
#define AK09918_TEST_LOLIM_Y	(-200)
#define AK09918_TEST_HILIM_Y	(200)
#define AK09918_TEST_LOLIM_Z	(-1000)
#define AK09918_TEST_HILIM_Z	(-200)

/* FIFO size */
#define AK09911_FIFO_SIZE	  (0)
#define AK09912_FIFO_SIZE	  (0)
#define AK09913_FIFO_SIZE	  (0)
#define AK09915_FIFO_SIZE	  (32)
#define AK09916_FIFO_SIZE	  (0)
#define AK09918_FIFO_SIZE	  (0)
#define AK0991X_FIFO_SIZE_MAX (32)

#define AK0991X_WAIT_TIME_MS	10
#define AK0991X_CNTL_DELAY_US	100
#define AK0991X_POR_DELAY_US	100

#define AK0991X_INFO_SIZE (AK0991X_WIA_SIZE + AK0991X_FUSE_SIZE)

/* macro functions */
#define AK0991X_NSF(NSF) (((NSF) & 0x03) << 5)
#define AK0991X_WM(WM)	 (((WM) - 1) & 0x1F)
#define IS_PDN(mode)	 (!(mode))
#define IS_SNG(mode)	 ((mode) == AK0991X_CNTL_SNG)
#define IS_TST(mode)	 ((mode) == AK0991X_CNTL_TEST)
#define IS_FUS(mode)	 ((mode) == AK0991X_CNTL_FUSE)

#define AK0991X_MAG_CHANNEL(axis, index) {	\
	.type = IIO_MAGN,\
	.modified = 1,\
	.channel2 = IIO_##axis,\
	.info_mask_separate = \
		BIT(IIO_CHAN_INFO_RAW) |\
		BIT(IIO_CHAN_INFO_SCALE),\
	.info_mask_shared_by_type = \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.scan_index = index,\
	.scan_type = {\
		.sign = 's',\
		.realbits = 32,\
		.storagebits = 32,\
		.shift = 0,\
	},\
}

#define AK0991X_STATUS_CHANNEL(index) {	\
	.type = IIO_MAGN,\
	.modified = 1,\
	.channel2 = IIO_NO_MOD,\
	.info_mask_separate = \
		BIT(IIO_CHAN_INFO_RAW),\
	.scan_index = index,\
	.scan_type = {\
		.sign = 'u',\
		.realbits = 16,\
		.storagebits = 32,\
	},\
}

enum AKM_DEVICES {
	AK09911,
	AK09912,
	AK09913,
	AK09915,
	AK09915D,
	AK09916,
	AK09916D,
	AK09918,
	SUPPORTED_DEVICES
};

struct ak0991x_part_info {
	/* a company id, i.e. WIA1 */
	u8	company_id;
	/* a device id, i.e. WIA2 */
	u8	device_id;
	/* part number of the device */
	char	*name;

	/* a list of frequencies which is supported by this device */
	u16	*freq_table;
	/* a list of register values correspond to a supported frequency */
	u8	*reg_table;
	/* the number of elements of above list */
	u8	num_freq;

	/* 0: the device does not have FUSEROM */
	/* +: the device has FUSEROM, the number shows amount of shift */
	u8	fuse_shift;
	/* coefficient of FUSE value */
	s32	fuse_coef;

	/* IRQ trigger direction. */
	unsigned long irqflag;

	/* 0: the device does not have NSF */
	/* 1: the device has NSF */
	u8	has_nsf;

	/* selfttest threshold */
	s16	test_lolim[NUM_OF_AXIS];
	s16	test_hilim[NUM_OF_AXIS];

	/* the size of fifo. */
	/*  0: device does not support fifo. */
	/*  +: device supports fifo. */
	u8 device_fifo_size;
};

struct ak0991x_data {
	struct device		*dev;
	struct iio_trigger	*trig;
	struct delayed_work	work;
	struct work_struct	flush_work;
	struct workqueue_struct *wq;

	const struct ak0991x_part_info *part_info;

	/* this value represents current operation mode.
	 * 0x00: power down mode
	 * 0x01: single measurement mode
	 * 0xXX: table
	 * 0x10: self-test mode
	 */
	atomic_t	mode;
	/* selftest result flag.
	 * -1: not done yet.
	 *  0: success.
	 *  +: the number of error.
	 */
	atomic_t	selftest;
	/* NSF flag.
	 *  0 or +: valid.
	 * -1: This device does not have NSF.
	 */
	atomic_t	cntl1;
	/* FIFO flush flag
	 * 0 : disable
	 * 1 : enable
	 */
	atomic_t flush_device_fifo;

	/* IRQ number.	0:not set,	+:set. */
	int		irq;
	/* a buffer to save WIA & FUSEROM value */
	u8		info[AK0991X_INFO_SIZE];
	/* scale factor for raw data */
	s32		raw_to_micro_q16[NUM_OF_AXIS];
	/* Axis conversion */
	u8		axis_order[NUM_OF_AXIS];
	u8		axis_sign[NUM_OF_AXIS];

	/* mutex */
	struct mutex buffer_mutex;
	struct mutex fifo_mutex;

	/* Data buffer*/
	/* 0:X, 1:Y, 2:Z, 3:ST1&2 */
	s32		buffer[4];

	/* current watermark (WM) setting for fifo operation. */
	/*  0: fifo is disabled. */
	/*  +: fifo is activated */
	u8 watermark;

	/* previous timestamp [nsec] */
	s64 prev_time_ns;

	const struct ak0991x_bus_ops *bops;

	struct ak0991x_platform_data *pdata;
};

/* supported frequency (0 means stop measurement) */
static u16 ak09911_freq_table[] = {0, 10, 20, 50, 100};
static u16 ak09912_freq_table[] = {0, 10, 20, 50, 100};
static u16 ak09913_freq_table[] = {0, 10, 20, 50, 100};
static u16 ak09915_freq_table[] = {0, 1, 10, 20, 50, 100, 200};
static u16 ak09916_freq_table[] = {0, 10, 20, 50, 100};
static u16 ak09918_freq_table[] = {0, 10, 20, 50, 100};

/* register value correspond to the freq_table */
static u8 ak09911_reg_table[] = {AK0991X_CNTL_PDN, 0x02, 0x04, 0x06, 0x08};
static u8 ak09912_reg_table[] = {AK0991X_CNTL_PDN, 0x02, 0x04, 0x06, 0x08};
static u8 ak09913_reg_table[] = {AK0991X_CNTL_PDN, 0x02, 0x04, 0x06, 0x08};
static u8 ak09915_reg_table[] = {AK0991X_CNTL_PDN, 0x0C, 0x02, 0x04, 0x06,
								 0x08, 0x0A};
static u8 ak09916_reg_table[] = {AK0991X_CNTL_PDN, 0x02, 0x04, 0x06, 0x08};
static u8 ak09918_reg_table[] = {AK0991X_CNTL_PDN, 0x02, 0x04, 0x06, 0x08};

/* channel spec */
static const struct iio_chan_spec ak0991x_channels[] = {
	AK0991X_MAG_CHANNEL(MOD_X, 0),
	AK0991X_MAG_CHANNEL(MOD_Y, 1),
	AK0991X_MAG_CHANNEL(MOD_Z, 2),
	AK0991X_STATUS_CHANNEL(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

/* device specific information */
static const struct ak0991x_part_info ak0991x_part_info_array[] = {
	[AK09911] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09911_DEVICE_ID,
		.name		= "AK09911",
		.fuse_shift	= 7,
		.has_nsf	= 0,
		.freq_table	= ak09911_freq_table,
		.reg_table	= ak09911_reg_table,
		.num_freq	= ARRAY_SIZE(ak09911_freq_table),
		.fuse_coef	= AK09911_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_NONE,
		.test_lolim = {
			AK09911_TEST_LOLIM_X,
			AK09911_TEST_LOLIM_Y,
			AK09911_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09911_TEST_HILIM_X,
			AK09911_TEST_HILIM_Y,
			AK09911_TEST_HILIM_Z
		},
		.device_fifo_size = AK09911_FIFO_SIZE,
	},
	[AK09912] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09912_DEVICE_ID,
		.name		= "AK09912",
		.fuse_shift	= 8,
		.has_nsf	= 1,
		.freq_table	= ak09912_freq_table,
		.reg_table	= ak09912_reg_table,
		.num_freq	= ARRAY_SIZE(ak09912_freq_table),
		.fuse_coef	= AK09912_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_RISING,
		.test_lolim = {
			AK09912_TEST_LOLIM_X,
			AK09912_TEST_LOLIM_Y,
			AK09912_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09912_TEST_HILIM_X,
			AK09912_TEST_HILIM_Y,
			AK09912_TEST_HILIM_Z
		},
		.device_fifo_size = AK09912_FIFO_SIZE,
	},
	[AK09913] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09913_DEVICE_ID,
		.name		= "AK09913",
		.fuse_shift	= 0,
		.has_nsf	= 0,
		.freq_table	= ak09913_freq_table,
		.reg_table	= ak09913_reg_table,
		.num_freq	= ARRAY_SIZE(ak09913_freq_table),
		.fuse_coef	= AK09913_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_NONE,
		.test_lolim = {
			AK09913_TEST_LOLIM_X,
			AK09913_TEST_LOLIM_Y,
			AK09913_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09913_TEST_HILIM_X,
			AK09913_TEST_HILIM_Y,
			AK09913_TEST_HILIM_Z
		},
		.device_fifo_size = AK09913_FIFO_SIZE,
	},
	[AK09915] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09915_DEVICE_ID,
		.name		= "AK09915",
		.fuse_shift	= 0,
		.has_nsf	= 0,
		.freq_table	= ak09915_freq_table,
		.reg_table	= ak09915_reg_table,
		.num_freq	= ARRAY_SIZE(ak09915_freq_table),
		.fuse_coef	= AK09915_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_RISING,
		.test_lolim = {
			AK09915_TEST_LOLIM_X,
			AK09915_TEST_LOLIM_Y,
			AK09915_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09915_TEST_HILIM_X,
			AK09915_TEST_HILIM_Y,
			AK09915_TEST_HILIM_Z
		},
		.device_fifo_size = AK09915_FIFO_SIZE,
	},
	[AK09915D] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09915_DEVICE_ID,
		.name		= "AK09915D",
		.fuse_shift	= 0,
		.has_nsf	= 0,
		.freq_table	= ak09915_freq_table,
		.reg_table	= ak09915_reg_table,
		.num_freq	= ARRAY_SIZE(ak09915_freq_table),
		.fuse_coef	= AK09915_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_FALLING,
		.test_lolim = {
			AK09915_TEST_LOLIM_X,
			AK09915_TEST_LOLIM_Y,
			AK09915_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09915_TEST_HILIM_X,
			AK09915_TEST_HILIM_Y,
			AK09915_TEST_HILIM_Z
		},
		.device_fifo_size = AK09915_FIFO_SIZE,
	},
	[AK09916] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09916_DEVICE_ID,
		.name		= "AK09916",
		.fuse_shift	= 0,
		.has_nsf	= 0,
		.freq_table	= ak09916_freq_table,
		.reg_table	= ak09916_reg_table,
		.num_freq	= ARRAY_SIZE(ak09916_freq_table),
		.fuse_coef	= AK09916_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_NONE,
		.test_lolim = {
			AK09916_TEST_LOLIM_X,
			AK09916_TEST_LOLIM_Y,
			AK09916_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09916_TEST_HILIM_X,
			AK09916_TEST_HILIM_Y,
			AK09916_TEST_HILIM_Z
		},
		.device_fifo_size = AK09916_FIFO_SIZE,
	},
	[AK09916D] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09916D_DEVICE_ID,
		.name		= "AK09916D",
		.fuse_shift	= 0,
		.has_nsf	= 0,
		.freq_table	= ak09916_freq_table,
		.reg_table	= ak09916_reg_table,
		.num_freq	= ARRAY_SIZE(ak09916_freq_table),
		.fuse_coef	= AK09916_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_FALLING,
		.test_lolim = {
			AK09916_TEST_LOLIM_X,
			AK09916_TEST_LOLIM_Y,
			AK09916_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09916_TEST_HILIM_X,
			AK09916_TEST_HILIM_Y,
			AK09916_TEST_HILIM_Z
		},
		.device_fifo_size = AK09916_FIFO_SIZE,
	},
	[AK09918] = {
		.company_id	= AK0991X_COMPANY_ID,
		.device_id	= AK09918_DEVICE_ID,
		.name		= "AK09918",
		.fuse_shift	= 0,
		.has_nsf	= 0,
		.freq_table	= ak09918_freq_table,
		.reg_table	= ak09918_reg_table,
		.num_freq	= ARRAY_SIZE(ak09918_freq_table),
		.fuse_coef	= AK09918_FUSE_COEF,
		.irqflag = IRQF_TRIGGER_NONE,
		.test_lolim = {
			AK09918_TEST_LOLIM_X,
			AK09918_TEST_LOLIM_Y,
			AK09918_TEST_LOLIM_Z
		},
		.test_hilim = {
			AK09918_TEST_HILIM_X,
			AK09918_TEST_HILIM_Y,
			AK09918_TEST_HILIM_Z
		},
		.device_fifo_size = AK09918_FIFO_SIZE,
	},
};

static int akecs_check_measure_mode(struct ak0991x_data *akm,
		unsigned char mode)
{
	u8 i; /* the type of num_freq */

	if (AK0991X_CNTL_SNG == mode)
		/* single measurement mode */
		return 0;

	if (AK0991X_CNTL_TEST == mode)
		/* selft test mode */
		return 0;

	for (i = 0; i < akm->part_info->num_freq; i++) {
		/* continuous measurement mode */
		if (akm->part_info->reg_table[i] == mode)
			return 0;
	}

	return -EINVAL;
}

static int akecs_setmode_measure(struct ak0991x_data *akm,
		unsigned char mode)
{
	unsigned char buffer[3];
	unsigned char cntl1, cntl2;
	int err = 0;
	struct iio_dev *indio_dev = (struct iio_dev *)akm;

	dev_info(akm->dev, "%s called: mode=%d", __func__, mode);

	/* check mode */
	err = akecs_check_measure_mode(akm, mode);
	if (err)
		return err;

	/* set mode */
	if (atomic_cmpxchg(&akm->mode,
				AK0991X_CNTL_PDN, mode)
			!= AK0991X_CNTL_PDN) {
		dev_err(akm->dev, "%s: device is busy", __func__);
		return -EBUSY;
	}

	/* set NSF bit if it is available */
	if (akm->part_info->has_nsf)
		cntl1 = atomic_read(&akm->cntl1);
	else
		cntl1 = 0;

	cntl2 = mode;

	if ((akm->part_info->device_fifo_size > 0) && (akm->watermark > 0)) {
		cntl1 |= AK0991X_WM(akm->watermark);
		cntl2 |= AK0991X_FIFO_EN_BIT_MASK;
	}

	/* set previous timestamp */
	akm->prev_time_ns = iio_get_time_ns(indio_dev);

	/* set write buffer */
	buffer[0] = AK0991X_ADDR_CNTL1;
	buffer[1] = cntl1;
	buffer[2] = cntl2;
	err = akm->bops->txdata(akm->dev, buffer, 3);

	if (err) {
		/* Error recovery */
		atomic_set(&akm->mode, AK0991X_CNTL_PDN);
		return err;
	}

	return err;
}

static int akecs_setmode_measure_with_timer(
		struct ak0991x_data *akm, unsigned char mode)
{
	int err = 0;

	/* when irq is set, don't use timer */
	if (akm->irq)
		return -EINVAL;

	/* timer is used with single/test mode only */
	if ((AK0991X_CNTL_SNG != mode) && (AK0991X_CNTL_TEST == mode))
		return -EINVAL;

	err = akecs_setmode_measure(akm, mode);
	if (err)
		return err;

	schedule_delayed_work(
			&akm->work,
			msecs_to_jiffies(AK0991X_WAIT_TIME_MS));

	return 0;
}

/* If power down mode is set while DRDY is HIGH,
  (i.e. before work que function read out the measurement data)
  DRDY bit is reset to 0, then work que function will fail.*/
static int akecs_setmode_powerdown(struct ak0991x_data *akm)
{
	unsigned char buffer[2];
	int err;

	dev_dbg(akm->dev, "%s called", __func__);

	if ((atomic_read(&akm->mode) != AK0991X_CNTL_PDN) && (!akm->irq))
		cancel_delayed_work_sync(&akm->work);

	buffer[0] = AK0991X_ADDR_CNTL2;
	buffer[1] = AK0991X_CNTL_PDN;

	err = akm->bops->txdata(akm->dev, buffer, sizeof(buffer));
	if (err)
		dev_err(akm->dev, "%s: Can not set to powerdown.",
			__func__);

	udelay(AK0991X_CNTL_DELAY_US);

	atomic_set(&akm->mode, AK0991X_CNTL_PDN);
	return err;
}

#if 0
/* Reset function is also same behavior as powerdown mode */
static int akecs_setmode_reset(struct ak0991x_data *akm)
{
	unsigned char buffer[2];
	int err;

	dev_dbg(akm->dev, "%s called", __func__);

	if (0 <= atomic_read(&akm->interval)) {
		cancel_delayed_work_sync(&akm->work);
		atomic_set(&akm->interval, -1);
	}

	buffer[0] = AK0991X_ADDR_CNTL3;
	buffer[1] = AK0991X_CNTL_RESET;

	err = akm->bops->txdata(akm->dev, buffer, sizeof(buffer));
	if (err)
		dev_err(akm->dev, "%s: Can not reset the device.",
			__func__);

	/* Wait for bootup the device */
	udelay(AK0991X_POR_DELAY_US);

	atomic_set(&akm->mode, AK0991X_CNTL_PDN);
	atomic_set(&akm->interval, -1);
	return err;
}
#endif

/* No busy check */
/* This function is called only once in initialization. */
static int akecs_checkdevice(struct ak0991x_data *akm)
{
	unsigned char buffer[AK0991X_INFO_SIZE];
	unsigned char *bufp = buffer;
	int err;
	s32 coef;
	u8 shift;

	dev_dbg(akm->dev, "%s called", __func__);

	/* 1. get WIA registers value */
	bufp[0] = AK0991X_ADDR_WIA;
	err = akm->bops->rxdata(akm->dev, bufp, AK0991X_WIA_SIZE);
	if (err) {
		dev_err(akm->dev, "%s: Can not read WIA.", __func__);
		return err;
	}

	if (AK0991X_COMPANY_ID != buffer[0]) {
		dev_err(akm->dev, "%s: The device is not AKM.", __func__);
		return -ENXIO;
	}

	switch (buffer[1]) {
	case AK09911_DEVICE_ID:
		/* AK09911: copy part info */
		akm->part_info = &ak0991x_part_info_array[AK09911];
		break;
	case AK09912_DEVICE_ID:
		/* AK09912: set default nsf, copy part info */
		atomic_set(&akm->cntl1, AK0991X_NSF(AK0991X_DEFAULT_NSF));
		akm->part_info = &ak0991x_part_info_array[AK09912];
		break;
	case AK09913_DEVICE_ID:
		/* AK09913: copy part info */
		akm->part_info = &ak0991x_part_info_array[AK09913];
		break;
	case AK09915_DEVICE_ID:
		/* AK09915: set default WM, copy part info */
		akm->watermark = AK0991X_DEFAULT_WM;
		atomic_set(&akm->flush_device_fifo, 0);
		/* INFO2 value is different between AK09915C and AK09915D */
		if (buffer[3] == AK09915D_INFO_ID)
			akm->part_info = &ak0991x_part_info_array[AK09915D];
		else
			akm->part_info = &ak0991x_part_info_array[AK09915];
		break;
	case AK09916_DEVICE_ID:
		/* AK09916: copy part info */
		akm->part_info = &ak0991x_part_info_array[AK09916];
		break;
	case AK09916D_DEVICE_ID:
		/* AK09916: copy part info */
		akm->part_info = &ak0991x_part_info_array[AK09916D];
		break;
	case AK09918_DEVICE_ID:
		/* AK09918: copy part info */
		akm->part_info = &ak0991x_part_info_array[AK09918];
		break;
	default:
		/* Other: error */
		dev_err(akm->dev, "%s: This device is not supported.",
			__func__);
		return -ENXIO;
	}

	/* 2. get FUSEROM value */
	coef = akm->part_info->fuse_coef;
	shift = akm->part_info->fuse_shift;

	/* if device does not have FUSEROM, we cannot access the ROM area.
	 * So we have to use dummy/fixed value instead.
	 */
	if (shift == 0) {
		akm->raw_to_micro_q16[0] = coef;
		akm->raw_to_micro_q16[1] = coef;
		akm->raw_to_micro_q16[2] = coef;
		/* copy read data to 'info' buffer */
		memcpy(akm->info, buffer, AK0991X_WIA_SIZE);
		/* put dummy FUSE value */
		akm->info[AK0991X_WIA_SIZE + 0] = AK0991X_FUSE_DUMMY;
		akm->info[AK0991X_WIA_SIZE + 1] = AK0991X_FUSE_DUMMY;
		akm->info[AK0991X_WIA_SIZE + 2] = AK0991X_FUSE_DUMMY;
		return 0;
	}

	bufp = &buffer[AK0991X_WIA_SIZE];
	bufp[0] = AK0991X_ADDR_CNTL2;
	bufp[1] = AK0991X_CNTL_FUSE;
	err = akm->bops->txdata(akm->dev, bufp, 2);
	if (err) {
		dev_err(akm->dev, "%s: Can not set to fuse access mode.",
			__func__);
		return err;
	}

	bufp[0] = AK0991X_ADDR_FUSE;
	err = akm->bops->rxdata(akm->dev, bufp, AK0991X_FUSE_SIZE);
	if (err) {
		dev_err(akm->dev, "%s: Can not read the FUSEROM.", __func__);
		return err;
	}

	/* calculate conversion coefficiency, which converts from raw to
	 * microtesla in q16 format */
	akm->raw_to_micro_q16[0] = ((bufp[0] + 128) * coef) >> shift;
	akm->raw_to_micro_q16[1] = ((bufp[1] + 128) * coef) >> shift;
	akm->raw_to_micro_q16[2] = ((bufp[2] + 128) * coef) >> shift;
	/* copy read data to 'info' buffer */
	memcpy(akm->info, buffer, AK0991X_INFO_SIZE);

	/* Finally, set to powerdown mode */
	return akecs_setmode_powerdown(akm);
}

static void selftest_judgement(struct ak0991x_data *akm, const s32 data[])
{
	int i;
	int result;
	s32 tmp32;

	result = 0;
	for (i = 0; i < NUM_OF_AXIS; i++) {
		/* data is in Q16 format. */
		tmp32 = data[i] >> 16;
		if ((tmp32 < akm->part_info->test_lolim[i]) ||
				(akm->part_info->test_hilim[i] < tmp32))
			result |= (1 << i);
	}
	atomic_set(&akm->selftest, result);
}

static int ak0991x_set_samp_freq(struct ak0991x_data *akm, int freq)
{
	u16 idx;
	u8 i;
	unsigned char mode;

	dev_info(akm->dev, "%s called (freq=%d).", __func__, freq);

	/* avoid too large value */
	if (freq > akm->part_info->freq_table[akm->part_info->num_freq - 1])
		return -EINVAL;

	idx = 0;
	for (i = 0; i < akm->part_info->num_freq; i++) {
		idx = i;
		if (freq <= akm->part_info->freq_table[i])
			break;
	}

	mode = akm->part_info->reg_table[idx];

	if (mode == AK0991X_CNTL_PDN)
		return akecs_setmode_powerdown(akm);
	else
		return akecs_setmode_measure(akm, mode);
}

static void ak0991x_get_samp_freq(struct ak0991x_data *akm, int *freq)
{
	int mode = atomic_read(&akm->mode);
	u8 i;

	/* -1 means unknown. e.g. selftest mode etc. */
	*freq = -1;

	for (i = 0; i < akm->part_info->num_freq; i++)
		if (mode == akm->part_info->reg_table[i])
			*freq = akm->part_info->freq_table[i];
}

static void ak0991x_convert_3axis_raw_to_micro_q16(
	struct ak0991x_data *akm, u8 *src, s32 *dst)
{
	int i;
	s16 tmp16;

	if (akm == NULL || src == NULL || dst == NULL)
		return;

	for (i = 0; i < 3; i++) {
		tmp16 = (s16)(((u16)src[i * 2 + 1] << 8) | (u16)src[i * 2]);
		dst[i] = (s32)tmp16 * akm->raw_to_micro_q16[i];
	}
}


static int ak0991x_send_event(struct iio_dev *indio_dev, unsigned char *data,
						s64 timestamp)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);
	uint8_t reg_st1, reg_st2;
	/* Before axis swap, in Q16. */
	s32 tmp32[3];
	/* data(32bit) * 3-axis + status(32bit) + timestamp(64bit) */
	u8 event[sizeof(s32) * 4 + sizeof(s64)];
	int bit, i;

	/* convert data to 32-bit microtesla unit in q16 format */
	ak0991x_convert_3axis_raw_to_micro_q16(akm, &data[1], tmp32);

	dev_vdbg(akm->dev, "hval = %d,%d,%d timestamp = %lld",
			 tmp32[0], tmp32[1], tmp32[2], timestamp);

	/* Axis conversion */
	mutex_lock(&akm->buffer_mutex);
	for (i = 0; i < 3; i++) {
		akm->buffer[i] = tmp32[akm->axis_order[i]];
		if (akm->axis_sign[i])
			akm->buffer[i] *= -1;
	}

	/* combine ST1 & ST2 value */
	reg_st1 = data[0];
	reg_st2 = data[AK0991X_DATA_SIZE - 1];
	akm->buffer[3] = (reg_st1 << 8) | reg_st2;

	memset(event, 0, sizeof(event));

	i = 0;
	for_each_set_bit(bit, indio_dev->buffer->scan_mask,
					 indio_dev->masklength) {
		((s32 *)event)[i++] = akm->buffer[bit];
	}
	/* iio_push_to_buffers_with_timestamp(indio_dev, iobuf, time_ns); */
	{
		if (indio_dev->scan_timestamp) {
			size_t ts_offset =
				indio_dev->scan_bytes / sizeof(int64_t) - 1;
			((int64_t *)event)[ts_offset] = timestamp;
		}
		dev_vdbg(akm->dev, "%s: iio_push_to_buffers called", __func__);
		iio_push_to_buffers(indio_dev, event);
	}
	mutex_unlock(&akm->buffer_mutex);

	return 0;
}

static int ak0991x_batch_read(struct iio_dev *indio_dev, int mode, s64 now)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);
	unsigned char data[AK0991X_DATA_SIZE];
	int err;

	dev_vdbg(akm->dev, "%s called", __func__);

	/* read data */
	data[0] = AK0991X_ADDR_ST1;
	err = akm->bops->rxdata(akm->dev, data, AK0991X_DATA_SIZE);
	if (err)
		return err;

	/* If operation is self-test mode, don't report event */
	if (mode == AK0991X_CNTL_TEST) {
		/* Before axis swap, in Q16. */
		s32 tmp32[3];
		/* convert data to 32-bit microtesla unit in q16 format */
		ak0991x_convert_3axis_raw_to_micro_q16(akm, &data[1], tmp32);

		dev_vdbg(akm->dev, "hval = %d,%d,%d",
				tmp32[0], tmp32[1], tmp32[2]);

		/* the judgement must be done before axis converted */
		selftest_judgement(akm, tmp32);
		return 0;
	}

	/* send event */
	ak0991x_send_event(indio_dev, data, now);

	/* update previous timestamp */
	akm->prev_time_ns = now;

	return 0;
}

static int ak0991x_batch_fifo_read(struct iio_dev *indio_dev, int mode, s64 now)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);
	unsigned char  data[AK0991X_DATA_SIZE * AK0991X_FIFO_SIZE_MAX];
	unsigned char *buffer;
	uint8_t	 reg_st2;
	uint16_t samples;
	int err;
	int i, freq;
	s64 interval, time_ns;
	int flush_device_fifo;

	dev_vdbg(akm->dev, "%s called", __func__);

	for (i = 0; i < akm->part_info->device_fifo_size; i++) {
		buffer = &data[AK0991X_DATA_SIZE * i];

		/* read data */
		buffer[0] = AK0991X_ADDR_ST1;
		err = akm->bops->rxdata(akm->dev, buffer, AK0991X_DATA_SIZE);
		if (err)
			return err;

		reg_st2 = buffer[AK0991X_DATA_SIZE - 1];

		/* Check if there's valid data in FIFO */
		if (reg_st2 & AK0991X_INV_BIT_MASK)
			break;
	}

	samples = i;

	/* If read data size is 0, do nothing */
	if (samples == 0)
		return 0;

	/* If operation is self-test mode, don't report event */
	if (mode == AK0991X_CNTL_TEST) {
		/* Before axis swap, in Q16. */
		s32 tmp32[3];
		/* convert data to 32-bit microtesla unit in q16 format */
		ak0991x_convert_3axis_raw_to_micro_q16(akm, &data[1], tmp32);

		dev_vdbg(akm->dev, "hval = %d,%d,%d",
				tmp32[0], tmp32[1], tmp32[2]);

		/* the judgement must be done before axis converted */
		selftest_judgement(akm, tmp32);
		return 0;
	}

	flush_device_fifo = atomic_read(&akm->flush_device_fifo);

	/* calculate data sampling interval */
	if (flush_device_fifo) {
		ak0991x_get_samp_freq(akm, &freq);
		interval = 1000000000 / freq;
	} else {
		interval = (now - akm->prev_time_ns) / samples;
	}
	dev_vdbg(akm->dev, "interval=%lld", interval);

	for (i = 0; i < samples; i++) {
		if (flush_device_fifo)
			time_ns = akm->prev_time_ns + (i + 1) * interval;
		else
			time_ns = now - interval * (samples - (i + 1));

		dev_vdbg(akm->dev, "send event=%d/%d", i + 1, samples);

		/* send event */
		ak0991x_send_event(indio_dev,
			&data[AK0991X_DATA_SIZE * i], time_ns);
	}

	return samples;
}

static int ak0991x_send_flush(struct iio_dev *indio_dev)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);
	unsigned char data[AK0991X_DATA_SIZE * AK0991X_FIFO_SIZE_MAX];
	s64 now = iio_get_time_ns(indio_dev);

	memset(data, 0, sizeof(data));

	/* set INV bit flag in ST2 data */
	data[AK0991X_DATA_SIZE - 1] |= AK0991X_INV_BIT_MASK;

	dev_vdbg(akm->dev, "send flush event");

	/* send event */
	return ak0991x_send_event(indio_dev, data, now);
}

static int read_and_event(struct iio_dev *indio_dev)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);
	s64 now = iio_get_time_ns(indio_dev);
	int mode;
	int samples;
	int err;

	dev_vdbg(akm->dev, "%s called", __func__);

	mode = atomic_read(&akm->mode);

	/* If operation is already canceled, don't report values */
	if (mode == AK0991X_CNTL_PDN) {
		dev_info(akm->dev, "%s: Operation canceled.", __func__);
		return 0;
	}

	/* If one shot operation, set to powerdown state */
	if (IS_SNG(mode) || IS_TST(mode))
		/* Device automatically goes to PDN. */
		atomic_set(&akm->mode, AK0991X_CNTL_PDN);

	/* If device has FIFO, the DD reads the all data of FIFO.
	 * Otherwise it reads 1 data. */
	if (akm->part_info->device_fifo_size > 0) {
		mutex_lock(&akm->fifo_mutex);
		samples = ak0991x_batch_fifo_read(indio_dev, mode, now);
		mutex_unlock(&akm->fifo_mutex);
		err = samples < 0 ? samples : 0;
	} else {
		err = ak0991x_batch_read(indio_dev, mode, now);
	}
	/* update previous timestamp */
	akm->prev_time_ns = now;

	return err;
}

static int flush_fifo(struct iio_dev *indio_dev)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);
	s64 now = iio_get_time_ns(indio_dev);
	int freq;
	int mode;
	int samples;

	dev_vdbg(akm->dev, "%s called", __func__);

	if (akm->part_info->device_fifo_size == 0) {
		dev_err(akm->dev, "%s: this device has no fifo.", __func__);
		return -EIO;
	}

	mode = atomic_read(&akm->mode);

	/* If operation is already canceled, don't report values */
	if (!atomic_read(&akm->flush_device_fifo) || mode == AK0991X_CNTL_PDN) {
		dev_info(akm->dev, "%s: Operation canceled.", __func__);
		return 0;
	}

	/* If one shot operation, set to powerdown state */
	if (IS_SNG(mode) || IS_TST(mode))
		/* Device automatically goes to PDN. */
		atomic_set(&akm->mode, AK0991X_CNTL_PDN);


	/* read the all data of FIFO */
	mutex_lock(&akm->fifo_mutex);
	samples = ak0991x_batch_fifo_read(indio_dev, mode, now);
	mutex_unlock(&akm->fifo_mutex);

	/* get sampling freq. */
	ak0991x_get_samp_freq(akm, &freq);

	/* update previous timestamp */
	akm->prev_time_ns += (s64)samples * 1000000000 / freq;

	dev_vdbg(akm->dev, "%s samples: %d", __func__, samples);

	/* send flush event */
	ak0991x_send_flush(indio_dev);

	return 0;
}

/*****************************************************************************/
static void ak0991x_tm_handler(struct work_struct *work)
{
	struct ak0991x_data *akm =
		container_of(work, struct ak0991x_data, work.work);
	struct iio_dev *indio_dev = iio_priv_to_dev(akm);

	dev_vdbg(akm->dev, "%s called", __func__);

	read_and_event(indio_dev);
}

/*****************************************************************************/
static irqreturn_t ak0991x_trigger_handler(int irq, void *handle)
{
	struct iio_poll_func *pf = handle;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ak0991x_data *akm = iio_priv(indio_dev);
	unsigned char reg_st1;
	int err;

	dev_vdbg(akm->dev, "%s called", __func__);

	/* Read ST1 first. Then check ST1 whether measurement is done or not */
	reg_st1 = AK0991X_ADDR_ST1;
	err = akm->bops->rxdata(akm->dev, &reg_st1, 1);
	if (err)
		return err;

	/* Check ST1 bit */
	if ((reg_st1 & AK0991X_DRDY_BIT_MASK) == 0) {
		dev_info(akm->dev, "%s: Operation canceled.", __func__);
		return IRQ_HANDLED;
	}

	read_and_event(indio_dev);

	if (indio_dev->trig)
		iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ak0991x_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ak0991x_data *akm = iio_priv(indio_dev);

	dev_dbg(akm->dev, "%s called. st=%s", __func__,
			(state ? "true" : "false"));

	return 0;
}

/*****************************************************************************/
static void ak0991x_flush_handler(struct work_struct *work)
{
	struct ak0991x_data *akm =
		container_of(work, struct ak0991x_data, flush_work);
	struct iio_dev *indio_dev = iio_priv_to_dev(akm);

	dev_vdbg(akm->dev, "%s called", __func__);

	flush_fifo(indio_dev);

	atomic_set(&akm->flush_device_fifo, 0);
}

/*
static int ak0991x_try_reenable(struct iio_trigger *trig)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ak0991x_data *akm = iio_priv(indio_dev);

	dev_dbg(akm->dev, "%s called", __func__);

	return 0;
}
*/

static const struct iio_trigger_ops ak0991x_trigger_ops = {
	.set_trigger_state = ak0991x_set_trigger_state,
	.owner = THIS_MODULE,
};

/*****************************************************************************
 *
 * SysFS attribute functions
 *
 * directory : /sys/bus/iio/devices/iio:deviceX
 * files :
 *  - nsf        [rw] : set digital filter (from 0 to 3)
 *  - selftest   [rw] : device's self test mode
 *  - reset      [w]  : soft reset
 *  - info       [r]  : get device part no. and ASA value.
 *
 * debug mode only:
 *  - axismap    [w]  : get axis map.
 *  - debug      [r]  : get driver status.
 *
 * notice:
 *   LF code is always included at the last of the string.
 *
 */

static int print_array(
	char **buf, ssize_t *buf_len,
	void *array, ssize_t array_len, u8 element_size)
{
	char *pbuf = *buf;
	u8 *par = (u8 *)array;
	s32 left = *buf_len;
	ssize_t n;
	int i;

	for (i = 0; i < array_len; i++) {
		switch (element_size) {
		case 1:
			n = scnprintf(pbuf, left, "%d ", *((u8 *)par));
			break;
		case 2:
			n = scnprintf(pbuf, left, "%d ", *((s16 *)par));
			break;
		case 4:
			n = scnprintf(pbuf, left, "%d ", *((s32 *)par));
			break;
		default:
			return -EINVAL;
		}
		/* print function failed */
		if (n == 0)
			return -EINVAL;

		pbuf += n;
		left -= n;
		par += element_size;

		/* when this loop finished, 'LF + NULL' will be put */
		/* to the buffer 2 or more length must be left at least */
		if (left < 2)
			return -EINVAL;
	}

	/* put LF at the last of string */
	n = scnprintf(pbuf, left, "\n");
	pbuf += n;
	left -= n;

	/* total length of written data */
	n = *buf_len - left;
	/* the pointer to the last of string */
	*buf = pbuf;
	/* the length of remained buffer */
	*buf_len = left;

	return n;
}

/*********** nsf ***********/
static ssize_t attr_nsf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	u8 read_cntl = AK0991X_ADDR_CNTL1;
	int ret;

	dev_dbg(dev, "%s called", __func__);

	if (!akm->part_info->has_nsf)
		return -ENXIO;

	ret = akm->bops->rxdata(akm->dev, &read_cntl, 1);
	if (ret < 0)
		return ret;

	read_cntl = ((read_cntl >> 5) & 0x03);
	return snprintf(buf, 5, "%d\n", read_cntl);
}

static ssize_t attr_nsf_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	unsigned char buffer[2];
	long nsf;
	int ret;

	dev_dbg(dev, "%s called: '%s'(%zu)", __func__, buf, count);

	if (!akm->part_info->has_nsf)
		return -ENXIO;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (kstrtol(buf, 10, &nsf))
		return -EINVAL;

	if ((nsf < 0) || (3 < nsf))
		return -EINVAL;

	buffer[0] = AK0991X_ADDR_CNTL1;
	buffer[1] = AK0991X_NSF(nsf);

	ret = akm->bops->txdata(akm->dev, buffer, 2);

	if (ret < 0)
		return ret;

	return count;
}

static IIO_DEVICE_ATTR(nsf,
		S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
		attr_nsf_show,
		attr_nsf_store,
		0);

/*********** watermark ***********/
static ssize_t attr_watermark_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	u8 read_cntl = AK0991X_ADDR_CNTL1;
	u8 wm;
	int ret;

	dev_dbg(dev, "%s called", __func__);

	if (akm->part_info->device_fifo_size == 0)
		return -ENXIO;

	ret = akm->bops->rxdata(akm->dev, &read_cntl, 1);
	if (ret < 0)
		return ret;

	wm = (read_cntl & 0x1f) + 1;

	return snprintf(buf, 5, "%d\n", wm);
}

static ssize_t attr_watermark_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	long fifo_size, wm;

	dev_dbg(dev, "%s called: '%s'(%zu)", __func__, buf, count);

	fifo_size = akm->part_info->device_fifo_size;

	if (fifo_size == 0)
		return -ENXIO;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (kstrtol(buf, 10, &wm))
		return -EINVAL;

	if ((wm < 1) || (fifo_size < wm))
		return -EINVAL;

	akm->watermark = (u8)wm;

	return count;
}

static IIO_DEVICE_ATTR(watermark,
		S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
		attr_watermark_show,
		attr_watermark_store,
		0);

/*********** flush ***********/
static ssize_t attr_flush_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	int flush_device_fifo;

	dev_dbg(dev, "%s called", __func__);

	if (akm->part_info->device_fifo_size == 0)
		return -ENXIO;

	flush_device_fifo = atomic_read(&akm->flush_device_fifo);

	return snprintf(buf, 7, "%s\n",
					flush_device_fifo ? "true" : "false");
}

static ssize_t attr_flush_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));

	dev_dbg(dev, "%s called: '%s'(%zu)", __func__, buf, count);

	if (akm->part_info->device_fifo_size == 0)
		return -ENXIO;

	if (atomic_cmpxchg(&akm->flush_device_fifo, 0, 1) != 0)
		return -EBUSY;

	queue_work(akm->wq, &akm->flush_work);

	return count;
}

static IIO_DEVICE_ATTR(flush,
		S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
		attr_flush_show,
		attr_flush_store,
		0);

/*********** selftest ***********/
static ssize_t attr_selftest_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));

	dev_dbg(dev, "%s called", __func__);

	return scnprintf(buf, PAGE_SIZE, "%d", atomic_read(&akm->selftest));
}

static ssize_t attr_selftest_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));

	dev_dbg(dev, "%s called: '%s'(%zu)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	return akecs_setmode_measure_with_timer(akm, AK0991X_CNTL_TEST);
}

static IIO_DEVICE_ATTR(selftest,
		S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
		attr_selftest_show,
		attr_selftest_store,
		0);

/*********** reset ***********/
static ssize_t attr_reset_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	/* struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev)); */

	/* TODO: hard reset will be implemented? */

	dev_dbg(dev, "%s called: '%s'(%zu)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	/* soft reset */
	/* TODO: implement reset function */

	return count;
}

static IIO_DEVICE_ATTR(reset,
		S_IWUSR | S_IWGRP,
		NULL,
		attr_reset_store,
		0);

/*********** info ***********/
static ssize_t attr_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	ssize_t len;
	u8 cp_info[AK0991X_INFO_SIZE + NUM_OF_AXIS + NUM_OF_AXIS];
	u16 n;

	dev_dbg(dev, "%s called", __func__);

	/* copy axis order & info here */
	n = 0;
	memcpy(&cp_info[n], akm->info, AK0991X_INFO_SIZE);
	n += AK0991X_INFO_SIZE;
	memcpy(&cp_info[n], akm->axis_order, NUM_OF_AXIS);
	n += NUM_OF_AXIS;
	memcpy(&cp_info[n], akm->axis_sign, NUM_OF_AXIS);
	n += NUM_OF_AXIS;

	len = PAGE_SIZE;
	return print_array(&buf, &len, cp_info, n, sizeof(u8));
}

static IIO_DEVICE_ATTR(info,
		S_IRUSR | S_IRGRP,
		attr_info_show,
		NULL,
		0);

#ifdef DEBUG
/*********** for debug purpose ***********/
/* Format: order[0],order[1],order[2],sign[0],sign[1],sign[2] */
/* 'order' should be one of 0,1,2. sign should be 0 or 1. */
static ssize_t attr_axismap_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	char cp_buffer[2];
	unsigned char tmp_order[3];
	unsigned char tmp_sign[3];
	long val;
	int i;

	dev_dbg(dev, "%s called: '%s'(%zu)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	/* this function access buf[10] */
	if (11 > count)
		return -EINVAL;

	/* initialize buffer */
	cp_buffer[0] = cp_buffer[1] = '\0';

	for (i = 0; i < 3; i++) {
		/* read axis order */
		cp_buffer[0] = buf[i * 2];
		if (kstrtol(cp_buffer, 10, &val))
			return -EINVAL;
		if (0 > val || 2 < val)
			return -EINVAL;
		tmp_order[i] = val;

		/* read axis sign */
		cp_buffer[0] = buf[i * 2 + 6];
		if (kstrtol(cp_buffer, 10, &val))
			return -EINVAL;
		if (0 > val || 1 < val)
			return -EINVAL;
		tmp_sign[i] = val;
	}
	/* Copy */
	for (i = 0; i < 3; i++) {
		akm->axis_order[i] = tmp_order[i];
		akm->axis_sign[i] = tmp_sign[i];
	}

	return count;
}

static IIO_DEVICE_ATTR(axismap,
		S_IWUSR | S_IWGRP,
		NULL,
		attr_axismap_store,
		0);

static ssize_t attr_debug_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = iio_priv(dev_to_iio_dev(dev));
	char *curbuf;
	int curlen;
	int wrlen;
	char tmpbuf[32];
	char *ptmpbuf;
	ssize_t len;

	dev_dbg(dev, "%s called", __func__);

	/* set initial value */
	curbuf = buf;
	curlen = PAGE_SIZE;

	/* irq */
	wrlen = scnprintf(curbuf, curlen, "irq: %d\n",
			akm->irq);
	if (wrlen == 0)
		return -EINVAL;
	curbuf += wrlen;
	curlen -= wrlen;

	/* mode */
	wrlen = scnprintf(curbuf, curlen, "current mode: %d\n",
			atomic_read(&akm->mode));
	if (wrlen == 0)
		return -EINVAL;
	curbuf += wrlen;
	curlen -= wrlen;

	/* selftest */
	wrlen = scnprintf(curbuf, curlen, "selftest: %d\n",
			atomic_read(&akm->selftest));
	if (wrlen == 0)
		return -EINVAL;
	curbuf += wrlen;
	curlen -= wrlen;

	/* print axis order */
	len = 32;
	ptmpbuf = tmpbuf;
	wrlen = print_array(
			&ptmpbuf, &len,
			akm->axis_order, NUM_OF_AXIS, sizeof(u8));
	if (wrlen == 0)
		return -EINVAL;

	wrlen = scnprintf(curbuf, curlen, "axis order: %s", tmpbuf);
	if (wrlen == 0)
		return -EINVAL;
	curbuf += wrlen;
	curlen -= wrlen;

	/* print axis sign */
	len = 32;
	ptmpbuf = tmpbuf;
	wrlen = print_array(
			&ptmpbuf, &len,
			akm->axis_sign, NUM_OF_AXIS, sizeof(u8));
	if (wrlen == 0)
		return -EINVAL;

	wrlen = scnprintf(curbuf, curlen, "axis sign: %s", tmpbuf);
	if (wrlen == 0)
		return -EINVAL;
	curbuf += wrlen;
	curlen -= wrlen;

	return (ssize_t)(PAGE_SIZE - curlen);
}

static IIO_DEVICE_ATTR(debug,
		S_IRUSR | S_IRGRP,
		attr_debug_show,
		NULL,
		0);
#endif

static struct attribute *ak0991x_attributes[] = {
	&iio_dev_attr_nsf.dev_attr.attr,
	&iio_dev_attr_watermark.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_reset.dev_attr.attr,
	&iio_dev_attr_info.dev_attr.attr,
#ifdef DEBUG
	&iio_dev_attr_axismap.dev_attr.attr,
	&iio_dev_attr_debug.dev_attr.attr,
#endif
	NULL
};

static const struct attribute_group ak0991x_attrs_group = {
	.attrs = ak0991x_attributes,
};

/*****************************************************************************/
static int ak0991x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);

	dev_dbg(akm->dev, "%s called (index=%d)", __func__, chan->scan_index);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&akm->buffer_mutex);
		*val = akm->buffer[chan->scan_index];
		mutex_unlock(&akm->buffer_mutex);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* scale coefficiency is fixed at boot time */
		*val = akm->raw_to_micro_q16[chan->scan_index];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ak0991x_get_samp_freq(akm, val);
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ak0991x_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);

	dev_info(akm->dev, "%s called (addr=%ld)", __func__, chan->address);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ak0991x_set_samp_freq(akm, val);
	}
	return -EINVAL;
}

static const struct iio_info ak0991x_info = {
	.attrs = &ak0991x_attrs_group,
	.read_raw = &ak0991x_read_raw,
	.write_raw = &ak0991x_write_raw,
	.driver_module = THIS_MODULE,
};

/******************************************************************************/
static int ak0991x_device_power_on(struct ak0991x_data *akm)
{
	int ret = 0;

	dev_dbg(akm->dev, "%s called", __func__);

	/* TODO: platform data will not be able to use with device tree */
	if (akm->pdata)
		if (akm->pdata->power_on)
			ret = akm->pdata->power_on();

	return ret;
}

static void ak0991x_device_power_off(struct ak0991x_data *akm)
{
	dev_dbg(akm->dev, "%s called", __func__);

	/* TODO: platform data will not be able to use with device tree */
	if (akm->pdata)
		if (akm->pdata->power_off)
			akm->pdata->power_off();
}

int ak0991x_resume(struct ak0991x_data *akm)
{
	dev_dbg(akm->dev, "%s called", __func__);
	return 0;
}
EXPORT_SYMBOL(ak0991x_resume);

int ak0991x_suspend(struct ak0991x_data *akm)
{
	dev_dbg(akm->dev, "%s called", __func__);
	return 0;
}
EXPORT_SYMBOL(ak0991x_suspend);

static void ak0991x_init_axis(struct ak0991x_data *akm)
{
	struct device_node *node = akm->dev->of_node;
	int i;

	if (node) {
		/* get from device node */
		/* parameters are declared as 'unsigned char' */
		/* if parameter cannot be get, use default value */
		if (of_property_read_u8(node, "axis_order_x",
					&akm->axis_order[0]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_order_y",
					&akm->axis_order[1]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_order_z",
					&akm->axis_order[2]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_sign_x",
					&akm->axis_sign[0]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_sign_y",
					&akm->axis_sign[1]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_sign_z",
					&akm->axis_sign[2]) != 0)
			goto SET_DEFAULT_AXIS;
	} else {
		/* platform data is NULL, use default value. */
		if (!akm->pdata)
			goto SET_DEFAULT_AXIS;

		/* get from pdata */
		for (i = 0; i < 3; i++) {
			akm->axis_order[i] = akm->pdata->axis_order[i];
			akm->axis_sign[i] = akm->pdata->axis_sign[i];
		}
	}

	dev_dbg(akm->dev, "%s : axis=[%d,%d,%d] sign=[%d,%d,%d]", __func__,
		akm->axis_order[0], akm->axis_order[1], akm->axis_order[2],
		akm->axis_sign[0], akm->axis_sign[1], akm->axis_sign[2]);
	return;

SET_DEFAULT_AXIS:
	dev_err(akm->dev, "%s: Axis info read failed. Use default value.",
			__func__);
	/* set default axis value */
	for (i = 0; i < 3; i++) {
		akm->axis_order[i] = i;
		akm->axis_sign[i] = 0;
	}
	return;
}

struct iio_dev *ak0991x_probe(struct device *dev, int irq,
		const struct ak0991x_bus_ops *bops)
{
	struct ak0991x_data *akm;
	struct iio_dev *indio_dev;
	int err = 0;
	/* platform data will not be able to use with device tree */

	dev_dbg(dev, "%s called", __func__);

	indio_dev = iio_device_alloc(sizeof(*akm));
	if (!indio_dev) {
		err = -ENOMEM;
		dev_err(dev, "%s: Failed to allocate iio device.", __func__);
		goto err_iio_device_allocate;
	}

	/* get private data */
	akm = iio_priv(indio_dev);


	/*** setup akm parameter ***/
	akm->dev = dev;
	akm->bops = bops;
	akm->pdata = dev->platform_data;
	akm->irq = irq;

	/* device is in powerdown mode just after the power on */
	atomic_set(&akm->mode, AK0991X_CNTL_PDN);
	/* -1 means 'selftest is not done yet' */
	atomic_set(&akm->selftest, -1);
	/* don't initialize with default value.
	 * because some device does not have NSF */
	atomic_set(&akm->cntl1, 0);
	mutex_init(&akm->buffer_mutex);
	mutex_init(&akm->fifo_mutex);

	INIT_DELAYED_WORK(&akm->work, ak0991x_tm_handler);
	INIT_WORK(&akm->flush_work, ak0991x_flush_handler);

	akm->wq = create_singlethread_workqueue("ak0991x");
	if (akm->wq == NULL) {
		dev_err(dev, "%s: create single thread wq failed.", __func__);
		goto err_create_thread_wq;
	}
	/* axis info */
	ak0991x_init_axis(akm);

	/* device check and set device specific parameters */
	err = ak0991x_device_power_on(akm);
	if (err) {
		dev_err(dev, "%s: device power on failed.", __func__);
		goto err_power_on;
	}

	err = akecs_checkdevice(akm);
	if (err < 0) {
		ak0991x_device_power_off(akm);
		goto err_check_device;
	}

	ak0991x_device_power_off(akm);

	/*** setup iio parameter ***/
	indio_dev->dev.parent = dev;
	indio_dev->channels = ak0991x_channels;
	indio_dev->num_channels = ARRAY_SIZE(ak0991x_channels);
	indio_dev->name = AKM_DRIVER_NAME;
	indio_dev->info = &ak0991x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if ((akm->irq) &&
			(akm->part_info->irqflag != IRQF_TRIGGER_NONE)) {
		akm->trig = iio_trigger_alloc(
				"%s-dev%d",
				indio_dev->name, indio_dev->id);
		if (!akm->trig) {
			err = -ENOMEM;
			goto err_trigger_alloc;
		}

		err = devm_request_irq(
				dev, akm->irq,
				iio_trigger_generic_data_rdy_poll,
				akm->part_info->irqflag, dev_name(akm->dev),
				akm->trig);
		if (err) {
			dev_err(dev, "%s: request irq failed.", __func__);
			goto err_request_irq;
		}
		/* register trigger */
		akm->trig->dev.parent = dev;
		akm->trig->ops = &ak0991x_trigger_ops;
		iio_trigger_set_drvdata(akm->trig, indio_dev);
		/* indio_dev->trig = iio_trigger_get(akm->trig); */
		err = iio_trigger_register(akm->trig);
		if (err)
			goto err_trigger_register;
		indio_dev->trig = akm->trig;
	}

	err = iio_triggered_buffer_setup(indio_dev, NULL,
			ak0991x_trigger_handler, NULL);
	if (err) {
		dev_err(dev, "%s: iio_triggered_buffer_setup failed.",
				__func__);
		goto err_iio_buffer_setup;
	}

	err = iio_device_register(indio_dev);
	if (err) {
		dev_err(dev, "%s: iio_device_register failed.", __func__);
		goto err_iio_device_register;
	}

	dev_info(dev, "%s successfully probed (irq=%d).",
			akm->part_info->name, akm->irq);

	return indio_dev;

err_iio_device_register:
	iio_triggered_buffer_cleanup(indio_dev);
err_iio_buffer_setup:
	if (akm->irq)
		iio_trigger_unregister(akm->trig);
err_trigger_register:
	if (akm->irq)
		devm_free_irq(dev, akm->irq, akm);
err_request_irq:
	if (akm->irq)
		iio_trigger_free(akm->trig);
err_trigger_alloc:
err_check_device:
err_power_on:
	iio_device_free(indio_dev);
err_create_thread_wq:
	if (akm->wq)
		destroy_workqueue(akm->wq);
err_iio_device_allocate:
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(ak0991x_probe);

int ak0991x_remove(struct iio_dev *indio_dev)
{
	struct ak0991x_data *akm = iio_priv(indio_dev);

	dev_dbg(akm->dev, "%s called", __func__);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	if (akm->irq) {
		iio_trigger_unregister(akm->trig);
		devm_free_irq(akm->dev, akm->irq, akm->trig);
		iio_trigger_free(akm->trig);
	}
	iio_device_free(indio_dev);

	if (akm->wq)
		destroy_workqueue(akm->wq);

	dev_info(akm->dev, "successfully removed.");
	return 0;
}
EXPORT_SYMBOL_GPL(ak0991x_remove);

MODULE_AUTHOR("AKM");
MODULE_DESCRIPTION("AK0991x compass driver for IIO");
MODULE_LICENSE("GPL");
