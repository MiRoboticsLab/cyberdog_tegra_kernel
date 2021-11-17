/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef DRIVERS_IIO_PROXIMITY_CH101_DATA_H_
#define DRIVERS_IIO_PROXIMITY_CH101_DATA_H_

#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/completion.h>

#include <asm-generic/int-ll64.h>

#include "chx01_client.h"

struct chx01_data {
	struct device *dev;
	struct chx01_client client;
	struct chx01_buffer buffer;
	struct regmap *regmap;
	// struct mutex lock;
	struct completion completion;
	struct iio_trigger *trig;
	bool trigger_enabled;
	struct hrtimer timer;
	int irq;
	int scan_rate;
	ktime_t period;
	spinlock_t period_lock;
};

#endif /* DRIVERS_IIO_PROXIMITY_CH101_DATA_H_ */
