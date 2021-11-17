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

#ifndef DRIVERS_IIO_PROXIMITY_INV_CH101_CH101_CLIENT_H_
#define DRIVERS_IIO_PROXIMITY_INV_CH101_CH101_CLIENT_H_

#include <linux/regmap.h>

#define MAX_SAMPLES 130

struct chx01_iq_data {
	s16 I;
	s16 Q;
};

struct chx01_buffer {
	int distance;
	int nb_samples;
	struct chx01_iq_data iq_data[MAX_SAMPLES];
};

struct chx01_client;
struct chx01_callbacks {
	int (*write)(void *client, u16 i2c_addr, u8 reg, u16 length, u8 *data);
	int (*read)(void *client, u16 i2c_addr, u8 reg, u16 length, u8 *data);
	void (*data_complete)(struct chx01_client *data);
};

struct chx01_gpios {
	struct i2c_client *i2c_client;
	struct gpio_desc *gpiod_rst;
	struct gpio_desc *gpiod_prg;
	struct gpio_desc *gpiod_cal;
	struct gpio_desc *gpiod_int;
};

struct chx01_client {
	void *i2c_client;
	struct chx01_gpios gpios;
	struct chx01_callbacks *cbk;
};

int chx01_core_probe(struct i2c_client *client, struct regmap *regmap,
			struct chx01_callbacks *cbk, const char *name);
int chx01_core_remove(struct i2c_client *client);

#endif /* DRIVERS_IIO_PROXIMITY_INV_CH101_CH101_CLIENT_H_ */
