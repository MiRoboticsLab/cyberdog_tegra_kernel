/*
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OV7251_H__
#define __OV7251_H__

#include <media/nvc.h>
#include <uapi/media/nvc_image.h>
#include <uapi/media/ov7251.h>

#define OV7251_INVALID_COARSE_TIME	-1

#define OV7251_EEPROM_ADDRESS		0x52 
#define OV7251_EEPROM_SIZE		1024
#define OV7251_EEPROM_STR_SIZE		(OV7251_EEPROM_SIZE * 2)
#define OV7251_EEPROM_BLOCK_SIZE	(1 << 8)
#define OV7251_EEPROM_NUM_BLOCKS \
	(OV7251_EEPROM_SIZE / OV7251_EEPROM_BLOCK_SIZE)

#define OV7251_OTP_LOAD_CTRL_ADDR	0x3D81
#define OV7251_OTP_BANK_SELECT_ADDR	0x3D84
#define OV7251_OTP_BANK_START_ADDR	0x3D00
#define OV7251_OTP_BANK_END_ADDR	0x3D0F
#define OV7251_OTP_NUM_BANKS		(32)
#define OV7251_OTP_BANK_SIZE \
	 (OV7251_OTP_BANK_END_ADDR - OV7251_OTP_BANK_START_ADDR + 1)
#define OV7251_OTP_SIZE \
	 (OV7251_OTP_BANK_SIZE * OV7251_OTP_NUM_BANKS)
#define OV7251_OTP_STR_SIZE (OV7251_OTP_SIZE * 2)

/* See notes in the nvc.h file on the GPIO usage */
enum ov7251_gpio_type {
	OV7251_GPIO_TYPE_PWRDN = 0,
	OV7251_GPIO_TYPE_RESET,
};

struct ov7251_eeprom_data {
	struct i2c_client *i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap *regmap;
};

struct ov7251_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *dovdd;
};

struct ov7251_regulators {
	const char *avdd;
	const char *dvdd;
	const char *dovdd;
};

struct ov7251_platform_data {
	unsigned cfg;
	unsigned num;
	const char *dev_name;
	unsigned gpio_count; /* see nvc.h GPIO notes */
	struct nvc_gpio_pdata *gpio; /* see nvc.h GPIO notes */
	struct nvc_imager_static_nvc *static_info;
	bool use_vcm_vdd;
	int (*probe_clock)(unsigned long);
	int (*power_on)(struct ov7251_power_rail *);
	int (*power_off)(struct ov7251_power_rail *);
	const char *mclk_name;
	struct nvc_imager_cap *cap;
	struct ov7251_regulators regulators;
	bool has_eeprom;
	bool use_cam_gpio;
};

#endif  /* __OV7251_H__ */
