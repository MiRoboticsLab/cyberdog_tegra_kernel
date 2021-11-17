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

#include "i2c_hal.h"
#include "chirp_hal.h"
#include "init_driver.h"
#include "../chx01_client.h"

unsigned long i2c_master_read_register(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	struct chx01_client *data = get_chirp_data();
	const void *client = data->i2c_client;
	int ret;

	dbg_info("<<chx01>>%s: %02x %02x %d\n", __func__, (u32)Address, RegAddr,
		RegLen);

	ret = data->cbk->read((void *)client, (u16)Address, (u8)RegAddr,
		(u16)RegLen, (u8 *)RegValue);

	//--yd return RegLen;
	return ret;
}
unsigned long i2c_master_read_register_client(void *data, unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	struct chx01_client *ch_client = (struct chx01_client *)data;
	void *client = ch_client->i2c_client;
	int ret;

	dbg_info("<<chx01>>%s: %02x %02x %d\n", __func__, (u32)Address, RegAddr,
		RegLen);

	ret = ch_client->cbk->read(client, (u16)Address, (u8)RegAddr,
		(u16)RegLen, (u8 *)RegValue);

	//--yd return RegLen;
	return ret;
}
unsigned long i2c_master_read_register0(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	return i2c_master_read_register(Address, RegAddr, RegLen, RegValue);
}

unsigned long i2c_master_read_register1(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	return i2c_master_read_register(Address, RegAddr, RegLen, RegValue);
}

unsigned long i2c_master_read_register2(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	return i2c_master_read_register(Address, RegAddr, RegLen, RegValue);
}

unsigned long i2c_master_read_register0_sync(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_read_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_read_register1_sync(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_read_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_read_register2_sync(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_read_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_read_register0_nb(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_read_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_read_register1_nb(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_read_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_read_register2_nb(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_read_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_write_register(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	struct chx01_client *data = get_chirp_data();
	const void *client = data->i2c_client;
	int ret;

	dbg_info("<<chx01>>%s: %02x %02x %d\n", __func__, (u32)Address, RegAddr, RegLen);

	ret = data->cbk->write((void *)client, (u16)Address, (u8)RegAddr,
		(u16)RegLen, (u8 *)RegValue);

	//--yd return RegLen;
	return ret;
}
unsigned long i2c_master_write_register_client(void *data, unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	struct chx01_client *ch_client = (struct chx01_client *)data;
	void *client = ch_client->i2c_client;
	int ret;

	dbg_info("<<chx01>>%s: %02x %02x %d\n", __func__, (u32)Address, RegAddr, RegLen);

	ret = ch_client->cbk->write(client, (u16)Address, (u8)RegAddr,
		(u16)RegLen, (u8 *)RegValue);

	//--yd return RegLen;
	return ret;
}
/*
unsigned long i2c_master_read_register_client(void *data, unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	struct chx01_client *ch_client = (struct chx01_client *)data;
	const void *client = data->i2c_client;
	int ret;

	dbg_info("<<chx01>>%s: %02x %02x %d\n", __func__, (u32)Address, RegAddr,
		RegLen);

	ret = ch_client->cbk->read(client, (u16)Address, (u8)RegAddr,
		(u16)RegLen, (u8 *)RegValue);

	//--yd return RegLen;
	return ret;
}
*/
unsigned long i2c_master_write_register0(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	return i2c_master_write_register(Address, RegAddr, RegLen, RegValue);
}

unsigned long i2c_master_write_register1(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	return i2c_master_write_register(Address, RegAddr, RegLen, RegValue);
}

unsigned long i2c_master_write_register2(unsigned char Address,
	unsigned char RegAddr, unsigned short RegLen,
	unsigned char *RegValue)
{
	return i2c_master_write_register(Address, RegAddr, RegLen, RegValue);
}

unsigned long i2c_master_write_register0_sync(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_write_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_write_register1_sync(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_write_register(Address, 0, RegLen, RegValue);
}

unsigned long i2c_master_write_register2_sync(unsigned char Address,
	unsigned short RegLen, unsigned char *RegValue)
{
	return i2c_master_write_register(Address, 0, RegLen, RegValue);
}


void i2c_master_initialize0(void)
{
}

void i2c_master_initialize1(void)
{
}

void i2c_master_initialize2(void)
{
}

void i2c_master_init(void)
{
	i2c_master_initialize0();
	i2c_master_initialize1();
	i2c_master_initialize2();
}

void ext_int_init(void)
{
}

unsigned long i2c_master_write_raw(unsigned char Address,
	unsigned short len, unsigned char *data)
{
	int res;
	int i;
	struct i2c_msg msg[] = {
	// { .addr = i2c_addr, .flags = 0, .len = 1, .buf = &reg },
	{ .addr = Address, .flags = 0, .len = len, .buf = data }
	};

	struct chx01_client *chirpdata = get_chirp_data();
	const void *client = chirpdata->i2c_client;
	struct i2c_client *i2c_client = (struct i2c_client *)client;
	struct i2c_adapter *i2c_adapter = i2c_client->adapter;

	dbg_info("<<chx01>>%s --> i2c_addr: 0x%x, length: %d\n", __func__, Address, len);

	res = i2c_transfer(i2c_adapter, msg, 1);
	if(len <= 4)
		for(i=0; i< len; i++) dbg_info("<<chx01>>%s -> data[%d] = 0x%x\n", __func__, i, data[i]); //debug--yd
			
	if (res < 1) {
		if (res == 0)
		res = -EIO;
		return res;
	} else 
		return 0;

}

unsigned long i2c_master_read_raw(unsigned char Address, unsigned short len, unsigned char *data)
{

	int res;
	int i;
	struct i2c_msg msg[] = {
		// { .addr = i2c_addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = Address, .flags = I2C_M_RD, .len = len, .buf =(char *)data }
	};

	struct chx01_client *chirpdata = get_chirp_data();
	const void *client = chirpdata->i2c_client;
	struct i2c_client *i2c_client = (struct i2c_client *)client;
	struct i2c_adapter *i2c_adapter = i2c_client->adapter;

	// dbg_info("<<chx01>>%s: %x\n", __func__, (u32)i2c_client);
	// dbg_info("<<chx01>>%s: %x\n", __func__, (u32)i2c_adapter);
	dbg_info("<<chx01>>%s --> i2c_addr: 0x%x, len: %d\n", __func__, Address, len);

	res = i2c_transfer(i2c_adapter, msg, 1);
	if(len <= 4)
		for(i=0; i< len; i++) dbg_info("<<chx01>>%s -> data[%d] = 0x%x\n", __func__, i, data[i]); //debug--yd

	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else 
		return 0;

}
