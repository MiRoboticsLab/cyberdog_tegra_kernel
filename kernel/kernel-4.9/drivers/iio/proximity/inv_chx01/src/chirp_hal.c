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

#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/export.h>
#include <linux/timekeeping.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/time.h>
#include <asm/div64.h>

#include "chirp_hal.h"
#include "chbsp_init.h"

static struct chx01_gpios *_gpios;

struct gpio_desc *os_get_pin_desc(ioport_pin_t pin);

void set_chirp_gpios(struct chx01_gpios *gpios)
{
	_gpios = gpios;
	dbg_info("<<chx01>>%s: data: %x\n", __func__, (u32)_gpios);
}

struct chx01_gpios *get_chirp_gpios(void)
{
	return _gpios;
}

void ioport_set_pin_dir_setval(ioport_pin_t pin, enum ioport_direction dir, int value)
{
	struct gpio_desc *desc = os_get_pin_desc(pin);

	dbg_info("<<chx01>>%s: pin: %d(%x) dir: %d\n", __func__, pin, (u32)desc,
		dir);

	if (dir == IOPORT_DIR_INPUT)
		gpiod_direction_input(desc);
	else
		gpiod_direction_output(desc, value);
}

void ioport_set_pin_dir(ioport_pin_t pin, enum ioport_direction dir)
{//--yd
	struct gpio_desc *desc = os_get_pin_desc(pin);
	int value;

	dbg_info("<<chx01>>%s: pin: %d(%x) dir: %d\n", __func__, pin, (u32)desc,
		dir);

	value = (pin == CHIRP_RST)? 1:0; //set Chirp-RST=output-high, other gpios (Chirp-CAL/Chirp-PROG) set to output-low --yd

	if (dir == IOPORT_DIR_INPUT)
		gpiod_direction_input(desc);
	else
		gpiod_direction_output(desc, value);
}


void ioport_set_pin_level_0(ioport_pin_t pin, bool level)
{
	struct gpio_desc *desc = os_get_pin_desc(pin);

	// dbg_info("<<chx01>>%s: pin: %d(%x) level: %d\n", __func__, pin, (u32)desc,	level);

	gpiod_set_value(desc, level);
}

void ioport_set_pin_level(ioport_pin_t pin, bool level)
{
	struct gpio_desc *desc = os_get_pin_desc(pin);

	dbg_info("<<chx01>>%s: pin: %d(%x) level: %d\n", __func__, pin, (u32)desc,
		level);

	gpiod_set_value(desc, level);
}

struct gpio_desc *os_get_pin_desc(ioport_pin_t pin)
{
	if (pin == CHIRP_RST)
		return _gpios->gpiod_rst;
	if (pin == CHIRP0_PROG_0)
		return _gpios->gpiod_prg;
	if (pin == CHIRP0_INT_0)
		return _gpios->gpiod_int;
	if (pin == CHIRP0_CAL_0)
		return _gpios->gpiod_cal;

	dbg_info("<<chx01>>%s: pin: %d undefined\n", __func__, (u32)pin);
	return 0;
}

int32_t os_enable_interrupt(const uint32_t pin)
{
	struct gpio_desc *desc = os_get_pin_desc(pin);
	unsigned int irq = gpiod_to_irq(desc);

	enable_irq_wake(irq);

	return 0;
}

int32_t os_disable_interrupt(const uint32_t pin)
{
	struct gpio_desc *desc = os_get_pin_desc(pin);
	unsigned int irq = gpiod_to_irq(desc);

	disable_irq_wake(irq);

	return 0;
}

void os_clear_interrupt(const uint32_t pin)
{
}

// delay in microseconds
void os_delay_us(const uint16_t us)
{
	usleep_range(us, us + 1);
}

// delay in miliseconds
void os_delay_ms(const uint16_t ms)
{
	// msleep(ms);
	mdelay(ms);
}

u64 os_timestamp_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	return timespec_to_ns(&ts);
}

u64 os_timestamp_ms(void)
{
	u64 ns = os_timestamp_ns();
	u32 div = 1000000;

	//return __div64_const32(ns, div);
	return div_u64(ns, div);   //--yd
}

void os_print_str(char *str)
{
	dbg_info("<<chx01>>%s", str);
}

