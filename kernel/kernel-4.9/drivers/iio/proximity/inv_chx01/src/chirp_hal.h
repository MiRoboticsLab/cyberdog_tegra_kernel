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

#ifndef CHIRPHAL_H
#define CHIRPHAL_H

#include "system.h"
#include "init_driver.h"
#include "chirp_hal.h"
#include "../chx01_client.h"

typedef u32 ioport_pin_t;

enum ioport_direction {
	IOPORT_DIR_INPUT, /*!< IOPORT input direction */
	IOPORT_DIR_OUTPUT, /*!< IOPORT output direction */
};

enum ioport_value {
	IOPORT_PIN_LEVEL_LOW, /*!< IOPORT pin value low */
	IOPORT_PIN_LEVEL_HIGH, /*!< IOPORT pin value high */
};

void ioport_set_pin_dir(ioport_pin_t pin, enum ioport_direction dir);
void ioport_set_pin_dir_setval(ioport_pin_t pin, enum ioport_direction dir, int value); //--yd
void ioport_set_pin_level(ioport_pin_t pin, bool level);
void ioport_set_pin_level_0(ioport_pin_t pin, bool level);

int32_t os_enable_interrupt(const u32 pin);
int32_t os_disable_interrupt(const u32 pin);
void os_clear_interrupt(const u32 pin);

void os_delay_us(const u16 us);
void os_delay_ms(const u16 ms);

uint64_t os_timestamp_ms(void);

void set_chirp_gpios(struct chx01_gpios *gpios);

void os_print_str(char *str);

#endif /* CHIRPHAL_H */
