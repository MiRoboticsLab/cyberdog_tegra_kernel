/* SPDX-License-Identifier: GPL-2.0 */
#ifndef SYSTEM_H_
#define SYSTEM_H_

#if defined(__KERNEL__)

#include <linux/string.h>
#include <linux/printk.h>

//to disable the debug message, undefine both CHDRV_DEBUG and CHIRP_DEBUG --yd
// #define CHIRP_DEBUG

#ifdef CHIRP_DEBUG
#define dbg_info(...) pr_info(__VA_ARGS__)
#else
/*to disable debug message globally*/
#define dbg_info(...)     
#endif

#define printf dbg_info

#define UINT8_MAX	0xFF
#define UINT16_MAX	0xFFFF

#else

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define printf(...)

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

#endif

#endif
