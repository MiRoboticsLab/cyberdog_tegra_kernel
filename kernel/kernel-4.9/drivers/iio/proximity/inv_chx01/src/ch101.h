/* SPDX-License-Identifier: GPL-2.0 */
/*! \file ch101.h
 *
 * \brief Internal definitions for the Chirp CH101 ultrasonic sensor.
 *
 * This file contains various hardware-defined values for the CH101 sensor.
 *
 * You should not need to edit this file or call the driver functions directly.
 * Doing so will reduce your ability to benefit from future enhancements and
 * releases from Chirp.
 *
 */

/*
 * Copyright (c) 2016-2019, Chirp Microsystems.  All rights reserved.
 *
 * Chirp Microsystems CONFIDENTIAL
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CH101_H_
#define CH101_H_

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

#define CH101_DATA_MEM_SIZE		0x800
#define CH101_DATA_MEM_ADDR		0x0200
#define CH101_PROG_MEM_SIZE		0x800
#define CH101_PROG_MEM_ADDR		0xF800
#define	CH101_FW_SIZE			CH101_PROG_MEM_SIZE

#define CH101_FREQCOUNTERCYCLES		128

#endif
