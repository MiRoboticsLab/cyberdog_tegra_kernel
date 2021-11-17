/* include/linux/i2c/bu27030_driver.h - ROHM bu27030 Linux kernel driver
 *
 * Copyright (C) 2012-2018
 * Written by Aaron Liu <aaron-liu@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __BU27030_DRIVER_H__
#define __BU27030_DRIVER_H__

/*************** Definitions ******************/
/* GENERAL */
#define BU27030_DRV_NAME "bu27030"
#define DRIVER_VERSION    "1.0"



#define QCOM_SENSORS       (0)
#define AGC_SUPPORT        (1)


/*-----------------------DEBUG------------------------------*/
#define BU27030_DGB_SWITCH         // debug switch
#define BU27030_TAG             "[ALS]BU27030"

#ifdef BU27030_DGB_SWITCH
#define BU27030_DEBUG   1
#else
#define BU27030_DEBUG   0
#endif

#define  BU27030_ERR(f, a...)        do {printk(KERN_ERR BU27030_TAG "ERROR (%s(), %d):"   f, __func__,  __LINE__, ## a);} while (0)
#define  BU27030_WARNING(f, a...)    do {printk(KERN_WARNING BU27030_TAG "(%s(), %d):"         f, __func__,  __LINE__, ## a);} while (0)
#define  BU27030_INFO(f, a...)       do {printk(KERN_INFO BU27030_TAG "INFO (%s(), %d):"    f, __func__,  __LINE__, ## a);} while (0)


#if BU27030_DEBUG
#define  BU27030_FUN()               do {printk(KERN_INFO BU27030_TAG "(%s(), %d)\n",          __func__,  __LINE__);} while (0)
#define  BU27030_DBG(f, a...)        do {printk(KERN_DEBUG BU27030_TAG "DEBUG (%s(), %d):"   f, __func__,  __LINE__, ## a);} while (0)
#else
#define  BU27030_FUN()   do {} while (0)
#define  BU27030_DBG(f, a...)   do {} while (0)
#endif
/*-----------------------------------------------------*/





/* BU27030 REGSTER */
#define BU27030_REG_SYSTEM_CTL         (0x40)
#define BU27030_REG_MODE_CTL1          (0x41)
#define BU27030_REG_MODE_CTL2          (0x42)
#define BU27030_REG_MODE_CTL3          (0x43)
#define BU27030_REG_DATA0              (0x50)
#define BU27030_REG_DATA1              (0x52)
#define BU27030_REG_PART_ID            (0x92)
#define BU27030_PART_ID_VALUE          (0xE0)

#define BU27030_REG_SOFT_RST           (BU27030_REG_SYSTEM_CTL)
#define BU27030_REG_CONTROL            (BU27030_REG_MODE_CTL3)
#define BU27030_REG_TIMING             (BU27030_REG_MODE_CTL1)
#define BU27030_REG_GAIN               (BU27030_REG_MODE_CTL2)


/************ define parameter for register ************/
#define BU27030_ENABLE      (1)
#define BU27030_DISABLE     (0)

#define POWER_ON      (1)
#define POWER_OFF     (0)

#define ALS_VALID_HIGH     (1 << 7)
#define BU27030_RST        (1 << 7)

/* Time(0x41) */
#define MEASURE_50MS       (1)
#define MEASURE_100MS      (0)
#define ALPHA_JUDGE        (10)
#define ALPHA_FACTOR       (100)


/* Gain (0x87) */
#define DATA0_GAIN_X1       (0x02 << 4)
#define DATA1_GAIN_X1       (0x02)

#define DATA0_GAIN_X32      (0x0a << 4)
#define DATA1_GAIN_X32      (0x0a)

#define DATA0_GAIN_X256     (0x0c << 4)
#define DATA1_GAIN_X256     (0x0c)

#define DATA0_DATA1_GAIN_X1      (DATA0_GAIN_X1   | DATA1_GAIN_X1)
#define DATA0_DATA1_GAIN_X32     (DATA0_GAIN_X32  | DATA1_GAIN_X32)
#define DATA0_DATA1_GAIN_X256    (DATA0_GAIN_X256 | DATA1_GAIN_X256)

#define DATA_TRANSFER_COFF (100 * 256)  //100ms, 256x

#define DATA0_GAIN_MASK (0xF0)
#define DATA1_GAIN_MASK (0x0F)

#define MEASURE_DEFAULT_TIME    (MEASURE_50MS)
#define MEASURE_DEFAULT_GAIN    (DATA0_DATA1_GAIN_X1)

#define BU27030_1X          (1)
#define BU27030_32X         (32)
#define BU27030_256X        (256)

#define BU27030_SATURATION_THRESH       (60000)
#define BU27030_INSUFFICIENCE_THRESH    (100)

/** Default values loaded in probe function */
#define BU27030_WHOAMI_VALUE              (BU27030_PART_ID_VALUE)  /** Who Am I default value */

/* POWER SUPPLY VOLTAGE RANGE */
#define BU27030_VDD_MIN_UV  (2000000)
#define BU27030_VDD_MAX_UV  (3300000)
#define BU27030_VIO_MIN_UV  (1750000)
#define BU27030_VIO_MAX_UV  (1950000)

#define ALS_SET_MIN_DELAY_TIME  (100)



/*************** Structs ******************/
typedef struct {
    struct i2c_client *client;
    struct regulator *vdd;
    struct regulator *vio;
    struct mutex update_lock;

    struct delayed_work als_dwork; /* for ALS polling */

    struct input_dev *input_dev_als;

#if QCOM_SENSORS
    struct sensors_classdev als_cdev;
#endif

    unsigned int enable;
    
    unsigned int als_time;


    /* control flag from HAL */
    unsigned int enable_als_sensor;
    unsigned int als_suspend_flag;


    /* ALS parameters */
    unsigned int als_data;      /* to store ALS data */
    unsigned int als_level;
    unsigned int gain;
    unsigned int als_poll_delay;

    unsigned int dev_id;
}BU27030_ALS_DATA;

/* structure to read data value from sensor */
typedef struct {
    unsigned int data0;
    unsigned int data1;
    } READ_DATA_ARG;

typedef struct  {
    unsigned char  als_time; 
    unsigned short gain0;
    unsigned short gain1;
    unsigned int   als_data0;
    unsigned int   als_data1;
    }CALC_DATA;

#endif
