/* Copyright (C) 2012-2020
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
/*
 *  This is Linux kernel modules for ambient light sensor
 *  Revision History
 *  2020-10-22:    Ver. 1.0    New release 
 */
#include <linux/debugfs.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>


#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include "bu27030_driver.h"

#if QCOM_SENSORS
#include <linux/sensors.h>
#endif


/*************** Global Data ******************/
////////////////////////////////////////////////////////////////

#define     COEFFICIENT_SIZE               (14)
#define     GAIN_FACTOR                    (128) //this value is related with gain when make coefficient  

#define JUDGE_FIXED_COEF   (100000)
#if 0
const int data_coefficient[COEFFICIENT_SIZE] = {
    32000, 
    850000, //850 * 1000
    518, -68, -32000,  471000,
    396, -52, -32000,  471000,
    518, -68, -32000, -80000 
    };
#else
const int data_coefficient[COEFFICIENT_SIZE] = {
    38600, 
    850000, //850 * 1000
    21300, 100000, -38600, 130000,
    21300, 100000, -38600,  -30000,
    21300, 100000, -38600,  -30000
    };
#endif

static const uint8_t time_table[] = {100, 50};    
static const uint16_t gain_table[] = {
    0,                  // 0
    0,                  // 1
    BU27030_1X,         // 2
    0,                  // 3
    0,                  // 4
    0,                  // 5
    0,                  // 6
    0,                  // 7
    0,                  // 8
    0,                  // 9
    BU27030_32X,        // 10
    0,                  // 11
    BU27030_256X,       // 12
    0,                  // 13
    0,                  // 14
    0                   // 15
    };


#if QCOM_SENSORS

/* static struct bu27030_chip *pdev_data;*/
static struct sensors_classdev sensors_light_cdev = {
    .name = "lightsensor",
    .vendor = "rohm",
    .version = 1,
    .handle = SENSORS_LIGHT_HANDLE,
    .type = SENSOR_TYPE_LIGHT,
    .max_range = "43000",
    .resolution = "0.0125",
    .sensor_power = "0.20",
    .min_delay = 1000, /* in microseconds */
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = 100,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .flags = 3,
    .max_latency = 0,
};

#endif

/******************************************************************************
 * NAME       : bu27030_driver_reset
 * FUNCTION   : reset BU27030 register
 * REMARKS    :
 *****************************************************************************/
static int bu27030_driver_reset(struct i2c_client *client)
{
    int result = 0;        

    if (!client)
    {
        BU27030_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft reset */
    result |= i2c_smbus_write_byte_data(client, BU27030_REG_SOFT_RST, BU27030_RST);

    return (result);
}


/**
 * @Brief: bu27030_register_dump print register value for debug
 *
 * @Param: reg_address regsiter address
 *
 * @return: return register's value
 */
static int bu27030_register_dump(BU27030_ALS_DATA * data, int addr)
{
    int  read_data = 0;

    if (!data || !data->client){
        printk(" Parameter error \n");
        return -EINVAL;;
    }

    if( addr > BU27030_REG_PART_ID  || addr < BU27030_REG_SYSTEM_CTL){
        printk( "reg =0x%x  is out of range!!!", addr);
        return -EINVAL;
    }

    /*read */
    read_data = i2c_smbus_read_byte_data(data->client, addr);

    if (read_data < 0) {
        printk( "bu27030_driver_general_read : transfer error \n");
        return -EINVAL;

    }

    printk( "reg(0x%x) = 0x%x \n", addr, read_data);

    return read_data;
}


/*************** Initialze Functions ******************/
static int bu27030_init_client(BU27030_ALS_DATA *als)
{
    int result = 0;

    /* execute software reset */
    result = bu27030_driver_reset(als->client);
    if (result != 0) {
        return (result);
    }

    //100ms
    result = i2c_smbus_write_byte_data(als->client, BU27030_REG_TIMING, MEASURE_100MS);
    if (result == 0) {
        //gain0 gain1 : 1X, 1X
        result = i2c_smbus_write_byte_data(als->client, BU27030_REG_GAIN, MEASURE_DEFAULT_GAIN);

    }
    else{
        BU27030_ERR(" I2c write failed! \n");
    }

    return (result);
}


/**
 * @Brief: bu27030_calculate_light Calculate lux base on rgbc
 *
 * @Param: data RGBC value from sensor
 *
 * @Returns: lux value or failed.
 */
static int bu27030_calculate_light(READ_DATA_ARG data, unsigned char gain, unsigned char time)
{    

    long long lx, tmp1, tmp2;
    int tmp_coef;
    CALC_DATA  calculated_data;

    /* set the value of measured als data */
    calculated_data.als_time   = time_table[time & 0x1];
    calculated_data.gain0 = gain_table[gain >> 4];
    calculated_data.gain1 = gain_table[gain & DATA1_GAIN_MASK];
    calculated_data.als_data0  = data.data0;
    calculated_data.als_data1  = data.data1;

    if(data.data0 == 0xFFFF){
        BU27030_WARNING("Data0 is 0xFFFF, return max lux 65535.");
        return 65535;
    }

    if(!calculated_data.als_time || !calculated_data.gain0 || !calculated_data.gain1)
    {
        BU27030_WARNING( "parameter error, als_time:%d, gain0:%d, gain1:%d",
                calculated_data.als_time, calculated_data.gain0, calculated_data.gain1);
        return 0;
    }

    calculated_data.als_data0 = 
        data.data0 * DATA_TRANSFER_COFF / calculated_data.als_time/ calculated_data.gain0;
    calculated_data.als_data1 = 
        data.data1 * DATA_TRANSFER_COFF / calculated_data.als_time/ calculated_data.gain1;


    if ( calculated_data.als_data1 < ALPHA_JUDGE) {
        tmp_coef = ALPHA_FACTOR * calculated_data.als_data1 / ALPHA_JUDGE; 
    } else  {
        tmp_coef = ALPHA_FACTOR;
    }


    if ( calculated_data.als_data1 * JUDGE_FIXED_COEF < calculated_data.als_data0 * data_coefficient[0]) {
        if( calculated_data.als_data0 < data_coefficient[1]) {


            tmp1 = data_coefficient[2] * calculated_data.als_data0 + data_coefficient[3] * calculated_data.als_data1 ;
            
            tmp2 = (JUDGE_FIXED_COEF * calculated_data.als_data1 / calculated_data.als_data0 - data_coefficient[4]) * data_coefficient[5] ;

            BU27030_WARNING("%s %d\n", __FUNCTION__, __LINE__);

        } 
        else {

            tmp1 = data_coefficient[6] * calculated_data.als_data0 + data_coefficient[7] * calculated_data.als_data1 ;
            
            tmp2 = (JUDGE_FIXED_COEF * calculated_data.als_data1 / calculated_data.als_data0 - data_coefficient[8]) * data_coefficient[9] ;

            BU27030_WARNING("%s %d\n", __FUNCTION__, __LINE__);

        }
    } 
    else {

        tmp1 = data_coefficient[10] * calculated_data.als_data0 + data_coefficient[11] * calculated_data.als_data1 ;
        
        tmp2 = (JUDGE_FIXED_COEF * calculated_data.als_data1 / calculated_data.als_data0 - data_coefficient[12]) * data_coefficient[13] ;

        BU27030_WARNING("%s %d\n", __FUNCTION__, __LINE__);

    } 


    if (tmp1 > JUDGE_FIXED_COEF) {

        lx = tmp1 / JUDGE_FIXED_COEF *
            (
                tmp2 * tmp_coef / ALPHA_FACTOR / JUDGE_FIXED_COEF 
                +  JUDGE_FIXED_COEF 
            ) 
            / JUDGE_FIXED_COEF;

    }
    else {

        lx = tmp1  * 
            (
                tmp2 * tmp_coef / ALPHA_FACTOR  / JUDGE_FIXED_COEF 
                +  JUDGE_FIXED_COEF 
            ) 
            / JUDGE_FIXED_COEF  / JUDGE_FIXED_COEF;
    }


    if(lx < 0)
    {
        lx = 0;
        BU27030_WARNING("lx is minus, error!!!");
    }
    
    BU27030_WARNING("tmp1=%lld, tmp2=%lld, tmp_coef=%d, JUDGE_FIXED_COEF=%d\n", 
        tmp1, tmp2, tmp_coef, JUDGE_FIXED_COEF);


    BU27030_WARNING("lux:%lld, data0=%d, data1=%d, gain0=%d, gain1=%d, als_time:%d\n",
            lx, 
            data.data0, data.data1,
            calculated_data.gain0, calculated_data.gain1,
            calculated_data.als_time);

    return (int)(lx);
}


static unsigned int bu27030_als_data_to_level(unsigned int als_data)
{
    return als_data;
}



int bu27030_auto_change_gain0(BU27030_ALS_DATA * als, unsigned int data0)
{
    unsigned char target_gain = 0;
    unsigned short curret_gain0 = 0;
    uint8_t  buffer;

    int gain_changed = 0;

    if(!als) {
        BU27030_ERR( "Parameter error  !!! \n");
        return gain_changed;
    }    
    
    //get gain reg_value
    buffer = i2c_smbus_read_byte_data(als->client, BU27030_REG_GAIN);
    if (buffer < 0)
    {
        BU27030_ERR("Read data from IC error.\n");
        return gain_changed;
    }

    //gain0
    curret_gain0 = gain_table[buffer >> 4];
    if(data0 > BU27030_SATURATION_THRESH) 
    {
        if(curret_gain0 > BU27030_32X){ //current is  256X
            target_gain = DATA0_GAIN_X32 | (buffer & DATA1_GAIN_MASK);
        }else if(curret_gain0 > BU27030_1X){ //current is 32X
            target_gain = DATA0_GAIN_X1 | (buffer & DATA1_GAIN_MASK);
        }
    }
    else if (data0 < BU27030_INSUFFICIENCE_THRESH)
    {
        if(curret_gain0 < BU27030_32X){ //current is  1X
            target_gain = DATA0_GAIN_X32 | (buffer & DATA1_GAIN_MASK);
        }else if(curret_gain0 < BU27030_256X){ //current is 32X
            target_gain = DATA0_GAIN_X256 | (buffer & DATA1_GAIN_MASK);
        }
    }

    if(target_gain) {
        
        gain_changed = 1;
       
        i2c_smbus_write_byte_data(als->client, BU27030_REG_GAIN, target_gain);

        BU27030_INFO("bu27030_auto_change_gain current_gain0=%d, target=%d\n", 
            curret_gain0, gain_table[target_gain >> 4]);

    }

    return gain_changed;

}

int bu27030_auto_change_gain1(BU27030_ALS_DATA * als,  unsigned int data1)
{

    unsigned char target_gain = 0;
    unsigned short curret_gain1 = 0;
    uint8_t  buffer;

    int gain_changed = 0;

    if (!als) {
        BU27030_ERR( "Parameter error  !!! \n");
        return gain_changed;
    }

    //get gain reg_value
    buffer = i2c_smbus_read_byte_data(als->client, BU27030_REG_GAIN);
    if (buffer < 0)
    {
        BU27030_ERR("Read data from IC error.\n");
        return gain_changed;
    }
    
    curret_gain1 = gain_table[buffer & DATA1_GAIN_MASK];
    if(data1 > BU27030_SATURATION_THRESH) 
    {
        if(curret_gain1 > BU27030_32X){ //current is  256X
            target_gain = DATA1_GAIN_X32 | (buffer & DATA0_GAIN_MASK);
        }else if(curret_gain1 > BU27030_1X){ //current is 32X
            target_gain = DATA1_GAIN_X1 | (buffer & DATA0_GAIN_MASK);
        }
    }
    else if (data1 < BU27030_INSUFFICIENCE_THRESH)
    {
        if(curret_gain1 < BU27030_32X){ //current is  1X
            target_gain = DATA1_GAIN_X32 | (buffer & DATA0_GAIN_MASK);
        }else if(curret_gain1 < BU27030_256X){ //current is 32X
            target_gain = DATA1_GAIN_X256 | (buffer & DATA0_GAIN_MASK);
        }
    }

    if(target_gain) {
        gain_changed = 1;
       
        i2c_smbus_write_byte_data(als->client, BU27030_REG_GAIN, target_gain);

        BU27030_INFO( "bu27030_auto_change_gain1 current_gain1=%d, target=%d\n", 
            curret_gain1, gain_table[target_gain & DATA1_GAIN_MASK]);
    }

    return gain_changed;
}


/* ALS polling routine */
static void bu27030_als_polling_work_handler(struct work_struct *work)
{
    BU27030_ALS_DATA * als = container_of(work,
            BU27030_ALS_DATA, als_dwork.work);
    struct i2c_client *client = als->client;
    int tmp = 0;

    int gain_changed = 0;

    //get valid from BU27030_REG_CONTROL(0x43)
    tmp = i2c_smbus_read_byte_data(client, BU27030_REG_CONTROL);
    if (tmp < 0)
    {
        BU27030_ERR("Read data from IC error.\n");
        return ;
    }

    if(0 == (tmp & POWER_ON)) {
        BU27030_WARNING( " ic is abnormal, re-initialize, and re-enable \n");
        bu27030_init_client(als);
        i2c_smbus_write_byte_data(als->client, BU27030_REG_CONTROL, POWER_ON);
    }

    //BU27030_WARNING("Data valid BU27030_REG_CONTROL(0x%x) = 0x%x\n", BU27030_REG_CONTROL, result);
    if ((tmp & ALS_VALID_HIGH) == 0)//not valid
    {
        BU27030_WARNING("Data Not valid. But it does not matter, please ignore it.\n");
    }
    else
    {
        READ_DATA_ARG  data = {0};
        unsigned char  gain = 0;
        unsigned char  time = 0;

        //read data0
        tmp = i2c_smbus_read_word_data(client, BU27030_REG_DATA0);
        if (tmp < 0){
            BU27030_DBG("%s: i2c read data0 fail.\n", __func__);
            return ;
        }
        data.data0 = (unsigned int)tmp;

        //read data1
        tmp = i2c_smbus_read_word_data(client, BU27030_REG_DATA1);
        if (tmp < 0){
            BU27030_DBG("%s: i2c read data1 fail.\n", __func__);
            return ;
        }
        data.data1 = (unsigned int)tmp;


        //read gain
        tmp = i2c_smbus_read_byte_data(client, BU27030_REG_GAIN);
        if (tmp < 0){
            BU27030_DBG("%s: i2c read gain fail.\n", __func__);
            return ;
        }
        gain = (unsigned char)tmp;

        //read time
        tmp = i2c_smbus_read_byte_data(client, BU27030_REG_TIMING);
        if (tmp < 0){
            BU27030_DBG("%s: i2c read time fail.\n", __func__);
            return ;
        }
        time = (unsigned char)tmp;

        als->als_data = bu27030_calculate_light(data, gain, time);

        #if AGC_SUPPORT
            //auto change gain
            //Be noted: if agc was enabled, you must make sure that agc can not be happened on the first time.
            //Then you should set the default value carefullly in bu27030_init_client() to avoid data overflow on the first time.
            gain_changed  = bu27030_auto_change_gain0(als, data.data0);
            gain_changed |= bu27030_auto_change_gain1(als, data.data1);
        #endif

        if (als->als_data == 0){
            als->als_data++;
        }
        
        als->als_level = bu27030_als_data_to_level(als->als_data);

    }

    //do not report if gain is changed
    if(0 == gain_changed){
        input_report_abs(als->input_dev_als,  ABS_MISC, als->als_level);
        input_sync(als->input_dev_als);
    }
    
    schedule_delayed_work(&als->als_dwork, msecs_to_jiffies(als->als_poll_delay));

}




static int bu27030_enable_als_sensor(struct i2c_client *client, int val)
{
    BU27030_ALS_DATA *als = i2c_get_clientdata(client);

    int result = 0;

    if (!als || ( val != POWER_ON && val != POWER_OFF))
    {
        BU27030_ERR(" Parameter error \n");
        return EINVAL;
    }

    BU27030_WARNING(" val=%d\n", val);

    mutex_lock(&als->update_lock);

    if (val == POWER_ON) {
        if (als->enable_als_sensor == POWER_OFF) {
            
            result = i2c_smbus_write_byte_data(als->client, BU27030_REG_CONTROL, POWER_ON);
            if (result < 0) {
                // i2c communication error 
                BU27030_ERR(" I2C write error \n");
                mutex_unlock(&als->update_lock);  
                return (result);
            }
            als->enable_als_sensor = POWER_ON;
        }
        
        cancel_delayed_work(&als->als_dwork);
        schedule_delayed_work(&als->als_dwork, msecs_to_jiffies(als->als_poll_delay));
        
    } else {
        if (als->enable_als_sensor == POWER_ON) {
            result = i2c_smbus_write_byte_data(als->client, BU27030_REG_CONTROL, POWER_OFF);
            if (result < 0) {
                // i2c communication error 
                BU27030_ERR(" I2C write error \n");
                mutex_unlock(&als->update_lock);  
                return (result);
            }
            als->enable_als_sensor = POWER_OFF;
        }
        
        cancel_delayed_work(&als->als_dwork);
    }

    mutex_unlock(&als->update_lock);

    return result;
}

static ssize_t bu27030_show_enable_als_sensor(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA *als = i2c_get_clientdata(client);

    return snprintf(buf, PAGE_SIZE, "%d\n", als->enable_als_sensor);
}

static ssize_t bu27030_store_enable_als_sensor(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    /*unsigned long val = simple_strtoul(buf, NULL, 10);*/
    unsigned long val;
    int result = 0;

    result = kstrtoul(buf, 10, &val);
    if (result)
        return result;

    if ((val != POWER_ON) && (val != POWER_OFF)) {
        BU27030_DBG("%s: enable als sensor=%ld\n", __func__, val);
        return count;
    }

    bu27030_enable_als_sensor(client, val);

    return count;
}

#if QCOM_SENSORS

static int bu27030_set_als_poll_delay(struct i2c_client *client,
        unsigned int val)
{
    BU27030_ALS_DATA *als = i2c_get_clientdata(client);

    if (val < ALS_SET_MIN_DELAY_TIME * 1000)
        val = ALS_SET_MIN_DELAY_TIME * 1000;

    mutex_lock(&als->update_lock);

    als->als_poll_delay = val / 1000;    /* convert us => ms */

    if (als->enable_als_sensor == 1) {
        cancel_delayed_work(&als->als_dwork);
        schedule_delayed_work(&als->als_dwork,
                msecs_to_jiffies(als->als_poll_delay));
    }
    mutex_unlock(&als->update_lock);

    return 0;
}



static int bu27030_als_set_enable(struct sensors_classdev *sensors_cdev,
        unsigned int enable)
{
    BU27030_ALS_DATA *als = container_of(sensors_cdev,
            BU27030_ALS_DATA, als_cdev);

    if ((enable != POWER_ON) && (enable != POWER_OFF)) {
        pr_err("%s: invalid value(%d)\n", __func__, enable);
        return -EINVAL;
    }

    return bu27030_enable_als_sensor(als->client, enable);
}

static int bu27030_als_poll_delay(struct sensors_classdev *sensors_cdev,
        unsigned int delay_msec)
{
    BU27030_ALS_DATA *als = container_of(sensors_cdev,
            BU27030_ALS_DATA, als_cdev);
    bu27030_set_als_poll_delay(als->client, delay_msec);
    return 0;
}
#endif

/******************************************************************************/

static ssize_t bu27030_show_als_poll_delay(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA *als = i2c_get_clientdata(client);

    return snprintf(buf, PAGE_SIZE, "%d\n", als->als_poll_delay*1000);
}

static ssize_t bu27030_store_als_poll_delay(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA *als = i2c_get_clientdata(client);
    /*unsigned long val = simple_strtoul(buf, NULL, 10);*/
    unsigned long val;
    int error;

    error = kstrtoul(buf, 10, &val);
    if (error)
        return error;

    if (val < ALS_SET_MIN_DELAY_TIME * 1000)
        val = ALS_SET_MIN_DELAY_TIME * 1000;

    mutex_lock(&als->update_lock);

    als->als_poll_delay = val / 1000;

    if (als->enable_als_sensor == POWER_ON) {
        cancel_delayed_work(&als->als_dwork);
        schedule_delayed_work(&als->als_dwork, msecs_to_jiffies(als->als_poll_delay));
    }

    mutex_unlock(&als->update_lock);

    return count;
}

static ssize_t bu27030_show_als_data(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int data0;
    int data1;
    data0 = i2c_smbus_read_word_data(client, BU27030_REG_DATA0);
    data1 = i2c_smbus_read_word_data(client, BU27030_REG_DATA1);

    return snprintf(buf, PAGE_SIZE, "%d %d\n", data0, data1);
}

static ssize_t bu27030_show_type(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA *data = i2c_get_clientdata(client);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->dev_id);
}


static ssize_t bu27030_show_allreg(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA *data = i2c_get_clientdata(client);

    ssize_t len = 0;
    int i = 0;
    

    if(!client || !data){
        len += snprintf(buf+len, PAGE_SIZE-len, "obj is null!!\n");
        return len;
    }

    for (i=0; i<COEFFICIENT_SIZE; i++) {
        len += snprintf(buf+len, PAGE_SIZE-len, "data_coefficient[%i]= %d \n",i, data_coefficient[i]);
    }

    len += snprintf(buf+len, PAGE_SIZE-len,
            "You can read/write a register just like the follow:\n        read:  echo \"r 0x40     \" > reg\n        write: echo \"w 0x40 0xFF\" > reg\n        para:  echo \"para       \" > reg\n        (Use dmesg to see kernel log)\n\n");

    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_SYSTEM_CTL , bu27030_register_dump( data,  BU27030_REG_SYSTEM_CTL  ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_MODE_CTL1  , bu27030_register_dump( data,  BU27030_REG_MODE_CTL1   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_MODE_CTL2  , bu27030_register_dump( data,  BU27030_REG_MODE_CTL2   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_MODE_CTL3  , bu27030_register_dump( data,  BU27030_REG_MODE_CTL3   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_DATA0      , bu27030_register_dump( data,  BU27030_REG_DATA0       ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_DATA0 + 1  , bu27030_register_dump( data,  BU27030_REG_DATA0 + 1   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_DATA1      , bu27030_register_dump( data,  BU27030_REG_DATA1       ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_DATA1 + 1  , bu27030_register_dump( data,  BU27030_REG_DATA1 + 1   ));
    len += snprintf(buf+len, PAGE_SIZE-len, " 0x%02x 0x%02x\n", BU27030_REG_PART_ID    , bu27030_register_dump( data,  BU27030_REG_PART_ID     ));


    return len;
}


static ssize_t bu27030_store_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA *data = i2c_get_clientdata(client);

#define MAX_LENGTH (3)

    int reg , i2c_data;
    int i = 0;
    int ret = 0;

    char * str_dest[MAX_LENGTH] = {0};
    char str_src[128];

    char delims[] = " ";
    char *str_result = NULL;
    char *cur_str = str_src;

    if(!data){
        printk("obj is null !!!\n");
        return 0;
    }

    memcpy(str_src, buf, count);
    printk("Your input buf is: %s\n", str_src );

    //spilt buf by space(" "), and seperated string are saved in str_src[]
    while(( str_result = strsep( &cur_str, delims ))) {
        if( i < MAX_LENGTH){  //max length should be 3
            str_dest[i++] = str_result;
        }
        else{
            //printk("break\n");
            break;
        }
    }

    if (!strncmp(str_dest[0], "r", 1)){
        reg = simple_strtol(str_dest[1], NULL, 16);

        //check reg valid
        if(((reg&0xFF) > BU27030_REG_PART_ID ) || ((reg&0xFF) < BU27030_REG_SYSTEM_CTL )){
            printk("reg=0x%x is out of range !!!\n", reg );
            return -1;
        }

        //read i2c data
        bu27030_register_dump(data, reg&0xFF);
    }else if (!strncmp(str_dest[0], "w",  1)) {
        reg      = simple_strtol(str_dest[1], NULL, 16);
        i2c_data = simple_strtol(str_dest[2], NULL, 16);

        //check reg valid
        if(((reg&0xFF) > BU27030_REG_PART_ID) || ((reg&0xFF) < BU27030_REG_SYSTEM_CTL)){
            printk("reg=0x%x is out of range !!!\n", reg );
            return -1;
        }

        //write i2c data
        ret = i2c_smbus_write_byte_data(data->client, reg&0xFF, i2c_data&0xFF);
        if (ret < 0) {
            printk( " I2C read error !!!  \n" );
            return -1;
        }
        printk("writing...reg=0x%x, i2c_data=0x%x success\n", reg, i2c_data);
    }else if(!strncmp(str_dest[0], "para",  4)){  //print parameter
        int i;
        for(i=0;i < COEFFICIENT_SIZE; i++){
            printk ("data_coefficient[%d] = %5d \n ", i, data_coefficient[i]);
        }
    } else{
        printk("Please input right format: \"r 0x40\", \"w 0x40 0xFF\"\n");
    }

    printk( "bu27030_store_reg count=%d\n", (int)count);

    return count;
}


static DEVICE_ATTR(als_poll_delay, 0660, bu27030_show_als_poll_delay, bu27030_store_als_poll_delay);
static DEVICE_ATTR(enable, 0660, bu27030_show_enable_als_sensor, bu27030_store_enable_als_sensor);
static DEVICE_ATTR(als_data, S_IRUGO, bu27030_show_als_data, NULL);
static DEVICE_ATTR(type, S_IRUGO, bu27030_show_type, NULL);
static DEVICE_ATTR(reg, 0660, bu27030_show_allreg, bu27030_store_reg);

static struct attribute *bu27030_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_als_poll_delay.attr,
    &dev_attr_als_data.attr,
    &dev_attr_type.attr,
    &dev_attr_reg.attr,
    NULL
};

static const struct attribute_group bu27030_attr_group = {
    .attrs = bu27030_attributes,
};



static int bu27030_power_on(BU27030_ALS_DATA *data, bool on)
{
    int rc;

    if (!on)
        goto power_off;

    rc = regulator_enable(data->vdd);
    if (rc) {
        dev_err(&data->client->dev,
                "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }

    rc = regulator_enable(data->vio);
    if (rc) {
        dev_err(&data->client->dev,
                "Regulator vio enable failed rc=%d\n", rc);
        regulator_disable(data->vdd);
    }

    return rc;

power_off:
    rc = regulator_disable(data->vdd);
    if (rc) {
        dev_err(&data->client->dev,
                "Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }

    rc = regulator_disable(data->vio);
    if (rc) {
        dev_err(&data->client->dev,
                "Regulator vio disable failed rc=%d\n", rc);
    }

    return rc;
}
static int bu27030_power_init(BU27030_ALS_DATA *data, bool on)
{
    int rc;

    if (!on)
        goto pwr_deinit;

    data->vdd = regulator_get(&data->client->dev, "vdd");
    if (IS_ERR(data->vdd)) {
        rc = PTR_ERR(data->vdd);
        dev_err(&data->client->dev,
                "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(data->vdd) > 0) {
        rc = regulator_set_voltage(data->vdd, BU27030_VDD_MIN_UV,
                BU27030_VDD_MAX_UV);
        if (rc) {
            dev_err(&data->client->dev,
                    "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_put;
        }
    }

    data->vio = regulator_get(&data->client->dev, "vio");
    if (IS_ERR(data->vio)) {
        rc = PTR_ERR(data->vio);
        dev_err(&data->client->dev,
                "Regulator get failed vio rc=%d\n", rc);
        goto reg_vdd_set_vtg;
    }

    if (regulator_count_voltages(data->vio) > 0) {
        rc = regulator_set_voltage(data->vio, BU27030_VIO_MIN_UV,
                BU27030_VIO_MAX_UV);
        if (rc) {
            dev_err(&data->client->dev,
                    "Regulator set_vtg failed vio rc=%d\n", rc);
            goto reg_vio_put;
        }
    }

    return 0;

reg_vio_put:
    regulator_put(data->vio);
reg_vdd_set_vtg:
    if (regulator_count_voltages(data->vdd) > 0)
        regulator_set_voltage(data->vdd, 0, BU27030_VDD_MAX_UV);
reg_vdd_put:
    regulator_put(data->vdd);
    return rc;

pwr_deinit:
    if (regulator_count_voltages(data->vdd) > 0)
        regulator_set_voltage(data->vdd, 0, BU27030_VDD_MAX_UV);

    regulator_put(data->vdd);

    if (regulator_count_voltages(data->vio) > 0)
        regulator_set_voltage(data->vio, 0, BU27030_VIO_MAX_UV);

    regulator_put(data->vio);
    return 0;
}


static int bu27030_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
#define ROHM_ALS_MAX (65535)

    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    BU27030_ALS_DATA * als;

    int err = 0;
    int dev_id;
    BU27030_INFO("%s probe started.\n", __func__);

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
        err = -EIO;
        goto exit;
    }

    als = kzalloc(sizeof(BU27030_ALS_DATA), GFP_KERNEL);
    if (!als) {
        err = -ENOMEM;
        goto exit;
    }
    als->client = client;
    i2c_set_clientdata(client, als);

    err = bu27030_power_init(als, true);
    if (err)
        dev_err(&client->dev, "power init failed");

    if(!err) {
        err = bu27030_power_on(als, true);
        if (err)
            dev_err(&client->dev, "power on failed");
    }

    BU27030_INFO("enable = %x\n", als->enable);

    dev_id = i2c_smbus_read_byte_data(client, BU27030_REG_PART_ID);
    if (dev_id != BU27030_PART_ID_VALUE) {
        kfree(als);
        return -EPERM;
    }
    als->dev_id = dev_id;
    BU27030_INFO("%s: id(0x%x), this is bu27030!\n", __func__, dev_id);

    mutex_init(&als->update_lock);

    INIT_DELAYED_WORK(&als->als_dwork, bu27030_als_polling_work_handler);

    /* Initialize the bu27030 chip */
    err = bu27030_init_client(als);
    if (err)
        goto exit_kfree;

    als->als_poll_delay = ALS_SET_MIN_DELAY_TIME;

    /* Register to Input Device */
    als->input_dev_als = input_allocate_device();
    if (!als->input_dev_als) {
        err = -ENOMEM;
        BU27030_ERR("%s: Failed to allocate input device als\n", __func__);
        goto exit_kfree;
    }

    input_set_drvdata(als->input_dev_als, als);
    set_bit(EV_ABS, als->input_dev_als->evbit);
    input_set_abs_params(als->input_dev_als, ABS_MISC, 0, ROHM_ALS_MAX, 0, 0);

    als->input_dev_als->name = "lightsensor";
    als->input_dev_als->id.bustype = BUS_I2C;
    als->input_dev_als->dev.parent = &als->client->dev;

    err = input_register_device(als->input_dev_als);
    if (err) {
        err = -ENOMEM;
        BU27030_ERR("%s:register input device als fail: %s\n", __func__,
                als->input_dev_als->name);
        goto exit_free_dev_als;
    }


    /* Register sysfs hooks */
    err = sysfs_create_group(&client->dev.kobj, &bu27030_attr_group);
    if (err) {
        BU27030_ERR("%s sysfs_create_groupX\n", __func__);
        goto exit_unregister_dev_als;
    }

    BU27030_INFO("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);

#if QCOM_SENSORS

    /* Register to sensors class for qcom  */
    als->als_cdev = sensors_light_cdev;
    als->als_cdev.sensors_enable = bu27030_als_set_enable;
    als->als_cdev.sensors_poll_delay = bu27030_als_poll_delay;

    err = sensors_classdev_register(&client->dev, &als->als_cdev);
    if (err) {
        pr_err("%s: Unable to register to sensors class: %d\n",
                __func__, err);
        goto exit_remove_sysfs_group;
    }
#endif

    BU27030_INFO("%s(): sizeof(int)=%d, sizeof(long)=%d, sizeof(long long)=%d\n",
        __FUNCTION__, sizeof(int), sizeof(long), sizeof(long long));
    
    BU27030_INFO("bu27030 probe success!");
    
    return 0;
    
#if QCOM_SENSORS
exit_create_class_sysfs:
    sensors_classdev_unregister(&als->als_cdev);
exit_remove_sysfs_group:
#endif

    sysfs_remove_group(&client->dev.kobj, &bu27030_attr_group);
exit_unregister_dev_als:
    input_unregister_device(als->input_dev_als);
exit_free_dev_als:
    input_free_device(als->input_dev_als);
exit_kfree:
    kfree(als);
exit:
    BU27030_ERR("%s : bu27030 probe failed\n", __func__);
    return err;

#undef ROHM_ALS_MAX
}

static int bu27030_remove(struct i2c_client *client)
{
    BU27030_ALS_DATA *als = i2c_get_clientdata(client);

#if QCOM_SENSORS
    sensors_classdev_unregister(&als->als_cdev);
#endif

    cancel_delayed_work(&als->als_dwork);

    input_unregister_device(als->input_dev_als);

    input_free_device(als->input_dev_als);

    sysfs_remove_group(&client->dev.kobj, &bu27030_attr_group);

    /* Power down the device */
    bu27030_enable_als_sensor(client, POWER_OFF);

    kfree(als);

    return 0;
}

static int bu27030_suspend(struct device *dev, pm_message_t mesg)
{
    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA * als = i2c_get_clientdata(client);
    int ret = 0;

    BU27030_INFO("%s\n", __func__);


    if(als->enable_als_sensor == POWER_ON){
        als->als_suspend_flag = 1;
    }
    
    ret =  bu27030_enable_als_sensor(client, POWER_OFF);
    
    return ret;
}

static int bu27030_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    BU27030_ALS_DATA * als = i2c_get_clientdata(client);

    int ret = 0;
    
    BU27030_INFO("%s\n", __func__);
    
    if(als->als_suspend_flag == 1){
        als->als_suspend_flag = 0;
    }
    
    ret = bu27030_enable_als_sensor(client, POWER_ON);

    return ret; 
}


MODULE_DEVICE_TABLE(i2c, bu27030_id);

static const struct i2c_device_id bu27030_id[] = {
    { "bu27030", 0 },
    { }
};

#ifdef CONFIG_OF
static struct of_device_id bu27030_match_table[] = {
    { .compatible = "rohm,bu27030",},
    { },
};
#else
#define bu27030_match_table NULL
#endif

static struct i2c_driver bu27030_driver = {
    .driver = {
        .suspend = bu27030_suspend,
        .resume  = bu27030_resume,
        .name    = BU27030_DRV_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = bu27030_match_table,
    },

    .probe    = bu27030_probe,
    .remove    = bu27030_remove,
    .id_table = bu27030_id,
};

static int __init bu27030_init(void)
{
    struct device_node *np;
    
    printk("aaron %s-line%d\n", __FUNCTION__, __LINE__);
    
    np = of_find_node_by_name(NULL, "rohm,bu27030");
    if (np)
        printk("bu27030 node found...\n");
    else
        printk("bu27030 node not found...\n");

    
    return i2c_add_driver(&bu27030_driver);
}

static void __exit bu27030_exit(void)
{
    i2c_del_driver(&bu27030_driver);
}

MODULE_AUTHOR("aaron_liu@rohm.com.cn");
MODULE_DESCRIPTION("bu27030 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(bu27030_init);
module_exit(bu27030_exit);
