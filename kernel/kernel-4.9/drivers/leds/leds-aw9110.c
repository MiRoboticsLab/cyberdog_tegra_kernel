/*
 * leds-aw9110.c   aw9110 led module
 *
 * Version: 1.0.0
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/leds.h>
#include <linux/leds-aw9110.h>
/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW9110_I2C_NAME "aw9110_led"

#define AW9110_VERSION "v1.0.0"

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5

#define REG_INPUT_P0        0x00
#define REG_INPUT_P1        0x01
#define REG_OUTPUT_P0       0x02
#define REG_OUTPUT_P1       0x03
#define REG_CONFIG_P0       0x04
#define REG_CONFIG_P1       0x05
#define REG_INT_P0          0x06
#define REG_INT_P1          0x07
#define REG_ID              0x10
#define REG_CTRL            0x11
#define REG_WORK_MODE_P0    0x12
#define REG_WORK_MODE_P1    0x13
#define REG_EN_BREATH       0x14
#define REG_FADE_TIME       0x15
#define REG_FULL_TIME       0x16
#define REG_DLY0_BREATH     0x17
#define REG_DLY1_BREATH     0x18
#define REG_DLY2_BREATH     0x19
#define REG_DLY3_BREATH     0x1a
#define REG_DLY4_BREATH     0x1b
#define REG_DLY5_BREATH     0x1c
#define REG_DIM00           0x20
#define REG_DIM01           0x21
#define REG_DIM02           0x22
#define REG_DIM03           0x23
#define REG_DIM04           0x24
#define REG_DIM05           0x25
#define REG_DIM06           0x26
#define REG_DIM07           0x27
#define REG_DIM08           0x28
#define REG_DIM09           0x29
#define REG_SWRST           0x7F


/* aw9110 register read/write access*/
#define REG_NONE_ACCESS                 0
#define REG_RD_ACCESS                   1 << 0
#define REG_WR_ACCESS                   1 << 1
#define AW9110_REG_MAX                  0xFF

const unsigned char aw9110_reg_access[AW9110_REG_MAX] = {
  [REG_INPUT_P0    ] = REG_RD_ACCESS,
  [REG_INPUT_P1    ] = REG_RD_ACCESS,
  [REG_OUTPUT_P0   ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_OUTPUT_P1   ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_CONFIG_P0   ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_CONFIG_P1   ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_INT_P0      ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_INT_P1      ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_ID          ] = REG_RD_ACCESS,
  [REG_CTRL        ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_WORK_MODE_P0] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_WORK_MODE_P1] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_EN_BREATH   ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_FADE_TIME   ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_FULL_TIME   ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_DLY0_BREATH ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_DLY1_BREATH ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_DLY2_BREATH ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_DLY3_BREATH ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_DLY4_BREATH ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_DLY5_BREATH ] = REG_RD_ACCESS|REG_WR_ACCESS,
  [REG_DIM00       ] = REG_WR_ACCESS,
  [REG_DIM01       ] = REG_WR_ACCESS,
  [REG_DIM02       ] = REG_WR_ACCESS,
  [REG_DIM03       ] = REG_WR_ACCESS,
  [REG_DIM04       ] = REG_WR_ACCESS,
  [REG_DIM05       ] = REG_WR_ACCESS,
  [REG_DIM06       ] = REG_WR_ACCESS,
  [REG_DIM07       ] = REG_WR_ACCESS,
  [REG_DIM08       ] = REG_WR_ACCESS,
  [REG_DIM09       ] = REG_WR_ACCESS,
  [REG_SWRST       ] = REG_WR_ACCESS,
};


/******************************************************
 *
 * aw9110 i2c write/read
 *
 ******************************************************/
static int aw9110_i2c_write(struct aw9110 *aw9110, 
         unsigned char reg_addr, unsigned char reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_smbus_write_byte_data(aw9110->i2c, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }

    return ret;
}

static int aw9110_i2c_read(struct aw9110 *aw9110, 
        unsigned char reg_addr, unsigned char *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_smbus_read_byte_data(aw9110->i2c, reg_addr);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            *reg_data = ret;
            break;
        }
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }

    return ret;
}

/******************************************************
 *
 * aw9110 i2c write bits
 *
static int aw9110_i2c_write_bits(struct aw9110 *aw9110, 
         unsigned char reg_addr, unsigned char mask, unsigned char reg_data)
{
    unsigned char reg_val;

    aw9110_i2c_read(aw9110, reg_addr, &reg_val);
    reg_val &= mask;
    reg_val |= reg_data;
    aw9110_i2c_write(aw9110, reg_addr, reg_val);

    return 0;
}
 ******************************************************/

/******************************************************
 *
 * aw9110 led
 *
 ******************************************************/
static void aw9110_brightness_work(struct work_struct *work)
{
    struct aw9110 *aw9110 = container_of(work, struct aw9110,
          brightness_work);

    unsigned char i;

    if(aw9110->cdev.brightness > aw9110->cdev.max_brightness) {
        aw9110->cdev.brightness = aw9110->cdev.max_brightness;
    }

    aw9110_i2c_write(aw9110, REG_WORK_MODE_P0, 0x00);   // led mode
    aw9110_i2c_write(aw9110, REG_WORK_MODE_P1, 0x00);   // led mode

    aw9110_i2c_write(aw9110, REG_EN_BREATH, 0x00);      // disable breath

    aw9110_i2c_write(aw9110, REG_CTRL, 0x03);           // imax

    for(i=0; i<10; i++) {
        aw9110_i2c_write(aw9110, REG_DIM00+i,
            aw9110->cdev.brightness);                   // dimming
    }
}

static void aw9110_set_brightness(struct led_classdev *cdev,
           enum led_brightness brightness)
{
    struct aw9110 *aw9110 = container_of(cdev, struct aw9110, cdev);

    aw9110->cdev.brightness = brightness;

    schedule_work(&aw9110->brightness_work);
}

static void aw9110_led_blink(struct aw9110 *aw9110, unsigned char blink)
{
    unsigned char i;

    if(aw9110->cdev.brightness > aw9110->cdev.max_brightness) {
        aw9110->cdev.brightness = aw9110->cdev.max_brightness;
    }

    if(blink) {
        aw9110_i2c_write(aw9110, REG_WORK_MODE_P0, 0x00);   // led mode
        aw9110_i2c_write(aw9110, REG_WORK_MODE_P1, 0x00);   // led mode

        aw9110_i2c_write(aw9110, REG_EN_BREATH, 0x3f);      // enable breath

        aw9110_i2c_write(aw9110, REG_CONFIG_P0, 0x03);      // blink mode
        aw9110_i2c_write(aw9110, REG_CONFIG_P1, 0x0f);      // blink mode

        aw9110_i2c_write(aw9110, REG_FADE_TIME,
            (aw9110->fall_time<<3)|(aw9110->rise_time));    // fade time
        aw9110_i2c_write(aw9110, REG_FULL_TIME,
            (aw9110->off_time<<3)|(aw9110->on_time));       // on/off time

        for(i=0; i<6; i++) {
            aw9110_i2c_write(aw9110, REG_DIM00+i,
                aw9110->cdev.brightness);                   // dimming
        }

        aw9110_i2c_write(aw9110, REG_CTRL,
            0x80 | aw9110->imax);                           // blink enable | imax
    } else {
        aw9110_i2c_write(aw9110, REG_WORK_MODE_P0, 0x00);   // led mode
        aw9110_i2c_write(aw9110, REG_WORK_MODE_P1, 0x00);   // led mode

        aw9110_i2c_write(aw9110, REG_EN_BREATH, 0x00);      // disable breath

        aw9110_i2c_write(aw9110, REG_CTRL, 0x03);           // imax

        for(i=0; i<10; i++) {
            aw9110_i2c_write(aw9110, REG_DIM00+i, 0x00);    // dimming
        }

    }
}



/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw9110_parse_dt(struct device *dev, struct aw9110 *aw9110,
        struct device_node *np)
{
    aw9110->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw9110->reset_gpio < 0) {
        dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
        return -1;
    } else {
        dev_info(dev, "%s: reset gpio provided ok, reset_gpio:%d\n", __func__, aw9110->reset_gpio);
    }

    return 0;
}

static int aw9110_hw_reset(struct aw9110 *aw9110)
{
    pr_info("%s enter\n", __func__);

    if (aw9110 && gpio_is_valid(aw9110->reset_gpio)) {
        gpio_set_value_cansleep(aw9110->reset_gpio, 0);
        msleep(1);
        gpio_set_value_cansleep(aw9110->reset_gpio, 1);
        msleep(1);
    } else {
        dev_err(aw9110->dev, "%s:  failed\n", __func__);
    }
    return 0;
}

static int aw9110_hw_off(struct aw9110 *aw9110)
{
    pr_info("%s enter\n", __func__);

    if (aw9110 && gpio_is_valid(aw9110->reset_gpio)) {
        gpio_set_value_cansleep(aw9110->reset_gpio, 0);
        msleep(1);
    } else {
        dev_err(aw9110->dev, "%s:  failed\n", __func__);
    }
    return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw9110_read_chipid(struct aw9110 *aw9110)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char reg_val = 0;
  
    while(cnt < AW_READ_CHIPID_RETRIES) {
        ret = aw9110_i2c_read(aw9110, REG_ID, &reg_val);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "%s: failed to read register AW9110_REG_ID: %d\n",
                __func__, ret);
            return -EIO;
        }
        switch (reg_val) {
        case AW9110_ID:
            pr_info("%s aw9110 detected\n", __func__);
            aw9110->chipid = AW9110_ID;
            return 0;
        default:
            pr_info("%s unsupported device revision (0x%x)\n",
                __func__, reg_val );
            break;
        }
        cnt ++;

        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }

    return -EINVAL;
}


/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw9110_reg_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw9110 *aw9110 = container_of(led_cdev, struct aw9110, cdev);

    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw9110_i2c_write(aw9110, (unsigned char)databuf[0], (unsigned char)databuf[1]);
    }

    return count;
}

static ssize_t aw9110_reg_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw9110 *aw9110 = container_of(led_cdev, struct aw9110, cdev);
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    for(i = 0; i < AW9110_REG_MAX; i ++) {
        if(!(aw9110_reg_access[i]&REG_RD_ACCESS))
           continue;
        aw9110_i2c_read(aw9110, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}

static ssize_t aw9110_hwen_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw9110 *aw9110 = container_of(led_cdev, struct aw9110, cdev);

    unsigned int databuf[1] = {0};

    if(1 == sscanf(buf, "%x", &databuf[0])) {
        if(1 == databuf[0]) {
            aw9110_hw_reset(aw9110);
        } else {
            aw9110_hw_off(aw9110);
        }
    }

    return count;
}

static ssize_t aw9110_hwen_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw9110 *aw9110 = container_of(led_cdev, struct aw9110, cdev);
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "hwen=%d\n",
            gpio_get_value(aw9110->reset_gpio));

    return len;
}

static ssize_t aw9110_blink_store(struct device* dev, struct device_attribute *attr,
                const char* buf, size_t len)
{
    unsigned int databuf[1];
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct aw9110 *aw9110 = container_of(led_cdev, struct aw9110, cdev);

    sscanf(buf,"%d",&databuf[0]);
    aw9110_led_blink(aw9110, databuf[0]);

    return len;
}

static ssize_t aw9110_blink_show(struct device* dev,struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "aw9110_blink()\n");
    len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > blink\n");
    len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 > blink\n");

    return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw9110_reg_show, aw9110_reg_store);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO, aw9110_hwen_show, aw9110_hwen_store);
static DEVICE_ATTR(blink, S_IWUSR | S_IRUGO, aw9110_blink_show, aw9110_blink_store);

static struct attribute *aw9110_attributes[] = {
    &dev_attr_reg.attr,
    &dev_attr_hwen.attr,
    &dev_attr_blink.attr,
    NULL
};

static struct attribute_group aw9110_attribute_group = {
    .attrs = aw9110_attributes
};


/******************************************************
 *
 * led class dev
 *
 ******************************************************/
static int aw9110_parse_led_cdev(struct aw9110 *aw9110,
        struct device_node *np)
{
    struct device_node *temp;
    int ret = -1;

    for_each_child_of_node(np, temp) {
        ret = of_property_read_string(temp, "aw9110,name",
            &aw9110->cdev.name);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading led name, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw9110,imax",
            &aw9110->imax);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading imax, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw9110,brightness",
            &aw9110->cdev.brightness);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading brightness, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw9110,max_brightness",
            &aw9110->cdev.max_brightness);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading max brightness, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw9110,rise_time",
            &aw9110->rise_time);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading rise_time, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw9110,on_time",
            &aw9110->on_time);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading on_time, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw9110,fall_time",
            &aw9110->fall_time);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading fall_time, ret = %d\n", ret);
            goto free_pdata;
        }
        ret = of_property_read_u32(temp, "aw9110,off_time",
            &aw9110->off_time);
        if (ret < 0) {
            dev_err(aw9110->dev,
                "Failure reading off_time, ret = %d\n", ret);
            goto free_pdata;
        }
    }

    INIT_WORK(&aw9110->brightness_work, aw9110_brightness_work);

    aw9110->cdev.brightness_set = aw9110_set_brightness;
    ret = led_classdev_register(aw9110->dev, &aw9110->cdev);
    if (ret) {
        dev_err(aw9110->dev,
            "unable to register led ret=%d\n", ret);
        goto free_pdata;
    }

    ret = sysfs_create_group(&aw9110->cdev.dev->kobj,
            &aw9110_attribute_group);
    if (ret) {
        dev_err(aw9110->dev, "led sysfs ret: %d\n", ret);
        goto free_class;
    }

    return 0;

free_class:
    led_classdev_unregister(&aw9110->cdev);
free_pdata:
    return ret;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw9110_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct aw9110 *aw9110;
    struct device_node *np = i2c->dev.of_node;
    int ret;

    pr_info("%s enter\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    aw9110 = devm_kzalloc(&i2c->dev, sizeof(struct aw9110), GFP_KERNEL);
    if (aw9110 == NULL)
        return -ENOMEM;

    aw9110->dev = &i2c->dev;
    aw9110->i2c = i2c;

    i2c_set_clientdata(i2c, aw9110);

    /* aw9110 rst & int */
    if (np) {
        ret = aw9110_parse_dt(&i2c->dev, aw9110, np);
        if (ret) {
            dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
            goto err;
        }
    } else {
        aw9110->reset_gpio = -1;
    }

    if (gpio_is_valid(aw9110->reset_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw9110->reset_gpio,
            GPIOF_OUT_INIT_LOW, "aw9110_rst");
        if (ret){
            dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
            goto err;
        }
    }

    /* hardware reset */
    aw9110_hw_reset(aw9110);

    /* aw9110 chip id */
    ret = aw9110_read_chipid(aw9110);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw9110_read_chipid failed ret=%d\n", __func__, ret);
        goto err_id;
    }

    dev_set_drvdata(&i2c->dev, aw9110);

    aw9110_parse_led_cdev(aw9110, np);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s error creating led class dev\n", __func__);
        goto err_sysfs;
    }

    pr_info("%s probe completed successfully!\n", __func__);

    return 0;

err_sysfs:
err_id:
err:
    return ret;
}

static int aw9110_i2c_remove(struct i2c_client *i2c)
{
    struct aw9110 *aw9110 = i2c_get_clientdata(i2c);

    pr_info("%s enter\n", __func__);

    if (gpio_is_valid(aw9110->reset_gpio))
        devm_gpio_free(&i2c->dev, aw9110->reset_gpio);

    return 0;
}

static const struct i2c_device_id aw9110_i2c_id[] = {
    { AW9110_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw9110_i2c_id);

static struct of_device_id aw9110_dt_match[] = {
    { .compatible = "awinic,aw9110_led" },
    { },
};

static struct i2c_driver aw9110_i2c_driver = {
    .driver = {
        .name = AW9110_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw9110_dt_match),
    },
    .probe = aw9110_i2c_probe,
    .remove = aw9110_i2c_remove,
    .id_table = aw9110_i2c_id,
};


static int __init aw9110_i2c_init(void)
{
    int ret = 0;

    pr_info("aw9110 driver version %s\n", AW9110_VERSION);

    ret = i2c_add_driver(&aw9110_i2c_driver);
    if(ret){
        pr_err("fail to add aw9110 device into i2c\n");
        return ret;
    }

    return 0;
}
module_init(aw9110_i2c_init);


static void __exit aw9110_i2c_exit(void)
{
    i2c_del_driver(&aw9110_i2c_driver);
}
module_exit(aw9110_i2c_exit);


MODULE_DESCRIPTION("AW9110 LED Driver");
MODULE_LICENSE("GPL v2");
