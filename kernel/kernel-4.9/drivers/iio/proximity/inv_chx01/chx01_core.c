// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 InvenSense, Inc.
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

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/timekeeping.h>

#include <linux/iio/iio.h>

#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>

#include <asm/delay.h>

#include "chx01_client.h"
#include "chx01_data.h"
#include "src/ch101_reg.h"
#include "src/chirp_hal.h"
#include "src/i2c_hal.h"
#include "src/chbsp_init.h"

#include "src/init_driver.h"


#define CHx01_GPIO_RESET	"rst"
#define CHx01_GPIO_PROG		"prg"
#define CHx01_GPIO_CAL		"cal"
#define CHx01_GPIO_INT		"int"

#define CH101_IRQ_NAME		"ch101_event"
#define CHx01_MIN_FREQ_HZ			1
#define CHx01_MAX_FREQ_HZ			10
#define CHx01_DEFAULT_SAMPLING_PERIOD_NS	(NSEC_PER_SEC / 10)  /* 100ms */

static const struct iio_event_spec chx01_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

#define CH101_CHANNEL(idx)					\
{								\
	.type = IIO_PROXIMITY,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.indexed = 1,						\
	.channel = idx,						\
	.event_spec = chx01_events,				\
	.num_event_specs = ARRAY_SIZE(chx01_events),		\
	.scan_index = idx,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.shift = 0,					\
	},							\
}


enum chx01_channel_index {
	CHx01_CHANNEL_PRESSURE,
	CHx01_CHANNEL_DISTANCE,
	CH101_CHANNEL_TIMESTAMP,
};

static const struct iio_chan_spec chx01_channels[] = {

	{
		.type = IIO_PROXIMITY,					
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		
		// .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET) | BIT(IIO_CHAN_INFO_SCALE),		
		// .indexed = 1,						
		.channel = CHx01_CHANNEL_PRESSURE,						
		.event_spec = chx01_events,				
		.num_event_specs = ARRAY_SIZE(chx01_events),		
		.scan_index = CHx01_CHANNEL_PRESSURE,					
		.scan_type = {						
			.sign = 'u',					
			.realbits = 16,					
			.storagebits = 16,				
			.shift = 0,					
		},	
	},						

	{
		.type = IIO_DISTANCE,
		// .info_mask_separate =   BIT(IIO_CHAN_INFO_RAW) |
		// 			BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_separate =   BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET) | BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = CHx01_CHANNEL_DISTANCE,
		.channel = CHx01_CHANNEL_DISTANCE,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(CH101_CHANNEL_TIMESTAMP),
};
extern struct ch_group_t chirp_group;

static int chx01_read_range_data(struct chx01_data *data, u32 *range)
{
	struct ch_group_t *grp_ptr = &chirp_group;
	struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, 0);
 	u32 raw_range;	

	// int ret; 
	// int i;
	// struct chx01_buffer *buffer = &data->buffer;

	dbg_info("<<chx01>>%s:\n", __func__);
	// ret = wait_for_completion_interruptible_timeout(&data->completion,
			// 1 * HZ);
	wait_for_completion(&data->completion);
	// if (ret != -ERESTARTSYS)
	// 	dbg_info("<<chx01>>%s: timeout\n", __func__);

	ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY, &raw_range);
	dbg_info("<<chx01>>%s: raw_range = %u\n", __func__, raw_range);	
	
	*range = raw_range /32;
 
	reinit_completion(&data->completion);

	return IIO_VAL_INT;
}
static int chx01_read_amplitude_data(struct chx01_data *data, u16 *amplitude)
{
	struct ch_group_t *grp_ptr = &chirp_group;
	struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, 0);

	// int ret; 
	// int i;
	// struct chx01_buffer *buffer = &data->buffer;

	dbg_info("<<chx01>>%s:\n", __func__);
	// ret = wait_for_completion_interruptible_timeout(&data->completion,
			// 1 * HZ);
	wait_for_completion(&data->completion);
	// if (ret != -ERESTARTSYS)
	// 	dbg_info("<<chx01>>%s: timeout\n", __func__);

	// ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY, &raw_range);
	ch_get_amplitude(dev_ptr, amplitude);
	dbg_info("<<chx01>>%s: amplitude = %d\n", __func__, *amplitude);	
	
	// *amplitude = raw_range /32;

 
	reinit_completion(&data->completion);

	return IIO_VAL_INT;
}


static int chx01_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	struct chx01_data *data = iio_priv(indio_dev);
	// struct device *dev_ptr = data->dev;
	// struct ch_group_t *grp_ptr = &chirp_group;
	// struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, 0);

	int ret;
	// struct ch_group_t *grp_ptr = &chirp_group;
	// struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, 0);
 	u32 range;	
	u16 amplitude;

	dbg_info("<<chx01>>%s: channel: %d, type: %d, mask: %lu\n", __func__, chan->channel, chan->type, mask);

	switch (chan->type) {
	case IIO_PROXIMITY:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
	dbg_info("<<chx01>>%s: 	IIO_CHAN_INFO_RAW..IIO_PROXIMITY..1\n", __func__);
			mutex_lock(&indio_dev->mlock);
			// mutex_lock(&data->lock);
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto out_info_raw;
			}
	dbg_info("<<chx01>>%s: 	IIO_CHAN_INFO_RAW..IIO_PROXIMITY..2\n", __func__);
			chx01_read_amplitude_data(data, &amplitude);
			// ch_get_amplitude(dev_ptr, &amplitude);

			// *val = 100;
			*val = amplitude;
			*val2 = 0;
			ret = IIO_VAL_INT;
out_info_raw:
	dbg_info("<<chx01>>%s: 	IIO_CHAN_INFO_RAW..IIO_PROXIMITY..3\n", __func__);
			mutex_unlock(&indio_dev->mlock);	
	dbg_info("<<chx01>>%s: 	IIO_CHAN_INFO_RAW..IIO_PROXIMITY..4\n", __func__);		
			// mutex_lock(&data->lock);		
			return ret;

		default:
			return -EINVAL;
		}

		case IIO_DISTANCE:
			switch (mask) {
			case IIO_CHAN_INFO_RAW:
	dbg_info("<<chx01>>%s: 	IIO_CHAN_INFO_RAW..IIO_DISTANCE..1\n", __func__);		
				// *val = data->buffer.distance;
			mutex_lock(&indio_dev->mlock);
			// mutex_lock(&data->lock);
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto out_info_raw_1;
			}				
				chx01_read_range_data(data, &range);
				// ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY, &range);
				dbg_info("<<chx01>>%s: range = %u\n", __func__, range);			
				// *val = 1000;
				*val = range;
				*val2 = 0;
out_info_raw_1:
	dbg_info("<<chx01>>%s: 	IIO_CHAN_INFO_RAW..IIO_DISTANCE..2\n", __func__);
			mutex_unlock(&indio_dev->mlock);
	dbg_info("<<chx01>>%s: 	IIO_CHAN_INFO_RAW..IIO_DISTANCE..3\n", __func__);		
				return IIO_VAL_INT;
			case IIO_CHAN_INFO_SCALE:
				*val = 32;
				*val2 = 0;
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}

	default:
		return -EINVAL;
	}
}


enum chx01_attributes {
	CHx01_ATTR_SAMP_AVAIL_FREQ,
	CHx01_ATTR_SAMP_FREQ,
};

static int chx01_validate_trigger(struct iio_dev *indio_dev,
				    struct iio_trigger *trig)
{
	struct chx01_data *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static ssize_t chx01_attr_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct chx01_data *st = iio_priv(dev_to_iio_dev(dev));
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	ktime_t period;
	uint32_t period_us;
	unsigned long flags;
dbg_info("<<chx01>>%s: this_attr->address: %d\n", __func__, this_attr->address);

	switch (this_attr->address) {
		case CHx01_ATTR_SAMP_AVAIL_FREQ:
			return scnprintf(buf, PAGE_SIZE, "%u--%u Hz\n", CHx01_MIN_FREQ_HZ,
					CHx01_MAX_FREQ_HZ);
		case CHx01_ATTR_SAMP_FREQ:
			spin_lock_irqsave(&st->period_lock, flags);
			period = st->period;
			spin_unlock_irqrestore(&st->period_lock, flags);
			period_us = div_u64(ktime_to_ns(period), 1000UL);
			return scnprintf(buf, PAGE_SIZE, "%lu\n", USEC_PER_SEC / period_us);

		default:
			return -EINVAL;
	}
}

static ssize_t chx01_attr_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct chx01_data *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	// unsigned val;
	// int64_t min_period;
	// int ret;
	unsigned long flags;
	int32_t sampling_frequency;
	ktime_t period;
	// unsigned long flags;	
dbg_info("<<chx01>>%s: this_attr->address: %d\n", __func__, this_attr->address);

	switch (this_attr->address) {
	case CHx01_ATTR_SAMP_FREQ:
		if (kstrtoint(buf, 10, &sampling_frequency))
			return -EINVAL;
		dbg_info("<<chx01>>%s: sampling_frequency(to set): %d\n", __func__, sampling_frequency);
		// if (sampling_frequency < CHx01_Min_freq || sampling_frequency > CHx01_Max_freq)
		if (sampling_frequency < CHx01_MIN_FREQ_HZ || sampling_frequency > CHx01_MAX_FREQ_HZ)
			return -EINVAL;

		period = ns_to_ktime(div_u64(NSEC_PER_SEC, sampling_frequency));
		mutex_lock(&indio_dev->mlock);
		if (!iio_buffer_enabled(indio_dev)) {		
			spin_lock_irqsave(&st->period_lock, flags);
			st->period = period;
			spin_unlock_irqrestore(&st->period_lock, flags);
		}
		mutex_unlock(&indio_dev->mlock);
		return count;
	default:
		return -EINVAL;
	}

	// return count;
}


static IIO_DEVICE_ATTR(sampling_frequency, S_IRUGO | S_IWUSR, chx01_attr_show,
		       chx01_attr_store, CHx01_ATTR_SAMP_FREQ);
			   
static IIO_DEVICE_ATTR(sampling_frequency_available, S_IRUGO, chx01_attr_show,
		       NULL, CHx01_ATTR_SAMP_AVAIL_FREQ);
static struct attribute *chx01_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	// &iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};


static const struct attribute_group chx01_attribute_group = {
	.name = "chx01_attr",
	.attrs = chx01_attributes
};

static const struct iio_info chx01_info = {
	.attrs = &chx01_attribute_group,
	.read_raw = &chx01_read_raw,
	// .write_raw = &chx01_write_raw,
	.validate_trigger = chx01_validate_trigger,
};

static irqreturn_t chx01_irq_handler(int irq, void *private)
{
	
	struct iio_dev *indio_dev = private;
	struct chx01_data *data = iio_priv(indio_dev);
	// struct device *dev = data->dev;

	if (data->trigger_enabled)
		iio_trigger_poll(data->trig);

	return IRQ_WAKE_THREAD;
}


unsigned long long systime_us;

extern struct chirp_data_t chirp_data[1];

extern struct ch_group_t chirp_group;

static irqreturn_t chx01_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct chx01_data *data = iio_priv(indio_dev);

	if((jiffies_to_usecs(jiffies) - systime_us) < 1000) 
		return IRQ_HANDLED; //fake INT (caused by Chirp hw triggering pulse)


	// mutex_lock(&indio_dev->mlock);

	complete(&data->completion);

	// mutex_lock(&indio_dev->mlock);

	return IRQ_HANDLED;
}


static void complete_done(struct chx01_client *data)
{
	struct chx01_data *data_drv;

	data_drv = container_of(data, struct chx01_data, client);
	complete(&data_drv->completion);
}


static enum hrtimer_restart chx01_hrtimer_handler(struct hrtimer *t)
{
	struct chx01_data *data;
	//dbg_info("<<chx01>>%s...1\n", __func__);
	data = container_of(t, struct chx01_data, timer);

	systime_us = jiffies_to_usecs(jiffies);

	ioport_set_pin_level_0(CHIRP0_CAL_0, IOPORT_PIN_LEVEL_HIGH); //reset high
	udelay(5); //Chirp hw triggering pulse --yd
	ioport_set_pin_level_0(CHIRP0_CAL_0, IOPORT_PIN_LEVEL_LOW);
	udelay(10);
	

	hrtimer_forward_now(t, data->period);
	//dbg_info("<<chx01>>%s...2\n", __func__);
	// iio_trigger_poll(data->trig); //debug--yd
	//dbg_info("<<chx01>>%s...3\n", __func__);
	return HRTIMER_RESTART;
}

static int chx01_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct chx01_data *st = iio_priv(indio_dev);
	struct device *dev = st->dev;
#if 0	
	ktime_t period;
	unsigned long flags;

	if (state) {
		spin_lock_irqsave(&st->period_lock, flags);
		period = st->period;
		spin_unlock_irqrestore(&st->period_lock, flags);
		hrtimer_start(&st->timer, period, HRTIMER_MODE_REL);
	} else {
		hrtimer_cancel(&st->timer);
	}
#endif	
	dev_info(dev, "%s: state: %d\n", __func__, state);

	st->trigger_enabled = state;
	return 0;
}

static const struct iio_trigger_ops chx01_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = chx01_trig_set_state,
};

static inline ktime_t chx01_get_time(void)
{
	ktime_t ts;

#ifdef CONFIG_RTC_INTF_ALARM
	/* Workaround for some platform on which monotonic clock and
	 * Android SystemClock has a gap.
	 */
	ts = alarm_get_elapsed_realtime();
#else
	ts = ktime_get_boottime();
#endif
	return ts;
}

static irqreturn_t chx01_trigger_handler(int irq, void *handle)
{
	struct iio_poll_func *pf = handle;
	struct iio_dev *indio_dev = pf->indio_dev;
	// struct chx01_data *data = iio_priv(indio_dev);
	int ret;

	struct {
		u32 distance_measure;
		u32 amplitude_measure;
		int64_t ts;
		} __packed buff_data;

	u32 range;	
	u16 amplitude;	
	struct ch_group_t *grp_ptr = &chirp_group;
	struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, 0);


	// dbg_info("<<chx01>>%s\n", __func__);

	// mutex_lock(&data->lock);
	mutex_lock(&indio_dev->mlock);


	ch_get_amplitude(dev_ptr, &amplitude);
	ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY, &range);

	buff_data.distance_measure = range;
	buff_data.amplitude_measure = (u32)amplitude;

	//buff_data.ts = ktime_to_ns(chx01_get_time()); //iio_get_time_ns
	buff_data.ts = iio_get_time_ns(indio_dev);

	ret = iio_push_to_buffers(indio_dev, (uint8_t *)&buff_data);
	if (ret)
		// dev_err(&st->client->dev, "iio push error %d\n", ret);
		dbg_info("iio push error %d\n", ret);

	// mutex_unlock(&data->lock);
	mutex_unlock(&indio_dev->mlock);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

#if 0
int chx01_util_find(struct chx01_data *data)
{
	int ret;
	u32 regval;

	ret = regmap_read(data->regmap, CH_PROG_REG_PING, &regval);
	if (ret < 0)
		return ret;

	if (regval != CH101_SIG_BYTE_0)
		return -1;

	return 0;
}
#endif

void set_gpios(struct chx01_data *data)
{
	struct device *dev;
	struct chx01_gpios *gpios;

	gpios = &data->client.gpios;
	dev = data->dev;

dev_info(dev, ">>%s: data->dev = 0x%08x\n", __func__, (long)dev); //debug--yd
dbg_info("<<chx01>>(dbg_info)>>%s: data->dev = 0x%08x\n", __func__, (long)dev); //debug--yd

	gpios->gpiod_rst = gpiod_get_index(dev, CHx01_GPIO_RESET, 0,
			GPIOD_OUT_HIGH);
	if (IS_ERR(gpios->gpiod_rst)) {
		dev_warn(dev, "gpio get reset pin failed\n");
		gpios->gpiod_rst = NULL;
	}
	gpios->gpiod_prg = gpiod_get_index(dev, CHx01_GPIO_PROG, 0,
			GPIOD_OUT_LOW);
	if (IS_ERR(gpios->gpiod_prg)) {
		dev_warn(dev, "gpio get prog pin failed\n");
		gpios->gpiod_prg = NULL;
	}
	gpios->gpiod_cal = gpiod_get_index(dev, CHx01_GPIO_CAL, 0,
			GPIOD_OUT_LOW);
	if (IS_ERR(gpios->gpiod_cal)) {
		dev_warn(dev, "gpio get int pin failed\n");
		gpios->gpiod_cal = NULL;
	}
	gpios->gpiod_int = gpiod_get_index(dev, CHx01_GPIO_INT, 0,
			GPIOD_IN);
	if (IS_ERR(gpios->gpiod_int)) {
		dev_warn(dev, "gpio get int pin failed\n");
		gpios->gpiod_int = NULL;
	}
	dev_info(dev, "%s: %08x %08x %08x\n", __func__,
			(long)gpios->gpiod_rst,
			(long)gpios->gpiod_prg,
			(long)gpios->gpiod_cal);
}

int chx01_core_probe(struct i2c_client *client, struct regmap *regmap,
		struct chx01_callbacks *cbk, const char *name)
{
	struct chx01_data *data;
	struct device *dev;
	struct iio_dev *indio_dev;
	int irq = 0;
	int ret = 0;

	// dev_info(dev, "%s: Start", __func__);
	dbg_info("new<<chx01>>%s: Start", __func__);

	indio_dev = iio_device_alloc(sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	
	i2c_set_clientdata(client, indio_dev); //-yd
	dev = &client->dev;
	
	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &chx01_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chx01_channels;
	indio_dev->num_channels = ARRAY_SIZE(chx01_channels);

	data = iio_priv(indio_dev);

	data->dev = dev;
	data->regmap = regmap;
	data->client.i2c_client = client;
	data->client.cbk = cbk;
	data->client.cbk->data_complete = complete_done;
	data->trigger_enabled = false;

	// mutex_init(&data->lock);

	init_completion(&data->completion);

	set_gpios(data);

	irq = gpiod_to_irq(data->client.gpios.gpiod_int);
	data->irq = irq;

	dev_info(dev, "%s: irq: %d\n", __func__, irq);

	ret = request_threaded_irq(irq,
			chx01_irq_handler, chx01_irq_thread_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			CH101_IRQ_NAME, indio_dev);
	if (ret < 0) {
		dev_err(dev, "request irq %d failed\n", irq);
		goto error_request_threaded_irq;
	}

	
dev_info(dev, ">>%s: debug 1\n", __func__);
	ret = iio_triggered_buffer_setup(indio_dev, NULL,
			chx01_trigger_handler, NULL);
	if (ret) {
		dev_err(&client->dev, "iio triggered buffer error %d\n", ret);
		goto error_triggered_buffer_setup;
	}


dev_info(dev, ">>%s: debug 1.1\n", __func__);
	data->trig = iio_trigger_alloc("%s-dev%d", indio_dev->name, indio_dev->id);
	if (data->trig == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "iio trigger alloc error\n");
		goto error_trigger_alloc;
	}

dev_info(dev, ">>%s: debug 2\n", __func__);
	data->trig->dev.parent = &client->dev;
	data->trig->ops = &chx01_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = iio_trigger_register(data->trig);
	if (ret) {
		dev_err(&client->dev, "iio trigger register error %d\n", ret);
		goto error_trigger_register;
	}

dev_info(dev, ">>%s: debug 3\n", __func__);
	iio_trigger_get(data->trig);
	indio_dev->trig = data->trig;

dev_info(dev, ">>%s: debug 4\n", __func__);
	ret = iio_device_register(indio_dev);
	if (ret < 0)
	{
		dev_err(dev, "iio_device_register failed: %d\n", ret);
		goto error_device_register;
	}

	set_chirp_data(&data->client);
	set_chirp_buffer(&data->buffer);
dev_info(dev, ">>%s: debug 5\n", __func__);
	/* Test all ch101 functions */
	ret = find_sensors();
	if (ret < 0) {
		dev_err(dev, ">>%s: find_sensors: %d\n",__func__, ret);
		goto error_find_sensors;
	}

dev_info(dev, ">>%s: debug 5.5\n", __func__);
	init_driver();
	// return 0; //debug--yd
dev_info(dev, ">>%s: debug 6\n", __func__);
	config_driver();

	//init the hrtimer after driver init --yd
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->period = ktime_set(0, CHx01_DEFAULT_SAMPLING_PERIOD_NS);
	hrtimer_start(&data->timer, data->period, HRTIMER_MODE_REL);
	data->timer.function = chx01_hrtimer_handler;
	data->scan_rate = (NSEC_PER_SEC/CHx01_DEFAULT_SAMPLING_PERIOD_NS);


	dev_info(dev, "%s: 9 End\n", __func__);
	return 0;

error_find_sensors:
	iio_device_unregister(indio_dev);

error_device_register:
	iio_trigger_unregister(data->trig);

error_trigger_register:
	iio_trigger_free(data->trig);

error_trigger_alloc:
error_triggered_buffer_setup:
error_request_threaded_irq:
	iio_device_free(indio_dev);

	dev_err(dev, "%s: Error %d:\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(chx01_core_probe);

int chx01_core_remove(struct i2c_client *client)
{
	int ret = 0;

	struct iio_dev *indio_dev = dev_get_drvdata(&client->dev);
	struct chx01_data *data = iio_priv(indio_dev);

	iio_trigger_unregister(data->trig);
	iio_trigger_free(data->trig);

	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);

	return ret;
}
EXPORT_SYMBOL_GPL(chx01_core_remove);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense CH101 core device driver");
MODULE_LICENSE("GPL");
