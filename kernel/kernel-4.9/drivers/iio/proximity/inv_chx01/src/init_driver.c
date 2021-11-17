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

#include "chbsp_init.h"
#include "chirp_bsp.h"
#include "chirp_hal.h"
#include "i2c_hal.h"
#include "soniclib.h"
#include "init_driver.h"
#include "../chx01_client.h"

/* Forward declarations */
//static bool check_sensor(int reg, ioport_pin_t pin);

static void sensor_int_callback(struct ch_group_t *grp_ptr, u8 dev_num);
//static void set_ch101_pitch_catch_config(void);
static u8 display_config_info(struct ch_dev_t *dev_ptr);
// static u8 handle_data_ready(struct ch_group_t *grp_ptr);
// static void trigger_driver(void);

struct chirp_drv {
	bool	driver_active;
	/* Device tracking variables
	*   These are bit-field variables which contain a separate bit assigned to
	*   each (possible) sensor, indexed by the device number.  The active_devices
	*   variable contains the bit pattern describing which ports have active
	*   sensors connected.  The data_ready_devices variable is set bit-by-bit
	*   as sensors interrupt, indicating they have completed a measurement
	*   cycle.  The two variables are compared to determine when all active
	*   devices have interrupted.
	*/	
	u32	active_devices;
	u32	data_ready_devices;

	u8	ch101_pitch;
	u16	chirp_odr_ms;
};

struct chirp_drv drv_data = { 0 };

/* Task flag word
 *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags
 *   that are set in I/O processing routines.  The flags are checked in the
 *   main() loop and, if set, will cause an appropriate handler function to
 *   be called to process sensor data.
 */
static u32 taskflags;

/* Chirp application version */
#define APP_VERSION_MAJOR	1
#define APP_VERSION_MINOR	0

#define CH_PART_NUMBER 101

/* Select sensor firmware to use
 *   The sensor firmware is specified during the call to ch_init(), by
 *   giving the name (address) of the firmware initialization function
 *   that will be called.
 */

/* SHORT RANGE OPTION:
 *   Uncomment the following line to use different sensor f/w optimized for
 *   short range. The short range firmware has 4 times the resolution, but
 *   only 1/4 the maximum range.  If you use this option, you should redefine
 *   the CHIRP_SENSOR_MAX_RANGE_MM symbol, below, to 250mm or less.
 */

#define	USE_RANGE			/* use range firmware */

#if defined(CONFIG_CH101_I2C)
//#define	USE_SHORT_RANGE			/* use short-range firmware */

#ifdef USE_RANGE
   
#ifndef USE_SHORT_RANGE
/* use CH101 GPR OPEN firmware (normal) */
#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_open_init
// #define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprmt_init	/* CH201 GPR Multi-Threshold firmware */
#else
/* use CH101 GPR SR OPEN firmware (short range) */
#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_sr_open_init
#endif

#else
#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch101_gpr_init
#endif
#endif
#if defined(CONFIG_CH201_I2C)
#define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprmt_init	/* CH201 GPR Multi-Threshold firmware */
#endif

#define INT_LINE_LATCH_TIME_US		50	/* latch time in microseconds */

int total_time_ms;

bool chirp_present[CHIRP_MAX_NUM_SENSORS] = { 0 };
u8 connected_sensor_array[CHIRP_MAX_NUM_SENSORS] = { 0 };
int num_connected_device;

/* Array of interrupts timestamp in ms*/
u64 chirp_timestamps_ms[CHIRP_MAX_NUM_SENSORS] = { 0 };

/* Array of structs to hold measurement data, one for each possible device */
struct chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS] = { { 0 } };

/* Array of ch_dev_t device descriptors, one for each possible device */
struct ch_dev_t chirp_devices[CHIRP_MAX_NUM_SENSORS] = { { 0 } };

/* Configuration structure for group of sensors */
struct ch_group_t chirp_group = { 0 };

/* Chirp sensor group pointer */
static struct ch_group_t *sensor_group_ptr;

static struct chx01_buffer *_buffer;
static struct chx01_client *ultra_data;

void set_chirp_buffer(struct chx01_buffer *buffer)
{
	_buffer = buffer;
}

struct chx01_buffer *get_chirp_buffer(void)
{
	return _buffer;
}

void set_chirp_data(struct chx01_client *data)
{
	ultra_data = data;

	if (data)
		set_chirp_gpios(&data->gpios);
}

struct chx01_client *get_chirp_data(void)
{
	return ultra_data;
}


static bool check_sensor(int reg, ioport_pin_t pin)
{
	u8 sig_bytes[2];
	bool good=0;
	// const char *ch;

	sig_bytes[0] = 0;
	sig_bytes[1] = 0;

	// dbg_info("<<chx01>>%s: 1\n", __func__);

	/* check sensor */
	ioport_set_pin_dir(pin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(pin, IOPORT_PIN_LEVEL_HIGH);
	// ioport_set_pin_dir(CHIRP_RST, IOPORT_DIR_OUTPUT); //reset=output-high
	// ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_LOW); //set=L
	// ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_HIGH); //reset=H	

	// dbg_info("<<chx01>>%s: 2\n", __func__);

	i2c_master_read_register0(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);

	// if (reg == 0)
	// 	i2c_master_read_register0(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);
	// else if (reg == 1)
	// 	i2c_master_read_register1(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);
	// else if (reg == 2)
	// 	i2c_master_read_register2(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);

	dbg_info("<<chx01>>%s:  sig_bytes[0]=0x%02X\n", __func__, (unsigned char) sig_bytes[0]);
	dbg_info("<<chx01>>%s:  sig_bytes[1]=0x%02X\n", __func__, (unsigned char) sig_bytes[1]);

	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		good = 1;
		dbg_info("<<chx01>>found chirp_0 !\n");
	} else {
		dbg_info("<<chx01>>did not find chirp_0\n");
	}

	ioport_set_pin_level(pin, IOPORT_PIN_LEVEL_LOW);

	return good;
}



int find_sensors(void)
{
	int i;
	bool find = false;
dbg_info("<<chx01>>%s: line %d\n", __func__,__LINE__);

	// ioport_set_pin_dir(CHIRP_RST, IOPORT_DIR_OUTPUT);        //reset=output
	// ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_HIGH);  //reset=H
	//reset the Chirp device
	ioport_set_pin_dir(CHIRP_RST, IOPORT_DIR_OUTPUT); //reset=output-high
	ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_LOW); //set=L
	ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_HIGH); //reset=H

	dbg_info("<<chx01>>%s: size: %x\n", __func__,
		(u32)ARRAY_SIZE(chirp_pin_prog));

	/* check sensors*/
	for (i = 0; i < ARRAY_SIZE(chirp_pin_prog); i++) {
		if (chirp_pin_enabled[i])
			chirp_present[i] = check_sensor(chirp_i2c_buses[i],
				chirp_pin_prog[i]);
		find |= chirp_present[i];
	}
dbg_info("<<chx01>>%s: line %d\n", __func__,__LINE__);
	return find ? 0 : 1;
}


void init_driver(void)
{
	struct ch_group_t *grp_ptr = &chirp_group;
	u8 chirp_error = 0;
	u8 num_ports = 0;
	u8 dev_num = 0;

	/* Initialize board hardware functions
	 *   This call to the board support package (BSP) performs all necessary
	 *   hardware initialization for the application to run on this board.
	 *   This includes setting up memory regions, initialization clocks and
	 *   peripherals (including I2C and serial port), and any
	 *   processor-specific startup sequences.
	 *
	 *   The chbsp_board_init() function also initializes fields within the
	 *   sensor group descriptor, including number of supported sensors and
	 *   the RTC clock calibration pulse length.
	 */

	/* Make local copy of group pointer */
	sensor_group_ptr = grp_ptr;

	chbsp_board_init(grp_ptr);

	dbg_info("<<chx01>>CH-%d driver", CH_PART_NUMBER);
	dbg_info("<<chx01>>    Version: %u.%u\n\n", APP_VERSION_MAJOR, APP_VERSION_MINOR);
	dbg_info("<<chx01>>    num_ports: %d\n", grp_ptr->num_ports);
	dbg_info("<<chx01>>    num_i2c_buses: %d\n", grp_ptr->num_i2c_buses);

	/* Get the number of (possible) sensor devices on the board
	 *   Set by the BSP during chbsp_board_init()
	 */
	num_ports = ch_get_num_ports(grp_ptr);

	/* Initialize sensor descriptors.
	 *   This loop initializes each (possible) sensor's ch_dev_t descriptor,
	 *   although we don't yet know if a sensor is actually connected.
	 *
	 *   The call to ch_init() specifies the sensor descriptor, the sensor
	 *   group it will be added to, the device number within the group, and
	 *   the sensor firmware initialization routine that will be used.
	 *   (The sensor firmware selection effectively specifies whether it is
	 *   a CH101 or CH201 sensor, as well as the exact feature set.)
	 */
	dbg_info("<<chx01>>Initializing sensor(s)... ");
	num_connected_device = 0;

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		if (chirp_pin_enabled[dev_num]) {
			// init struct in array
			struct ch_dev_t *dev_ptr = &chirp_devices[dev_num];

			/* Init device descriptor
			 * Note that this assumes all sensors will use the same
			 * sensor firmware.
			 */
			chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num,
						CHIRP_SENSOR_FW_INIT_FUNC);
			dev_ptr->sensor_connected = chirp_present[dev_num];
			if (dev_ptr->sensor_connected)
				connected_sensor_array[num_connected_device++] =
					dev_num;
		}
	}

	/* Start all sensors.
	 * The ch_group_start() function will search each port (that was
	 * initialized above) for a sensor. If it finds one, it programs it(with
	 * the firmware specified above during ch_init()) and waits for it to
	 * perform a self-calibration step.  Then, once it has found all the
	 * sensors, ch_group_start() completes a timing reference calibration by
	 * applying a pulse of known length to the sensor's INT line.
	 */
	if (chirp_error == 0) {
		dbg_info("<<chx01>>starting group... ");
		chirp_error = ch_group_start(grp_ptr);
	}

	if (chirp_error == 0)
		dbg_info("<<chx01>>OK\n");
	else
		dbg_info("<<chx01>>starting group FAILED: %d\n", chirp_error);
	dbg_info("<<chx01>>\n");

	/* Get and display the initialization results for each connected sensor.
	 *  This loop checks each device number in the sensor group to determine
	 *  if a sensor is actually connected.  If so, it makes a series of
	 *  function calls to get different operating values, including the
	 *  operating frequency, clock calibration values, and firmware version.
	 */
	dbg_info("<<chx01>>Sensor\tType \t   Freq\t\t RTC Cal \tFirmware\n");

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			dbg_info("<<chx01>>%d\tCH%d\t %u Hz\t%u@%u ms\t%s\n", dev_num,
				ch_get_part_number(dev_ptr),
				ch_get_frequency(dev_ptr),
				ch_get_rtc_cal_result(dev_ptr),
				ch_get_rtc_cal_pulselength(dev_ptr),
				ch_get_fw_version_string(dev_ptr));
		}
	}
	dbg_info("<<chx01>>\n");

	// Register callback function to be called when Chirp sensor interrupts
	ch_io_int_callback_set(grp_ptr, sensor_int_callback);
	chbsp_delay_ms(100); //--yd
}


/*
 * display_config_info() - display the configuration values for a sensor
 *
 * This function displays the current configuration settings for an individual
 * sensor.  The operating mode, maximum range, and static target rejection
 * range (if used) are displayed.
 *
 * For CH201 sensors only, the multiple detection threshold values are also
 * displayed.
 */
static u8 display_config_info(struct ch_dev_t *dev_ptr)
{
	struct ch_config_t read_config;
	struct ch_thresh_t *tr;
	u8 chirp_error;
	int i = 0;

#ifdef CHIRP_DEBUG
	u8 dev_num = ch_get_dev_num(dev_ptr);
#endif
	/* Read configuration values for the device into ch_config_t structure*/
	chirp_error = ch_get_config(dev_ptr, &read_config);

	if (!chirp_error) {
		char *mode_string;

		switch (read_config.mode) {
		case CH_MODE_IDLE:
			mode_string = "IDLE";
			break;
		case CH_MODE_FREERUN:
			mode_string = "FREERUN";
			break;
		case CH_MODE_TRIGGERED_TX_RX:
			mode_string = "TRIGGERED_TX_RX";
			break;
		case CH_MODE_TRIGGERED_RX_ONLY:
			mode_string = "TRIGGERED_RX_ONLY";
			break;
		default:
			mode_string = "UNKNOWN";
		}

		/* Display sensor number, mode and max range */
		// dbg_info("<<chx01>>Sensor %d:\tmax_range=%dmm \tmode=%s  ", ch_get_config(dev_ptr, &read_config),		
		dbg_info("<<chx01>>Sensor %d:\tmax_range=%dmm \tmode=%s  ", dev_num,
			read_config.max_range, mode_string);

		/* Display static target rejection range, if used */
		if (read_config.static_range != 0) {
			dbg_info("<<chx01>>static_range=%d samples",
				read_config.static_range);
		}

		/* Display detection thresholds (only supported on CH201) */
		if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
			struct ch_thresholds_t read_thresholds;

			/* Get threshold values in structure */
			chirp_error = ch_get_thresholds(dev_ptr,
				&read_thresholds);

			if (!chirp_error) {
				dbg_info("<<chx01>>\n  Detection thresholds:\n");
				for (i = 0; i < CH_NUM_THRESHOLDS; i++) {
					tr = &read_thresholds.threshold[i];
					dbg_info("<<chx01>> %d\tstart: %2d\tlevel: %d\n",
						i,
						tr->start_sample,
						tr->level);
				}
			} else {
				dbg_info("<<chx01>> Device %d: Error ch_get_thresholds()",
					dev_num);
			}
		}
		// dbg_info("<<chx01>>\n");

	} else {
		dbg_info("<<chx01>> Device %d: Error during ch_get_config()\n", dev_num);
	}

	return chirp_error;
}

void config_driver(void)
{
	int num_samples;
	struct ch_group_t *grp_ptr = &chirp_group;
	u8 chirp_error = 0;
	u8 num_ports = 0;
	u8 dev_num = 0;
	uint8_t	num_connected = 0;

	num_ports = ch_get_num_ports(grp_ptr);

	dbg_info("<<chx01>>Configuring sensor(s)...\n");
	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		struct ch_config_t dev_config;
		struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			/* Select sensor mode
			 * All connected sensors are placed in hardware
			 * triggered mode. The first connected (lowest numbered)
			 * sensor will transmit and receive, all others will
			 * only receive.
			 */

			// // add to active device bit mask
			drv_data.active_devices |= (1 << dev_num);
			// dev_config.mode = CH_MODE_TRIGGERED_RX_ONLY;
			num_connected++;					// count one more connected
			
			if (num_connected == 1) {			// if this is the first sensor
				dev_config.mode = CH_MODE_TRIGGERED_TX_RX;
				// dev_config.mode = CH_MODE_FREERUN; //--yd
			} else {									
				dev_config.mode = CH_MODE_TRIGGERED_RX_ONLY;
			}

			/* Init config structure with default values */
			dev_config.max_range = CHIRP_SENSOR_MAX_RANGE_MM;
			dev_config.static_range = CHIRP_SENSOR_STATIC_RANGE;
			dev_config.sample_interval = CHIRP_SENSOR_SAMPLE_INTERVAL;

			/* Set detection thresholds (CH201 only) */
			dev_config.thresh_ptr = 0;

			/* Apply sensor configuration */
			chirp_error = ch_set_config(dev_ptr, &dev_config);

			num_samples = ch_get_num_samples(dev_ptr);
			if (num_samples > MAX_RX_SAMPLES) {
				ch_set_num_samples(dev_ptr, MAX_RX_SAMPLES);
				dev_ptr->num_rx_samples = MAX_RX_SAMPLES;
			}

			/* Enable sensor interrupt if using free-running mode
			 *   Note that interrupt is automatically enabled if
			 *   using triggered modes.
			 */
			if (!chirp_error &&
				dev_config.mode == CH_MODE_FREERUN)
				chbsp_io_interrupt_enable(dev_ptr);

			/* Read back and display config settings */
			if (!chirp_error)
				display_config_info(dev_ptr);
			else
				dbg_info("<<chx01>>Device %d: Error ch_set_config()\n",
					dev_num);

			/* Turn on an LED to indicate device connected */
			if (!chirp_error)
				chbsp_led_on(dev_num);
		}
	}
	// set_ch101_pitch_catch_config();
}

//static void periodic_timer_callback(void)
//{
//	struct ch_group_t *grp_ptr = &chirp_group;
//
//	ch_group_trigger(grp_ptr);
//}

void set_complete(void)
{
	struct chx01_client *data = get_chirp_data();

	if (data->cbk->data_complete)
		data->cbk->data_complete(data);
}

/*
 * sensor_int_callback() - sensor interrupt callback routine
 *
 * This function is called by the board support package's interrupt handler for
 * the sensor's INT line every time that the sensor interrupts.  The device
 * number parameter, dev_num, is used to identify the interrupting device
 * within the sensor group.  (Generally the device number is same as the port
 * number used in the BSP to manage I/O pins, etc.)
 *
 * This callback function is registered by the call to ch_io_int_callback_set()
 * in main().
 */
static void sensor_int_callback(struct ch_group_t *grp_ptr, u8 dev_num)
{
	struct ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

	// time of interrupt in ms
	chirp_timestamps_ms[dev_num] = os_timestamp_ms();

	// add to data-ready bit mask
	drv_data.data_ready_devices |= (1 << dev_num);

	//INV_LOGD("t: %lld INT: %d", inv_pal_get_time_ms(), dev_num);

	if (drv_data.data_ready_devices == drv_data.active_devices) {
		/* All active sensors have interrupted
		 * after performing a measurement
		 */
		drv_data.data_ready_devices = 0;

		/* Set data-ready flag - it will be checked in main() loop */
		taskflags |= DATA_READY_FLAG;

		/* Disable interrupt unless in free-running mode
		 *   It will automatically be re-enabled during the next trigger
		 */
		if (ch_get_mode(dev_ptr) != CH_MODE_FREERUN)
			chbsp_group_io_interrupt_disable(grp_ptr);
	}
}
