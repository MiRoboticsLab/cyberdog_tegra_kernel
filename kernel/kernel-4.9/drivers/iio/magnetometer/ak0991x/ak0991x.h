/*
 * Partly modified 2012 Sony Mobile Communications AB.
 * Partly modified 2014 Asahi Kasei Microdevices Corp.
 */

/*
 * Definitions for ak0991x compass chip.
 */
#ifndef AK0991X_H
#define AK0991X_H

#ifdef __KERNEL__
#include <linux/input.h>
#endif

#define AKM_DRIVER_NAME "ak0991x"
#define MSC_RX  MSC_SERIAL
#define MSC_RY  MSC_PULSELED
#define MSC_RZ  MSC_GESTURE
#define MSC_ST2 MSC_RAW
#define MSC_TSL MSC_SCAN
#define MSC_TSH (MSC_SCAN + 1)

#ifdef __KERNEL__
struct ak0991x_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned char	axis_order[3];
	unsigned char	axis_sign[3];
};
#endif

#endif

