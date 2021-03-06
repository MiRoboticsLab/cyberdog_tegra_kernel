#ifndef __AW21036_H__
#define __AW21036_H__

/******************************************************
 *
 * Register List
 *
 *****************************************************/
#define AW21036_REG_GCR		0x00
#define AW21036_REG_BR0		0x01
#define AW21036_REG_BR1		0x02
#define AW21036_REG_BR2		0x03
#define AW21036_REG_BR3		0x04
#define AW21036_REG_BR4		0x05
#define AW21036_REG_BR5		0x06
#define AW21036_REG_BR6		0x07
#define AW21036_REG_BR7		0x08
#define AW21036_REG_BR8		0x09
#define AW21036_REG_BR9		0x0A
#define AW21036_REG_BR10	0x0B
#define AW21036_REG_BR11	0x0C
#define AW21036_REG_BR12	0x0D
#define AW21036_REG_BR13	0x0E
#define AW21036_REG_BR14	0x0F
#define AW21036_REG_BR15	0x10
#define AW21036_REG_BR16	0x11
#define AW21036_REG_BR17	0x12
#define AW21036_REG_BR18	0x13
#define AW21036_REG_BR19	0x14
#define AW21036_REG_BR20	0x15
#define AW21036_REG_BR21	0x16
#define AW21036_REG_BR22	0x17
#define AW21036_REG_BR23	0x18
#define AW21036_REG_BR24	0x19
#define AW21036_REG_BR25	0x1A
#define AW21036_REG_BR26	0x1B
#define AW21036_REG_BR27	0x1C
#define AW21036_REG_BR28	0x1D
#define AW21036_REG_BR29	0x1E
#define AW21036_REG_BR30	0x1F
#define AW21036_REG_BR31	0x20
#define AW21036_REG_BR32	0x21
#define AW21036_REG_BR33	0x22
#define AW21036_REG_BR34	0x23
#define AW21036_REG_BR35	0x24
#define AW21036_REG_UPDATE	0x49
#define AW21036_REG_COL0	0x4A
#define AW21036_REG_COL1	0x4B
#define AW21036_REG_COL2	0x4C
#define AW21036_REG_COL3	0x4D
#define AW21036_REG_COL4	0x4E
#define AW21036_REG_COL5	0x4F
#define AW21036_REG_COL6	0x50
#define AW21036_REG_COL7	0x51
#define AW21036_REG_COL8	0x52
#define AW21036_REG_COL9	0x53
#define AW21036_REG_COL10	0x54
#define AW21036_REG_COL11	0x55
#define AW21036_REG_COL12	0x56
#define AW21036_REG_COL13	0x57
#define AW21036_REG_COL14	0x58
#define AW21036_REG_COL15	0x59
#define AW21036_REG_COL16	0x5A
#define AW21036_REG_COL17	0x5B
#define AW21036_REG_COL18	0x5C
#define AW21036_REG_COL19	0x5D
#define AW21036_REG_COL20	0x5E
#define AW21036_REG_COL21	0x5F
#define AW21036_REG_COL22	0x60
#define AW21036_REG_COL23	0x61
#define AW21036_REG_COL24	0x62
#define AW21036_REG_COL25	0x63
#define AW21036_REG_COL26	0x64
#define AW21036_REG_COL27	0x65
#define AW21036_REG_COL28	0x66
#define AW21036_REG_COL29	0x67
#define AW21036_REG_COL30	0x68
#define AW21036_REG_COL31	0x69
#define AW21036_REG_COL32	0x6A
#define AW21036_REG_COL33	0x6B
#define AW21036_REG_COL34	0x6C
#define AW21036_REG_COL35	0x6D
#define AW21036_REG_GCCR	0x6E
#define AW21036_REG_PHCR	0x70
#define AW21036_REG_OSDCR	0x71
#define AW21036_REG_OSST0	0x72
#define AW21036_REG_OSST1	0x73
#define AW21036_REG_OSST2	0x74
#define AW21036_REG_OSST3	0x75
#define AW21036_REG_OSST4	0x76
#define AW21036_REG_OTCR	0x77
#define AW21036_REG_SSCR	0x78
#define AW21036_REG_UVCR	0x79
#define AW21036_REG_GCR2	0x7A
#define AW21036_REG_GCR4	0x7C
#define AW21036_REG_VER		0x7E
#define AW21036_REG_RESET	0x7F
#define AW21036_REG_WBR		0x90
#define AW21036_REG_WBG		0x91
#define AW21036_REG_WBB		0x92
#define AW21036_REG_PATCFG	0xA0
#define AW21036_REG_PATGO	0xA1
#define AW21036_REG_PATT0	0xA2
#define AW21036_REG_PATT1	0xA3
#define AW21036_REG_PATT2	0xA4
#define AW21036_REG_PATT3	0xA5
#define AW21036_REG_FADEH	0xA6
#define AW21036_REG_FADEL	0xA7
#define AW21036_REG_GCOLR	0xA8
#define AW21036_REG_GCOLG	0xA9
#define AW21036_REG_GCOLB	0xAA
#define AW21036_REG_GCFG0	0xAB
#define AW21036_REG_GCFG1	0xAC

/******************************************************
 *
 * Register Write/Read Access
 *
 *****************************************************/
#define REG_NONE_ACCESS		0
#define REG_RD_ACCESS		(1 << 0)
#define REG_WR_ACCESS		(1 << 1)
#define AW21036_REG_MAX		0xFF

const unsigned char aw21036_reg_access[AW21036_REG_MAX] = {
	[AW21036_REG_GCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR3]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR4]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR5]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR6]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR7]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR8]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR9]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR10]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR11]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR12]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR13]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR14]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR15]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR16]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR17]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR18]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR19]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR20]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR21]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR22]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR23]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR24]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR25]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR26]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR27]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR28]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR29]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR30]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR31]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR32]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR33]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR34]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_BR35]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_UPDATE]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL3]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL4]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL5]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL6]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL7]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL8]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL9]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL10]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL11]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL12]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL13]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL14]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL15]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL16]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL17]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL18]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL19]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL20]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL21]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL22]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL23]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL24]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL25]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL26]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL27]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL28]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL29]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL30]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL31]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL32]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL33]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL34]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_COL35]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_PHCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_OSDCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_OSST0]	= REG_RD_ACCESS,
	[AW21036_REG_OSST1]	= REG_RD_ACCESS,
	[AW21036_REG_OSST2]	= REG_RD_ACCESS,
	[AW21036_REG_OSST3]	= REG_RD_ACCESS,
	[AW21036_REG_OSST4]	= REG_RD_ACCESS,
	[AW21036_REG_OTCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_SSCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_UVCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCR2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCR4]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_VER]	= REG_RD_ACCESS,
	[AW21036_REG_RESET]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_WBR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_WBG]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_WBB]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_PATCFG]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_PATGO]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_PATT0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_PATT1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_PATT2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_PATT3]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_FADEH]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_FADEL]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCOLR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCOLG]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCOLB]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCFG0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[AW21036_REG_GCFG1]	= REG_RD_ACCESS | REG_WR_ACCESS,
};

/******************************************************
 *
 * Register Detail
 *
 *****************************************************/
#define AW21036_BIT_GCR_CHIPEN_MASK		(~(1<<0))
#define AW21036_BIT_GCR_CHIPEN_ENABLE		(1<<0)
#define AW21036_BIT_GCR_CHIPEN_DISABLE		(0<<0)
#define AW21036_BIT_GCR_CLKFRQ_MASK		(~(7<<4))
#define AW21036_BIT_GCR_CLKFRQ_16M		(0<<4)
#define AW21036_BIT_GCR_CLKFRQ_8M		(1<<4)
#define AW21036_BIT_GCR_CLKFRQ_1M		(2<<4)
#define AW21036_BIT_GCR_CLKFRQ_512K		(3<<4)
#define AW21036_BIT_GCR_CLKFRQ_256K		(4<<4)
#define AW21036_BIT_GCR_CLKFRQ_125K		(5<<4)
#define AW21036_BIT_GCR_CLKFRQ_62K		(6<<4)
#define AW21036_BIT_GCR_CLKFRQ_31K		(7<<4)
#define AW21036_BIT_GCR_APSE_MASK		(~(1<<7))
#define AW21036_BIT_GCR_APSE_ENABLE		(1<<7)
#define AW21036_BIT_GCR_APSE_DISABLE		(0<<7)

#define AW21036_BIT_OSDCR_OTH_0V1		(0<<3)
#define AW21036_BIT_OSDCR_OTH_0V2		(1<<3)

#define AW21036_BIT_OSDCR_STH_1V		(0<<2)
#define AW21036_BIT_OSDCR_STH_0V5		(1<<2)

/*********************************************************
 *
 * chip info
 *
 ********************************************************/
#define AW21036_CHIP_VERSION	0xA8
#define AW21036_CHIPID		0x18

enum aw21036_pwm_freq {
	AW21036_PWM_FREQ_62K = 0,
	AW21036_PWM_FREQ_32K = 1,
	AW21036_PWM_FREQ_4K  = 2,
	AW21036_PWM_FREQ_2K  = 3,
	AW21036_PWM_FREQ_1K  = 4,
	AW21036_PWM_FREQ_500 = 5,
	AW21036_PWM_FREQ_244 = 6,
	AW21036_PWM_FREQ_122 = 7,
};

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct aw21036 {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct work_struct brightness_work;
	struct work_struct cfg_work;
	struct mutex cfg_lock;
	int reset_gpio;

	unsigned char flags;
	unsigned char chipid;
	unsigned int imax;
	unsigned int fw_version;

	unsigned int pwm_freq;
	unsigned int led_current;

	unsigned char effect;
	unsigned char cfg;

	unsigned int rgbcolor;
	unsigned int rgbbrightness;
};

typedef struct aw21036_cfg {
	unsigned char *p;
	unsigned int count;
} AW21036_CFG;

#endif
