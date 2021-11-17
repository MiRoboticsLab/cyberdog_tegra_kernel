#
# Makefile for CH101 proximity sensors
#
# obj-y += inv_ch101/
#

#obj-$(CONFIG_CH101_I2C)	+= chx01-i2c.o
obj-$(CONFIG_CHIRP_I2C)	+= chx01-i2c.o

chx01-i2c-y		+= chx01_core.o
chx01-i2c-y 	+= src/init_driver.o
chx01-i2c-y 	+= src/chbsp_dummy.o 
chx01-i2c-y 	+= src/chbsp_init.o 
chx01-i2c-y 	+= src/chirp_hal.o 
chx01-i2c-y 	+= src/i2c_hal.o 

ifeq ($(CONFIG_CH101_I2C),y)
#chx01-i2c-y 	+= src/ch101_gpr.o 
#chx01-i2c-y 	+= src/ch101_gpr_fw.o 
chx01-i2c-y 	+= src/ch101_gpr_open.o 
chx01-i2c-y 	+= src/ch101_gpr_open_fw.o 
#chx01-i2c-y 	+= src/ch101_gpr_sr_open.o 
#chx01-i2c-y 	+= src/ch101_gpr_sr_open_fw.o 
else ifeq ($(CONFIG_CH201_I2C),y)
chx01-i2c-y 	+= src/ch201_gprmt.o
chx01-i2c-y 	+= src/ch201_gprmt_fw.o
endif

chx01-i2c-y 	+= src/ch_api.o 
chx01-i2c-y 	+= src/ch_common.o 
chx01-i2c-y 	+= src/ch_driver.o

chx01-i2c-y 	+= chx01_i2c.o

# disable some warning flags
ccflags-y += -Wno-format
