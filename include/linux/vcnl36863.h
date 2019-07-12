/* include/linux/VCNL36863.h
 *
 * Copyright (C) 2018 Vishay Capella Microsystems Limited
 * Author: Frank Hsieh <Frank.Hsieh@vishay.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_VCNL36863_H
#define __LINUX_VCNL36863_H

#define VCNL36863_I2C_NAME "vcnl36863"

/* Define Slave Address*/
#define	VCNL36863_SLAVE_ADDR	0x60

/*Define Command Code*/
#define		CS_CONF      0x00
#define		CS_THDH      0x01
#define		CS_THDL      0x02
#define		PS_CONF1      0x03
#define		PS_CONF3      0x04

#define		PS_THDL       0x05
#define		PS_THDH       0x06
#define		PS_CANC       0x07

#define		CS_R_DATA     0xF0
#define		CS_G_DATA     0xF1
#define		CS_B_DATA     0xF2
#define		CS_IR_DATA    0xF3
#define		PS_DATA       0xF4

#define		INT_FLAG      0xF5
#define		ID_REG        0xF6

/*vcnl36863*/
/*for CS CONF command*/
#define CS_RESERVED_1           (1 << 9)

#define VCNL36863_CS_INT_MASK    0xFEFF

#define VCNL36863_CS_START      (1 << 7)
#define VCNL36863_CS_IT_50MS    (0 << 2)
#define VCNL36863_CS_IT_100MS   (1 << 2)
#define VCNL36863_CS_IT_200MS   (2 << 2)
#define VCNL36863_CS_IT_400MS   (3 << 2)

#define VCNL36863_CS_STANDBY    (1 << 1)

#define VCNL36863_CS_SD         (1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define VCNL36863_CS_SD_MASK     0xFFFE

/*for PS CONF1 command*/
#define VCNL36863_PS_IT_1T       (0 << 14)
#define VCNL36863_PS_IT_2T       (1 << 14)
#define VCNL36863_PS_IT_4T       (2 << 14)
#define VCNL36863_PS_IT_8T       (3 << 14)

#define VCNL36863_PS_MPS_1       (0 << 12)
#define VCNL36863_PS_MPS_2       (1 << 12)
#define VCNL36863_PS_MPS_4       (2 << 12)
#define VCNL36863_PS_MPS_8       (3 << 12)

#define VCNL36863_PS_START       (1 << 11) 
#define VCNL36863_PS_HG_ENABLE   (1 << 10)

#define VCNL36863_PS_INT_SEL     (1 <<  8)

#define VCNL36863_PS_INT_IN_AND_OUT  (2 <<  2) /*enable/disable Interrupt*/
#define VCNL36863_PS_INT_MASK      0xFFF3  /*enable/disable Interrupt*/

#define VCNL36863_PS_PERS_1      (0 << 4)
#define VCNL36863_PS_PERS_2      (1 << 4)
#define VCNL36863_PS_PERS_3      (2 << 4)
#define VCNL36863_PS_PERS_4      (3 << 4)

#define VCNL36863_PS_SMART_PERS  (1 << 1)
#define VCNL36863_PS_SD	         (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define VCNL36863_PS_SD_MASK      0xFFFE

/*for PS CONF3 command*/
#define VCNL36863_PS_SUNLIGHT_DEFAULT   (7 << 13)
#define VCNL36863_LED_I_7mA             (0 <<  8)
#define VCNL36863_LED_I_11mA            (1 <<  8)
#define VCNL36863_LED_I_14mA            (2 <<  8)
#define VCNL36863_LED_I_17mA            (3 <<  8)
#define VCNL36863_LED_I_20mA            (4 <<  8)
#define VCNL36863_PS_ACTIVE_FORCE_MODE  (1 <<  6)
#define VCNL36863_PS_ACTIVE_FORCE_TRIG  (1 <<  5)
#define VCNL36863_PS_START2             (1 <<  3)

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG              (1 << 14)
#define INT_FLAG_PS_IF_CLOSE            (1 <<  9)  //PS rises above PS_THDH INT trigger event
#define INT_FLAG_PS_IF_AWAY             (1 <<  8)  //PS drops below PS_THDL INT trigger event

/*Calibration*/
#define ALS_CALIBRATION_50MS "/vendor/factory/sensors/als_cal_50ms"
#define ALS_CALIBRATION_100MS "/vendor/factory/sensors/als_cal_100ms"

#define PS_CALIBRATION_INF "/vendor/factory/sensors/ps_cal_inf"
#define PS_CALIBRATION_1CM "/vendor/factory/sensors/ps_cal_1cm"
#define PS_CALIBRATION_2CM "/vendor/factory/sensors/ps_cal_2cm"
#define PS_CALIBRATION_4CM "/vendor/factory/sensors/ps_cal_4cm"

#define FRGB_RED_RATIO    "/vendor/factory/sensors/frgb_red_ratio"
#define FRGB_GREEN_RATIO  "/vendor/factory/sensors/frgb_green_ratio"
#define FRGB_BLUE_RATIO   "/vendor/factory/sensors/frgb_blue_ratio"
#define FRGB_LIGHT1_RED   "/vendor/factory/sensors/frgb_light1_red"
#define FRGB_LIGHT1_GREEN "/vendor/factory/sensors/frgb_light1_green"
#define FRGB_LIGHT1_BLUE  "/vendor/factory/sensors/frgb_light1_blue"
#define FRGB_LIGHT1_IR    "/vendor/factory/sensors/frgb_light1_ir"
#define FRGB_LIGHT2_RED   "/vendor/factory/sensors/frgb_light2_red"
#define FRGB_LIGHT2_GREEN "/vendor/factory/sensors/frgb_light2_green"
#define FRGB_LIGHT2_BLUE  "/vendor/factory/sensors/frgb_light2_blue"
#define FRGB_LIGHT2_IR    "/vendor/factory/sensors/frgb_light2_ir"
#define FRGB_LIGHT3_RED   "/vendor/factory/sensors/frgb_light3_red"
#define FRGB_LIGHT3_GREEN "/vendor/factory/sensors/frgb_light3_green"
#define FRGB_LIGHT3_BLUE  "/vendor/factory/sensors/frgb_light3_blue"
#define FRGB_LIGHT3_IR    "/vendor/factory/sensors/frgb_light3_ir"

extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;

struct vcnl36863_platform_data {
	int intr;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t slave_addr; 
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	uint16_t ls_cmd;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;
        struct regulator *vcc_l8c_1p8;
        struct regulator *vcc_17c_3p0;
};

#endif
