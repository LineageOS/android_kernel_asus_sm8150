/* include/linux/VCNL36812.h
 *
 * Copyright (C) 2016 Vishay Capella Microsystems Limited
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

#ifndef __LINUX_VCNL36812_H
#define __LINUX_VCNL36812_H

#define VCNL36812_I2C_NAME "vcnl36812"

/*Define Command Code*/
//#define		ID_REG    		0x0F

#define		VCNL36812_CS_SLAVE_ADDR    		0x51
#define		VCNL36812_PS_SLAVE_ADDR    		0x60


#define		ID_REG    		0x0E


/*Define CS Command Code*/
#define		CS_CONF		    0x10
#define		CS_THDH  	    0x12
#define		CS_THDL	      0x13
#define		CS_R_DATA     0x15
#define		CS_G_DATA     0x16
#define		CS_B_DATA     0x17
#define		CS_IR_DATA    0x18
#define		CS_C_DATA     0x14

/*Define PS Command Code*/
#define		PS_CONF1      0x00
#define		PS_CONF3      0x01
#define		PS_THDL       0x03
#define		PS_THDH       0x04
#define		PS_CANC       0x05
#define		PS_DATA       0x0C
#define		PS_INT_FLAG  	0x0D

#define PS_SET1	    		0x07
#define PS_SET2	    		0x08

#define CS_PST	    		0x1A
#define CS_PSC	    		0x1B

/*vcnl36812*/
/*for CS CONF command*/
#define VCNL36812_CS_START      (1 << 15) //have to be check

#define VCNL36812_CS_HS_1 	    (0 << 12)
#define VCNL36812_CS_HS_2 	    (1 << 12)
#define VCNL36812_CS_HS_4 	    (2 << 12)

#define VCNL36812_CS_GAIN_1     (0 << 10)
#define VCNL36812_CS_GAIN_2	    (1 << 10)
#define VCNL36812_CS_GAIN_4	    (2 << 10)
#define VCNL36812_CS_GAIN_1_2   (3 << 10)

#define VCNL36812_CS_PERS_1     (0 << 8)
#define VCNL36812_CS_PERS_2     (1 << 8)
#define VCNL36812_CS_PERS_4     (2 << 8)
#define VCNL36812_CS_PERS_8     (3 << 8)
#define VCNL36812_CS_IT_50MS    (0 << 4)
#define VCNL36812_CS_IT_100MS   (1 << 4)
#define VCNL36812_CS_IT_200MS   (2 << 4)
#define VCNL36812_CS_IT_400MS   (3 << 4)
#define VCNL36812_CS_INT_EN     (1 << 1) /*enable/disable Interrupt*/
#define VCNL36812_CS_INT_MASK   0xFFFD
#define VCNL36812_CS_SD         (1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define VCNL36812_CS_SD_MASK    0xFFFE

/*for PS CONF1 command*/
//#define VCNL36812_PS_START 	(1 << 11)
#define VCNL36812_PS_RESERVED1_BIT_1    (1 << 11) //=> have to be check
#define VCNL36812_PS_INT_OFF	          (0 << 2) /*enable/disable Interrupt*/
#define VCNL36812_PS_INT_IN_AND_OUT     (2 << 2)
#define VCNL36812_PS_INT_MASK           0xFFF3

#define VCNL36812_PS_PERS_1     (0 << 4)
#define VCNL36812_PS_PERS_2     (1 << 4)
#define VCNL36812_PS_PERS_3 	  (2 << 4)
#define VCNL36812_PS_PERS_4 	  (3 << 4)
#define VCNL36812_PS_IT_1T 	    (0 << 14)
#define VCNL36812_PS_IT_2T 	    (1 << 14)
#define VCNL36812_PS_IT_4T 	    (2 << 14)
#define VCNL36812_PS_IT_8T 	    (3 << 14)

#define VCNL36812_PS_MP_1 	    (0 << 12)
#define VCNL36812_PS_MP_2 	    (1 << 12)
#define VCNL36812_PS_MP_4 	    (2 << 12)
#define VCNL36812_PS_MP_8 	    (3 << 12)

#define VCNL36812_PS_HG         (1 << 10)


#define VCNL36812_PS_SMART_PERS (1 << 1)
#define VCNL36812_PS_SD	        (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define VCNL36812_PS_SD_MASK    0xFFFE

/*for PS CONF3 command*/
#define VCNL36812_LED_I_70              (0 << 8)
#define VCNL36812_LED_I_95              (1 << 8)
#define VCNL36812_LED_I_110             (2 << 8)
#define VCNL36812_LED_I_130             (3 << 8)
#define VCNL36812_LED_I_170             (4 << 8)
#define VCNL36812_LED_I_200             (5 << 8)
#define VCNL36812_LED_I_220             (6 << 8)
#define VCNL36812_LED_I_240             (7 << 8)
#define VCNL36812_PS_ACTIVE_FORCE_MODE  (1 << 6)
#define VCNL36812_PS_ACTIVE_FORCE_TRIG  (1 << 5)
#define VCNL36812_PS_RESERVED2_BIT_1    (1 << 3) //=> have to be check

/*for INT FLAG*/
#define INT_FLAG_PS_SPFLAG              (1 << 12)
#define INT_FLAG_PS_IF_CLOSE            (1 << 9)
#define INT_FLAG_PS_IF_AWAY             (1 << 8)

extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;

struct vcnl36812_platform_data {
	int intr;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t  cs_slave_addr;
	uint8_t  ps_slave_addr;  
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	uint16_t ls_cmd;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;
	struct regulator *vcc_l8c_1p8;
	struct regulator *vcc_17c_3p0;
};

#endif
