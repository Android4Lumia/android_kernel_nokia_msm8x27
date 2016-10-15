/*
 *  etzkx: lisn3dsh/kxtnk 3d accelerometer driver
 *
 *  Copyright(C) 2013  Andi Shyti <andi@etezian.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/i2c/etzkx.h>

/* register map */
#define ETZKX_REG_INFO1		0x0D
#define ETZKX_REG_INFO2		0x0E
#define ETZKX_REG_WIA		0x0F
#define ETZKX_REG_OUTX_L	0x10
#define ETZKX_REG_OUTX_H	0x11
#define ETZKX_REG_OUTY_L	0x12
#define ETZKX_REG_OUTY_H	0x13
#define ETZKX_REG_OUTZ_L	0x14
#define ETZKX_REG_OUTZ_H	0x15
#define ETZKX_REG_LC_L		0x16
#define ETZKX_REG_LC_H		0x17
#define ETZKX_REG_STAT		0x18
#define ETZKX_REG_PEAK1		0x19
#define ETZKX_REG_PEAK2		0x1A
#define ETZKX_REG_CNTL1		0x1B
#define ETZKX_REG_CNTL2		0x1C
#define ETZKX_REG_CNTL3		0x1D
#define ETZKX_REG_CNTL4		0x1E
#define ETZKX_REG_THRS3		0x1F
#define ETZKX_REG_OFF_X		0x20
#define ETZKX_REG_OFF_Y		0x21
#define ETZKX_REG_OFF_Z		0x22
#define ETZKX_REG_CS_X		0x24
#define ETZKX_REG_CS_Y		0x25
#define ETZKX_REG_CS_Z		0x26
#define ETZKX_REG_X_DEBUG	0x28
#define ETZKX_REG_Y_DEBUG	0x29
#define ETZKX_REG_Z_DEBUG	0x2A
#define ETZKX_REG_VFC_1		0x2C
#define ETZKX_REG_VFC_2		0x2D
#define ETZKX_REG_VFC_3		0x2E
#define ETZKX_REG_VFC_4		0x2F

#define ETZKX_REG_ST1_1		0x40
#define ETZKX_REG_ST2_1		0x41
#define ETZKX_REG_ST3_1		0x42
#define ETZKX_REG_ST4_1		0x43
#define ETZKX_REG_ST5_1		0x44
#define ETZKX_REG_ST6_1		0x45
#define ETZKX_REG_ST7_1		0x46
#define ETZKX_REG_ST8_1		0x47
#define ETZKX_REG_ST9_1		0x48
#define ETZKX_REG_ST10_1	0x49
#define ETZKX_REG_ST11_1	0x4A
#define ETZKX_REG_ST12_1	0x4B
#define ETZKX_REG_ST13_1	0x4C
#define ETZKX_REG_ST14_1	0x4D
#define ETZKX_REG_ST15_1	0x4E
#define ETZKX_REG_ST16_1	0x4F
#define ETZKX_REG_TIM4_1	0x50
#define ETZKX_REG_TIM3_1	0x51
#define ETZKX_REG_TIM2_L_1	0x52
#define ETZKX_REG_TIM2_H_1	0x53
#define ETZKX_REG_TIM1_L_1	0x54
#define ETZKX_REG_TIM1_H_1	0x55
#define ETZKX_REG_THRS2_1	0x56
#define ETZKX_REG_THRS1_1	0x57
#define ETZKX_REG_SA_1		0x59
#define ETZKX_REG_MA_1		0x5A
#define ETZKX_REG_SETT_1	0x5B
#define ETZKX_REG_PR_1		0x5C
#define ETZKX_REG_TC_L_1	0x5D
#define ETZKX_REG_TC_H_1	0x5E
#define ETZKX_REG_OUTS_1	0x5F

#define ETZKX_REG_ST1_2		0x60
#define ETZKX_REG_ST2_2		0x61
#define ETZKX_REG_ST3_2		0x62
#define ETZKX_REG_ST4_2		0x63
#define ETZKX_REG_ST5_2		0x64
#define ETZKX_REG_ST6_2		0x65
#define ETZKX_REG_ST7_2		0x66
#define ETZKX_REG_ST8_2		0x67
#define ETZKX_REG_ST9_2		0x68
#define ETZKX_REG_ST10_2	0x69
#define ETZKX_REG_ST11_2	0x6A
#define ETZKX_REG_ST12_2	0x6B
#define ETZKX_REG_ST13_2	0x6C
#define ETZKX_REG_ST14_2	0x6D
#define ETZKX_REG_ST15_2	0x6E
#define ETZKX_REG_ST16_2	0x6F
#define ETZKX_REG_TIM4_2	0x70
#define ETZKX_REG_TIM3_2	0x71
#define ETZKX_REG_TIM2_L_2	0x72
#define ETZKX_REG_TIM2_H_2	0x73
#define ETZKX_REG_TIM1_L_2	0x74
#define ETZKX_REG_TIM1_H_2	0x75
#define ETZKX_REG_THRS2_2	0x76
#define ETZKX_REG_THRS1_2	0x77
#define ETZKX_REG_DES_2		0x78
#define ETZKX_REG_SA_2		0x79
#define ETZKX_REG_MA_2		0x7A
#define ETZKX_REG_SETT_2	0x7B
#define ETZKX_REG_PR_2		0x7C
#define ETZKX_REG_TC_L_2	0x7D
#define ETZKX_REG_TC_H_2	0x7E
#define ETZKX_REG_OUTS_2	0x7F

/* registers mask */
/* CNTL1 masks */
#define ETZKX_CNTL1_IEN_MASK		1
#define ETZKX_CNTL1_ODR_MASK		(7 << 2)
#define ETZKX_CNTL1_G_MASK		(3 << 5)
#define ETZKX_CNTL1_PC_MASK		(1 << 7)

/* CNTL2/CNTL3 masks */
#define ETZKX_CNTLX_SMX_EN_MASK		1
#define ETZKX_CNTLX_SMX_PIN_MASK	(1 << 3)

/* CNTL4 masks */
#define ETZKX_CNTL4_STRT_MASK		1
#define ETZKX_CNTL4_STP_MASK		(1 << 1)
#define ETZKX_CNTL4_VFILT_MASK		(1 << 2)
#define ETZKX_CNTL4_INT1_EN_MASK	(1 << 3)
#define ETZKX_CNTL4_INT2_EN_MASK	(1 << 4)
#define ETZKX_CNTL4_IEL_MASK		(1 << 5)
#define ETZKX_CNTL4_IEA_MASK		(1 << 6)
#define ETZKX_CNTL4_DR_EN_MASK		(1 << 7)

/* SETT_X masks */
#define ETZKX_SETT_D_CS_MASK		(1 << 3)
#define ETZKX_SETT_RADI_MASK		(1 << 4)

/* ODR masks */
#define ETZKX_ODR_3_125_MASK		(ETZKX_ODR_3_125 << 2)
#define ETZKX_ODR_6_25_MASK		(ETZKX_ODR_6_25  << 2)
#define ETZKX_ODR_12_5_MASK		(ETZKX_ODR_12_5  << 2)
#define ETZKX_ODR_25_MASK		(ETZKX_ODR_25    << 2)
#define ETZKX_ODR_50_MASK		(ETZKX_ODR_50    << 2)
#define ETZKX_ODR_100_MASK		(ETZKX_ODR_100   << 2)
#define ETZKX_ODR_400_MASK		(ETZKX_ODR_400   << 2)
#define ETZKX_ODR_1600_MASK		(ETZKX_ODR_1600  << 2)

/* etzkx driver states */
#define ETZKX_STATE_POWER_OFF		0
#define ETZKX_STATE_STDBY		1
#define ETZKX_STATE_ACTIVE		(1 << 1)
#define ETZKX_STATE_STRM		(1 << 2)
#define ETZKX_STATE_STM_1		(1 << 3)
#define ETZKX_STATE_STM_2		(1 << 4)
#define ETZKX_STATE_DRDY		(1 << 5)
#define ETZKX_STATE_SELF_TEST		(1 << 6)
#define ETZKX_STATE_STRM_STM1		(ETZKX_STATE_STRM | ETZKX_STATE_STM_1)
#define ETZKX_STATE_STRM_STM2		(ETZKX_STATE_STRM | ETZKX_STATE_STM_2)
#define ETZKX_STATE_STM1_STM2		(ETZKX_STATE_STM_1 | ETZKX_STATE_STM_2)
#define ETZKX_STATE_STRM_STM1_STM2	(ETZKX_STATE_STRM | \
					ETZKX_STATE_STM_1 | \
					ETZKX_STATE_STM_2)
#define ETZKX_STATE_DRDY_STM2		(ETZKX_STATE_DRDY | ETZKX_STATE_STM_2)

#define ETZKX_MAX_POLL_RATE		65535

/* State machine definitions */
#define ETZKX_STM_MAX_STEP		16
#define ETZKX_STM_LEN			28
#define ETZKX_NO_STM_RUNNING		0xFF
#define ETZKX_STM_REG_GAP		(ETZKX_REG_ST1_2 - ETZKX_REG_ST1_1)

/* algo selection algorithm */
#define ETZKX_STM_MATCH_ID		1
#define ETZKX_STM_MATCH_ODR		2
#define ETZKX_STM_MATCH_RANGE		4
#define ETZKX_STM_MATCH_OK		(ETZKX_STM_MATCH_ID | \
					ETZKX_STM_MATCH_ODR | \
					ETZKX_STM_MATCH_RANGE)

/* Instructions set: Next/Reset conditions */
#define ETZKX_NOP			0x0
#define ETZKX_TI1			0x1
#define ETZKX_TI2			0x2
#define ETZKX_TI3			0x3
#define ETZKX_TI4			0x4
#define ETZKX_GNTH1			0x5
#define ETZKX_GNTH2			0x6
#define ETZKX_LNTH1			0x7
#define ETZKX_LNTH2			0x8
#define ETZKX_GTTH1			0x9
#define ETZKX_LLTH2			0xA
#define ETZKX_GRTH1			0xB
#define ETZKX_LRTH1			0xC
#define ETZKX_GRTH2			0xD
#define ETZKX_LRTH2			0xE
#define ETZKX_NZERO			0xF
/* Instruction set: Commands */
#define ETZKX_STOP			0x00
#define ETZKX_CONT			0x11
#define ETZKX_JMP			0x22
#define ETZKX_SRP			0x33
#define ETZKX_CRP			0x44
#define ETZKX_SETP			0x55
#define ETZKX_SETS1			0x66
#define ETZKX_STHR1			0x77
#define ETZKX_OUTC			0x88
#define ETZKX_OUTW			0x99
#define ETZKX_STHR2			0xAA
#define ETZKX_DEC			0xBB
#define ETZKX_SISW			0xCC
#define ETZKX_REL			0xDD
#define ETZKX_STHR3			0xEE
#define ETZKX_SSYNC			0xFF

/* specific state machine definitions */
/* orientation algorithm */
#define ETZKX_ORIENTATION_PORTRAIT	0x20
#define ETZKX_ORIENTATION_LANDSCAPE	0x80

/* double tap algorithm */
#define ETZKX_DOUBLE_TAP_PLUS_X		0x40
#define ETZKX_DOUBLE_TAP_MINUS_X	0x80
#define ETZKX_DOUBLE_TAP_PLUS_Y		0x10
#define ETZKX_DOUBLE_TAP_MINUS_Y	0x20
#define ETZKX_DOUBLE_TAP_PLUS_Z		0x04
#define ETZKX_DOUBLE_TAP_MINUS_Z	0x08

/* accelerometer thresholds for landscape detection */
#define ETZKX_ORIENTATION_LIMIT		150
#define ETZKX_MAX_RANGE			8

/* accelerometer dimension x, y, z, v */
#define ETZKX_DIMENSION			8

#define ETZKX_ALGO_DES_IDX		24
#define ETZKX_ALGO_MASK_IDX		25
#define ETZKX_ALGO_SETT_IDX		27

#define etzkx_load_stm1(s, i)		__etzkx_load_stm(s, i, 0)
#define etzkx_load_stm2(s, i)		__etzkx_load_stm(s, i, \
						ETZKX_STM_REG_GAP)
#define etzkx_enable_stm1(s)		__etzkx_enable_stm(s, ETZKX_REG_CNTL2)
#define etzkx_enable_stm2(s)		__etzkx_enable_stm(s, ETZKX_REG_CNTL3)
#define etzkx_disable_stm1(s)		__etzkx_disable_stm(s, ETZKX_REG_CNTL2)
#define etzkx_disable_stm2(s)		__etzkx_disable_stm(s, ETZKX_REG_CNTL3)
#define etzkx_switch_on_vfilter(s, i)	__etzkx_switch_vfilter(s, i, 1)
#define etzkx_switch_off_vfilter(s, i)	__etzkx_switch_vfilter(s, i, 0)

/* conditions for an algorithm to be loaded in state machine 2 */
#define ETZKX_ALGO_STM2(s, o)	((s[ETZKX_ALGO_SETT_IDX] & \
						ETZKX_SETT_RADI_MASK) || \
				((o == 1600) && (s[ETZKX_ALGO_DES_IDX])))

/* WAI register values */
#define ETZKX_WIA_LISN3DSH		0x3B
#define ETZKX_WIA_KXTNK			0x6E
#define ETZKX_WIA_KXCNL			0x0B

/* ETZKX accelerometer type names */
#define ETZKX_LISN3DSH_NAME		"lisn3dsh"
#define ETZKX_KXTNK_NAME		"kxtnk-1000"

#define ETZKX_CHARDEV_NAME		"etzkx_stm"

struct etzkx_data {
	struct i2c_client *client;
	struct etzkx_platform_data *pdata;
	struct mutex mutex;

	struct input_dev *input_dev;
	struct delayed_work poll_read_work;

	u8 wai;
	u8 hw_version;
	u8 drv_state;

	u16 poll_rate;
	u16 odr;
	u8 range;

	u8 stm1;
	u8 stm2;
	u8 range_back;
	u8 mask_matrix[ETZKX_DIMENSION];

	struct cdev cdev;
	struct etzkx_stm_data running_stm;
	wait_queue_head_t wq;
};

struct etzkx_algo_data {
	u8 stm_id;
	u8 stm[ETZKX_STM_LEN];
	u8 thrs3;
	u8 v[4];
	u8 odr;
	u8 range;
};

static const struct etzkx_algo_data etzkx_algos[] = {
	{	ETZKX_STM_ID_TIMING,
		{	ETZKX_NOP << 4 | ETZKX_TI3,	/* 0 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 1 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 2 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 3 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 4 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 5 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 6 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 7 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 8 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 9 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 10 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 11 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 12 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 13 */
			ETZKX_NOP << 4 | ETZKX_TI3,	/* 14 */
			ETZKX_CONT,			/* 15 */
			0x00, /* TIM4 */
			0x02, /* TIM3 */
			0x00, /* TIM2 high */
			0x00, /* TIM2 low */
			0x00, /* TIM1 high */
			0x00, /* TIM1 low */
			0x00, /* THRS2 */
			0x00, /* THRS1 */
			0x00, /* DES */
			0x00, /* mask 2 */
			0x00, /* mask 1 */
			0x01, /* settings */
		},
		0x00, /* THRS3 */
		{0x00, 0x00, 0x00, 0x00}, /* V filter */
		ETZKX_ODR_DONT_CARE,
		ETZKX_G_RANGE_DONT_CARE,
	},
	{	/* orientation, 1600Hz, 4g */
		ETZKX_STM_ID_ORIENTATION,
		{	ETZKX_NOP   << 4 | ETZKX_GNTH1,	/* 0 */
			ETZKX_TI2   << 4 | ETZKX_TI4,	/* 1 */
			ETZKX_TI1   << 4 | ETZKX_LNTH1,	/* 2 */
			ETZKX_GNTH1 << 4 | ETZKX_TI2,	/* 3 */
			ETZKX_TI1   << 4 | ETZKX_TI4,	/* 4 */
			ETZKX_LNTH2 << 4 | ETZKX_TI3,	/* 5 */
			ETZKX_LNTH2 << 4 | ETZKX_LNTH2,	/* 6 */
			ETZKX_TI3   << 4 | ETZKX_TI3,	/* 7 */
			ETZKX_TI2   << 4 | ETZKX_TI4,	/* 8 */
			ETZKX_NOP   << 4 | ETZKX_GNTH1,	/* 9 */
			ETZKX_TI1   << 4 | ETZKX_TI4,	/* 10 */
			ETZKX_TI1   << 4 | ETZKX_LNTH1,	/* 11 */
			ETZKX_GNTH1 << 4 | ETZKX_TI2,	/* 12 */
			ETZKX_TI2   << 4 | ETZKX_TI4,	/* 13 */
			ETZKX_LNTH2 << 4 | ETZKX_TI3,	/* 14 */
			ETZKX_TI1   << 4 | ETZKX_TI1,	/* 15 */
			0x00, /* TIM4 */
			0xa0, /* TIM3 */
			0x78, /* TIM2 high */
			0x01, /* TIM2 low */
			0xa0, /* TIM1 high */
			0x00, /* TIM1 low */
			0x10, /* THRS2 */
			0x0b, /* THRS1 */
			0x00, /* DES */
			0x20, /* mask 2 */
			0x80, /* mask 1 */
			0x21, /* settings */
		},
		0x00, /* THRS3 */
		{0x00, 0x00, 0x00, 0x00}, /* V filter */
		ETZKX_ODR_1600,
		ETZKX_G_RANGE_4G,
	},
	{	/* double tap, 1600Hz, 4g */
		ETZKX_STM_ID_DOUBLE_TAP,
		{	ETZKX_GNTH1 << 4 | ETZKX_TI1,   /* 0 */
			ETZKX_GNTH1 << 4 | ETZKX_TI1,   /* 1 */
			ETZKX_NOP   << 4 | ETZKX_GNTH2, /* 2 */
			ETZKX_TI3   << 4 | ETZKX_LNTH2, /* 3 */
			ETZKX_NOP   << 4 | ETZKX_TI4,   /* 4 */
			ETZKX_GTTH1 << 4 | ETZKX_TI1,   /* 5 */
			ETZKX_TI2   << 4 | ETZKX_GNTH2, /* 6 */
			ETZKX_TI3   << 4 | ETZKX_LNTH2, /* 7 */
			ETZKX_NOP   << 4 | ETZKX_TI4,   /* 8 */
			ETZKX_GTTH1 << 4 | ETZKX_TI1,   /* 9 */
			ETZKX_TI1   << 4 | ETZKX_TI1,   /* 10 */
			ETZKX_NOP   << 4 | ETZKX_NOP,   /* 11 */
			ETZKX_NOP   << 4 | ETZKX_NOP,   /* 12 */
			ETZKX_NOP   << 4 | ETZKX_NOP,   /* 13 */
			ETZKX_NOP   << 4 | ETZKX_NOP,   /* 14 */
			ETZKX_NOP   << 4 | ETZKX_NOP,   /* 15 */
			0x20, /* TIM4 */
			0x0d, /* TIM3 */
			0x20, /* TIM2 high */
			0x03, /* TIM2 low */
			0x70, /* TIM1 high */
			0x00, /* TIM1 low */
			0x44, /* THRS2 */
			0x3a, /* THRS1 */
			0x00, /* DES */
			0x00, /* mask 2 */
			0xfc, /* mask 1 */
			0xa1, /* settings */
		},
		0x00, /* THRS3 */
		{0x00, 0x00, 0x00, 0x00}, /* V filter */
		ETZKX_ODR_1600,
		ETZKX_G_RANGE_4G,
	},
	{	/* double tap, 1600Hz, 4g */
		ETZKX_STM_ID_WAKEUP,
		{
			ETZKX_NOP   << 4 | ETZKX_TI3,	/* 0 */
			ETZKX_GNTH1 << 4 | ETZKX_TI1,	/* 1 */
			ETZKX_NOP   << 4 | ETZKX_OUTC,	/* 2 */
			ETZKX_NOP   << 4 | ETZKX_GNTH1,	/* 3 */
			ETZKX_CONT,			/* 4 */
			ETZKX_NOP,			/* 5 */
			ETZKX_NOP,			/* 6 */
			ETZKX_NOP,			/* 7 */
			ETZKX_NOP,			/* 8 */
			ETZKX_NOP,			/* 9 */
			ETZKX_NOP,			/* 10 */
			ETZKX_NOP,			/* 11 */
			ETZKX_NOP,			/* 12 */
			ETZKX_NOP,			/* 13 */
			ETZKX_NOP,			/* 14 */
			ETZKX_NOP,			/* 15 */
			0x00, /* TIM4 */
			0x00, /* TIM3 */
			0x00, /* TIM2 high */
			0x00, /* TIM2 low */
			0x00, /* TIM1 high */
			0x10, /* TIM1 low */
			0x00, /* THRS2 */
			0x03, /* THRS1 */
			0x00, /* DES */
			0x00, /* mask 2 */
			0xFC, /* mask 1 */
			0x31, /* settings */
		},
		0x00, /* THRS3 */
		{0x00, 0x00, 0x00, 0x00}, /* V filter */
		ETZKX_ODR_1600,
		ETZKX_G_RANGE_4G,
	},
	{	/* double tap based on V filter */
		ETZKX_STM_ID_V_DOUBLE_TAP,
		{
			ETZKX_GNTH1 << 4 | ETZKX_TI1,	/* 0 */
			ETZKX_GNTH1 << 4 | ETZKX_TI1,	/* 1 */
			ETZKX_NOP   << 4 | ETZKX_GNTH2,	/* 2 */
			ETZKX_TI3   << 4 | ETZKX_LNTH2,	/* 3 */
			ETZKX_NOP   << 4 | ETZKX_TI4,	/* 4 */
			ETZKX_GTTH1 << 4 | ETZKX_TI1,	/* 5 */
			ETZKX_TI2   << 4 | ETZKX_GNTH2,	/* 6 */
			ETZKX_TI3   << 4 | ETZKX_LNTH2,	/* 7 */
			ETZKX_NOP   << 4 | ETZKX_TI4,	/* 8 */
			ETZKX_GTTH1 << 4 | ETZKX_TI1,	/* 9 */
			ETZKX_TI1   << 4 | ETZKX_TI1,	/* 10 */
			ETZKX_NOP,			/* 11 */
			ETZKX_NOP,			/* 12 */
			ETZKX_NOP,			/* 13 */
			ETZKX_NOP,			/* 14 */
			ETZKX_NOP,			/* 15 */
			0x98, /* TIM4 */
			0x0d, /* TIM3 */
			0x20, /* TIM2 high */
			0x03, /* TIM2 low */
			0x70, /* TIM1 high */
			0x00, /* TIM1 low */
			0x05, /* THRS2 */
			0x03, /* THRS1 */
			0x00, /* DES */
			0xf0, /* mask 2 */
			0x03, /* mask 1 */
			0xe1, /* settings */
		},
		0x46, /* THRS3 */
		{0x09, 0x27, 0x57, 0x7f}, /* V filter */
		ETZKX_ODR_1600,
		ETZKX_G_RANGE_4G,
	},
};

/* driver state prototypes */
static int etzkx_state_go_stdby(struct etzkx_data *);
static int etzkx_state_go_active(struct etzkx_data *);
static int etzkx_state_enable_streaming(struct etzkx_data *);
static int etzkx_state_disable_streaming(struct etzkx_data *);
static int etzkx_state_enable_st(struct etzkx_data *);
static int etzkx_state_disable_st(struct etzkx_data *);
static int etzkx_state_enable_stm1(struct etzkx_data *, u8, u8);
static int etzkx_state_enable_stm2(struct etzkx_data *, u8, u8);
static int etzkx_state_disable_stm1(struct etzkx_data *, u8);
static int etzkx_state_disable_stm2(struct etzkx_data *, u8);

static const u16 etzkx_rate_ms[] = {320, 160, 80, 40, 20, 10, 2, 0};
static const u16 etzkx_rate_hz[] = {3, 6, 12, 25, 50, 100, 400, 1600};
static const char * const etzkx_rate_str[] = {"3.125", "6.25", "12.5", "25",
					"50", "100", "400", "1600"};
static dev_t etzkx_dev_number;
static struct class *etzkx_class;

/* get mutex lock before calling set_odr and set_range */
static int etzkx_set_odr(struct etzkx_data *sdata, u16 new_odr)
{
	u8 odr_mask;
	s32 cntl1_reg;

	if (sdata->odr == new_odr)
		return 0;

	if ((new_odr < ETZKX_ODR_3_125) || (new_odr > ETZKX_ODR_1600))
		return -EINVAL;

	odr_mask = new_odr << 2;

	cntl1_reg = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL1);
	if (cntl1_reg < 0)
		return -EIO;
	cntl1_reg = (cntl1_reg & ~ETZKX_CNTL1_ODR_MASK) | odr_mask;
	cntl1_reg = i2c_smbus_write_byte_data(sdata->client,
						ETZKX_REG_CNTL1, cntl1_reg);
	if (cntl1_reg < 0)
		return -EIO;

	sdata->odr = new_odr;

	return 0;
}

static int etzkx_set_range(struct etzkx_data *sdata, u8 new_range)
{
	u8 new_mask_range;
	s32 cntl1_val;

	if ((new_range != ETZKX_G_RANGE_2G) &&
		(new_range != ETZKX_G_RANGE_4G) &&
		(new_range != ETZKX_G_RANGE_6G) &&
		(new_range != ETZKX_G_RANGE_8G))
		return -EINVAL;

	cntl1_val = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL1);
	if (cntl1_val < 0)
		return cntl1_val;
	cntl1_val &= ~ETZKX_CNTL1_G_MASK;
	new_mask_range = (new_range / 2 - 1) << 5;

	cntl1_val = i2c_smbus_write_byte_data(sdata->client, ETZKX_REG_CNTL1,
				new_mask_range | cntl1_val);
	if (cntl1_val < 0)
		return -EIO;

	sdata->range = new_range;

	return 0;
}

/* get mutex lock before calling state functions */
static int etzkx_state_go_stdby(struct etzkx_data *sdata)
{
	s32 cntl1_reg;

	cntl1_reg = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL1);
	if (cntl1_reg < 0)
		return cntl1_reg;
	cntl1_reg &= ~ETZKX_CNTL1_PC_MASK;
	cntl1_reg = i2c_smbus_write_byte_data(sdata->client, ETZKX_REG_CNTL1,
				cntl1_reg);
	if (cntl1_reg < 0)
		return cntl1_reg;

	usleep_range(500, 700);
	sdata->drv_state = ETZKX_STATE_STDBY;
	sdata->stm1 = ETZKX_NO_STM_RUNNING;
	sdata->stm2 = ETZKX_NO_STM_RUNNING;

	return 0;
}

static int etzkx_state_go_active(struct etzkx_data *sdata)
{
	s32 reg;

	switch (sdata->drv_state) {
	case ETZKX_STATE_STDBY:
		reg = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL1);
		reg &= ETZKX_CNTL1_G_MASK | ETZKX_CNTL1_ODR_MASK;
		reg |= ETZKX_CNTL1_PC_MASK | sdata->odr << 2 |
						(sdata->range / 2 - 1) << 5;
		reg = i2c_smbus_write_byte_data(sdata->client,
							ETZKX_REG_CNTL1, reg);
		if (reg)
			return reg;

		usleep_range(500, 600);
		sdata->drv_state = ETZKX_STATE_ACTIVE;

	case ETZKX_STATE_SELF_TEST:
		etzkx_state_disable_st(sdata);
	case ETZKX_STATE_STRM:
		etzkx_state_disable_streaming(sdata);
		sdata->drv_state = ETZKX_STATE_ACTIVE;
		return 0;
	case ETZKX_STATE_STM_1:
		return etzkx_state_disable_stm1(sdata,
					etzkx_algos[sdata->stm1].stm_id);
	case ETZKX_STATE_STM_2:
		return etzkx_state_disable_stm2(sdata,
					etzkx_algos[sdata->stm2].stm_id);
	}
	return -EPERM;
}

static int etzkx_state_enable_streaming(struct etzkx_data *sdata)
{
	int err;

	if (sdata->drv_state & ETZKX_STATE_STRM)
		return 0;

	switch (sdata->drv_state) {
	case ETZKX_STATE_STDBY:
		err = etzkx_state_go_active(sdata);
		if (err)
			return err;
	case ETZKX_STATE_ACTIVE:
		if (sdata->odr > ETZKX_ODR_100) {
			err = etzkx_set_odr(sdata, ETZKX_ODR_100);
			sdata->poll_rate = etzkx_rate_ms[ETZKX_ODR_100];
		}
		schedule_delayed_work(&sdata->poll_read_work,
			msecs_to_jiffies(sdata->poll_rate));
		sdata->drv_state = ETZKX_STATE_STRM;
		return 0;
	case ETZKX_STATE_SELF_TEST:
		err = etzkx_state_disable_st(sdata);
		return err ? err : 0;
	case ETZKX_STATE_STM_1:
	case ETZKX_STATE_STM_2:
	case ETZKX_STATE_STM1_STM2:
		schedule_delayed_work(&sdata->poll_read_work,
			msecs_to_jiffies(sdata->poll_rate));
		sdata->drv_state |= ETZKX_STATE_STRM;
		return 0;
	}
	return -EPERM;
}

static int etzkx_state_disable_streaming(struct etzkx_data *sdata)
{
	int err;

	switch (sdata->drv_state) {
	case ETZKX_STATE_POWER_OFF:
	case ETZKX_STATE_STDBY:
	case ETZKX_STATE_ACTIVE:
	case ETZKX_STATE_STM_1:
	case ETZKX_STATE_STM_2:
	case ETZKX_STATE_STM1_STM2:
		return 0;
	case ETZKX_STATE_SELF_TEST:
		err = etzkx_state_disable_st(sdata);
		if (err)
			return err;
	case ETZKX_STATE_STRM:
	case ETZKX_STATE_STRM_STM1:
	case ETZKX_STATE_STRM_STM2:
	case ETZKX_STATE_STRM_STM1_STM2:
		cancel_delayed_work_sync(&sdata->poll_read_work);
		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_STRM) ?
					ETZKX_STATE_ACTIVE :
					(sdata->drv_state & ~ETZKX_STATE_STRM);
		return 0;
	}

	return -EPERM;
}

static int etzkx_state_enable_st(struct etzkx_data *sdata)
{
	int reg;

	switch (sdata->drv_state) {
	case ETZKX_STATE_STRM:
	case ETZKX_STATE_POWER_OFF:
	case ETZKX_STATE_STDBY:
		reg = etzkx_state_go_active(sdata);
		if (reg)
			return reg;
	case ETZKX_STATE_ACTIVE:
		if (sdata->drv_state == ETZKX_STATE_STRM)
			cancel_delayed_work_sync(&sdata->poll_read_work);

		if (sdata->odr > ETZKX_ODR_100)
			return -EPERM;

		reg = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL4);
		if (reg < 0)
			return reg;

		reg |= ETZKX_CNTL4_STP_MASK;
		reg = i2c_smbus_write_byte_data(sdata->client,
						ETZKX_REG_CNTL4, reg);
		if (reg < 0)
			return reg;
		/*
		 * schedule the polling 4 odr later, due to some
		 * hardware limitations
		 */
		schedule_delayed_work(&sdata->poll_read_work,
			4*msecs_to_jiffies(etzkx_rate_ms[sdata->odr]));
		sdata->drv_state = ETZKX_STATE_SELF_TEST;

		return 0;
	}
	return -EPERM;
}

static int etzkx_state_disable_st(struct etzkx_data *sdata)
{
	int reg;

	if (sdata->drv_state != ETZKX_STATE_SELF_TEST)
		return 0;

	reg = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL4);
	if (reg < 0)
		return reg;

	reg &= ~ETZKX_CNTL4_STP_MASK;
	reg = i2c_smbus_write_byte_data(sdata->client, ETZKX_REG_CNTL4, reg);
	if (reg < 0)
		return 0;

	sdata->drv_state = ETZKX_STATE_STRM;

	return 0;
}

/*
 * etzkx_select_stm - selects the desired state machine
 * @sdata: running odr and g range value
 * @stm_id: desired state machine
 * @match_bitops: pointer to the bit operations that has the following meaning:
 *
 * 000: no stm found
 * 001: g range and odr must be changed
 * 011: g range must be changed
 * 101: odr must be changed
 * 111: the state machine can be applied without any change
 *
 * The return value is the index of the algo from the global algorithm vector.
 * Lower than 0 if an error occured.
 *
 * The matchings have a different relevance that are weighed:
 * 3: algo id (redundant)
 * 2: odr
 * 1: range
 */
static u8 etzkx_select_stm(struct etzkx_data *sdata,
			int stm_id, u8 *match_bitops)
{
	u8 i, algo_idx;
	u8 match, final_match;

	for (i = 0, match = 0, *match_bitops = 0, algo_idx = 0, final_match = 0;
		(i < ARRAY_SIZE(etzkx_algos)) &&
		(*match_bitops != ETZKX_STM_MATCH_OK);
			i++, match = 0) {

		if (etzkx_algos[i].stm_id != stm_id)
			continue;

		match += 3;
		if (match > final_match) {
			*match_bitops |= ETZKX_STM_MATCH_ID;
			algo_idx = i;
			final_match = match;
		}

		if ((etzkx_algos[i].odr == sdata->odr) ||
			(etzkx_algos[i].odr == ETZKX_ODR_DONT_CARE) ||
			((sdata->odr == etzkx_rate_hz[ETZKX_ODR_1600]) &&
			(etzkx_algos[i].stm[ETZKX_ALGO_DES_IDX]))) {

			match += 2;
			if (match > final_match) {
				final_match = match;
				algo_idx = i;
				*match_bitops |= ETZKX_STM_MATCH_ODR;
			}
		}

		if ((etzkx_algos[i].range == sdata->range) ||
			(etzkx_algos[i].range ==
				ETZKX_G_RANGE_DONT_CARE)) {
			match++;
			if (match > final_match) {
				final_match = match;
				algo_idx = i;
				*match_bitops |= ETZKX_STM_MATCH_RANGE;
			}
		}
	}

	return algo_idx;
}

static u8 etzkx_mask_orientation(struct etzkx_data *sdata, u8 val)
{
	int i;
	u8 new_val = 0;

	if (!val)
		return 0;

	for (i = 0; i < ETZKX_DIMENSION; i++)
		if (sdata->mask_matrix[i] & val)
			new_val |= (1 << (ETZKX_DIMENSION - 1 - i));

	return new_val;
}

static int __etzkx_switch_vfilter(struct etzkx_data *sdata, u8 stm_id, u8 onoff)
{
	s32 cntl4, err;

	cntl4 = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL4);

	if (onoff) {
		err = i2c_smbus_write_i2c_block_data(sdata->client,
							ETZKX_REG_VFC_1, 4,
							etzkx_algos[stm_id].v);
		if (err)
			return err;

		cntl4 |= ETZKX_CNTL4_VFILT_MASK;
	} else {
		cntl4 &= ~ETZKX_CNTL4_VFILT_MASK;
	}

	cntl4 = i2c_smbus_write_byte_data(sdata->client,
					ETZKX_REG_CNTL4, cntl4);

	return (cntl4 < 0) ? cntl4 : 0;
}

static int __etzkx_load_stm(struct etzkx_data *sdata, u8 stm_id, u8 offset)
{
	s32 err;
	u8 mask[2];
	u8 idx;

	idx = !offset ? sdata->stm1 : sdata->stm2;

	if (etzkx_algos[stm_id].thrs3 &&
		(idx == ETZKX_NO_STM_RUNNING || !etzkx_algos[idx].thrs3)) {

		err = i2c_smbus_write_byte_data(sdata->client, ETZKX_REG_THRS3,
						etzkx_algos[stm_id].thrs3);
		if (err)
			return err;
	}

	if (*((u32 *) etzkx_algos[stm_id].v) &&
					(idx == ETZKX_NO_STM_RUNNING ||
					(*((u32 *) etzkx_algos[idx].v))))

		etzkx_switch_on_vfilter(sdata, stm_id);

	err = i2c_smbus_write_i2c_block_data(sdata->client,
						ETZKX_REG_ST1_1 + offset,
						ETZKX_STM_LEN,
						etzkx_algos[stm_id].stm);
	if (err)
		return err;


	mask[0] = etzkx_mask_orientation(sdata,
			etzkx_algos[stm_id].stm[ETZKX_ALGO_MASK_IDX]);
	mask[1] = etzkx_mask_orientation(sdata,
			etzkx_algos[stm_id].stm[ETZKX_ALGO_MASK_IDX+1]);

	err = i2c_smbus_write_i2c_block_data(sdata->client,
					ETZKX_REG_SA_1 + offset, 2, mask);

	return (err < 0) ? err : 0;
}

static int __etzkx_enable_stm(struct etzkx_data *sdata, u8 reg)
{
	int cntl1, cntl4, cntlx;

	cntl1 = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL1);
	if (cntl1 < 0)
		return cntl1;
	cntlx = i2c_smbus_read_byte_data(sdata->client, reg);
	if (cntlx < 0)
		return cntlx;
	cntl4 = i2c_smbus_read_byte_data(sdata->client, ETZKX_REG_CNTL4);
	if (cntl4 < 0)
		return cntl4;

	cntl1 |= ETZKX_CNTL1_IEN_MASK;
	cntlx |= ETZKX_CNTLX_SMX_EN_MASK;
	cntl4 |= ETZKX_CNTL4_IEA_MASK |	ETZKX_CNTL4_IEL_MASK; /* pulsed irq */

	if (reg == ETZKX_REG_CNTL2)
		cntl4 |= ETZKX_CNTL4_INT1_EN_MASK;
	else if (reg == ETZKX_REG_CNTL3) {
		cntl4 |= ETZKX_CNTL4_INT2_EN_MASK;
		cntlx |= ETZKX_CNTLX_SMX_PIN_MASK;
	} else
		return -EINVAL;

	cntl1 = i2c_smbus_write_byte_data(sdata->client,
						ETZKX_REG_CNTL1, cntl1);
	if (cntl1 < 0)
		return cntl1;
	cntl4 = i2c_smbus_write_byte_data(sdata->client,
						ETZKX_REG_CNTL4, cntl4);
	if (cntl4 < 0)
		return cntl4;
	cntlx = i2c_smbus_write_byte_data(sdata->client, reg, cntlx);
	if (cntlx < 0)
		return cntlx;

	return 0;
}

static int __etzkx_disable_stm(struct etzkx_data *sdata, u8 reg)
{
	s32 cntlx;

	cntlx = i2c_smbus_read_byte_data(sdata->client, reg);
	if (cntlx < 0)
		return cntlx;

	cntlx &= ~ETZKX_CNTLX_SMX_EN_MASK;
	cntlx = i2c_smbus_write_byte_data(sdata->client, reg, cntlx);

	return (cntlx) ? cntlx : 0;
}

static int etzkx_set_stm_params(struct etzkx_data *sdata,
				u8 match_bitops, u8 new_odr, u8 new_range)
{
	int ret;
	u8 odr_back = sdata->odr;
	sdata->range_back = sdata->range;

	/*
	 * '0' means that odr/range doesn't match with the running
	 * config, therefore, needs to be changed
	 */
	if (!(match_bitops & ETZKX_STM_MATCH_ODR)) {
		ret = etzkx_set_odr(sdata, new_odr);
		if (ret)
			return ret;
	}

	if (!(match_bitops & ETZKX_STM_MATCH_RANGE)) {
		ret = etzkx_set_range(sdata, new_range);
		if (ret)
			goto restore_odr;
	}

	return 0;

restore_odr:
	etzkx_set_odr(sdata, odr_back);

	return ret;
}

static int __etzkx_state_enable_stm(struct etzkx_data *sdata, u8 i, u8 state)
{
	int ret;

	switch (state) {
	case ETZKX_STATE_STM_1:
		ret = etzkx_load_stm1(sdata, i);
		return ret ? ret : etzkx_enable_stm1(sdata);

	case ETZKX_STATE_STM_2:
		ret = etzkx_load_stm2(sdata, i);
		return ret ? ret : etzkx_enable_stm2(sdata);
	}
	return -EINVAL;
}

static int etzkx_move_stm_2_to_1(struct etzkx_data *sdata)
{
	int err;

	err = etzkx_disable_stm2(sdata);
	if (err)
		return err;

	err = etzkx_load_stm1(sdata, sdata->stm2);
	if (err)
		return err;

	err = etzkx_enable_stm1(sdata);
	if (err)
		return err;

	sdata->drv_state &= ~ETZKX_STATE_STM_2;
	sdata->drv_state |= ETZKX_STATE_STM_1;
	sdata->stm1 = sdata->stm2;
	sdata->stm2 = ETZKX_NO_STM_RUNNING;

	return 0;
}

static int etzkx_move_stm_1_to_2(struct etzkx_data *sdata)
{
	int err;

	err = etzkx_disable_stm1(sdata);
	if (err)
		return err;

	err = etzkx_load_stm2(sdata, sdata->stm1);
	if (err)
		return err;

	err = etzkx_enable_stm2(sdata);
	if (err)
		return err;

	sdata->drv_state &= ~ETZKX_STATE_STM_1;
	sdata->drv_state |= ETZKX_STATE_STM_2;
	sdata->stm2 = sdata->stm1;
	sdata->stm1 = ETZKX_NO_STM_RUNNING;

	return 0;
}

static int etzkx_state_enable_stm1(struct etzkx_data *sdata, u8 stm_id, u8 i)
{
	int ret;
	u8 match_bitops;

	if (!i) {
		i = etzkx_select_stm(sdata, stm_id, &match_bitops);
		if (!match_bitops)
			return 0;
	}

	switch (sdata->drv_state) {
	case ETZKX_STATE_STDBY:
		ret = etzkx_state_go_active(sdata);
		if (ret)
			return ret;

	case ETZKX_STATE_ACTIVE:
	case ETZKX_STATE_STRM:
		if (match_bitops) {
			ret = etzkx_set_stm_params(sdata, match_bitops,
							etzkx_algos[i].odr,
							etzkx_algos[i].range);
			if (ret)
				return ret;
		} else if ((etzkx_algos[i].odr != sdata->odr) ||
				(etzkx_algos[i].range != sdata->range))
				return -EPERM;

		if ((etzkx_algos[i].stm[ETZKX_ALGO_DES_IDX]) ||
			(etzkx_algos[i].stm[ETZKX_ALGO_SETT_IDX] &
							ETZKX_SETT_RADI_MASK))
			return etzkx_state_enable_stm2(sdata, stm_id, i);

		ret = __etzkx_state_enable_stm(sdata, i, ETZKX_STATE_STM_1);
		if (ret)
			return ret;

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_ACTIVE) ?
					ETZKX_STATE_STM_1 :
					ETZKX_STATE_STRM_STM1;
		sdata->stm1 = i;
		return 0;

	case ETZKX_STATE_STM_2:
	case ETZKX_STATE_STRM_STM2:
		if (etzkx_algos[i].range != sdata->range)
			return -EPERM;

		if (ETZKX_ALGO_STM2(etzkx_algos[i].stm, sdata->odr) &&
			!ETZKX_ALGO_STM2(etzkx_algos[sdata->stm2].stm,
								sdata->odr)) {
			ret = etzkx_move_stm_2_to_1(sdata);
			return ret ? ret :
				etzkx_state_enable_stm2(sdata, stm_id, i);
		} else if (sdata->odr != etzkx_algos[i].odr) {
			return -EPERM;
		}

		ret = __etzkx_state_enable_stm(sdata, i, ETZKX_STATE_STM_1);
		if (ret)
			return ret;

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_STM_2) ?
					ETZKX_STATE_STM1_STM2 :
					ETZKX_STATE_STRM_STM1_STM2;
		sdata->stm1 = i;
		return 0;
	}

	return -EPERM;
}

static int etzkx_state_enable_stm2(struct etzkx_data *sdata, u8 stm_id, u8 i)
{
	int ret;
	u8 match_bitops;

	if (!i) {
		i = etzkx_select_stm(sdata, stm_id, &match_bitops);
		if (!match_bitops)
			return 0;
	}

	switch (sdata->drv_state) {
	case ETZKX_STATE_STDBY:
		ret = etzkx_state_go_active(sdata);
		if (ret)
			return ret;

	case ETZKX_STATE_ACTIVE:
	case ETZKX_STATE_STRM:
		if (match_bitops) {
			ret = etzkx_set_stm_params(sdata, match_bitops,
							etzkx_algos[i].odr,
							etzkx_algos[i].range);
			if (ret)
				return ret;
		} else if ((etzkx_algos[i].odr != sdata->odr) ||
				(etzkx_algos[i].range != sdata->range)) {
				return -EPERM;
		}

		ret = __etzkx_state_enable_stm(sdata, i, ETZKX_STATE_STM_2);
		if (ret)
			return ret;

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_ACTIVE) ?
					ETZKX_STATE_STM_2 :
					ETZKX_STATE_STRM_STM2;
		sdata->stm2 = i;
		return 0;

	case ETZKX_STATE_STM_1:
	case ETZKX_STATE_STRM_STM1:
		/* check if the selected stm is compatible with stm2 */
		if ((etzkx_algos[i].range != sdata->range) &&
			(etzkx_algos[i].range != ETZKX_G_RANGE_DONT_CARE))
			return -EPERM;

		if (!ETZKX_ALGO_STM2(etzkx_algos[i].stm, etzkx_algos[i].odr) &&
			ETZKX_ALGO_STM2(etzkx_algos[sdata->stm1].stm,
					etzkx_rate_hz[ETZKX_ODR_1600])) {
			if ((etzkx_algos[i].odr ==
					etzkx_rate_hz[ETZKX_ODR_1600]) &&
				(sdata->odr !=
					etzkx_rate_hz[ETZKX_ODR_1600])) {

				ret = etzkx_set_odr(sdata,
						etzkx_rate_hz[ETZKX_ODR_1600]);
				if (ret)
					return ret;
			}
			ret = etzkx_move_stm_1_to_2(sdata);
			return ret ? ret :
				etzkx_state_enable_stm1(sdata, stm_id, i);
		} else if ((etzkx_algos[i].odr != sdata->odr) &&
				(etzkx_algos[i].odr != ETZKX_ODR_DONT_CARE)) {
			return -EPERM;
		}

		ret = __etzkx_state_enable_stm(sdata, i, ETZKX_STATE_STM_2);
		if (ret)
			return ret;

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_STM_1) ?
					ETZKX_STATE_STM1_STM2 :
					ETZKX_STATE_STRM_STM1_STM2;
		sdata->stm2 = i;
		return 0;
	}

	return -EPERM;
}

static int etzkx_restore_after_stm(struct etzkx_data *sdata)
{
	int i, err;

	for (i = 0;
		i < (ARRAY_SIZE(etzkx_rate_ms) - 2) &&
		(sdata->poll_rate < etzkx_rate_ms[i]);
			i++)
		;

	err = etzkx_set_odr(sdata, i);
	if (sdata->range != sdata->range_back)
		err |= etzkx_set_range(sdata, sdata->range_back);

	return (err) ? err : 0;
}
static int etzkx_state_disable_stm1(struct etzkx_data *sdata, u8 stm_id)
{
	int err;

	if (sdata->stm1 == ETZKX_NO_STM_RUNNING)
		return 0;
	if (stm_id != etzkx_algos[sdata->stm1].stm_id)
		return 0;

	switch (sdata->drv_state) {
	case ETZKX_STATE_STDBY:
	case ETZKX_STATE_ACTIVE:
		return 0;
	case ETZKX_STATE_STM_1:
	case ETZKX_STATE_STRM_STM1:
		err = etzkx_disable_stm1(sdata);
		if (err)
			return err;

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_STM_1) ?
					ETZKX_STATE_ACTIVE :
					ETZKX_STATE_STRM;
		sdata->stm1 = ETZKX_NO_STM_RUNNING;

		return etzkx_restore_after_stm(sdata);

	case ETZKX_STATE_STM1_STM2:
	case ETZKX_STATE_STRM_STM1_STM2:
		err = etzkx_disable_stm1(sdata);
		if (err)
			return err;

		if (*((u32 *) etzkx_algos[sdata->stm1].v)) {
			err = etzkx_switch_off_vfilter(sdata, sdata->stm2);
			if (err)
				return err;
		}

		if ((etzkx_algos[sdata->stm1].odr ==
					etzkx_rate_hz[ETZKX_ODR_1600]) &&
			etzkx_algos[sdata->stm2].stm[ETZKX_ALGO_DES_IDX])

			err = etzkx_set_odr(sdata,
						etzkx_algos[sdata->stm2].odr);

		else if (etzkx_algos[sdata->stm2].odr == ETZKX_ODR_DONT_CARE)
			err = etzkx_restore_after_stm(sdata);

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_STM1_STM2) ?
					ETZKX_STATE_STM_2 :
					ETZKX_STATE_STRM_STM2;
		sdata->stm1 = ETZKX_NO_STM_RUNNING;

		return err;
	}

	return -EPERM;
}

static int etzkx_state_disable_stm2(struct etzkx_data *sdata, u8 stm_id)
{
	int err;

	if (sdata->stm2 == ETZKX_NO_STM_RUNNING)
		return 0;
	if (stm_id != etzkx_algos[sdata->stm2].stm_id)
		return 0;

	switch (sdata->drv_state) {
	case ETZKX_STATE_STDBY:
	case ETZKX_STATE_ACTIVE:
		return 0;
	case ETZKX_STATE_STM_2:
	case ETZKX_STATE_STRM_STM2:
		err = etzkx_disable_stm2(sdata);
		if (err < 0)
			return err;

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_STM_2) ?
					ETZKX_STATE_ACTIVE :
					ETZKX_STATE_STRM;
		sdata->stm2 = ETZKX_NO_STM_RUNNING;

		return etzkx_restore_after_stm(sdata);

	case ETZKX_STATE_STM1_STM2:
	case ETZKX_STATE_STRM_STM1_STM2:
		err = etzkx_disable_stm2(sdata);
		if (err)
			return err;

		if (etzkx_algos[sdata->stm1].odr == ETZKX_ODR_DONT_CARE) {
			err = etzkx_restore_after_stm(sdata);
			if (err)
				return err;
		}

		if (*((u32 *) etzkx_algos[sdata->stm2].v))
			err = etzkx_switch_off_vfilter(sdata, sdata->stm2);

		sdata->drv_state = (sdata->drv_state == ETZKX_STATE_STM1_STM2) ?
					ETZKX_STATE_STM_1 :
					ETZKX_STATE_STRM_STM1;
		sdata->stm2 = ETZKX_NO_STM_RUNNING;

		return err;
	}

	return -EPERM;
}

static void etzkx_report_xyz(struct input_dev *dev, int *xyz)
{
	input_report_abs(dev, ABS_X, xyz[0]);
	input_report_abs(dev, ABS_Y, xyz[1]);
	input_report_abs(dev, ABS_Z, xyz[2]);
	input_sync(dev);
}

static int etzkx_read_xyz(struct etzkx_data *sdata, int *xyz)
{

	int err;
	u8 reg_xyz[6];
	s16 raw_xyz[3] = { 0 };

	err = i2c_smbus_read_i2c_block_data(sdata->client, ETZKX_REG_OUTX_L,
					6, reg_xyz);
	if (err != 6)
		return -EIO;

	raw_xyz[0] = ((s16) ((reg_xyz[1] << 8) | reg_xyz[0]));
	raw_xyz[1] = ((s16) ((reg_xyz[3] << 8) | reg_xyz[2]));
	raw_xyz[2] = ((s16) ((reg_xyz[5] << 8) | reg_xyz[4]));

	xyz[0] = ((sdata->pdata->x_negate) ? (-raw_xyz[sdata->pdata->x_map])
		   : (raw_xyz[sdata->pdata->x_map]));
	xyz[1] = ((sdata->pdata->y_negate) ? (-raw_xyz[sdata->pdata->y_map])
		   : (raw_xyz[sdata->pdata->y_map]));
	xyz[2] = ((sdata->pdata->z_negate) ? (-raw_xyz[sdata->pdata->z_map])
		   : (raw_xyz[sdata->pdata->z_map]));

	return 0;
}

static void etzkx_poll_read_work(struct work_struct *work)
{
	int err;
	int xyz[3];
	struct etzkx_data *etzkx = container_of((struct delayed_work *) work,
				struct etzkx_data, poll_read_work);

	err = etzkx_read_xyz(etzkx, xyz);
	if (err) {
		dev_err(&etzkx->client->dev, "i2c read/write error\n");
		mutex_lock(&etzkx->mutex);
		etzkx_state_disable_streaming(etzkx);
		mutex_unlock(&etzkx->mutex);
	} else {
		etzkx_report_xyz(etzkx->input_dev, xyz);
	}

	if (etzkx->drv_state & (ETZKX_STATE_STRM | ETZKX_STATE_SELF_TEST)) {
		schedule_delayed_work(&etzkx->poll_read_work,
			msecs_to_jiffies(etzkx->poll_rate));
	}
}

static int etzkx_stm_handle(struct etzkx_data *sdata, u8 i, u8 outs)
{
	switch (etzkx_algos[i].stm_id) {
	case ETZKX_STM_ID_TIMING:
		sdata->running_stm.id = ETZKX_STM_ID_TIMING;
		break;
	case ETZKX_STM_ID_ORIENTATION:
		sdata->running_stm.id = ETZKX_STM_ID_ORIENTATION;
		switch (outs) {
		case ETZKX_ORIENTATION_PORTRAIT:
			sdata->running_stm.algo.portrait  = 1;
			sdata->running_stm.algo.landscape = 0;
			break;
		case ETZKX_ORIENTATION_LANDSCAPE:
			sdata->running_stm.algo.portrait  = 0;
			sdata->running_stm.algo.landscape = 1;
			break;
		}
		break;
	case ETZKX_STM_ID_DOUBLE_TAP:
		sdata->running_stm.id = ETZKX_STM_ID_DOUBLE_TAP;
		switch (outs) {
		case ETZKX_DOUBLE_TAP_PLUS_X:
			sdata->running_stm.algo.x    = 1;
			sdata->running_stm.algo.y    = 0;
			sdata->running_stm.algo.z    = 0;
			sdata->running_stm.algo.peak = 0;
			break;
		case ETZKX_DOUBLE_TAP_MINUS_X:
			sdata->running_stm.algo.x    = 2;
			sdata->running_stm.algo.y    = 0;
			sdata->running_stm.algo.z    = 0;
			sdata->running_stm.algo.peak = 0;
			break;
		case ETZKX_DOUBLE_TAP_PLUS_Y:
			sdata->running_stm.algo.x    = 0;
			sdata->running_stm.algo.y    = 1;
			sdata->running_stm.algo.z    = 0;
			sdata->running_stm.algo.peak = 0;
			break;
		case ETZKX_DOUBLE_TAP_MINUS_Y:
			sdata->running_stm.algo.x    = 0;
			sdata->running_stm.algo.y    = 2;
			sdata->running_stm.algo.z    = 0;
			sdata->running_stm.algo.peak = 0;
			break;
		case ETZKX_DOUBLE_TAP_PLUS_Z:
			sdata->running_stm.algo.x    = 0;
			sdata->running_stm.algo.y    = 0;
			sdata->running_stm.algo.z    = 1;
			sdata->running_stm.algo.peak = 0;
			break;
		case ETZKX_DOUBLE_TAP_MINUS_Z:
			sdata->running_stm.algo.x    = 0;
			sdata->running_stm.algo.y    = 0;
			sdata->running_stm.algo.z    = 2;
			sdata->running_stm.algo.peak = 0;
			break;
		}
		break;
	case ETZKX_STM_ID_WAKEUP:
		sdata->running_stm.id = ETZKX_STM_ID_WAKEUP;
		sdata->running_stm.algo.sleep = !outs;
		sdata->running_stm.algo.wakeup =
					!sdata->running_stm.algo.sleep;
		break;
	case ETZKX_STM_ID_V_DOUBLE_TAP:
		sdata->running_stm.id = ETZKX_STM_ID_V_DOUBLE_TAP;
		sdata->running_stm.algo.vtap = 1;
		break;
	default:
		return 0;
	}

	wake_up_interruptible(&sdata->wq);
	return 0;
}

static irqreturn_t etzkx_irq_handler(int irq, void *dev)
{
	struct etzkx_data *sdata = dev;
	s32 outs;
	u8 masked_outs;

	if (irq == sdata->pdata->irq1) {
		outs = i2c_smbus_read_byte_data(sdata->client,
						ETZKX_REG_OUTS_1);
		masked_outs = etzkx_mask_orientation(sdata, outs);
		etzkx_stm_handle(dev, sdata->stm1, masked_outs);
	} else if (irq == sdata->pdata->irq2) {
		outs = i2c_smbus_read_byte_data(sdata->client,
						ETZKX_REG_OUTS_2);
		masked_outs = etzkx_mask_orientation(sdata, outs);
		etzkx_stm_handle(dev, sdata->stm2, masked_outs);
	}

	return IRQ_HANDLED;
}

static int etzkx_hw_detect(struct etzkx_data *sdata)
{
	int err;
	u8 wai_reg[3];

	dev_info(&sdata->client->dev, "Trying to read from device\n");
	err = i2c_smbus_read_i2c_block_data(sdata->client, ETZKX_REG_INFO1,
					3, wai_reg);
	if (err < 0)
		return err;
	if (err != 3)
		return -EIO;

	dev_info(&sdata->client->dev, "Device code is 0x%hhx\n", wai_reg[2]);

	switch (wai_reg[2]) {
	case ETZKX_WIA_LISN3DSH:
		dev_info(&sdata->client->dev,
			"ST lisn3dsh vers. %u accelerometer detected\n",
			wai_reg[0]);
		break;

	case ETZKX_WIA_KXTNK:
		dev_info(&sdata->client->dev,
			"Kionix kxtnk-1000 vers %u accelerometer detected\n",
			wai_reg[0]);
		break;

	case ETZKX_WIA_KXCNL:
		dev_info(&sdata->client->dev,
			"Kionix kxcnl-1010 vers %u accelerometer detected\n",
			wai_reg[0]);
		break;

	default:
		return -ENODEV;
	}

	sdata->wai = wai_reg[2];
	sdata->hw_version = wai_reg[0];

	return 0;
}

static ssize_t etzkx_sysfs_read_hwid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	switch (sdata->wai) {
	case ETZKX_WIA_LISN3DSH:
		return sprintf(buf, "lisn3dsh (%u)\n", sdata->hw_version);
	case ETZKX_WIA_KXTNK:
		return sprintf(buf, "kxtnk-1000 (%u)\n", sdata->hw_version);
	case ETZKX_WIA_KXCNL:
		return sprintf(buf, "kxcnl-1010 (%u)\n", sdata->hw_version);
	}

	return -ENODEV;
}

static ssize_t etzkx_sysfs_get_strm(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", !!(sdata->drv_state & ETZKX_STATE_STRM));
}

static ssize_t etzkx_sysfs_set_strm(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		err = etzkx_state_enable_streaming(sdata);
	else
		err = etzkx_state_disable_streaming(sdata);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t etzkx_sysfs_get_odr(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", etzkx_rate_str[sdata->odr]);
}

static ssize_t etzkx_sysfs_set_odr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned long value;
	int ret, i;
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;
	/*
	 * odr = 1600 Hz is not allowed for streaming
	 * 1600Hz should be set only for state machine purpose
	 */

	if ((sdata->stm1 != ETZKX_NO_STM_RUNNING) ||
			(sdata->stm2 != ETZKX_NO_STM_RUNNING))
		return -EPERM;

	for (i = 0;
		(i < ARRAY_SIZE(etzkx_rate_hz)) && (value >= etzkx_rate_hz[i]);
			i++)
		;

	mutex_lock(&sdata->mutex);
	if (i == 0)
		ret = etzkx_set_odr(sdata, i);
	else
		ret = etzkx_set_odr(sdata, i-1);
	if (etzkx_rate_ms[sdata->odr] < etzkx_rate_ms[ETZKX_ODR_100])
		sdata->poll_rate = etzkx_rate_ms[ETZKX_ODR_100];
	else
		sdata->poll_rate = etzkx_rate_ms[sdata->odr];
	mutex_unlock(&sdata->mutex);

	return (ret < 0) ? ret : len;
}

static ssize_t etzkx_sysfs_set_poll_rate(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int i, err = 0;
	unsigned long value;
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	/* doesn't allow polling rates faster than 10 ms */
	if (value < etzkx_rate_ms[ETZKX_ODR_100])
		value = etzkx_rate_ms[ETZKX_ODR_100];
	/* doesn't allow polling rates slower than 65535 ms */
	if (value > ETZKX_MAX_POLL_RATE)
		value = ETZKX_MAX_POLL_RATE;

	for (i = 0;
		i < ARRAY_SIZE(etzkx_rate_ms) && (value < etzkx_rate_ms[i]);
			i++)
		;

	/*
	 * set the device frequency one step lower
	 * that the polling rate frequency
	 */
	mutex_lock(&sdata->mutex);
	if ((sdata->stm1 == ETZKX_NO_STM_RUNNING) &&
			sdata->stm2 == ETZKX_NO_STM_RUNNING) {
		if (i > ETZKX_ODR_100)
			err = etzkx_set_odr(sdata, ETZKX_ODR_100);
		else
			err = etzkx_set_odr(sdata, i);
	}
	if (!err) {
		cancel_delayed_work_sync(&sdata->poll_read_work);
		sdata->poll_rate = value;
		if (sdata->drv_state & ETZKX_STATE_STRM)
			schedule_delayed_work(&sdata->poll_read_work,
				msecs_to_jiffies(sdata->poll_rate));
	}
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t etzkx_sysfs_get_poll_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->poll_rate);
}

static ssize_t etzkx_sysfs_get_range(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->range);
}

static ssize_t etzkx_sysfs_set_range(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	ssize_t ret;
	unsigned long value;
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if ((sdata->stm1 != ETZKX_NO_STM_RUNNING) ||
			(sdata->stm2 != ETZKX_NO_STM_RUNNING))
		return -EPERM;

	mutex_lock(&sdata->mutex);
	ret = etzkx_set_range(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (ret < 0) ? ret : len;
}

static ssize_t etzkx_sysfs_get_st(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n",
			!!(sdata->drv_state & ETZKX_STATE_SELF_TEST));
}

static ssize_t etzkx_sysfs_set_st(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int ret;
	unsigned long value;
	struct etzkx_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		ret = etzkx_state_enable_st(sdata);
	else
		ret = etzkx_state_disable_st(sdata);

	mutex_unlock(&sdata->mutex);

	return (ret < 0) ? ret : len;
}

static ssize_t etzkx_sysfs_read_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ETZKX_DRV_VERSION);
}

static DEVICE_ATTR(hwid, S_IRUGO, etzkx_sysfs_read_hwid, NULL);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		etzkx_sysfs_get_strm, etzkx_sysfs_set_strm);
static DEVICE_ATTR(odr, S_IRUGO | S_IWUSR,
		etzkx_sysfs_get_odr, etzkx_sysfs_set_odr);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR,
		etzkx_sysfs_get_poll_rate, etzkx_sysfs_set_poll_rate);
static DEVICE_ATTR(range, S_IRUGO | S_IWUSR,
		etzkx_sysfs_get_range, etzkx_sysfs_set_range);
static DEVICE_ATTR(self_test, S_IRUGO | S_IWUSR,
		etzkx_sysfs_get_st, etzkx_sysfs_set_st);
static DEVICE_ATTR(drv_version, S_IRUGO, etzkx_sysfs_read_version, NULL);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_hwid.attr,
	&dev_attr_enable.attr,
	&dev_attr_odr.attr,
	&dev_attr_delay.attr,
	&dev_attr_range.attr,
	&dev_attr_self_test.attr,
	&dev_attr_drv_version.attr,
	NULL
};

static struct attribute_group etzkx_attribute_group = {
	.attrs = sysfs_attrs
};

static int etzkx_chardev_open(struct inode *inode, struct file *file)
{
	struct etzkx_data *sdata =
			container_of(inode->i_cdev, struct etzkx_data, cdev);

	file->private_data = sdata;

	return 0;
}

static int etzkx_chardev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t etzkx_chardev_read(struct file *file, char __user *buffer,
					size_t length, loff_t *offset)
{
	int ret;
	struct etzkx_data *sdata = file->private_data;

	ret = wait_event_interruptible(sdata->wq, sdata->running_stm.id);
	if (ret != 0)
		return ret;

	ret = copy_to_user(buffer, &sdata->running_stm,
					sizeof(sdata->running_stm));
	if (ret != 0)
		ret = -EFAULT;

	mutex_lock(&sdata->mutex);
	sdata->running_stm.id = ETZKX_STM_ID_NO_STM;
	mutex_unlock(&sdata->mutex);

	return ret;
}

static ssize_t etzkx_chardev_write(struct file *file,
		const char __user *buffer, size_t length, loff_t *offset)
{
	return 0;
}

static unsigned int etzkx_chardev_poll(struct file *file,
					struct poll_table_struct *wait)
{
	struct etzkx_data *sdata = file->private_data;
	poll_wait(file, &sdata->wq, wait);

	return (sdata->running_stm.id) ? (POLLIN | POLLRDNORM) : 0;
}

static long etzkx_chardev_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	s32 pc;
	int err = 0;
	int reg_xyz[3];
	s16 range_mult;
	struct etzkx_stm_data info;
	int (*etzkx_state_enable_stm)(struct etzkx_data*, u8, u8);
	struct etzkx_data *sdata = file->private_data;

	/* switch on the latest two bits of cmd */
	switch ((cmd >> _IOC_NRSHIFT) & 0x03) {
	case ETZKXIO_ENABLE:
		/*
		 * check if there are free slots and if the
		 * requested state machine is already running
		 */

		if (sdata->stm1 == ETZKX_NO_STM_RUNNING) {
			if (sdata->stm2 != ETZKX_NO_STM_RUNNING &&
					etzkx_algos[sdata->stm2].stm_id ==
					(((cmd & 0xFF) >> _IOC_NRSHIFT) >> 2))
				return -EPERM;

			etzkx_state_enable_stm = etzkx_state_enable_stm1;

		} else if (sdata->stm2 == ETZKX_NO_STM_RUNNING) {
			if (sdata->stm1 != ETZKX_NO_STM_RUNNING &&
					etzkx_algos[sdata->stm1].stm_id ==
					(((cmd & 0xFF) >> _IOC_NRSHIFT) >> 2))
				return -EPERM;

			etzkx_state_enable_stm = etzkx_state_enable_stm2;

		} else {
			return -EPERM;
		}

		mutex_lock(&sdata->mutex);
		switch (cmd) {
		case ETZKXIO_ENABLE_TIMING:
			err = etzkx_state_enable_stm(sdata,
						ETZKX_STM_ID_TIMING, 0);
			break;
		case ETZKXIO_ENABLE_ORIENTATION:
			err = etzkx_state_enable_stm(sdata,
						ETZKX_STM_ID_ORIENTATION, 0);
			break;
		case ETZKXIO_ENABLE_DOUBLE_TAP:
			err = etzkx_state_enable_stm(sdata,
						ETZKX_STM_ID_DOUBLE_TAP, 0);
			break;
		case ETZKXIO_ENABLE_WAKEUP:
			err = etzkx_state_enable_stm(sdata,
						ETZKX_STM_ID_WAKEUP, 0);
			break;
		case ETZKXIO_ENABLE_V_DOUBLE_TAP:
			err = etzkx_state_enable_stm(sdata,
						ETZKX_STM_ID_V_DOUBLE_TAP, 0);
			break;
		default:
			err = -EINVAL;
		}
		break;

	case ETZKXIO_DISABLE:
		/* get the algo id from the ioctl command */
		cmd = (cmd & 0xFF) >> 2;
		mutex_lock(&sdata->mutex);
		if ((sdata->stm1 != ETZKX_NO_STM_RUNNING) &&
			(etzkx_algos[sdata->stm1].stm_id == cmd))
			err = etzkx_state_disable_stm1(sdata,
				etzkx_algos[sdata->stm1].stm_id);

		else if ((sdata->stm2 != ETZKX_NO_STM_RUNNING) &&
			(etzkx_algos[sdata->stm2].stm_id == cmd))
			err = etzkx_state_disable_stm2(sdata,
				etzkx_algos[sdata->stm2].stm_id);
		else
			err = -EINVAL;
		break;

	case ETZKXIO_STATE:
		mutex_lock(&sdata->mutex);
		switch (cmd) {
		case ETZKXIO_WHICH_ORIENTATION:
			if ((sdata->stm1 != ETZKX_NO_STM_RUNNING) &&
				(etzkx_algos[sdata->stm1].stm_id ==
						ETZKX_STM_ID_ORIENTATION)) {

				pc = i2c_smbus_read_byte_data(sdata->client,
							ETZKX_REG_PR_1);
				if (pc < 0) {
					err = pc;
					goto ioctl_err;
				}
			} else if ((sdata->stm2 != ETZKX_NO_STM_RUNNING) &&
				(etzkx_algos[sdata->stm2].stm_id ==
						ETZKX_STM_ID_ORIENTATION)) {
				pc = i2c_smbus_read_byte_data(sdata->client,
							ETZKX_REG_PR_2);
				if (pc < 0) {
					err = pc;
					goto ioctl_err;
				}
				/*
				 * address register gap between
				 * addresses for stm1 and stm2
				 */
				pc -= ETZKX_STM_REG_GAP;
			} else {
				err = -EINVAL;
				goto ioctl_err;
			}

			info.id = ETZKX_STM_ID_ORIENTATION;
			info.algo.portrait = (pc <= ETZKX_REG_ST7_1);
			info.algo.landscape = !info.algo.portrait;

			if (copy_to_user((void __user *) arg,
						&info, sizeof(info)))
				err = -EFAULT;
			break;

		case ETZKXIO_INSTANT_ORIENTATION:
			if (sdata->drv_state == ETZKX_STATE_STDBY) {
				err = -EFAULT;
				goto ioctl_err;
			}

			if (sdata->range == ETZKX_G_RANGE_DONT_CARE) {
				err = -EINVAL;
				goto ioctl_err;
			}

			err = etzkx_read_xyz(sdata, reg_xyz);
			if (err)
				goto ioctl_err;

			range_mult = ETZKX_MAX_RANGE / sdata->range;
			info.id = ETZKX_STM_ID_ORIENTATION;
			info.algo.landscape = ((reg_xyz[1] <
				ETZKX_ORIENTATION_LIMIT * range_mult) &&
				(abs(reg_xyz[0]) >
				ETZKX_ORIENTATION_LIMIT * range_mult));
			info.algo.portrait = !info.algo.landscape;

			if (copy_to_user((void __user *) arg,
						&info, sizeof(info)))
				err = -EFAULT;

			break;

		case ETZKXIO_RUNNING_ALGO:
			info.id = ETZKX_STM_ID_NO_STM;
			info.algo.stm1 =
				(sdata->stm1 == ETZKX_NO_STM_RUNNING) ?
					ETZKX_STM_ID_NO_STM :
					etzkx_algos[sdata->stm1].stm_id;
			info.algo.stm2 =
				(sdata->stm2 == ETZKX_NO_STM_RUNNING) ?
					ETZKX_STM_ID_NO_STM :
					etzkx_algos[sdata->stm2].stm_id;

			if (copy_to_user((void __user *) arg,
						&info, sizeof(info)))
				err = -EFAULT;
			break;

		default:
			err = -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}

ioctl_err:
	mutex_unlock(&sdata->mutex);
	return err;
}

static const struct file_operations etzkx_chardev_fops = {
	.owner   = THIS_MODULE,
	.open    = etzkx_chardev_open,
	.release = etzkx_chardev_release,
	.read    = etzkx_chardev_read,
	.write   = etzkx_chardev_write,
	.poll    = etzkx_chardev_poll,
	.unlocked_ioctl = etzkx_chardev_ioctl,
};

static int etzkx_input_register_device(struct etzkx_data *sdata)
{
	int err;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev)
		return -ENOMEM;

	sdata->input_dev->name       = ETZKX_DEV_NAME;
	sdata->input_dev->id.bustype = BUS_I2C;
	sdata->input_dev->id.vendor  = sdata->wai;
	sdata->input_dev->id.version = sdata->hw_version;
	sdata->input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->input_dev->evbit);
	input_set_abs_params(sdata->input_dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->input_dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->input_dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(sdata->input_dev);
	if (err)
		return err;

	return 0;
}

static void etzkx_input_cleanup(struct etzkx_data *sdata)
{
	input_unregister_device(sdata->input_dev);
}

static int etzkx_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	struct etzkx_data *sdata;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "no platform data declared\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev,
			"no algorithm associated to the i2c bus\n");
		return -ENODEV;
	}

	sdata = devm_kzalloc(&client->dev,
			sizeof(struct etzkx_data), GFP_KERNEL);
	if (!sdata) {
		dev_err(&client->dev, "no memory available\n");
		return -ENOMEM;
	}

	mutex_init(&sdata->mutex);
	mutex_lock(&sdata->mutex);

	sdata->client = client;
	i2c_set_clientdata(client, sdata);

	sdata->pdata = client->dev.platform_data;

	msleep(50);

	err = etzkx_hw_detect(sdata);
	if (err) {
		dev_err(&client->dev, "device not recognized\n");
		goto free_stmt;
	}

	if (sdata->pdata->init) {
		err = sdata->pdata->init();
		if (err) {
			dev_err(&client->dev,
				"impossible to initialize the device\n");
			goto free_stmt;
		}
	}

	err = etzkx_input_register_device(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"impossible to associate the device to an event\n");
		goto free_src;
	}

	err = sysfs_create_group(&sdata->client->dev.kobj,
					&etzkx_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"impossible to allocate sysfs resources\n");
		goto free_input;
	}

	err = alloc_chrdev_region(&etzkx_dev_number, 0, 1, ETZKX_CHARDEV_NAME);
	if (err)
		dev_err(&client->dev, "cannot register device\n");

	etzkx_class = class_create(THIS_MODULE, ETZKX_CHARDEV_NAME);
	cdev_init(&sdata->cdev, &etzkx_chardev_fops);
	sdata->cdev.owner = THIS_MODULE;

	err = cdev_add(&sdata->cdev, etzkx_dev_number, 1);
	if (err)
		dev_err(&client->dev, "cannot register device\n");

	device_create(etzkx_class, NULL, MKDEV(MAJOR(etzkx_dev_number), 0),
						NULL, ETZKX_CHARDEV_NAME);

	init_waitqueue_head(&sdata->wq);

	if (sdata->pdata->irq1) {
		err = request_threaded_irq(sdata->pdata->irq1, NULL,
				etzkx_irq_handler, IRQF_TRIGGER_RISING,
				"etzkx_irq1", sdata);
		if (err) {
			dev_err(&client->dev, "unable to request irq1\n");
			goto free_sysfs;
		}
	}

	if (sdata->pdata->irq2) {
		err = request_threaded_irq(sdata->pdata->irq2, NULL,
				etzkx_irq_handler, IRQF_TRIGGER_RISING,
				"etzkx_irq2", sdata);
		if (err) {
			dev_err(&client->dev, "unable to request irq2\n");
			goto free_irq1;
		}
	}

	if ((sdata->pdata->odr >= ETZKX_ODR_3_125) &&
		(sdata->pdata->odr <= ETZKX_ODR_1600))
		sdata->odr = sdata->pdata->odr;
	else
		sdata->odr = ETZKX_DEFAULT_ODR;
	if (sdata->pdata->range != ETZKX_G_RANGE_2G &&
		sdata->pdata->range != ETZKX_G_RANGE_4G &&
		sdata->pdata->range != ETZKX_G_RANGE_6G &&
		sdata->pdata->range != ETZKX_G_RANGE_8G)
		sdata->range = ETZKX_DEFAULT_G_RANGE;
	else
		sdata->range = sdata->pdata->range;
	sdata->poll_rate = etzkx_rate_ms[sdata->odr];
	err = etzkx_state_go_stdby(sdata);
	if (err) {
		dev_err(&client->dev, "unable to switch on the device\n");
		goto free_irq2;
	}

	/* build mask matrix */
	sdata->mask_matrix[0] = 1 <<
		(ETZKX_DIMENSION - (sdata->pdata->x_map + 1) * 2 +
						!sdata->pdata->x_negate);
	sdata->mask_matrix[1] = 1 <<
		(ETZKX_DIMENSION - (sdata->pdata->x_map + 1) * 2 +
						sdata->pdata->x_negate);
	sdata->mask_matrix[2] = 1 <<
		(ETZKX_DIMENSION - (sdata->pdata->y_map + 1) * 2 +
						!sdata->pdata->y_negate);
	sdata->mask_matrix[3] = 1 <<
		(ETZKX_DIMENSION - (sdata->pdata->y_map + 1) * 2 +
						sdata->pdata->y_negate);
	sdata->mask_matrix[4] = 1 <<
		(ETZKX_DIMENSION - (sdata->pdata->z_map + 1) * 2 +
						!sdata->pdata->z_negate);
	sdata->mask_matrix[5] = 1 <<
		(ETZKX_DIMENSION - (sdata->pdata->z_map + 1) * 2 +
						sdata->pdata->z_negate);
	sdata->mask_matrix[6] = 2;
	sdata->mask_matrix[7] = 1;

	INIT_DELAYED_WORK(&sdata->poll_read_work, etzkx_poll_read_work);

	mutex_unlock(&sdata->mutex);

	return 0;

free_irq2:
	if (sdata->pdata->irq2)
		free_irq(sdata->pdata->irq2, sdata);
free_irq1:
	if (sdata->pdata->irq1)
		free_irq(sdata->pdata->irq1, sdata);
free_sysfs:
	sysfs_remove_group(&sdata->client->dev.kobj, &etzkx_attribute_group);
free_input:
	etzkx_input_cleanup(sdata);
free_src:
	if (sdata->pdata->release)
		sdata->pdata->release();
free_stmt:
	mutex_unlock(&sdata->mutex);

	return err;
}

static int etzkx_remove(struct i2c_client *client)
{
	struct etzkx_data *sdata = i2c_get_clientdata(client);

	sysfs_remove_group(&sdata->client->dev.kobj, &etzkx_attribute_group);

	unregister_chrdev_region(etzkx_dev_number, 1);
	device_destroy(etzkx_class, MKDEV(MAJOR(etzkx_dev_number), 0));
	cdev_del(&sdata->cdev);
	class_destroy(etzkx_class);

	if (sdata->pdata->irq2)
		free_irq(sdata->pdata->irq2, sdata);

	if (sdata->pdata->irq1)
		free_irq(sdata->pdata->irq1, sdata);

	if (sdata->drv_state & ETZKX_STATE_STRM)
		cancel_delayed_work_sync(&sdata->poll_read_work);

	etzkx_input_cleanup(sdata);

	if (sdata->pdata->release)
		sdata->pdata->release();

	return 0;
}

#ifdef CONFIG_PM
static int etzkx_suspend(struct device *dev)
{
	return 0;
}

static int etzkx_resume(struct device *dev)
{
	return 0;
}
#else
#define etzkx_suspend	NULL
#define etzkx_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int etzkx_runtime_suspend(struct device *dev)
{
	return 0;
}

static int etzkx_runtime_resume(struct device *dev)
{
	return 0;
}
#else
#define etzkx_runtime_suspend	NULL
#define etzkx_runtime_resume	NULL
#endif

static const struct i2c_device_id etzkx_id[] = {
			{ ETZKX_DEV_NAME, 0 },
			{ ETZKX_LISN3DSH_NAME, 0 },
			{ ETZKX_KXTNK_NAME, 0 },
			{ },
		};

MODULE_DEVICE_TABLE(i2c, etzkx_id);

static const struct dev_pm_ops etzkx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(etzkx_suspend, etzkx_resume)
	SET_RUNTIME_PM_OPS(etzkx_runtime_suspend, etzkx_runtime_resume, NULL)
};

static struct i2c_driver etzkx_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name  = ETZKX_DEV_NAME,
			.pm    = &etzkx_pm_ops,
		  },
	.probe    = etzkx_probe,
	.remove   = etzkx_remove,
	.id_table = etzkx_id,
};

module_i2c_driver(etzkx_driver);

MODULE_DESCRIPTION("State Machine Interrupt Driven Accelerometer");
MODULE_AUTHOR("Andi Shyti");
MODULE_LICENSE("GPL v2");
