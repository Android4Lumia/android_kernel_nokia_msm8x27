/*
 *  etzkx: lisn3dsh/kxtnk 3d accelerometer driver
 *
 *  Copyright (C) 2013  Andi Shyti <andi@etezian.org>
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

#ifndef __ETZKX_H__
#define __ETZKX_H__

#define ETZKX_G_RANGE_DONT_CARE		0
#define ETZKX_G_RANGE_2G		2
#define ETZKX_G_RANGE_4G		4
#define ETZKX_G_RANGE_6G		6
#define ETZKX_G_RANGE_8G		8

#define ETZKX_ODR_3_125			0x00
#define ETZKX_ODR_6_25			0x01
#define ETZKX_ODR_12_5			0x02
#define ETZKX_ODR_25			0x03
#define ETZKX_ODR_50			0x04
#define ETZKX_ODR_100			0x05
#define ETZKX_ODR_400			0x06
#define ETZKX_ODR_1600			0x07
#define ETZKX_ODR_DONT_CARE		0xFF

#define ETZKX_STM_ID_NO_STM		0
#define ETZKX_STM_ID_TIMING		1
#define ETZKX_STM_ID_ORIENTATION	2
#define ETZKX_STM_ID_DOUBLE_TAP		3
#define ETZKX_STM_ID_WAKEUP		4
#define ETZKX_STM_ID_V_DOUBLE_TAP	5

#define ETZKX_DEFAULT_G_RANGE		ETZKX_G_RANGE_4G
#define ETZKX_DEFAULT_ODR		ETZKX_ODR_50

/* ioctl-number: 'x' 00-2F (32 commands) */
#define ETZKX_IOCTL_NUM			('x')

struct etzkx_stm_data {
	s32 id;
	union {
		/* double tap */
		struct {
			u8 x;
			u8 y;
			u8 z;
			u8 peak;
		};
		/* orientation */
		struct {
			u16 portrait;
			u16 landscape;
		};
		/* wake up */
		struct {
			u16 wakeup;
			u16 sleep;
		};
		/* v double tap */
		struct {
			u32 vtap;
		};
		/* running algos */
		struct {
			u16 stm1;
			u16 stm2;
		};
	} algo;
};

/* etzkx ioctl command types */
#define ETZKXIO_ENABLE			0x01
#define ETZKXIO_DISABLE			0x02
#define ETZKXIO_STATE			0x03
/*
 * etzkx ioctl commands
 *   - first 6 bits identify the algorithm id
 *   - last 2 bits identify the operation type on the algorithm
 */
#define ETZKXIO_ENABLE_TIMING		_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_TIMING << 2) | \
						ETZKXIO_ENABLE)
#define ETZKXIO_DISABLE_TIMING		_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_TIMING << 2) | \
						ETZKXIO_DISABLE)
#define ETZKXIO_ENABLE_ORIENTATION	_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_ORIENTATION << 2) | \
						ETZKXIO_ENABLE)
#define ETZKXIO_DISABLE_ORIENTATION	_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_ORIENTATION << 2) | \
						ETZKXIO_DISABLE)
#define ETZKXIO_ENABLE_WAKEUP		_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_WAKEUP << 2) | \
						ETZKXIO_ENABLE)
#define ETZKXIO_DISABLE_WAKEUP		_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_WAKEUP << 2) | \
						ETZKXIO_DISABLE)
#define ETZKXIO_ENABLE_V_DOUBLE_TAP	_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_V_DOUBLE_TAP << 2) | \
						ETZKXIO_ENABLE)
#define ETZKXIO_DISABLE_V_DOUBLE_TAP	_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_V_DOUBLE_TAP << 2) | \
						ETZKXIO_DISABLE)
#define ETZKXIO_WHICH_ORIENTATION	_IOR(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_ORIENTATION << 2) | \
						ETZKXIO_STATE, \
						struct etzkx_stm_data *)
#define ETZKXIO_ENABLE_DOUBLE_TAP	_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_DOUBLE_TAP << 2) | \
						ETZKXIO_ENABLE)
#define ETZKXIO_DISABLE_DOUBLE_TAP	_IO(ETZKX_IOCTL_NUM, \
					(ETZKX_STM_ID_DOUBLE_TAP << 2) | \
						ETZKXIO_DISABLE)
#define ETZKXIO_RUNNING_ALGO		_IOR(ETZKX_IOCTL_NUM, \
					(0x0A << 2) | ETZKXIO_STATE, \
						struct etzkx_stm_data *)
#define ETZKXIO_INSTANT_ORIENTATION	_IOR(ETZKX_IOCTL_NUM, \
					(0x0B << 2) | ETZKXIO_STATE, \
						struct etzkx_stm_data *)

#define ETZKX_DEV_NAME		"etzkx"
#define ETZKX_DRV_VERSION	5

struct etzkx_platform_data {
	int (*init)(void);
	void (*release)(void);

	u8 x_map;
	u8 y_map;
	u8 z_map;

	u8 x_negate;
	u8 y_negate;
	u8 z_negate;

	u8 odr;
	u8 range;

	u16 irq1;
	u16 irq2;
};

#endif
