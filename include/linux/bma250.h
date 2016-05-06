/* include/linux/bma250.h
 *
 * Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
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

#ifndef LINUX_BMA250_MODULE_H
#define LINUX_BMA250_MODULE_H

#define GSENSOR_NAME            "bma250"
#define GPIO_GS_INT             70

#define LEVEL0 0
#define LEVEL1 1
#define LEVEL2 2
#define LEVEL3 3
//#define GSENSOR_DBG
#ifdef GSENSOR_DBG
static int gsensor_debug_level = 0;
#define GSENSOR_DEBUG(level, fmt, ...) \
    do { \
        if (level <= gsensor_debug_level) \
            printk("[GSENSOR] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## __VA_ARGS__); \
    } while(0)
#else
#define GSENSOR_DEBUG(level, fmt, ...)
#endif

struct bma250_platform_data {
    int (*gpio_init)(void);
    int layout;
};

#endif	/* LINUX_BMA250_MODULE_H */
