/*
 * include/linux/input/ltr553.h
 *
 * Copyright (C) 2015 Balázs Triszka <balika011@protonmail.ch>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef LTR553_H
#define LTR553_H

struct ltr553_platform_data {
	int			irq_gpio;
	u32			irq_flags;
	int			als_ps_persist;
	int			ps_led;
	int			ps_pulses;
	int			als_integration_time;
	int			ps_wakeup_threshold;
};

#endif
