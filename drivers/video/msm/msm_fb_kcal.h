/*
 * Copyright (c) 2016, The CyanogenMod Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MSM_FB_KCAL_H
#define __MSM_FB_KCAL_H

int kcal_create_sysfs(struct msm_fb_data_type *mfd);

extern int kcal_tuning_apply(void);

#endif
