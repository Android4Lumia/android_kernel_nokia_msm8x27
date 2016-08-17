/* drivers/video/msm/logo.c
 *
 * Show Logo in RLE 565 format
 *
 * Copyright (C) 2008 Google Incorporated
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <asm/system.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include "msm_fb.h"
#include "mdp.h"
#include "mdp4.h"

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)

#if defined(CONFIG_FB_MSM_DEFAULT_DEPTH_RGBA8888)
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * 4)
#else
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * 2)
#endif

#ifndef CONFIG_FB_MSM_DEFAULT_DEPTH_RGBA8888
static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	count >>= 1;
	while (count--)
		*ptr++ = val;
}
#endif

/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
int load_565rle_image(char *filename)
{
	struct fb_info *info;
	int fd, count, err = 0;
	unsigned max;
	unsigned short *data, *bits, *ptr;
#if defined(CONFIG_FB_MSM_DEFAULT_DEPTH_RGBA8888)
	int bits_count;
#endif

	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = sys_lseek(fd, (off_t)0, 2);
	if (count <= 0) {
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if (sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}

	max = fb_width(info) * fb_height(info);
	ptr = data;
	if (info->screen_base) {
		bits = (unsigned short *)(info->screen_base);
		while (count > 3) {
			unsigned n = ptr[0];
			if (n > max)
				break;
#ifdef CONFIG_FB_MSM_DEFAULT_DEPTH_RGBA8888
			bits_count = n;
			while (bits_count--) {
				*bits++ = (ptr[1] & 0x7E0) << 5 | (ptr[1] & 0xF800) >> 8;
				*bits++ = 0xFF00 | (ptr[1] & 0x1F) << 3;
			}
#else
			memset16(bits, ptr[1], n << 1);
			bits += n;
#endif
			max -= n;
			ptr += 2;
			count -= 4;
		}
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565rle_image);
static void draw_logo(void)
{
	struct fb_info *fb_info = registered_fb[0];
	int ret = 0, i = 0;

	if (fb_info && fb_info->fbops->fb_open) {
		ret = fb_info->fbops->fb_open(fb_info, 0);
		if (ret != 0) {
			printk(KERN_ERR "[DISPLAY]%s: Can not open fb, ret <%d>\n",
					__func__, ret);
		}

		while (++i < 6) {
			ret = load_565rle_image(INIT_IMAGE_FILE);
			if (ret == 0) {
				ret = fb_info->fbops->fb_pan_display(&fb_info->var, fb_info);
				if (ret == 0) {
					printk(KERN_INFO "[DISPLAY]%s: Drawing logo\n", __func__);
					break;
				}
			}
			printk(KERN_ERR "[DISPLAY]%s: failed %d times, ret <%d>\n",
					__func__, i, ret);
			msleep(100);
		};
	}
}

int __init logo_init(void)
{
	if (mdp4_overlay_borderfill_supported())
		draw_logo();
	return 0;
}

module_init(logo_init);

