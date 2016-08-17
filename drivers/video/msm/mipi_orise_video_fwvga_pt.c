/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise.h"

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* DSIPHY_REGULATOR_CTRL */
	.regulator = {0x02, 0x08, 0x05, 0x00, 0x20},
	/* DSIPHY_CTRL */
	.ctrl = {0x5F, 0x00, 0x00, 0x10},
	/* DSIPHY_STRENGTH_CTRL */
	.strength = {0xFF, 0x00, 0x06, 0x00},
	/* DSIPHY_TIMING_CTRL */
	.timing = {0x67, 0x16, 0x0D, 0x00, 0x38, 0x3C, 0x12, 0x19, 0x18, 0x03, 0x04, 0xA0},
	/* DSIPHY_PLL_CTRL */
	.pll = {0x01, 0x25, 0x30, 0xC2, 0x00, 0x30, 0x0C, 0x62, 0x41, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01},
};

static int __init mipi_video_orise_pt_init(void)
{
	int ret;
	pr_info("[DISPLAY] +%s\n", __func__);

	pinfo.xres = 480;
	pinfo.yres = 800;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;

	pinfo.lcdc.h_back_porch = 44;
	pinfo.lcdc.h_front_porch = 45;
	pinfo.lcdc.h_pulse_width = 4;
	pinfo.lcdc.v_back_porch = 14;
	pinfo.lcdc.v_front_porch = 14;
	pinfo.lcdc.v_pulse_width = 1;

	/* Make sure change this value after modified pinfo.pll*/

	pinfo.clk_rate = 343848960;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xf0;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 127;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = FALSE;
	pinfo.mipi.data_lane3 = FALSE;

	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x04;
	pinfo.mipi.t_clk_pre = 0x18;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.dlane_swap = 0x01;
	pinfo.lcd.refx100 = 7000; /* adjust refx100 to prevent tearing */
	pinfo.mipi.te_sel = 0; /* TE from vsync gpio */
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = FALSE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;

	ret = mipi_orise_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_orise_pt_init);
