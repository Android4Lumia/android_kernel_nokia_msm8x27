/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 * Copyright(C) 2013 Foxconn International Holdings, Ltd. All rights reserved.
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
#include <mach/vreg.h>
#include <linux/gpio.h>

#define TRULY_PANEL_ID_EARLY 0x40
#define AUO_PANEL_ID 0x42
#define CMI_PANEL_ID 0x43
#define TRULY_PANEL_ID 0x45

#define TRULY_PANEL_V2_ID 0x46
#define TRULY_PANEL_V3_ID 0x49
#define TRULY_PANEL_V4_ID 0x50

#define CMI_PANEL_V2_ID 0x47
#define CMI_PANEL_V3_ID 0x48

#define RACE_PANEL_ID 0xe3

static struct dsi_buf orise_tx_buf;
static struct dsi_buf orise_rx_buf;
static int mipi_orise_lcd_init(void);

static struct msm_panel_common_pdata *mipi_orise_pdata = NULL;
static int display_initialize = 0;
static char gPanelModel = 0;

/* orise fwvga panel */
/* ----------- [For AUO panel setting Start] ----------- */
static char extc[4] = {0xFF, 0x80, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char cmd2[3] = {0xFF, 0x80, 0x09}; /* DTYPE_GEN_LWRITE */
static char cmdD8[2] = {0xD8, 0x87}; /* DTYPE_GEN_LWRITE */
static char auo_gamma_cmdE1[17] = {0xE1, 0x09, 0x11, 0x16, 0x0D, 0x05,
								0x0E, 0x0A, 0x07, 0x07, 0x08, 0x0A,
								0x05, 0x0C, 0x0A, 0x09, 0x08};
static char auo_gamma_cmdE2[17] = {0xE2, 0x09, 0x11, 0x16, 0x0D, 0x05,
								0x0E, 0x0A, 0x07, 0x07, 0x08, 0x0A,
								0x05, 0x0C, 0x0A, 0x09, 0x08};
static char auo_gamma_cmdEC[34] = {0xEC, 0x40, 0x44, 0x43, 0x43, 0x43, 0x32,
								0x43, 0x33, 0x46, 0x22, 0x23, 0x32, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x34, 0x33, 0x54,
								0x44, 0x33, 0x34, 0x34, 0x44, 0x33, 0x34,
								0x43, 0x44, 0x44, 0x44, 0x43, 0x44};
static char auo_gamma_cmdED[34] = {0xED, 0x40, 0x44, 0x43, 0x43, 0x44, 0x43,
								0x33, 0x33, 0x54, 0x34, 0x32, 0x33, 0x44,
								0x43, 0x44, 0x44, 0x43, 0x24, 0x44, 0x54,
								0x34, 0x43, 0x44, 0x44, 0x44, 0x33, 0x34,
								0x44, 0x45, 0x44, 0x53, 0x63, 0x45};
static char auo_gamma_cmdEE[34] = {0xEE, 0x40, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x04};
/* ----------- [For AUO panel setting End] ----------- */

/* ----------- [For CMI panel setting Start] ----------- */
static char cmi_extc[4] = {0xFF, 0x80, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char cmi_cmd2[3] = {0xFF, 0x80, 0x09}; /* DTYPE_GEN_LWRITE */
/* Set GVDD/NGVDD */
static char gamma_cmd1[3] = {0xD8, 0xAF, 0xAF};
/* Gammm Correction Characteristics Setting (3.0+) */
static char gamma_cmdE7[17] = {0xE7, 0x00, 0x10, 0x17, 0x0F, 0x08,
							0x0B, 0x04, 0x02, 0x0B, 0x0D, 0x0F,
							0x03, 0x06, 0x08, 0x07, 0x06};
/* Gammm Correction Characteristics Setting (3.0-) */
static char gamma_cmdE8[17] = {0xE8, 0x00, 0x10, 0x17, 0x0C, 0x05,
							0x0D, 0x0A, 0x07, 0x08, 0x09, 0x0E,
							0x04, 0x0B, 0x04, 0x03, 0x01};
/* Gamma  Set */
static char gamma_set[2] = {0x26, 0x08};
/* ----------- [For CMI panel setting End] ----------- */

static char ce_cmdD4[361] = {0xD4,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40,
	0xFE, 0x40, 0xFE, 0x40, 0xFE, 0x40, 0xFD, 0x40, 0xFD, 0x40,
	0xFD, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFB, 0x40, 0xFA, 0x40,
	0xFA, 0x40, 0xFA, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFC, 0x40,
	0xFD, 0x40, 0xFD, 0x40, 0xFD, 0x40, 0xFE, 0x40, 0xFE, 0x40,
	0xFE, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40,
	0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x03, 0x40, 0x03, 0x40,
	0x03, 0x40, 0x04, 0x40, 0x04, 0x40, 0x04, 0x40, 0x06, 0x40,
	0x06, 0x40, 0x05, 0x40, 0x04, 0x40, 0x04, 0x40, 0x04, 0x40,
	0x03, 0x40, 0x03, 0x40, 0x03, 0x40, 0x02, 0x40, 0x02, 0x40,
	0x02, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40,
	0xFE, 0x40, 0xFE, 0x40, 0xFE, 0x40, 0xFD, 0x40, 0xFD, 0x40,
	0xFD, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFB, 0x40, 0xFA, 0x40,
	0xFA, 0x40, 0xFA, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFC, 0x40,
	0xFD, 0x40, 0xFD, 0x40, 0xFD, 0x40, 0xFE, 0x40, 0xFE, 0x40,
	0xFE, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40
};
static char ce_cmdD5[361] = {0xD5,
	0x00, 0x60, 0x00, 0x5E, 0x00, 0x5D, 0x00, 0x5B, 0x00, 0x59,
	0x00, 0x57, 0x00, 0x56, 0x00, 0x54, 0x00, 0x52, 0x00, 0x51,
	0x00, 0x4F, 0x00, 0x4D, 0x00, 0x4C, 0x00, 0x4A, 0x00, 0x48,
	0x00, 0x47, 0x00, 0x4A, 0x00, 0x4C, 0x00, 0x4E, 0x00, 0x50,
	0x00, 0x52, 0x00, 0x54, 0x00, 0x56, 0x00, 0x58, 0x00, 0x5B,
	0x00, 0x5D, 0x00, 0x5F, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65,

	0x00, 0x66, 0xFF, 0x65, 0xFE, 0x64, 0xFE, 0x63, 0xFE, 0x62,
	0xFD, 0x61, 0xFD, 0x60, 0xFC, 0x5F, 0xFB, 0x5E, 0xFB, 0x5D,
	0xFB, 0x5C, 0xFA, 0x5A, 0xFA, 0x59, 0xF9, 0x58, 0xF8, 0x57,
	0xF8, 0x57, 0xF8, 0x5A, 0xFA, 0x5D, 0xF9, 0x5F, 0xF9, 0x62,
	0xFB, 0x65, 0xFB, 0x68, 0xFA, 0x6A, 0xFC, 0x6D, 0xFC, 0x70,
	0xFC, 0x73, 0xFE, 0x75, 0xFE, 0x78, 0xFE, 0x7B, 0x00, 0x7E,

	0x00, 0x7F, 0x00, 0x7C, 0x02, 0x7A, 0x02, 0x77, 0x02, 0x74,
	0x04, 0x71, 0x04, 0x6F, 0x04, 0x6C, 0x06, 0x69, 0x05, 0x66,
	0x05, 0x64, 0x07, 0x61, 0x07, 0x5E, 0x06, 0x5B, 0x08, 0x58,
	0x08, 0x56, 0x07, 0x56, 0x06, 0x56, 0x06, 0x56, 0x06, 0x56,
	0x05, 0x56, 0x05, 0x56, 0x04, 0x56, 0x03, 0x56, 0x03, 0x56,
	0x03, 0x56, 0x02, 0x56, 0x02, 0x56, 0x01, 0x56, 0x00, 0x56,

	0x00, 0x56, 0xFF, 0x56, 0xFE, 0x56, 0xFE, 0x56, 0xFE, 0x56,
	0xFD, 0x56, 0xFD, 0x56, 0xFC, 0x56, 0xFB, 0x56, 0xFB, 0x56,
	0xFB, 0x56, 0xFA, 0x56, 0xFA, 0x56, 0xF9, 0x56, 0xF8, 0x56,
	0xF9, 0x56, 0xF9, 0x55, 0xFA, 0x54, 0xFA, 0x54, 0xFA, 0x53,
	0xFC, 0x52, 0xFC, 0x51, 0xFC, 0x50, 0xFD, 0x50, 0xFD, 0x4F,
	0xFD, 0x4E, 0xFF, 0x4D, 0xFF, 0x4C, 0xFF, 0x4B, 0x00, 0x4A,

	0x00, 0x4A, 0x00, 0x4B, 0x00, 0x4C, 0x00, 0x4D, 0x00, 0x4D,
	0x00, 0x4E, 0x00, 0x4F, 0x00, 0x50, 0x00, 0x51, 0x00, 0x52,
	0x00, 0x53, 0x00, 0x53, 0x00, 0x54, 0x00, 0x55, 0x00, 0x56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,

	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X57, 0x00, 0X57, 0x00, 0X58, 0x00, 0X59, 0x00, 0X59,
	0x00, 0X5A, 0x00, 0X5B, 0x00, 0X5B, 0x00, 0X5C, 0x00, 0X5C,
	0x00, 0X5D, 0x00, 0X5E, 0x00, 0X5E, 0x00, 0X5F, 0x00, 0X60,
};
static char ce_cmdD6[2] = {0xD6, 0x08};
/* Program PWM frequency to about 33 KHz */
static char shift_addr00[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char shift_addr01[2] = {0x00, 0x01}; /* DTYPE_GEN_WRITE1 */
static char shift_addr80[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char shift_addrB1[2] = {0x00, 0xB1}; /* DTYPE_GEN_WRITE1 */
static char shift_addrB4[2] = {0x00, 0xB4}; /* DTYPE_GEN_WRITE1 */
static char pwm_para3[2] = {0xC6, 0x02}; /* DTYPE_GEN_WRITE1 */
static char pwm_para5[2] = {0xC6, 0x10}; /* DTYPE_GEN_WRITE1 */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
static char write_ctrl_display[2] = {0x53, 0x24}; /* DTYPE_DCS_WRITE1 */
static char write_display_brightness[2] = {0x51, 0x80}; /* DTYPE_DCS_WRITE1 */
static char orise_manufacture_idDA[2] = {0xDA, 0x00}; /* DTYPE_DCS_READ */

static char truly_cmd2_enable_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_cmd2_enable[4] = {0xFF, 0x80, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char truly_orise_cmd2_enable_offset[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char truly_orise_cmd2_enable[3] = {0xFF, 0x80, 0x09}; /* DTYPE_GEN_LWRITE */
static char truly_vdd_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_vdd[3] = {0xD8, 0x87, 0x87}; /* DTYPE_GEN_LWRITE */
static char truly_power_control_setting3_offset[2] = {0x00, 0xB1}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting3[2] = {0xC5, 0xA9}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting3_1_offset[2] = {0x00, 0x91}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting3_1[2] = {0xC5, 0x79}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting2_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting2[2] = {0xD9, 0x67}; /* DTYPE_GEN_LWRITE */
static char truly_power_control_setting2_1_offset[2] = {0x00, 0x92}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting2_1[2] = {0xC5, 0x01}; /* DTYPE_GEN_LWRITE */
static char truly_vcom_offset[2] = {0x00, 0xA1}; /* DTYPE_GEN_WRITE1 */
static char truly_vcom[2] = {0xC1, 0x08}; /* DTYPE_GEN_WRITE1 */
static char truly_osc_ref_offset[2] = {0x00, 0x81}; /* DTYPE_GEN_WRITE1 */
static char truly_osc_ref[2] = {0xC1, 0x77}; /* DTYPE_GEN_WRITE1 */
static char truly_osc_adjust_offset[2] = {0x00, 0xA3}; /* DTYPE_GEN_WRITE1 */
static char truly_osc_adjust[2] = {0xC0, 0x1B}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting1_offset[2] = {0x00, 0x82}; /* DTYPE_GEN_WRITE1 */
static char truly_power_control_setting1[2] = {0xC5, 0x83}; /* DTYPE_GEN_WRITE1 */
static char truly_source_driver1_offset[2] = {0x00, 0x81}; /* DTYPE_GEN_WRITE1 */
static char truly_source_driver1[2] = {0xC4, 0x83}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter1_offset[2] = {0x00, 0x90}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter1[2] = {0xB3, 0x02}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_offset[2] = {0x00, 0x92}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3[2] = {0xB3, 0x45}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_1_offset[2] = {0x00, 0xA0}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_1[2] = {0xC1, 0xEA}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_2_offset[2] = {0x00, 0xC0}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_2[2] = {0xC5, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_3_offset[2] = {0x00, 0x8B}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_3[2] = {0xB0, 0x40}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_4_offset[2] = {0x00, 0x87}; /* DTYPE_GEN_WRITE1 */
static char truly_iopad_parameter3_4[4] = {0xC4, 0x00, 0x80, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_disable_offset[2] = {0x00, 0xB2}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_disable[5] = {0xF5, 0x15, 0x00, 0x15, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_0[2] = {0x00, 0x81}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_0[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_1[2] = {0x00, 0x83}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_1[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_2[2] = {0x00, 0x85}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_2[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_3[2] = {0x00, 0x87}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_3[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_4[2] = {0x00, 0x89}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_4[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_5[2] = {0x00, 0x8B}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_5[2] = {0xF5, 0x20}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_6[2] = {0x00, 0x91}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_6[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_7[2] = {0x00, 0x93}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_7[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_8[2] = {0x00, 0x95}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_8[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_9[2] = {0x00, 0x97}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_9[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_10[2] = {0x00, 0x99}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_10[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_11[2] = {0x00, 0xA1}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_11[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_12[2] = {0x00, 0xA3}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_12[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_13[2] = {0x00, 0xA5}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_13[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_14[2] = {0x00, 0xA7}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_14[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_15[2] = {0x00, 0xB1}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_15[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_16[2] = {0x00, 0xB3}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_16[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_17[2] = {0x00, 0xB5}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_17[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_18[2] = {0x00, 0xB7}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_18[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_offset_19[2] = {0x00, 0xB9}; /* DTYPE_GEN_WRITE1 */
static char truly_power_off_sequence_disable_19[2] = {0xF5, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_minimum_offset[2] = {0x00, 0x93}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_minimum[2] = {0xC5, 0x03}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_minimum_offset_2[2] = {0x00, 0xA7}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_minimum_2[2] = {0xB3, 0x01}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_minimum_offset_3[2] = {0x00, 0xA6}; /* DTYPE_GEN_WRITE1 */
static char truly_vrgh_minimum_3[2] = {0xB3, 0x2B}; /* DTYPE_GEN_WRITE1 */
static char truly_setup_and_hold1_offset[2] = {0x00, 0x90}; /* DTYPE_GEN_WRITE1 */
static char truly_setup_and_hold1[7] = {0xC0, 0x00, 0x4E, 0x00, 0x00, 0x00, 0x03}; /* DTYPE_GEN_LWRITE */
static char truly_setup_and_hold2_offset[2] = {0x00, 0xA6}; /* DTYPE_GEN_WRITE1 */
static char truly_setup_and_hold2[4] = {0xC1, 0x01, 0x00, 0x00}; /* DTYPE_GEN_LWRITE */
/* Gamma 2.5 + */
static char truly_gamma_positive_offset[2] = {0x00, 0x00};
static char truly_gamma_positive[17] = {0xE1, 0x02, 0x0B, 0x0E, 0x0E, 0x07,
                                              0x0D, 0x0B, 0x0B, 0x02, 0x06,
                                              0x16, 0x0E, 0x14, 0x1E, 0x0A,
                                              0x00}; /* DTYPE_GEN_LWRITE */
static char truly_gamma_negitive_offset[2] = {0x00, 0x00};
static char truly_gamma_negitive[17] = {0xE2, 0x02, 0x0B, 0x0E, 0x0E, 0x07,
                                              0x0D, 0x0B, 0x0B, 0x02, 0x06,
                                              0x16, 0x0E, 0x14, 0x1E, 0x0A,
                                              0x00}; /* DTYPE_GEN_LWRITE */
/* Gamma 2.5 - */
static char truly_gamma24_positive_offset[2] = {0x00, 0x00};
static char truly_gamma24_positive[17] = {0xe1,0x02,0x09,0x0e,0x0e,0x07,0x0d,0x0b,0x0b,0x02,0x06,0x16,0x11,0x17,0x20,0x0c,0x00}; /* DTYPE_GEN_LWRITE */
static char truly_gamma24_negitive_offset[2] = {0x00, 0x00};
static char truly_gamma24_negitive[17] = {0xe2,0x02,0x09,0x0e,0x0e,0x07,0x0d,0x0b,0x0b,0x02,0x06,0x16,0x11,0x17,0x20,0x0c,0x00}; /* DTYPE_GEN_LWRITE */
static char truly_3gamma_EC[34] = {0xEC,
            0x40,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x34,
            0x33,
            0x33,
            0x43,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x44,
            0x55,
            0x55,
            0x55,
            0x04};

static char truly_3gamma_ED[34] = {0xED,
            0x40,                
            0x44,                
            0x44,         
            0x44,                
            0x44,                 
            0x44,                      
            0x44,                     
            0x44,                       
            0x44,                            
            0x44,              
            0x44,                        
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x04};

static char truly_3gamma_EE[34] = {0xEE,
            0x30,                                                                    
            0x33,                                                                    
            0x33,                                                                    
            0x33,                                                                    
            0x43,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x44,                                                                    
            0x54,                                                                    
            0x55,                                                                    
            0x55,                                                                    
            0x55,                                                                    
            0x45,                                                                    
            0x04};

static char truly_CEEN_offset[2] = {0x00, 0x80};
static char truly_CEEN[2] = {0xD6, 0x28};
static char truly_source_output_levels_offset[2] = {0x00, 0xB5};
static char truly_source_output_levels[2] = {0xC0, 0x28}; /* DTYPE_GEN_WRITE1 */
static char truly_zigzag_offset[2] = {0x00, 0x91};
static char truly_zigzag[2] = {0xB3, 0x40}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_vst_offset[2] = {0x00, 0x80};
static char truly_goa_vst[13] = {0xCE, 0x85, 0x01, 0x18, 0x84, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* DTYPE_GEN_LWRITE */
static char truly_goa_vend_offset[2] = {0x00, 0x90};
static char truly_goa_vend[15] = {0xCE, 0x13, 0x56, 0x18, 0x13, 0x57, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clka1_offset[2] = {0x00, 0xA0};
static char truly_goa_clka1[8] = {0xCE, 0x18, 0x0B, 0x03, 0x5E, 0x00, 0x18, 0x00}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clka2_offset[2] = {0x00, 0xA7};
static char truly_goa_clka2[8] = {0xCE, 0x18, 0x0A, 0x03, 0x5F, 0x00, 0x18, 0x00}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clka3_offset[2] = {0x00, 0xB0}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_clka3[8] = {0xCE, 0x18, 0x0D, 0x03, 0x5C, 0x00, 0x18, 0x00}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clka4_offset[2] = {0x00, 0xB7}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_clka4[8] = {0xCE, 0x18, 0x0C, 0x03, 0x5D, 0x00, 0x18, 0x00}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clkb1_offset[2] = {0x00, 0xC0}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_clkb1[8] = {0xCE, 0x38, 0x0E, 0x03, 0x5D, 0x00, 0x10, 0x07}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clkb2_offset[2] = {0x00, 0xC7}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_clkb2[8] = {0xCE, 0x38, 0x0F, 0x03, 0x5C, 0x00, 0x10, 0x07}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clkb3_offset[2] = {0x00, 0xD0}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_clkb3[8] = {0xCE, 0x38, 0x0A, 0x03, 0x59, 0x00, 0x10, 0x07}; /* DTYPE_GEN_LWRITE */
static char truly_goa_clkb4_offset[2] = {0x00, 0xD7}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_clkb4[8] = {0xCE, 0x38, 0x0B, 0x03, 0x58, 0x00, 0x10, 0x07}; /* DTYPE_GEN_LWRITE */
static char truly_goa_signal_toggle1_offset[2] = {0x00, 0xC7}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_signal_toggle1[2] = {0xCF, 0x04}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_signal_toggle2_offset[2] = {0x00, 0xC9}; /* DTYPE_GEN_WRITE1 */
static char truly_goa_signal_toggle2[2] = {0xCF, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings1_offset[2] = {0x00, 0xC0}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings1[2] = {0xCB, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings2_offset[2] = {0x00, 0xC2}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings2[6] = {0xCB, 0x14, 0x14, 0x14, 0x14, 0x14}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings3_offset[2] = {0x00, 0xD5}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings3[2] = {0xCB, 0x14}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings4_offset[2] = {0x00, 0xD7}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings4[6] = {0xCB, 0x14, 0x14, 0x14, 0x14, 0x14}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings5_offset[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings5[2] = {0xCC, 0x06}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings6_offset[2] = {0x00, 0x82}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings6[6] = {0xCC, 0x0F, 0x0D, 0x0C, 0x0A, 0x02}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings7_offset[2] = {0x00, 0x9A}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings7[2] = {0xCC, 0x05}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings8_offset[2] = {0x00, 0x9C}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings8[4] = {0xCC, 0x10, 0x0E, 0x0B}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings9_offset[2] = {0x00, 0xA0}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings9[3] = {0xCC, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings10_offset[2] = {0x00, 0xB0}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings10[2] = {0xCC, 0x06}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings11_offset[2] = {0x00, 0xB2}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings11[6] = {0xCC, 0x0F, 0x0D, 0x0C, 0x0A, 0x02}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings12_offset[2] = {0x00, 0xCA}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings12[2] = {0xCC, 0x05}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings13_offset[2] = {0x00, 0xCC}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings13[4] = {0xCC, 0x10, 0x0E, 0x0B}; /* DTYPE_GEN_LWRITE */
static char truly_panel_settings14_offset[2] = {0x00, 0xD0}; /* DTYPE_GEN_WRITE1 */
static char truly_panel_settings14[3] = {0xCC, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char truly_pwm_1_offset[2] = {0x00, 0xB4}; /* DTYPE_GEN_WRITE1 */
static char truly_pwm_1[2] = {0xC6, 0x12}; /* DTYPE_GEN_WRITE1 */
static char truly_pwm_2_offset[2] = {0x00, 0xB1}; /* DTYPE_GEN_WRITE1 */
static char truly_pwm_2[2] = {0xC6, 0x0A}; /* DTYPE_GEN_WRITE1 */
static char truly_bc_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_bc[2] = {0x53, 0x2C}; /* DTYPE_GEN_WRITE1 */
static char truly_tear_on_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_tear_on[2] = {0x35, 0x00};
static char truly_sleep_out_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_sleep_out[2] = {0x11, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_display_on_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_display_on[2] = {0x29, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_cmd2_disable_offset[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char truly_cmd2_disable[4] = {0xFF, 0xFF, 0xFF, 0xFF}; /* DTYPE_GEN_WRITE1 */
static char truly_orise_cmd2_disable_offset[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char truly_orise_cmd2_disable[3] = {0xFF, 0xFF, 0xFF}; /* DTYPE_GEN_WRITE1 */
static struct dsi_cmd_desc orise_truly_on_cmds[] = {
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_cmd2_enable_offset), truly_cmd2_enable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_cmd2_enable), truly_cmd2_enable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_orise_cmd2_enable_offset), truly_orise_cmd2_enable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_orise_cmd2_enable), truly_orise_cmd2_enable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vdd_offset), truly_vdd_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_vdd), truly_vdd},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting3_offset), truly_power_control_setting3_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting3), truly_power_control_setting3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting3_1_offset), truly_power_control_setting3_1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting3_1), truly_power_control_setting3_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting2_offset), truly_power_control_setting2_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting2), truly_power_control_setting2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting2_1_offset), truly_power_control_setting2_1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting2_1), truly_power_control_setting2_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vcom_offset), truly_vcom_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vcom), truly_vcom},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_osc_ref_offset), truly_osc_ref_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_osc_ref), truly_osc_ref},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_osc_adjust_offset), truly_osc_adjust_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_osc_adjust), truly_osc_adjust},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting1_offset), truly_power_control_setting1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_control_setting1), truly_power_control_setting1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_source_driver1_offset), truly_source_driver1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_source_driver1), truly_source_driver1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter1_offset), truly_iopad_parameter1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter1), truly_iopad_parameter1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_offset), truly_iopad_parameter3_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3), truly_iopad_parameter3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_1_offset), truly_iopad_parameter3_1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_1), truly_iopad_parameter3_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_2_offset), truly_iopad_parameter3_2_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_2), truly_iopad_parameter3_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_3_offset), truly_iopad_parameter3_3_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_3), truly_iopad_parameter3_3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_4_offset), truly_iopad_parameter3_4_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_iopad_parameter3_4), truly_iopad_parameter3_4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vrgh_disable_offset), truly_vrgh_disable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_vrgh_disable), truly_vrgh_disable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_0), truly_power_off_sequence_disable_offset_0},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_0), truly_power_off_sequence_disable_0},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_1), truly_power_off_sequence_disable_offset_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_1), truly_power_off_sequence_disable_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_2), truly_power_off_sequence_disable_offset_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_2), truly_power_off_sequence_disable_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_3), truly_power_off_sequence_disable_offset_3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_3), truly_power_off_sequence_disable_3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_4), truly_power_off_sequence_disable_offset_4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_4), truly_power_off_sequence_disable_4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_5), truly_power_off_sequence_disable_offset_5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_5), truly_power_off_sequence_disable_5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_6), truly_power_off_sequence_disable_offset_6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_6), truly_power_off_sequence_disable_6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_7), truly_power_off_sequence_disable_offset_7},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_7), truly_power_off_sequence_disable_7},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_8), truly_power_off_sequence_disable_offset_8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_8), truly_power_off_sequence_disable_8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_9), truly_power_off_sequence_disable_offset_9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_9), truly_power_off_sequence_disable_9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_10), truly_power_off_sequence_disable_offset_10},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_10), truly_power_off_sequence_disable_10},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_11), truly_power_off_sequence_disable_offset_11},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_11), truly_power_off_sequence_disable_11},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_12), truly_power_off_sequence_disable_offset_12},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_12), truly_power_off_sequence_disable_12},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_13), truly_power_off_sequence_disable_offset_13},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_13), truly_power_off_sequence_disable_13},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_14), truly_power_off_sequence_disable_offset_14},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_14), truly_power_off_sequence_disable_14},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_15), truly_power_off_sequence_disable_offset_15},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_15), truly_power_off_sequence_disable_15},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_16), truly_power_off_sequence_disable_offset_16},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_16), truly_power_off_sequence_disable_16},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_17), truly_power_off_sequence_disable_offset_17},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_17), truly_power_off_sequence_disable_17},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_18), truly_power_off_sequence_disable_offset_18},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_18), truly_power_off_sequence_disable_18},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_offset_19), truly_power_off_sequence_disable_offset_19},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_power_off_sequence_disable_19), truly_power_off_sequence_disable_19},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vrgh_minimum_offset), truly_vrgh_minimum_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vrgh_minimum), truly_vrgh_minimum},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vrgh_minimum_offset_2), truly_vrgh_minimum_offset_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vrgh_minimum_2), truly_vrgh_minimum_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vrgh_minimum_offset_3), truly_vrgh_minimum_offset_3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_vrgh_minimum_3), truly_vrgh_minimum_3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_setup_and_hold1_offset), truly_setup_and_hold1_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_setup_and_hold1), truly_setup_and_hold1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_setup_and_hold2_offset), truly_setup_and_hold2_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_setup_and_hold2), truly_setup_and_hold2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_gamma_positive_offset), truly_gamma_positive_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_gamma_positive), truly_gamma_positive},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_gamma_negitive_offset), truly_gamma_negitive_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_gamma_negitive), truly_gamma_negitive},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_source_output_levels_offset), truly_source_output_levels_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_source_output_levels), truly_source_output_levels},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_zigzag_offset), truly_zigzag_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_zigzag), truly_zigzag},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_vst_offset), truly_goa_vst_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_vst), truly_goa_vst},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_vend_offset), truly_goa_vend_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_vend), truly_goa_vend},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clka1_offset), truly_goa_clka1_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clka1), truly_goa_clka1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clka2_offset), truly_goa_clka2_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clka2), truly_goa_clka2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clka3_offset), truly_goa_clka3_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clka3), truly_goa_clka3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clka4_offset), truly_goa_clka4_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clka4), truly_goa_clka4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clkb1_offset), truly_goa_clkb1_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clkb1), truly_goa_clkb1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clkb2_offset), truly_goa_clkb2_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clkb2), truly_goa_clkb2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clkb3_offset), truly_goa_clkb3_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clkb3), truly_goa_clkb3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_clkb4_offset), truly_goa_clkb4_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_goa_clkb4), truly_goa_clkb4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_signal_toggle1_offset), truly_goa_signal_toggle1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_signal_toggle1), truly_goa_signal_toggle1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_signal_toggle2_offset), truly_goa_signal_toggle2_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_goa_signal_toggle2), truly_goa_signal_toggle2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings1_offset), truly_panel_settings1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings1), truly_panel_settings1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings2_offset), truly_panel_settings2_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings2), truly_panel_settings2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings3_offset), truly_panel_settings3_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings3), truly_panel_settings3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings4_offset), truly_panel_settings4_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings4), truly_panel_settings4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings5_offset), truly_panel_settings5_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings5), truly_panel_settings5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings6_offset), truly_panel_settings6_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings6), truly_panel_settings6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings7_offset), truly_panel_settings7_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings7), truly_panel_settings7},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings8_offset), truly_panel_settings8_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings8), truly_panel_settings8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings9_offset), truly_panel_settings9_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings9), truly_panel_settings9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings10_offset), truly_panel_settings10_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings10), truly_panel_settings10},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings11_offset), truly_panel_settings11_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings11), truly_panel_settings11},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings12_offset), truly_panel_settings12_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings12), truly_panel_settings12},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings13_offset), truly_panel_settings13_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings13), truly_panel_settings13},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_panel_settings14_offset), truly_panel_settings14_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_panel_settings14), truly_panel_settings14},

	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_CEEN_offset), truly_CEEN_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_CEEN), truly_CEEN},

	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_cmd2_disable_offset), truly_cmd2_disable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_cmd2_disable), truly_cmd2_disable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_orise_cmd2_disable_offset), truly_orise_cmd2_disable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_orise_cmd2_disable), truly_orise_cmd2_disable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_1_offset), truly_pwm_1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_1), truly_pwm_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_2_offset), truly_pwm_2_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_2), truly_pwm_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_bc_offset), truly_bc_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_bc), truly_bc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_tear_on_offset), truly_tear_on_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_tear_on), truly_tear_on},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_sleep_out_offset), truly_sleep_out_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 120, sizeof(truly_sleep_out), truly_sleep_out},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_display_on_offset), truly_display_on_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_display_on), truly_display_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,	sizeof(write_display_brightness), write_display_brightness}
};
/* LCM ID 0x49*/
static struct dsi_cmd_desc orise_truly_v3_on_cmds[] = {
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_cmd2_enable_offset), truly_cmd2_enable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_cmd2_enable), truly_cmd2_enable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_orise_cmd2_enable_offset), truly_orise_cmd2_enable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_orise_cmd2_enable), truly_orise_cmd2_enable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_CEEN_offset), truly_CEEN_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_CEEN), truly_CEEN},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_cmd2_disable_offset), truly_cmd2_disable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_cmd2_disable), truly_cmd2_disable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_orise_cmd2_disable_offset), truly_orise_cmd2_disable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_orise_cmd2_disable), truly_orise_cmd2_disable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_1_offset), truly_pwm_1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_1), truly_pwm_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_2_offset), truly_pwm_2_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_2), truly_pwm_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_bc_offset), truly_bc_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_bc), truly_bc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_tear_on_offset), truly_tear_on_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_tear_on), truly_tear_on},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_sleep_out_offset), truly_sleep_out_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 120, sizeof(truly_sleep_out), truly_sleep_out},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_display_on_offset), truly_display_on_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_display_on), truly_display_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,	sizeof(write_display_brightness), write_display_brightness}
};
static struct dsi_cmd_desc orise_truly_v4_on_cmds[] = {
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_cmd2_enable_offset), truly_cmd2_enable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_cmd2_enable), truly_cmd2_enable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_orise_cmd2_enable_offset), truly_orise_cmd2_enable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_orise_cmd2_enable), truly_orise_cmd2_enable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_gamma24_positive_offset), truly_gamma24_positive_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_gamma24_positive), truly_gamma24_positive},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_gamma24_negitive_offset), truly_gamma24_negitive_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_gamma24_negitive), truly_gamma24_negitive},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_3gamma_EC), truly_3gamma_EC}, 
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_3gamma_ED), truly_3gamma_ED}, 
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_3gamma_EE), truly_3gamma_EE}, 
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_CEEN_offset), truly_CEEN_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_CEEN), truly_CEEN},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_cmd2_disable_offset), truly_cmd2_disable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_cmd2_disable), truly_cmd2_disable},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_orise_cmd2_disable_offset), truly_orise_cmd2_disable_offset},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(truly_orise_cmd2_disable), truly_orise_cmd2_disable},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_1_offset), truly_pwm_1_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_1), truly_pwm_1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_2_offset), truly_pwm_2_offset},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_pwm_2), truly_pwm_2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_bc_offset), truly_bc_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_bc), truly_bc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_tear_on_offset), truly_tear_on_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_tear_on), truly_tear_on},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_sleep_out_offset), truly_sleep_out_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 120, sizeof(truly_sleep_out), truly_sleep_out},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(truly_display_on_offset), truly_display_on_offset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(truly_display_on), truly_display_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,	sizeof(write_display_brightness), write_display_brightness}
};

static struct dsi_cmd_desc orise_auo_video_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(extc), extc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmd2), cmd2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(cmdD8), cmdD8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr01), shift_addr01},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmdD8), cmdD8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdE1), auo_gamma_cmdE1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdE2), auo_gamma_cmdE2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdEC), auo_gamma_cmdEC},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdED), auo_gamma_cmdED},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdEE), auo_gamma_cmdEE},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD4), ce_cmdD4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD5), ce_cmdD5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(ce_cmdD6), ce_cmdD6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB1), shift_addrB1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para3), pwm_para3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB4), shift_addrB4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para5), pwm_para5},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_ctrl_display), write_ctrl_display},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(display_on), display_on}
};

static struct dsi_cmd_desc orise_cmi_video_on_cmds[] = {
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmi_extc), cmi_extc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmi_cmd2), cmi_cmd2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB1), shift_addrB1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para3), pwm_para3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB4), shift_addrB4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para5), pwm_para5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_ctrl_display), write_ctrl_display},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_cmd1), gamma_cmd1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_cmdE7), gamma_cmdE7},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_cmdE8), gamma_cmdE8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(gamma_set), gamma_set},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD4), ce_cmdD4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD5), ce_cmdD5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(ce_cmdD6), ce_cmdD6},
	{DTYPE_DCS_WRITE, 1, 0, 0, 20,
		sizeof(display_on), display_on}
};

static struct dsi_cmd_desc orise_video_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 40,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

/* MM-KW-CMI_V2-00+{ */
/* ----------- [For CMI V2 panel setting Start] ----------- */
static char shift_addr83[2] = {0x00, 0x83}; /* DTYPE_GEN_WRITE1 */
static char x47_cmi_extc[4] = {0xFF, 0x80, 0x12, 0x01}; /* DTYPE_GEN_LWRITE */
static char x47_cmi_cmd2[3] = {0xFF, 0x80, 0x12}; /* DTYPE_GEN_LWRITE */
/* Set GVDD/NGVDD */
static char x47_vdd_cmd1[3] = {0xD8, 0xAF, 0xAF};
static char shift_addr92[2] = {0x00, 0x92}; /* DTYPE_GEN_WRITE1 */
static char shift_addrA1[2] = {0x00, 0xA1}; /* DTYPE_GEN_WRITE1 */
static char shift_addrA6[2] = {0x00, 0xA6}; /* DTYPE_GEN_WRITE1 */
static char shift_addrB0[2] = {0x00, 0xB0}; /* DTYPE_GEN_WRITE1 */
static char shift_addrB2[2] = {0x00, 0xB2}; /* DTYPE_GEN_WRITE1 */
static char x47_set_A_data_latch[2] = {0xC4, 0x00};
static char x47_set_A_data_latch_to_0x08[2] = {0xC4, 0x08};
static char x47_set_B_source_gate[2] = {0xB3, 0x00};
static char x47_set_C_data_latch[2] = {0xC4, 0x08};
static char x47_set_D_improve_tear[2] = {0xC0, 0x30};
static char x47_set_E_improve_tear[3] = {0xC1, 0x82, 0x00};
static char x47_set_F_improve_tear[2] = {0xB3, 0x18};
static char x47_osc_ref_80[3] = {0xC1, 0x25, 0x77}; /* DTYPE_GEN_LWRITE */
static char x47_osc_ref_83[3] = {0xC1, 0x50}; /* DTYPE_GEN_LWRITE */
/* Gamma Correction Characteristics Setting (2.2+) */
static char x47_gamma_cmdE1[21] = {0xE1, 0x1F, 0x3D, 0x53, 0x63, 0x6E,
									0x7A, 0x77, 0x9A, 0x83, 0x98, 0x6D,
									0x57, 0x6A, 0x45, 0x45, 0x42, 0x38,
									0x1B, 0x17, 0x11};
/* Gamma Correction Characteristics Setting (2.2-) */
static char x47_gamma_cmdE2[21] = {0xE2, 0x1F, 0x3D, 0x53, 0x63, 0x6E,
									0x7A, 0x77, 0x9A, 0x83, 0x98, 0x6D,
									0x57, 0x6A, 0x45, 0x45, 0x42, 0x38,
									0x1B, 0x17, 0x11};
static struct dsi_cmd_desc x47_orise_cmi_video_on_cmds[] = {
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_cmi_extc), x47_cmi_extc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_cmi_cmd2), x47_cmi_cmd2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr92), shift_addr92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_A_data_latch), x47_set_A_data_latch},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrA1), shift_addrA1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_B_source_gate), x47_set_B_source_gate},
	{DTYPE_DCS_WRITE, 1, 0, 0, 110,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr92), shift_addr92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(x47_set_C_data_latch), x47_set_C_data_latch},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr92), shift_addr92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(x47_set_A_data_latch), x47_set_A_data_latch},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr92), shift_addr92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(x47_set_C_data_latch), x47_set_C_data_latch},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr92), shift_addr92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(x47_set_A_data_latch), x47_set_A_data_latch},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr92), shift_addr92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(x47_set_C_data_latch), x47_set_C_data_latch},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_vdd_cmd1), x47_vdd_cmd1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB2), shift_addrB2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_D_improve_tear), x47_set_D_improve_tear},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrA6), shift_addrA6},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_set_E_improve_tear), x47_set_E_improve_tear},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB0), shift_addrB0},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_F_improve_tear), x47_set_F_improve_tear},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_gamma_cmdE1), x47_gamma_cmdE1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_gamma_cmdE2), x47_gamma_cmdE2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_osc_ref_80), x47_osc_ref_80},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr83), shift_addr83},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_osc_ref_83), x47_osc_ref_83},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_DCS_WRITE, 1, 0, 0, 100,
		sizeof(display_on), display_on}
};

/* LCM ID 0x48*/
static struct dsi_cmd_desc x47_orise_cmi_v2_video_on_cmds[] = {
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_cmi_extc), x47_cmi_extc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_cmi_cmd2), x47_cmi_cmd2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr92), shift_addr92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_A_data_latch_to_0x08), x47_set_A_data_latch_to_0x08},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrA1), shift_addrA1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_B_source_gate), x47_set_B_source_gate},
	{DTYPE_DCS_WRITE, 1, 0, 0, 110,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB2), shift_addrB2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_D_improve_tear), x47_set_D_improve_tear},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrA6), shift_addrA6},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_set_E_improve_tear), x47_set_E_improve_tear},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB0), shift_addrB0},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_set_F_improve_tear), x47_set_F_improve_tear},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(x47_osc_ref_80), x47_osc_ref_80},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr83), shift_addr83},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(x47_osc_ref_83), x47_osc_ref_83},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_on), display_on}
};
/* ----------- [For CMI V2 panel setting End] ----------- */
/* ----------- [For RACE panel setting Start] ----------- */
/* Nokia RACE WVGA DSI CMD Panel (480x800x24) START */
static char race_set_pixel_format[2] = {0x3A, 0x70};	/* DTYPE_DCS_WRITE1 */
static char race_ctrl_display[2] = {0x53, 0x2C};	/* DTYPE_DCS_WRITE1 */
static char race_cabc_on[2] = {0x55, 0x01};		/* DTYPE_DCS_WRITE1 */
static char race_cabc_off[2] = {0x55, 0x00};		/* DTYPE_DCS_WRITE1 */

static char race_unknown_cmd[2] = {0xff, 0x78};		/* DTYPE_GEN_LWRITE? */


static struct dsi_cmd_desc orise_race_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(race_unknown_cmd), race_unknown_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(race_set_pixel_format), race_set_pixel_format},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(race_ctrl_display), race_ctrl_display},
	//{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(write_display_brightness), write_display_brightness},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(race_cabc_on), race_cabc_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(race_set_pixel_format), race_set_pixel_format},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_on), display_on},
};

static struct dsi_cmd_desc orise_race_video_off_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(race_cabc_off), race_cabc_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(enter_sleep), enter_sleep},
};
/* ----------- [For RACE panel setting End] ----------- */
//#if 0
static struct dsi_cmd_desc orise_video_bkl_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(write_display_brightness), write_display_brightness}
};
//#endif
static struct dsi_cmd_desc x47_orise_cmi_video_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 40,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};
static struct dsi_cmd_desc orise_ReadDA = {
	DTYPE_DCS_READ, 1, 0, 1, 20, sizeof(orise_manufacture_idDA), orise_manufacture_idDA
};
static int mipi_orise_manufacture_id(struct msm_fb_data_type *mfd)
{
	char retDA = 0;

	struct dsi_buf *tp = &orise_tx_buf;
	struct dsi_buf *rp = &orise_rx_buf;

	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);
	mipi_dsi_cmds_rx(mfd, tp, rp, &orise_ReadDA, 1);
	retDA = *((char *) rp->data);

	printk(KERN_ALERT "[DISPLAY] Panel ID <0x%02x>\n", retDA);
	return retDA;
}

static int mipi_orise_lcd_on(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_fb_data_type *mfd = NULL;
	pr_info("[DISPLAY] +%s\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (unlikely(display_initialize))
		return 0;

	if (unlikely(gPanelModel == 0)) {
		gPanelModel = mipi_orise_manufacture_id(mfd);
	}

	switch (gPanelModel) {
		case TRULY_PANEL_V4_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_truly_v4_on_cmds,
					ARRAY_SIZE(orise_truly_v4_on_cmds));
			break;
		case TRULY_PANEL_V3_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_truly_v3_on_cmds,
					ARRAY_SIZE(orise_truly_v3_on_cmds));
			break;
		case TRULY_PANEL_V2_ID:
		case TRULY_PANEL_ID_EARLY:
		case TRULY_PANEL_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_truly_on_cmds,
					ARRAY_SIZE(orise_truly_on_cmds));
			break;
		case AUO_PANEL_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_auo_video_on_cmds,
					ARRAY_SIZE(orise_auo_video_on_cmds));
			break;
		case CMI_PANEL_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_cmi_video_on_cmds,
					ARRAY_SIZE(orise_cmi_video_on_cmds));
			break;
		case CMI_PANEL_V3_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, x47_orise_cmi_v2_video_on_cmds,
					ARRAY_SIZE(x47_orise_cmi_v2_video_on_cmds));
			break;
		case CMI_PANEL_V2_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, x47_orise_cmi_video_on_cmds,
					ARRAY_SIZE(x47_orise_cmi_video_on_cmds));
			break;
		case RACE_PANEL_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_race_video_on_cmds,
					ARRAY_SIZE(orise_race_video_on_cmds));
			break;
		default:
			printk(KERN_ERR "[DISPLAY] illegal PID <0x%02x>\n", gPanelModel);
			break;
	}
    if(gPanelModel)
	{
		printk(KERN_ALERT "[DISPLAY] dsi commands done, rc <%d>, PID <0x%02x>\n",
			rc, gPanelModel);

		display_initialize = 1;
    }

	if (rc > 0)
		rc = 0;

	return rc;
}

static int mipi_orise_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = NULL;

	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (unlikely(!display_initialize))
		return 0;

	mipi_set_tx_power_mode(0);
	if (gPanelModel == CMI_PANEL_V2_ID)
		mipi_dsi_cmds_tx(&orise_tx_buf, x47_orise_cmi_video_off_cmds,
				ARRAY_SIZE(x47_orise_cmi_video_off_cmds));
	else if(gPanelModel == RACE_PANEL_ID)
		mipi_dsi_cmds_tx(&orise_tx_buf, orise_race_video_off_cmds,
				ARRAY_SIZE(orise_race_video_off_cmds));
	else
		mipi_dsi_cmds_tx(&orise_tx_buf, orise_video_off_cmds,
				ARRAY_SIZE(orise_video_off_cmds));
	mipi_set_tx_power_mode(1);

	display_initialize = 0;

	return 0;
}

static void mipi_orise_lcd_backlight(struct msm_fb_data_type *mfd)
{
	printk(KERN_ALERT "[DISPLAY] Enter %s(%i)\n", __func__, mfd->bl_level);
	//WARN_ON(1);
//#if 0
	if (unlikely(!display_initialize))
		return;

	write_display_brightness[1] = mfd->bl_level;

	down(&mfd->dma->mutex);
	mipi_set_tx_power_mode(0);
	mipi_dsi_cmds_tx(&orise_tx_buf, orise_video_bkl_cmds,
			ARRAY_SIZE(orise_video_bkl_cmds));
	mipi_set_tx_power_mode(1);
	up(&mfd->dma->mutex);
//#endif
}

static int __devinit mipi_orise_lcd_probe(struct platform_device *pdev)
{
	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);

	if (pdev->id == 0) {
		mipi_orise_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_orise_lcd_probe,
	.driver = {
		.name   = "mipi_orise",
	},
};

static struct msm_fb_panel_data orise_panel_data = {
	.on		= mipi_orise_lcd_on,
	.off	= mipi_orise_lcd_off,
	.set_backlight = mipi_orise_lcd_backlight,
};

static int ch_used[3];

int mipi_orise_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret = 0;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_orise_lcd_init();
	if (ret) {
		pr_err("mipi_orise_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_orise", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	orise_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &orise_panel_data,
		sizeof(orise_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_orise_lcd_init(void)
{
	mipi_dsi_buf_alloc(&orise_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&orise_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}
