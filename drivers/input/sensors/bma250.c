/* drivers/input/sensors/bma250.c
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

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/bma250.h>
#include <mach/gpio.h>
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
#include <linux/cyttsp_i2c_tma340.h>
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

/*MTD-PERIPHERAL-AC-Layout-01+{*/
#include <linux/fih_hw_info.h>
/*MTD-PERIPHERAL-AC-Layout-01+}*/

/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+{*/
#include <linux/regulator/msm-gpio-regulator.h>
#include <linux/mfd/pm8xxx/pm8038.h>
#include <linux/mfd/pm8xxx/pm8921.h>
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+}*/

#define BMA250_CHIP_ID_REG                      0x00
#define BMA250_VERSION_REG                      0x01
#define BMA250_X_AXIS_LSB_REG                   0x02
#define BMA250_X_AXIS_MSB_REG                   0x03
#define BMA250_Y_AXIS_LSB_REG                   0x04
#define BMA250_Y_AXIS_MSB_REG                   0x05
#define BMA250_Z_AXIS_LSB_REG                   0x06
#define BMA250_Z_AXIS_MSB_REG                   0x07
#define BMA250_TEMP_RD_REG                      0x08
#define BMA250_STATUS1_REG                      0x09
#define BMA250_STATUS2_REG                      0x0A
#define BMA250_STATUS_TAP_SLOPE_REG             0x0B
#define BMA250_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA250_RANGE_REG        0x0F
#define BMA250_BANDWIDTH_REG            0x10
#define BMA250_MODE_CTRL_REG                    0x11
#define BMA250_LOW_NOISE_CTRL_REG               0x12
#define BMA250_DATA_CTRL_REG                    0x13
#define BMA250_RESET_REG                        0x14
#define BMA250_INT_ENABLE1_REG                  0x16
#define BMA250_INT_ENABLE2_REG                  0x17
#define BMA250_INT1_PAD_SEL_REG                 0x19
#define BMA250_INT_DATA_SEL_REG                 0x1A
#define BMA250_INT2_PAD_SEL_REG                 0x1B
#define BMA250_INT_SRC_REG                      0x1E
#define BMA250_INT_SET_REG                      0x20
#define BMA250_INT_CTRL_REG                     0x21
#define BMA250_LOW_DURN_REG                     0x22
#define BMA250_LOW_THRES_REG                    0x23
#define BMA250_LOW_HIGH_HYST_REG                0x24
#define BMA250_HIGH_DURN_REG                    0x25
#define BMA250_HIGH_THRES_REG                   0x26
#define BMA250_SLOPE_DURN_REG                   0x27
#define BMA250_SLOPE_THRES_REG                  0x28
#define BMA250_TAP_PARAM_REG                    0x2A
#define BMA250_TAP_THRES_REG                    0x2B
#define BMA250_ORIENT_PARAM_REG                 0x2C
#define BMA250_THETA_BLOCK_REG                  0x2D
#define BMA250_THETA_FLAT_REG                   0x2E
#define BMA250_FLAT_HOLD_TIME_REG               0x2F
#define BMA250_STATUS_LOW_POWER_REG             0x31
#define BMA250_SELF_TEST_REG                    0x32
#define BMA250_EEPROM_CTRL_REG                  0x33
#define BMA250_SERIAL_CTRL_REG                  0x34
#define BMA250_CTRL_UNLOCK_REG                  0x35
#define BMA250_OFFSET1_REG      0x36
#define BMA250_OFFSET2_REG      0x37
#define BMA250_OFFSET_FILT_X_REG                0x38
#define BMA250_OFFSET_FILT_Y_REG                0x39
#define BMA250_OFFSET_FILT_Z_REG                0x3A
#define BMA250_OFFSET_UNFILT_X_REG              0x3B
#define BMA250_OFFSET_UNFILT_Y_REG              0x3C
#define BMA250_OFFSET_UNFILT_Z_REG              0x3D
#define BMA250_SPARE_0_REG                      0x3E
#define BMA250_SPARE_1_REG                      0x3F


#define BMA250_CHIP_ID          3
#define BMA250_NEW_DATA_POS     0
#define BMA250_NEW_DATA_LEN     1
#define BMA250_NEW_DATA_MSK     0x01
#define BMA250_NEW_DATA_REG     BMA250_X_AXIS_LSB_REG
#define BMA250_ACC_LSB_POS      6
#define BMA250_ACC_LSB_LEN      2
#define BMA250_ACC_LSB_MSK      0xC0
#define BMA250_ACC_LSB_REG      BMA250_X_AXIS_LSB_REG
#define BMA250_ACC_MSB_POS      0
#define BMA250_ACC_MSB_LEN      8
#define BMA250_ACC_MSB_MSK      0xFF
#define BMA250_ACC_MSB_REG      BMA250_X_AXIS_MSB_REG
#define BMA250_MODE_SUSPEND     0x80
#define BMA250_RESET            0xB6
#define BMA250_CAL_RDY          0x10
#define BMA250_NVM_RDY          0x04

#define BMA250_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA250_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


#define BMA250_EN_SELF_TEST__POS                0
#define BMA250_EN_SELF_TEST__LEN                2
#define BMA250_EN_SELF_TEST__MSK                0x03
#define BMA250_EN_SELF_TEST__REG                BMA250_SELF_TEST_REG

#define BMA250_NEG_SELF_TEST__POS               2
#define BMA250_NEG_SELF_TEST__LEN               1
#define BMA250_NEG_SELF_TEST__MSK               0x04
#define BMA250_NEG_SELF_TEST__REG               BMA250_SELF_TEST_REG


#define BMA250_ACC_X_LSB__POS           6
#define BMA250_ACC_X_LSB__LEN           2
#define BMA250_ACC_X_LSB__MSK           0xC0
#define BMA250_ACC_X_LSB__REG           BMA250_X_AXIS_LSB_REG

#define BMA250_ACC_X_MSB__POS           0
#define BMA250_ACC_X_MSB__LEN           8
#define BMA250_ACC_X_MSB__MSK           0xFF
#define BMA250_ACC_X_MSB__REG           BMA250_X_AXIS_MSB_REG

#define BMA250_ACC_Y_LSB__POS           6
#define BMA250_ACC_Y_LSB__LEN           2
#define BMA250_ACC_Y_LSB__MSK           0xC0
#define BMA250_ACC_Y_LSB__REG           BMA250_Y_AXIS_LSB_REG

#define BMA250_ACC_Y_MSB__POS           0
#define BMA250_ACC_Y_MSB__LEN           8
#define BMA250_ACC_Y_MSB__MSK           0xFF
#define BMA250_ACC_Y_MSB__REG           BMA250_Y_AXIS_MSB_REG

#define BMA250_ACC_Z_LSB__POS           6
#define BMA250_ACC_Z_LSB__LEN           2
#define BMA250_ACC_Z_LSB__MSK           0xC0
#define BMA250_ACC_Z_LSB__REG           BMA250_Z_AXIS_LSB_REG

#define BMA250_ACC_Z_MSB__POS           0
#define BMA250_ACC_Z_MSB__LEN           8
#define BMA250_ACC_Z_MSB__MSK           0xFF
#define BMA250_ACC_Z_MSB__REG           BMA250_Z_AXIS_MSB_REG

struct bma250acc {
    s16 x,
        y,
        z;
};

struct bma250cfg {
    unsigned char bw;
    unsigned char mode;
    unsigned char range;
};

struct bma250reg {
    unsigned char addr;
    unsigned char data;
    unsigned char flag;
};

struct bma250_data {
    bool enable;
    bool interrupt;
    bool irq_flag;
    struct bma250_platform_data *platform_data;
    struct bma250acc value;
    struct bma250cfg old_cfg;
    struct bma250cfg cfg;
    struct bma250reg reg;
    struct delayed_work work;
    struct delayed_work irq_work;	/* FIH-SW1-PERIPHERAL-FG-GSENSOR_INTERRUPT-00* */
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    struct i2c_client *client;
    struct input_dev *input;
    struct mutex enable_mutex;
    struct mutex value_mutex;
    int irq;
    int polling_times;	/* FIH-SW1-PERIPHERAL-FG-GSENSOR_INTERRUPT-00+ */
    unsigned int delay;
/*PERI-AC-Interrupt-00+{*/
    int polling_count;
    int pdelay;
    struct bma250acc gaxi;
/*PERI-AC-Interrupt-00+}*/
    int    selftest_result;
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+{*/
    struct regulator *vreg_l9;
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+}*/
};

static const int bma250_direction_mapping[][3][3] = {
    {{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}},	/* top/upper-left */
    {{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}},	/* top/upper-right */
    {{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}},	/* top/lower-right */
    {{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}},	/* top/lower-left */
    {{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}},	/* bottom/upper-right */
    {{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}},	/* bottom/upper-left */
    {{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}},	/* bottom/lower-left */
    {{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}},	/* bottom/lower-right */
};

static int bma250_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
    s32 dummy = i2c_smbus_read_byte_data(client, reg_addr);
    if (dummy < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed.");
        return -EPERM;
    }
    *data = dummy & 0x000000ff;

    return 0;
}

static int bma250_smbus_write_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char data)
{
    s32 dummy = i2c_smbus_write_byte_data(client, reg_addr, data);
    if (dummy < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed.");
        return -EPERM;
    }

    return 0;
}

static int bma250_smbus_read_byte_block(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
    if (dummy < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed.");
        return -EPERM;
    }

    return 0;
}

/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+{*/
static int bma250_power_control(struct bma250_data *bma250,int onoff)
{
    int rc=0;
    if(IS_ERR(bma250->vreg_l9))
    {
        GSENSOR_DEBUG(LEVEL0, "Failed.");
        return -ENODEV;
    }

    if(onoff)
    {
        rc = regulator_set_optimum_mode(bma250->vreg_l9, 154);
        if(rc < 0)
        {
            GSENSOR_DEBUG(LEVEL0, "Enable fail.");
            return rc;
        }
    }
    else
    {
        rc = regulator_set_optimum_mode(bma250->vreg_l9, 0);
        if(rc < 0)
        {
            GSENSOR_DEBUG(LEVEL0, "Disable fail.");
            return rc;
        }
    }
    return 0;
}

static int bma250_power_init(struct bma250_data *bma250)
{
    int rc = 0;
    if(bma250 == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed.");
        return -ENODEV;
    }

    bma250->vreg_l9 = regulator_get(&bma250->client->dev,"bma_vreg");
    if(IS_ERR(bma250->vreg_l9))
    {
        GSENSOR_DEBUG(LEVEL0, "Can't power on init.");
        return -ENODEV;
    }
    rc = bma250_power_control(bma250,1);
    if(rc)
    {
        return rc;
    }
    GSENSOR_DEBUG(LEVEL0, "Pass.");
    return 0;
}
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+}*/

static void bma250_read_accel_xyz(struct bma250_data *bma250)
{
    unsigned char buf[6];
    s16 raw[3], data[3];
    int i, j;
    ktime_t timestamp;

    if (bma250_smbus_read_byte_block(bma250->client, BMA250_X_AXIS_LSB_REG, buf, sizeof(buf)) < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed.");
        return;
    }

    memset(raw, 0 , sizeof(raw));
    for(i=0; i<3; i++)
    {
        raw[i] = ((buf[i*2] >> BMA250_ACC_LSB_POS) | (buf[i*2+1] << BMA250_ACC_LSB_LEN));
        /* Handle a negative number */
        raw[i] = raw[i] << (sizeof(short)*8-(BMA250_ACC_LSB_LEN+BMA250_ACC_MSB_LEN));
        raw[i] = raw[i] >> (sizeof(short)*8-(BMA250_ACC_LSB_LEN+BMA250_ACC_MSB_LEN));
    }
    memset(data, 0 , sizeof(data));
    for (i=0; i<3; i++)
    {
        for (j=0; j<3; j++)
            data[i] += raw[j] * bma250_direction_mapping[bma250->platform_data->layout][i][j];
    }
    mutex_lock(&bma250->value_mutex);
    bma250->value.x = data[0];
    bma250->value.y = data[1];
    bma250->value.z = data[2];
    mutex_unlock(&bma250->value_mutex);

/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD	
	if(bma250->value.z < -200)
		TOUCH_ESD_WORKAROUND(CY_FACE_DOWN);
	else
		TOUCH_ESD_WORKAROUND(CY_FACE_UP);
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

    timestamp = ktime_get_boottime();
    input_report_abs(bma250->input, ABS_X, bma250->value.x);
    input_report_abs(bma250->input, ABS_Y, bma250->value.y);
    input_report_abs(bma250->input, ABS_Z, bma250->value.z);
    input_event(bma250->input, EV_SYN, SYN_TIME_SEC,
		    ktime_to_timespec(timestamp).tv_sec);
    input_event(bma250->input, EV_SYN, SYN_TIME_NSEC,
		    ktime_to_timespec(timestamp).tv_nsec);
    input_sync(bma250->input);
}

static int bma250_set_selftest_st(struct i2c_client *client,
        unsigned char selftest)
{
    int comres = 0;
    unsigned char data;

    comres = bma250_smbus_read_byte(client,
        BMA250_EN_SELF_TEST__REG, &data);
    data = BMA250_SET_BITSLICE(data,
        BMA250_EN_SELF_TEST, selftest);
    comres = bma250_smbus_write_byte(client,
        BMA250_EN_SELF_TEST__REG, data);

    return comres;
}

static int bma250_set_selftest_stn(struct i2c_client *client, unsigned char stn)
{
    int comres = 0;
    unsigned char data;

    comres = bma250_smbus_read_byte(client,
        BMA250_NEG_SELF_TEST__REG, &data);
    data = BMA250_SET_BITSLICE(data, BMA250_NEG_SELF_TEST, stn);
    comres = bma250_smbus_write_byte(client,
        BMA250_NEG_SELF_TEST__REG, data);

    return comres;
}

static int bma250_read_accel_x(struct i2c_client *client, short *a_x)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client,
			BMA250_ACC_X_LSB__REG, data, 2);
	*a_x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB)
		| (BMA250_GET_BITSLICE(data[1], BMA250_ACC_X_MSB)
				<<BMA250_ACC_X_LSB__LEN);
	*a_x = *a_x << (sizeof(short)*8
			-(BMA250_ACC_X_LSB__LEN+BMA250_ACC_X_MSB__LEN));
	*a_x = *a_x >> (sizeof(short)*8
			-(BMA250_ACC_X_LSB__LEN+BMA250_ACC_X_MSB__LEN));

	return comres;
}
static int bma250_read_accel_y(struct i2c_client *client, short *a_y)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client,
			BMA250_ACC_Y_LSB__REG, data, 2);
	*a_y = BMA250_GET_BITSLICE(data[0], BMA250_ACC_Y_LSB)
		| (BMA250_GET_BITSLICE(data[1], BMA250_ACC_Y_MSB)
				<<BMA250_ACC_Y_LSB__LEN);
	*a_y = *a_y << (sizeof(short)*8
			-(BMA250_ACC_Y_LSB__LEN+BMA250_ACC_Y_MSB__LEN));
	*a_y = *a_y >> (sizeof(short)*8
			-(BMA250_ACC_Y_LSB__LEN+BMA250_ACC_Y_MSB__LEN));

	return comres;
}

static int bma250_read_accel_z(struct i2c_client *client, short *a_z)
{
	int comres;
	unsigned char data[2];

	comres = bma250_smbus_read_byte_block(client,
			BMA250_ACC_Z_LSB__REG, data, 2);
	*a_z = BMA250_GET_BITSLICE(data[0], BMA250_ACC_Z_LSB)
		| BMA250_GET_BITSLICE(data[1], BMA250_ACC_Z_MSB)
		<<BMA250_ACC_Z_LSB__LEN;
	*a_z = *a_z << (sizeof(short)*8
			-(BMA250_ACC_Z_LSB__LEN+BMA250_ACC_Z_MSB__LEN));
	*a_z = *a_z >> (sizeof(short)*8
			-(BMA250_ACC_Z_LSB__LEN+BMA250_ACC_Z_MSB__LEN));

	return comres;
}

static int bma250_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }

    return bma250_smbus_read_byte(client, BMA250_BANDWIDTH_REG, BW);
}

static int bma250_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }
    if (bma250->cfg.bw == BW)
        return 0;

    bma250->old_cfg.bw = bma250->cfg.bw;
    bma250->cfg.bw = BW;
    if (bma250_smbus_write_byte(client, BMA250_BANDWIDTH_REG, BW) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    return 0;
}

static int bma250_get_mode(struct i2c_client *client, unsigned char *Mode)
{
    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }

    return bma250_smbus_read_byte(client, BMA250_MODE_CTRL_REG, Mode);
}

static int bma250_set_mode(struct i2c_client *client, unsigned char Mode)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }
    if (bma250->cfg.mode == Mode)
        return 0;

    bma250->old_cfg.mode = bma250->cfg.mode;
    bma250->cfg.mode = Mode;
    if (bma250_smbus_write_byte(client, BMA250_MODE_CTRL_REG, Mode) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    return 0;
}

static int bma250_get_range(struct i2c_client *client, unsigned char *Range)
{
    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }

    return bma250_smbus_read_byte(client, BMA250_RANGE_REG, Range);
}

static int bma250_set_range(struct i2c_client *client, unsigned char Range)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }
    if (bma250->cfg.range == Range)
        return 0;

    bma250->old_cfg.range = bma250->cfg.range;
    bma250->cfg.range = Range;
    if (bma250_smbus_write_byte(client, BMA250_RANGE_REG, Range) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    return 0;
}

static int bma250_reset(struct i2c_client *client)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);
    unsigned char tmp = 0;
    int err = 0;

    err = bma250_smbus_write_byte(client, BMA250_RESET_REG, BMA250_RESET);
    if (err < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed.");
        return err;
    }

    msleep(4);
    bma250_get_bandwidth(client, &tmp);
    bma250->old_cfg.bw = bma250->cfg.bw = tmp;
    bma250_get_mode(client, &tmp);
    bma250->old_cfg.mode = bma250->cfg.mode = tmp;
    bma250_get_range(client, &tmp);
    bma250->old_cfg.range = bma250->cfg.range = tmp;

    GSENSOR_DEBUG(LEVEL0, "Done.");
    return err;
}

static int bma250_calibration_xyz(struct i2c_client *client, unsigned char XYZ)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);
    unsigned char buf[2]={0};
    unsigned char data = 0;
    s16 raw = 0;
    int i = 0;

    /*MTD-PERIPHERAL-AC-Calibration-01+{*/
    bma250_smbus_read_byte_block(bma250->client, BMA250_Z_AXIS_LSB_REG, buf, sizeof(buf));
    raw = ((buf[0] >> BMA250_ACC_LSB_POS) | (buf[1] << BMA250_ACC_LSB_LEN));
    /* Handle a negative number */
    raw = raw << (sizeof(short)*8-(BMA250_ACC_LSB_LEN+BMA250_ACC_MSB_LEN));
    raw = raw >> (sizeof(short)*8-(BMA250_ACC_LSB_LEN+BMA250_ACC_MSB_LEN));
    /*MTD-PERIPHERAL-AC-Calibration-01+}*/

    /* 3. Write 00 in bits[2:1] of register 0x37 =>calibrate on x-axis */
    /* 6. Write 00 in bits[4:3] of register 0x37 =>calibrate on y-axis */
    /* 9. Write 01(1g) or 10(-1g) in bits[6:5] of register 0x37 =>calibrate on z-axis */

    if (XYZ > 2)
    {
        /*MTD-PERIPHERAL-AC-Calibration-01*/
        /* Check IC z-axi true position value */
        if (raw > 0)
            data = 1 << 5;
        else
            data = 1 << 6;

    }
    if (bma250_smbus_write_byte(client, BMA250_OFFSET2_REG, data) < 0)
        return -EIO;

    /* 4. Write 01 in bits[6:5] of register 0x36 */
    /* 7. Write 10 in bits[6:5] of register 0x36 */
    /* 10. Write 11 in bits[6:5] of register 0x36 */
    data = XYZ << 5;
    if (bma250_smbus_write_byte(client, BMA250_OFFSET1_REG, data) < 0)
        return -EIO;

    /* 5. Keep reading bit4 of register 0x36 until the value is 1 */
    /* 8. Keep reading bit4 of register 0x36 until the value is 1 */
    /* 11. Keep reading bit4 of register 0x36 until the value is 1 */
    for (i=0; i<10; i++)
    {
        msleep(150);
        bma250_smbus_read_byte(client, BMA250_OFFSET1_REG, &data);
        GSENSOR_DEBUG(LEVEL2, "Reg[0x%2x] = 0x%2x(%d)", BMA250_OFFSET1_REG, data, i+1);
        if (data & BMA250_CAL_RDY)
            return 0;
    }

    GSENSOR_DEBUG(LEVEL0, "Failed.");
    return -EINVAL;
}
static void bma250_irq_work(struct work_struct *work)
{
    struct bma250_data *bma250 = container_of((struct delayed_work *)work, struct bma250_data, irq_work);

    if (bma250->enable)
    {
        bma250_read_accel_xyz(bma250);

        if (bma250->polling_times == 0 || (bma250->polling_times >= (bma250->polling_count-1)))
            GSENSOR_DEBUG(LEVEL1, "%d) x = %3d, y = %3d, z = %3d", bma250->polling_times,\
                                                                                                          bma250->value.x,\
                                                                                                          bma250->value.y,\
                                                                                                          bma250->value.z);
        if (abs(bma250->value.x) > bma250->gaxi.x||\
            abs(bma250->value.y) > bma250->gaxi.y||\
            abs(bma250->value.z) > bma250->gaxi.z)
            bma250->polling_times = 0;

        bma250->polling_times++;
        if (bma250->polling_times < bma250->polling_count)
            schedule_delayed_work(&bma250->irq_work, msecs_to_jiffies(bma250->pdelay));
    }
}

static irqreturn_t bma250_irq_handler(int irq, void *dev_id)
{
    struct bma250_data *bma250 = dev_id;

    bma250->polling_times = 0;
    schedule_delayed_work(&bma250->irq_work, msecs_to_jiffies(0));

    return IRQ_HANDLED;
}

static int bma250_calibration_store(struct i2c_client *client)
{
    unsigned char data = 0;
    int i = 0;

    /* 12. Write 1 in bit0 of register 0x33 =>store calibration data into EEPROM */
    bma250_smbus_read_byte(client, BMA250_EEPROM_CTRL_REG, &data);
    data = data | (1 << 0);
    if (bma250_smbus_write_byte(client, BMA250_EEPROM_CTRL_REG, data) < 0)
        return -EIO;

    /* 13. Write 1 in bit1 of register 0x33 */
    data = data | (1 << 1);
    if (bma250_smbus_write_byte(client, BMA250_EEPROM_CTRL_REG, data) < 0)
        return -EIO;

    /* 14. Keep reading bit2 of register 0x33 until the value is 1 */
    for (i=0; i<10; i++)
    {
        msleep(10);
        bma250_smbus_read_byte(client, BMA250_EEPROM_CTRL_REG, &data);
        GSENSOR_DEBUG(LEVEL2, "Reg[0x%2x] = 0x%2x(%d)", BMA250_EEPROM_CTRL_REG, data, i+1);
        if (data & BMA250_NVM_RDY)
            break;
    }
    if (i==10)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    /* 15. Write 0 in bit0 of register 0x33 and calibration is done. */
    data = data & 0xFE;
    if (bma250_smbus_write_byte(client, BMA250_EEPROM_CTRL_REG, data) < 0)
        return -EIO;

    bma250_smbus_read_byte(client, BMA250_EEPROM_CTRL_REG, &data);
    GSENSOR_DEBUG(LEVEL2, "Reg[0x%2x] = 0x%2x", BMA250_EEPROM_CTRL_REG, data);

    GSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static int bma250_calibration(struct i2c_client *client)
{
    struct bma250cfg cfg_value;

    /* 0. Write 0x00 in register 0x11 =>set default power mode */
    bma250_get_mode(client, &cfg_value.mode);
    bma250_set_mode(client, 0x00);

    /* 1. Write 0x03 in register 0x0F =>set 2g */
    bma250_get_range(client, &cfg_value.range);
    bma250_set_range(client, 0x03);

    /* 2. Write 0x0A in register 0x10 =>set 31Hz */
    bma250_get_bandwidth(client, &cfg_value.bw);
    bma250_set_bandwidth(client, 0x0A);

    bma250_calibration_xyz(client, 1);	/* calibrate on x-axis */
    bma250_calibration_xyz(client, 2);	/* calibrate on y-axis */
    bma250_calibration_xyz(client, 3);	/* calibrate on z-axis */
    bma250_calibration_store(client);	/* store calibration data into EEPROM */

    bma250_set_mode(client, cfg_value.mode);
    bma250_set_range(client, cfg_value.range);
    bma250_set_bandwidth(client, cfg_value.bw);

    GSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static void bma250_set_delay(struct i2c_client *client, unsigned int Delay)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return;
    }
    if (bma250->delay == Delay)
        return;

    bma250->delay = Delay;
    if (bma250->enable && (!bma250->interrupt))
    {
        cancel_delayed_work_sync(&bma250->work);
        schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
    }
}

static void bma250_set_enable(struct i2c_client *client, bool Enable)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return;
    }
    if (bma250->enable == Enable)
        return;

    mutex_lock(&bma250->enable_mutex);
    bma250->enable = Enable;
    if (bma250->enable)
    {
        bma250_set_mode(bma250->client, bma250->old_cfg.mode);
        if (!bma250->interrupt)
        {	/* Polling Mode */
            schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
        }
        else
        {	/* Interrupt Mode */
            if (!bma250->irq_flag)
            {
                bma250->irq_flag = true;
                enable_irq(bma250->irq);
            }
        }
    }
    else
    {
        /* Interrupt Mode */
        if (bma250->irq_flag)
        {
            bma250->irq_flag = false;
            disable_irq(bma250->irq);
        }
        /* Polling Mode */
        cancel_delayed_work_sync(&bma250->work);
        bma250_set_mode(bma250->client, BMA250_MODE_SUSPEND);
    }
    mutex_unlock(&bma250->enable_mutex);

    GSENSOR_DEBUG(LEVEL0, "enable = %d", bma250->enable);
}

static void bma250_set_value(struct i2c_client *client, unsigned int Value)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        GSENSOR_DEBUG(LEVEL0, "NULL.");
        return;
    }
    if (Value != 3112)	//cal
        return;

    if (bma250->enable)
        bma250_calibration(client);
}

static ssize_t bma250_bandwidth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned char data = 0;

    if (bma250_get_bandwidth(bma250->client, &data) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    GSENSOR_DEBUG(LEVEL0, "%d", data);
    return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t bma250_bandwidth_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    if (bma250_set_bandwidth(bma250->client, (unsigned char)data) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    GSENSOR_DEBUG(LEVEL2, "%ld", data);
    return count;
}

#ifdef GSENSOR_DBG
static ssize_t bma250_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct bma250_data *bma250 = dev_get_drvdata(dev);

    GSENSOR_DEBUG(LEVEL0, "%d", gsensor_debug_level);
    return snprintf(buf, PAGE_SIZE, "%d\n", gsensor_debug_level);
}

static ssize_t bma250_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    gsensor_debug_level = data;

    GSENSOR_DEBUG(LEVEL0, "%ld", data);
    return count;
}
#endif

static ssize_t bma250_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    GSENSOR_DEBUG(LEVEL0, "%dms", bma250->delay);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->delay);
}

static ssize_t bma250_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250_set_delay(bma250->client, (unsigned int)data);

    GSENSOR_DEBUG(LEVEL2, "%ldms", data);
    return count;
}

static ssize_t bma250_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    GSENSOR_DEBUG(LEVEL0, "%d", bma250->enable);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->enable);
}

static ssize_t bma250_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250_set_enable(bma250->client, (bool)data);

    GSENSOR_DEBUG(LEVEL2, "%ld", data);
    return count;
}

static ssize_t bma250_interrupt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    GSENSOR_DEBUG(LEVEL0, "%d", bma250->interrupt);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->interrupt);
}

static ssize_t bma250_interrupt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250->interrupt = (bool)data;
    if (!bma250->interrupt)
    {	/* Polling Mode */
        if (bma250->irq_flag)
        {
            bma250->irq_flag = false;
            disable_irq(bma250->irq);
        }
        schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
    }
    else
    {	/* Interrupt Mode */
        cancel_delayed_work_sync(&bma250->work);
        if (!bma250->irq_flag)
        {
            bma250->irq_flag = true;
            enable_irq(bma250->irq);
        }
    }

    GSENSOR_DEBUG(LEVEL2, "%ld", data);
    return count;
}

static ssize_t bma250_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned char data = 0;

    if (bma250_get_mode(bma250->client, &data) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    GSENSOR_DEBUG(LEVEL0, "%d", data);
    return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t bma250_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    if (bma250_set_mode(bma250->client, (unsigned char)data) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    GSENSOR_DEBUG(LEVEL2, "%ld", data);
    return count;
}

static ssize_t bma250_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned char data = 0;

    if (bma250_get_range(bma250->client, &data) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    GSENSOR_DEBUG(LEVEL0, "%d", data);
    return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t bma250_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    if (bma250_set_range(bma250->client, (unsigned char)data) < 0)
        GSENSOR_DEBUG(LEVEL0, "Failed.");

    GSENSOR_DEBUG(LEVEL2, "%ld", data);
    return count;
}

static ssize_t bma250_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    if (bma250->reg.flag)
    {
        if (bma250_smbus_read_byte(bma250->client, bma250->reg.addr, &bma250->reg.data) < 0)
            GSENSOR_DEBUG(LEVEL0, "Failed.");
    }

    GSENSOR_DEBUG(LEVEL0, "addr = 0x%2x, data = 0x%2x, flag = %d", bma250->reg.addr, bma250->reg.data, bma250->reg.flag);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->reg.data);
}

static ssize_t bma250_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long addr = 0, data = 0, flag = 0;
    int v = 0;
    char **p = argv_split(GFP_KERNEL, buf, &v);
    if (strict_strtoul(p[0], 10, &addr) || strict_strtoul(p[1], 10, &data) || strict_strtoul(p[2], 10, &flag))
    {
        argv_free(p);
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }
    argv_free(p);

    if (flag)
    {
        bma250->reg.addr = addr;
        bma250->reg.data = data;
        bma250->reg.flag = flag;
    }
    else
    {
        if (bma250_smbus_write_byte(bma250->client, (unsigned char)addr, (unsigned char)data) < 0)
            GSENSOR_DEBUG(LEVEL0, "Failed.");

        GSENSOR_DEBUG(LEVEL2, "Reg[0x%2lx] = 0x%2lx", addr, data);
    }

    GSENSOR_DEBUG(LEVEL2, "%s", buf);
    return count;
}

static ssize_t bma250_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    struct bma250acc acc_value;

    mutex_lock(&bma250->value_mutex);
    acc_value = bma250->value;
    mutex_unlock(&bma250->value_mutex);

    GSENSOR_DEBUG(LEVEL2, "x = %3d, y = %3d, z = %3d", acc_value.x, acc_value.y, acc_value.z);
    return snprintf(buf, PAGE_SIZE, "%d %d %d\n", acc_value.x, acc_value.y, acc_value.z);
}

static ssize_t bma250_value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        GSENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250_set_value(bma250->client, (unsigned int)data);

    GSENSOR_DEBUG(LEVEL2, "%ld", data);
    return count;
}

static ssize_t bma250_update_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct bma250_data *bma250 = dev_get_drvdata(dev);

	bma250_read_accel_xyz(bma250);
	return count;
}

static ssize_t bma250_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    return snprintf(buf, PAGE_SIZE,"%d\n", bma250->selftest_result);
}

#define AXI_AVERAGE(client,axi,loop,average_value,value,nav)   \
    do{         \
        for(loop=0;loop<15;loop++)  \
        {   \
            bma250_set_selftest_stn(client,nav);    \
            mdelay(30); \
            bma250_read_accel_##axi(client, &value);    \
            average_value +=value;  \
            value = 0;  \
        }   \
    } while(0)

#define AXI_CLEAN_DATA(a,b) \
    do{         \
        a = 0;  \
        b = 0;  \
    }while(0)

#define AXI_AVERAGE_DIFF(diff,value1,value2,n)  \
    do{     \
        diff = (value1/n)-(value2/n);   \
    }while(0)

static ssize_t bma250_selftest_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data;
    unsigned char clear_value = 0,mode=0,bandwidth=0,range=0;
    int error,i=0,ave_value1=0,ave_value2=0;
    short value1 = 0;
    short value2 = 0;
    short diff = 0;
    unsigned long result = 0;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    error = strict_strtoul(buf, 10, &data);
    if (error)
        return error;

    if (data != 1)
        return -EINVAL;

    bma250_get_mode(bma250->client, &mode);
    bma250_get_bandwidth(bma250->client,&bandwidth);
    bma250_get_range(bma250->client,&range);

    /* set to 2 G range */
    if (bma250_set_range(bma250->client, 0x3) < 0)
        return -EINVAL;

    if (bma250_set_bandwidth(bma250->client, 0x1f) < 0)
        return -EINVAL;

    bma250_smbus_write_byte(bma250->client, 0x32, clear_value);
    /* 1 for x-axis*/
    bma250_set_selftest_st(bma250->client, 1);
    /* positive direction*/
    AXI_AVERAGE(bma250->client,x,i,ave_value1,value1,0);
    /* negative direction*/
    AXI_AVERAGE(bma250->client,x,i,ave_value2,value2,1);
    AXI_AVERAGE_DIFF(diff,ave_value1,ave_value2,15);

    printk(KERN_INFO "diff x is %d, value1 is %d, value2 is %d\n",
    diff, ave_value1, ave_value2);
    AXI_CLEAN_DATA(ave_value1,ave_value2);
    if (abs(diff) < 200)
        result |= 1;

    /* 2 for y-axis*/
    bma250_set_selftest_st(bma250->client, 2);
    /* positive direction*/
    AXI_AVERAGE(bma250->client,y,i,ave_value1,value1,0);
    /* negative direction*/
    AXI_AVERAGE(bma250->client,y,i,ave_value2,value2,1);
    AXI_AVERAGE_DIFF(diff,ave_value1,ave_value2,15);

    printk(KERN_INFO "diff y is %d, value1 is %d, value2 is %d\n",
    diff, ave_value1, ave_value2);
    AXI_CLEAN_DATA(ave_value1,ave_value2);
    if (abs(diff) < 200)
        result |= 2;

    /* 3 for z-axis*/
    bma250_set_selftest_st(bma250->client, 3);
    /* positive direction*/
    AXI_AVERAGE(bma250->client,z,i,ave_value1,value1,0);
    /* negative direction*/
    AXI_AVERAGE(bma250->client,z,i,ave_value2,value2,1);
    AXI_AVERAGE_DIFF(diff,ave_value1,ave_value2,15);

    printk(KERN_INFO "diff z is %d, value1 is %d, value2 is %d\n",
    diff, ave_value1, ave_value2);
    if (abs(diff) < 100)
        result |= 4;

    bma250->selftest_result = result;


    printk(KERN_INFO "self test finished\n");
    bma250_smbus_write_byte(bma250->client, 0x32, clear_value);
    bma250_set_mode(bma250->client,mode);
    bma250_set_bandwidth(bma250->client,bandwidth);
    bma250_set_range(bma250->client,range);
    bma250_reset(bma250->client);
    
    return count;
}
    
/*PERI-AC-Interrupt-00+{*/
static ssize_t bma250_gaxi_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
   unsigned  long data = 0;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma250_data *bma250 = i2c_get_clientdata(client);
    int err=0;
    err = strict_strtoul(buf, 10, &data);
    if (err)
        return err;
    bma250->gaxi.x = data;
    bma250->gaxi.y = data;
    bma250->gaxi.z = data;
    return count;
}
static ssize_t bma250_polling_count_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
    unsigned long data = 0;
    int err = 0;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    err = strict_strtoul(buf, 10, &data);
    if (err)
        return err;

    bma250->polling_count = data;
    return count;
}
static ssize_t bma250_pdelay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
    unsigned long data = 0;
    int err = 0;
    struct i2c_client *client = to_i2c_client(dev);
    struct bma250_data *bma250 = i2c_get_clientdata(client);
    err = strict_strtoul(buf, 10, &data);

    if (err)
        return err;

    bma250->pdelay= data;
    return count;
}
/*PERI-AC-Interrupt-00+}*/
static DEVICE_ATTR(bandwidth, 0660, bma250_bandwidth_show, bma250_bandwidth_store);
#ifdef GSENSOR_DBG
static DEVICE_ATTR(debug, 0660, bma250_debug_show, bma250_debug_store);
#endif
static DEVICE_ATTR(delay, 0660, bma250_delay_show, bma250_delay_store);
static DEVICE_ATTR(enable, 0660, bma250_enable_show, bma250_enable_store);
static DEVICE_ATTR(interrupt, 0660, bma250_interrupt_show, bma250_interrupt_store);
static DEVICE_ATTR(mode, 0660, bma250_mode_show, bma250_mode_store);
static DEVICE_ATTR(range, 0660, bma250_range_show, bma250_range_store);
static DEVICE_ATTR(reg, 0660, bma250_reg_show, bma250_reg_store);
static DEVICE_ATTR(value, 0640, bma250_value_show, bma250_value_store);
static DEVICE_ATTR(update, 0660,NULL, bma250_update_store);
static DEVICE_ATTR(selftest, 0660,bma250_selftest_show, bma250_selftest_store);
/*PERI-AC-Interrupt-00+{*/
static DEVICE_ATTR(axi, 0660,NULL, bma250_gaxi_store);
static DEVICE_ATTR(count, 0660,NULL, bma250_polling_count_store);
static DEVICE_ATTR(pd, 0660,NULL, bma250_pdelay_store);
/*PERI-AC-Interrupt-00+}*/
static struct attribute *bma250_attributes[] = {
    &dev_attr_bandwidth.attr,
#ifdef GSENSOR_DBG
    &dev_attr_debug.attr,
#endif
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_interrupt.attr,
    &dev_attr_mode.attr,
    &dev_attr_range.attr,
    &dev_attr_reg.attr,
    &dev_attr_value.attr,
    &dev_attr_update.attr,
    &dev_attr_selftest.attr,
/*PERI-AC-Interrupt-00+{*/
    &dev_attr_axi.attr,
    &dev_attr_count.attr,
    &dev_attr_pd.attr,
/*PERI-AC-Interrupt-00+}*/
    NULL
};

static struct attribute_group bma250_attribute_group = {
    .attrs = bma250_attributes
};

static int bma250_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    struct i2c_adapter *adapter = client->adapter;

    if (!adapter)
    {
        GSENSOR_DEBUG(LEVEL0, "Find I2C adapter failed.");
        return -ENOMEM;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
    {
        GSENSOR_DEBUG(LEVEL0, "Check I2C functionality failed.");
        return -ENODEV;
    }

    strlcpy(info->type, dev_name(&client->dev), I2C_NAME_SIZE);

    GSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}


static void bma250_delayed_work(struct work_struct *work)
{
    struct bma250_data *bma250 = container_of((struct delayed_work *)work, struct bma250_data, work);

    bma250_read_accel_xyz(bma250);
    schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
}

static int bma250_input_init(struct bma250_data *bma250)
{
    struct input_dev *dev = input_allocate_device();
    int err = 0;

    if (!dev)
    {
        GSENSOR_DEBUG(LEVEL0, "Allocate input device failed.");
        return -ENOMEM;
    }
    dev->name = "bma250";
    dev->id.bustype = BUS_I2C;

    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_abs_params(dev, ABS_X, -4096, 4095, 0, 0);
    input_set_abs_params(dev, ABS_Y, -4096, 4095, 0, 0);
    input_set_abs_params(dev, ABS_Z, -4096, 4095, 0, 0);
    input_set_drvdata(dev, bma250);

    err = input_register_device(dev);
    if (err < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "Register input device failed.");
        input_free_device(dev);
        return err;
    }
    bma250->input = dev;

    GSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static void bma250_input_delete(struct bma250_data *bma250)
{
    struct input_dev *dev = bma250->input;

    if (!dev)
    {
        GSENSOR_DEBUG(LEVEL0, "Find input device failed.");
        return;
    }

    input_unregister_device(dev);
    input_free_device(dev);
    GSENSOR_DEBUG(LEVEL0, "Done.");
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h)
{
    struct bma250_data *data = container_of(h, struct bma250_data, early_suspend);
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+{*/
    /*As system in suspend state, framework will disable sensor.
      *Then driver suspend function can control LDO to low power mode*/
    bma250_power_control(data, 0);
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+}*/
    GSENSOR_DEBUG(LEVEL2, "Done.");
    return;
}

static void bma250_late_resume(struct early_suspend *h)
{
    struct bma250_data *data = container_of(h, struct bma250_data, early_suspend);
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+{*/
    /*As resume, driver can control LDO from low power mdoe to normal mode*/
    bma250_power_control(data, 1);
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+}*/
    GSENSOR_DEBUG(LEVEL2, "Done.");
    return;
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_PM
static int bma250_suspend(struct device *dev)
{
    //struct bma250_data *data = dev_get_drvdata(dev);

    GSENSOR_DEBUG(LEVEL2, "Done.");
    return 0;
}

static int bma250_resume(struct device *dev)
{
    //struct bma250_data *data = dev_get_drvdata(dev);

    GSENSOR_DEBUG(LEVEL2, "Done.");
    return 0;
}

static struct dev_pm_ops bma250_pm_ops = {
    .suspend = bma250_suspend,
    .resume  = bma250_resume,
};
#endif	/* CONFIG_PM */

static int bma250_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    int tempvalue = 0;
    struct bma250_data *data;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        GSENSOR_DEBUG(LEVEL0, "Check I2C functionality failed.");
        goto exit;
    }

    data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
    if (!data)
    {
        err = -ENOMEM;
        GSENSOR_DEBUG(LEVEL0, "Allocate kernel memory failed.");
        goto exit;
    }

    i2c_set_clientdata(client, data);
    data->platform_data = client->dev.platform_data;
    if (!data->platform_data)
    {
        GSENSOR_DEBUG(LEVEL0, "Check platform data failed.");
        goto kfree_exit;
    }

    err = data->platform_data->gpio_init();
    if (err < 0)
        goto kfree_exit;

    err = bma250_reset(client);
    if (err < 0)
        goto kfree_exit;

    tempvalue = i2c_smbus_read_word_data(client, BMA250_CHIP_ID_REG);
    if ((tempvalue&0x00FF) == BMA250_CHIP_ID)
    {
        GSENSOR_DEBUG(LEVEL0, "Chip ID = 0x%x.", tempvalue);
    }
    else
    {
        GSENSOR_DEBUG(LEVEL0, "Check Chip ID failed.");
        err = -1;
        goto kfree_exit;
    }
    i2c_set_clientdata(client, data);
    data->client = client;
    data->irq = client->irq;
    mutex_init(&data->enable_mutex);
    mutex_init(&data->value_mutex);

/*MTD-PERIPHERAL-AC-Layout-01+{*/
    if(fih_get_product_phase() == PHASE_PD)
    {
        data->platform_data->layout = 7;
    }
/*MTD-PERIPHERAL-AC-Layout-01+}*/
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+{*/
    err = bma250_power_init(data);
    if(err < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "regulator get fail.");
    }
/*MTD-PERIPHERAL-AC-VREG_CONTROL-00+}*/
    INIT_DELAYED_WORK(&data->work, bma250_delayed_work);
    INIT_DELAYED_WORK(&data->irq_work, bma250_irq_work);	/* FIH-SW1-PERIPHERAL-FG-GSENSOR_INTERRUPT-00* */
    err = request_irq(data->irq, bma250_irq_handler, IRQF_TRIGGER_RISING, dev_name(&client->dev), data);
    if (err < 0)
    {
        GSENSOR_DEBUG(LEVEL0, "Request IRQ(%d) failed.", data->irq);
        goto error_irq;
    }
    disable_irq(data->irq);
    data->delay = 200;
    data->enable = true;
    bma250_set_enable(data->client, false);

    err = bma250_input_init(data);
    if (err < 0)
        goto error_input;

    err = sysfs_create_group(&data->input->dev.kobj, &bma250_attribute_group);
    if (err < 0)
        goto error_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    data->early_suspend.suspend = bma250_early_suspend;
    data->early_suspend.resume = bma250_late_resume;
    register_early_suspend(&data->early_suspend);
#endif

    GSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;

error_sysfs:
    bma250_input_delete(data);
error_input:
    free_irq(data->irq, data);
error_irq:
    cancel_delayed_work_sync(&data->irq_work);
    cancel_delayed_work_sync(&data->work);
kfree_exit:
    kfree(data);
exit:
    GSENSOR_DEBUG(LEVEL0, "Failed.");
    return err;
}

static int bma250_remove(struct i2c_client *client)
{
    struct bma250_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&data->early_suspend);
#endif
    sysfs_remove_group(&data->input->dev.kobj, &bma250_attribute_group);
    bma250_input_delete(data);
    free_irq(data->irq, data);
    cancel_delayed_work_sync(&data->irq_work);
    cancel_delayed_work_sync(&data->work);
    kfree(data);

    GSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static const struct i2c_device_id bma250_id[] = {
    { GSENSOR_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = GSENSOR_NAME,
#ifdef CONFIG_PM
        .pm     = &bma250_pm_ops,
#endif
    },
    .class      = I2C_CLASS_HWMON,
    .id_table   = bma250_id,
    .probe      = bma250_probe,
    .remove     = bma250_remove,
    .detect     = bma250_detect,
};

static int __init bma250_init(void)
{
    GSENSOR_DEBUG(LEVEL2, "Done.");
    return i2c_add_driver(&bma250_driver);
}

static void __exit bma250_exit(void)
{
    GSENSOR_DEBUG(LEVEL2, "Done.");
    i2c_del_driver(&bma250_driver);
}

module_init(bma250_init);
module_exit(bma250_exit);

MODULE_AUTHOR("FIH-SW1-PERIPHERAL FromkerGu <fromkergu@fihspec.com>");
MODULE_DESCRIPTION("Bosch BMA250 Acceleration Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

