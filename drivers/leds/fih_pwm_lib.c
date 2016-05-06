/********************************************************************
@
@
@
@
@
@
@
@
@ Foxconn, Inc
@
@ 20120901, OwenHuang, Initial Version
@ 20121019, BennetJau  , Modify for PM8038 support
********************************************************************/
#include <linux/fih_pwm_lib.h>

#define LED_PWM_PERIOD 1000
#define MAX_BRIGHTNESS 511
#define LUT_TABLE_SIZE 64
#define FADE_IN_OUT_DURATION 1000 //ms

#define PWM_ERR(fmt, args...) printk(KERN_ERR "[%s][ERR] PWM_LIB_LED : "fmt" \n", __func__, ##args)
#define PWM_WAN(fmt, args...) printk(KERN_WARNING"[%s][WAN] PWM_LIB_LED : "fmt" \n", __func__, ##args)
#define PWM_MSG(fmt, args...) printk(KERN_INFO"[%s][MSG] PWM_LIB_LED : "fmt" \n", __func__, ##args)

//#define PWM_FIH_DEBUG
#ifdef PWM_FIH_DEBUG
#define PWM_IFO(fmt, args...) printk(KERN_INFO"[%s][IFO] PWM_LIB_LED : "fmt" \n", __func__, ##args)
#define PWM_DBG(fmt, args...) printk(KERN_DEBUG "[%s][DBG] PWM_LIB_LED : "fmt" \n", __func__, ##args)
#else
#define PWM_IFO(fmt, args...)
#define PWM_DBG(fmt, args...) 
#endif


//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
//Select one of them
//#define Config_LED_PM8921_FIH //for PM8921
#define Config_LED_PM8038_FIH //for PM8038

#define MAX_LC_LED_BRIGHTNESS 		20

#ifdef Config_LED_PM8921_FIH
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + 152)
//LED SSBI register control
#define PM8XXX_LED_OFFSET(id) 		((id) - 1) 
#define SSBI_REG_ADDR_LED_CTRL_BASE	0x131
#define SSBI_REG_ADDR_LED_CTRL(n)	(SSBI_REG_ADDR_LED_CTRL_BASE + (n))
#define PM8XXX_DRV_LED_CTRL_MASK	0xf8
#define PM8XXX_DRV_LED_CTRL_SHIFT	0x03
/*For lpg_mapp*/
enum {
LPG5_TO_PWM_CHANNEL = 0,
LPG6_TO_PWM_CHANNEL,
LPG7_TO_PWM_CHANNEL,
LPG_MAX,
};

/*{LED_ID, PWM_CHANNEL, CURRENT_VALUE}*/
static int lpg_mapping_setting[LPG_MAX][3] = {
	[LPG5_TO_PWM_CHANNEL] = {3, 1, MAX_LC_LED_BRIGHTNESS},
	[LPG6_TO_PWM_CHANNEL] = {2, 2, MAX_LC_LED_BRIGHTNESS},
	[LPG7_TO_PWM_CHANNEL] = {1, 3, MAX_LC_LED_BRIGHTNESS},
};
#endif

#ifdef Config_LED_PM8038_FIH
//Macros assume PMIC GPIOs and MPPs start at 1
#define PM8038_GPIO_BASE		152
#define PM8038_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8038_GPIO_BASE)
#define PM8038_MPP_BASE			(PM8038_GPIO_BASE + 12)
#define PM8038_MPP_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8038_MPP_BASE)

//for LPG control, "CNTL" to control on/off, "TEST" to control current value
#define SSBI_REG_ADDR_RGB_CNTL1		0x12D
#define SSBI_REG_ADDR_RGB_CNTL2		0x12E
#define SSBI_REG_ADDR_RGB_TEST		0x12F //PERI-BJ-SetCurrent-00+

//PERI-BJ-Set_PWM_register_default-00+{
#define SSBI_REG_ADDR_LPG_BANK_SEL	0x143
#define SSBI_REG_ADDR_LPG_CTL7		0x14D
#define SSBI_REG_ADDR_LPG_BANK_ENABLE_LO 0x130
//PERI-BJ-Set_PWM_register_default-00+}

#define PM8XXX_DRV_RGB_RED_LED		BIT(2)
#define PM8XXX_DRV_RGB_GREEN_LED	BIT(1)
#define PM8XXX_DRV_RGB_BLUE_LED		BIT(0)

/*For lpg_mapp*/
enum {
	LPG4_TO_PWM_CHANNEL = 0,
	LPG5_TO_PWM_CHANNEL,
	LPG6_TO_PWM_CHANNEL,
	LPG_MAX,
};

enum {
	PM8XXX_ID_RGB_LED_RED = 8,
	PM8XXX_ID_RGB_LED_GREEN,
	PM8XXX_ID_RGB_LED_BLUE,
};
#endif


//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}

/*look up table for LPG bank use*/
static int local_pwm_lut_table[PM_PWM_LUT_SIZE] = {0};
static int local_pwm_lut_table_diff[3][PM_PWM_LUT_SIZE] = {{0},{0},{0}};//MTD-SW3-PERIPHERAL-BJ-LED_FADE_IN_OUT_DIFF-00+


/*PMIC GPIO Config Settings*/
struct pm_gpio pmic_gpio_config_off = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
		.inv_int_pol    = 0,
		.vin_sel        = 2,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
};

struct pm_gpio pmic_gpio_config_on = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
		.inv_int_pol    = 0,
		.vin_sel        = 2,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
};

//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
#ifdef Config_LED_PM8038_FIH
/*PMIC MPP Config Settings*/
struct pm8xxx_mpp_config_data pmic_mpp_config_off = {
		.type			= PM8XXX_MPP_TYPE_SINK,
		.level			= PM8XXX_MPP_CS_OUT_5MA,
		.control 		= PM8XXX_MPP_CS_CTRL_MPP_LOW_EN,
};

struct pm8xxx_mpp_config_data pmic_mpp_config_on = {
	.type			= PM8XXX_MPP_TYPE_SINK,
	.level			= PM8XXX_MPP_CS_OUT_5MA,
	.control		= PM8XXX_MPP_CS_CTRL_MPP_LOW_EN,
};
#endif
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}


typedef int (* control_led)(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data);

typedef struct _led_functions {
	char *name;
	control_led led_actions[LED_ACTION_MAX];
} led_functions;


#ifdef Config_LED_PM8921_FIH
//MTD-SW3-PERIPHERAL-OH-LED_Porting-01+{
/*Config LED DRV with PWM Channel*/
/**************************************************************************************************************************
@ Qualcomm SR:
@ LPG 8 VIB 
@ LPG 7 LED_DRV<2:1> 
@ LPG 6 LED_DRV<2:0> 
@ LPG 5 LED_DRV<2:0> 
@ LPG 4 KYP_DRV 
@ LPG 3 GPIO26 
@ LPG 2 GPIO25 
@ LPG 1 GPIO24 
@
@ For LED_DRV0: 
@ PWM1 = ATC_LED_EN (from SMBC), PWM2 = LPG6, and PWM3 = LPG7 
@
@ For LED_DRV1: 
@ PWM1 = LPG5, PWM2 = LPG6, and PWM3 = LPG7 
@
@ For LED_DRV2: 
@ PWM1 = LPG5, PWM2 = LPG6, and PWM3 = LPG7 	
**************************************************************************************************************************/
static int led_drv_mapping_to_pwm_channel(struct pwm_device *pwm_dev, u8 led_id, u8 reg, int current_value)
{
	int ret = 0; 
	u16 offset = 0;
	u8 control_reg = reg;
	u8 level = 0;

	//got the chip
	if (pwm_dev->chip == NULL)
	{
		PWM_ERR("pwm chip data is NULL");
		return -EINVAL;
	}

	if (current_value > MAX_LC_LED_BRIGHTNESS)
	{
		current_value = MAX_LC_LED_BRIGHTNESS;
	}

	level = (current_value << PM8XXX_DRV_LED_CTRL_SHIFT) & PM8XXX_DRV_LED_CTRL_MASK;
	offset = PM8XXX_LED_OFFSET(led_id);
	control_reg &= ~PM8XXX_DRV_LED_CTRL_MASK;
	control_reg |= level;

	/*Write data through SSBI bus*/
	ret = pm8xxx_writeb(pwm_dev->chip->dev->parent, SSBI_REG_ADDR_LED_CTRL(offset), control_reg);
	if (ret)
	{
		PWM_ERR("write pm8xxx ssbi register error, ret = %d", ret);
	}

	return ret;
}
//MTD-SW3-PERIPHERAL-OH-LED_Porting-01+}
#endif

//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
#ifdef Config_LED_PM8038_FIH
static void
led_rgb_write(struct pwm_device *pwm_dev, u8 led_id, u16 addr, int value)
{
	int rc;
	u8 val, mask;

	if (led_id != PM8XXX_ID_RGB_LED_BLUE &&
		led_id != PM8XXX_ID_RGB_LED_RED &&
		led_id != PM8XXX_ID_RGB_LED_GREEN)
		return;

	rc = pm8xxx_readb(pwm_dev->chip->dev->parent, addr, &val);
	if (rc) {
		PWM_ERR("can't read rgb ctrl register rc=%d", rc);
		return;
	}

	//PWM_IFO("val=0x%x, led_id=%d", val, led_id);

	switch (led_id) {
	case PM8XXX_ID_RGB_LED_RED:
		mask = PM8XXX_DRV_RGB_RED_LED;
		//PWM_IFO("PM8XXX_DRV_RGB_RED_LED");

		break;
	case PM8XXX_ID_RGB_LED_GREEN:
		mask = PM8XXX_DRV_RGB_GREEN_LED;
		//PWM_IFO("PM8XXX_DRV_RGB_GREEN_LED");

		break;
	case PM8XXX_ID_RGB_LED_BLUE:
		mask = PM8XXX_DRV_RGB_BLUE_LED;
		//PWM_IFO("PM8XXX_DRV_RGB_BLUE_LED");
		break;
	default:
		return;
	}

	if (value)
		val |= mask;
	else
		val &= ~mask;

//PERI-BJ-SetCurrent-00+{
	if( addr== SSBI_REG_ADDR_RGB_TEST ) {
		switch(value)
		 {
			case RGB_12mA:
				val &= 0xFC;
				//PWM_IFO("SSBI_REG_ADDR_RGB_TEST:RGB_12MA, led_id=%d", led_id);
				break;
			case RGB_8mA:
				val &= 0xFC;
				val |= 0x1;
				//PWM_IFO("SSBI_REG_ADDR_RGB_TEST:RGB_8MA, led_id=%d", led_id);
				break;
			case RGB_4mA:
				val &= 0xFC;
				val |= 0x2;
				//PWM_IFO("SSBI_REG_ADDR_RGB_TEST:RGB_4MA, led_id=%d", led_id);
				break;
			default:
				val &= 0xFC;
				val |= 0x2;
				//PWM_IFO("SSBI_REG_ADDR_RGB_TEST:Default, led_id=%d", led_id);
				break;
		}
	}
//PERI-BJ-SetCurrent-00+}

	//PWM_IFO("val=0x%x, led_id=%d", val, led_id);


	rc = pm8xxx_writeb(pwm_dev->chip->dev->parent, addr, val);
	if (rc < 0)
		PWM_ERR("can't set rgb led %d level rc=%d", led_id, rc);
}

//PERI-BJ-Set_PWM_register_default-00+{
static void
led_rgb_set_ctl7_default(struct pwm_device *pwm_dev)
{
	int rc;
//	u8 val;
	u16 addr = SSBI_REG_ADDR_LPG_CTL7;

	rc = pm8xxx_writeb(pwm_dev->chip->dev->parent, SSBI_REG_ADDR_LPG_BANK_SEL, pwm_dev->pwm_id);
	if (rc < 0)
		PWM_ERR("pm8xxx_writeb(): rc=%d (Select PWM Bank), pwm_dev->pwm_id=%d\n", rc,pwm_dev->pwm_id);

//	rc = pm8xxx_readb(pwm_dev->chip->dev->parent, SSBI_REG_ADDR_LPG_BANK_SEL, &val);
//	PWM_MSG("val=0x%x", val);

//	rc = pm8xxx_readb(pwm_dev->chip->dev->parent, addr, &val);
//	PWM_MSG("val=0x%x", val);

	rc = pm8xxx_writeb(pwm_dev->chip->dev->parent, addr, 0x0);
	if (rc < 0)
		PWM_ERR("can't set SSBI_REG_ADDR_LPG_CTL7, rc=%d", rc);
}

static void
led_rgb_set_bank_enable_lo_default(struct pwm_device *pwm_dev)
{
	int rc;
	u16 addr = SSBI_REG_ADDR_LPG_BANK_ENABLE_LO;

	rc = pm8xxx_writeb(pwm_dev->chip->dev->parent, addr, 0x0);
	if (rc < 0)
		PWM_ERR("pm8xxx_writeb(): rc=%d (set bank enable lo default)\n", rc);
}
//PERI-BJ-Set_PWM_register_default-00+}


#endif
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}


/*calculate brightness percentage for lut used*/
static int cal_brightness_percentage(int raw_brightness)
{
	return (raw_brightness * 100 / MAX_BRIGHTNESS);
}

/*calculate each ramp up size*/
static int cal_each_ramp_size(int size, int raw_brightness)
{
	return (raw_brightness * 100 / MAX_BRIGHTNESS) / size;
}

//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
/*Based on hardware type to configure related pin, eg PMIC_GPIO, MSM_GPIO, LPG, MPP...*/
static int configure_hw_pin(struct pwm_device *pwm_dev, int on_off, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	
	if (pmic_data == NULL)
	{
		PWM_ERR("invalid pmic_data");
		return (-EIO);
	}
	
	switch(pmic_data->hardware)
	{
		case LED_HW_PMIC_GPIO:
			ret = pm8xxx_gpio_config(pmic_data->pmic_pin, 
										on_off? &pmic_gpio_config_on : &pmic_gpio_config_off);
			PWM_DBG("Configure PMIC GPIO, ret = %d", ret);		
			break;

#ifdef Config_LED_PM8921_FIH
		/*Currently, these are not needed!*/
		case LED_HW_PMIC_MPP:
		case LED_HW_MSM_GPIO:
		case LED_HW_PMIC_LPG:
#endif

#ifdef Config_LED_PM8038_FIH
		case LED_HW_PMIC_MPP:
			ret = pm8xxx_mpp_config(pmic_data->pmic_pin,
										on_off? &pmic_mpp_config_on : &pmic_mpp_config_off);
			PWM_DBG("Configure PMIC MPP, ret = %d", ret);		
			break;

//PERI-BJ-SetCurrent-00+{
		case LED_HW_PMIC_LPG:
			switch (pmic_data->lpg_out)
			{
				case PM_LPG_OUT4:
					PWM_IFO("PM_LPG_OUT4:%d, %d",pmic_data->current_sink,on_off);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_BLUE, SSBI_REG_ADDR_RGB_CNTL1, on_off? MAX_LC_LED_BRIGHTNESS:0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_BLUE, SSBI_REG_ADDR_RGB_CNTL2, on_off? MAX_LC_LED_BRIGHTNESS:0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_BLUE, SSBI_REG_ADDR_RGB_TEST, pmic_data->current_sink);
					break;
				case PM_LPG_OUT5:
					PWM_IFO("PM_LPG_OUT5:%d, %d",pmic_data->current_sink, on_off);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_GREEN, SSBI_REG_ADDR_RGB_CNTL1, on_off? MAX_LC_LED_BRIGHTNESS:0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_GREEN, SSBI_REG_ADDR_RGB_CNTL2, on_off? MAX_LC_LED_BRIGHTNESS:0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_GREEN, SSBI_REG_ADDR_RGB_TEST, pmic_data->current_sink);
					break;
				case PM_LPG_OUT6:
					PWM_IFO("PM_LPG_OUT6:%d, %d",pmic_data->current_sink, on_off);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_RED, SSBI_REG_ADDR_RGB_CNTL1, on_off? MAX_LC_LED_BRIGHTNESS:0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_RED, SSBI_REG_ADDR_RGB_CNTL2, on_off? MAX_LC_LED_BRIGHTNESS:0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_RED, SSBI_REG_ADDR_RGB_TEST, pmic_data->current_sink);
					break;
				default:
					break;
			}
			break;
//PERI-BJ-SetCurrent-00+}
		/*Currently, these are not needed!*/
		case LED_HW_MSM_GPIO:
#endif
		default:
			PWM_ERR("No such IO pin");
			break;

	}

	return ret;
}
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}

//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
#if 0
#ifdef Config_LED_PM8038_FIH
//Use blinking method to workaround original led on/off method
static int pmic_gpio_led_on(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int flags = 0;
	struct pm8xxx_pwm_period pwm_period;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data->pwm_clock;
	pwm_period.pre_div = pmic_data->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data->pwm_div_exp;

	ret = pm8xxx_pwm_config_period(pwm_dev, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm period error, ret = %d", ret);
		return -EIO;
	}

	/*Calculate Blink low and high level brightness percentage*/
	local_pwm_lut_table[0] = cal_brightness_percentage(pmic_data->blinking_pwm2); //PWM_ON_Value
	local_pwm_lut_table[1] = cal_brightness_percentage(pmic_data->blinking_pwm1); //PWM_OFF_Value

	if (pmic_data->invert)
	{
		local_pwm_lut_table[0] = 100 - local_pwm_lut_table[0];
		local_pwm_lut_table[1] = 100 - local_pwm_lut_table[1];
		PWM_MSG("Invert pwm value ");
	}

	/*Setup lut, for on/off usage, the idx_len set to 1*/
	flags = PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN;
	PWM_MSG("Blinking level[%d:%d], flags = 0x%X", local_pwm_lut_table[0], local_pwm_lut_table[1], flags);
	
	ret = pm8xxx_pwm_lut_config(pwm_dev, 1000, local_pwm_lut_table, 1000, 
								pmic_data->lut_table_start, 1, pmic_data->blinking_time2,
								pmic_data->blinking_time1, flags);
	if (ret)
	{
		PWM_ERR("setup lut config error! ret = %d ", ret);
		return ret;
	}

	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev, 1, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d ", ret);
	}

	/*Start*/
	ret = pm8xxx_pwm_lut_enable(pwm_dev, 1);
	if (ret)
	{
		PWM_ERR("enable lut channel error! ret = %d !", ret);
		return ret;
	}

	return ret;

}


static int pmic_gpio_led_off(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;

	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev, 0, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d ", ret);
	}
	/*Diable lpg channel*/
	ret = pm8xxx_pwm_lut_enable(pwm_dev, 0);
	if (ret)
	{
		PWM_ERR("disable pwm error! ret = %d ", ret);
	}

	return ret;

}
#endif
#endif
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}

static int pmic_gpio_led_on(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int duty_ms = 0;
	int brightness = 0;
	struct pm8xxx_pwm_period pwm_period;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data->pwm_clock;
	pwm_period.pre_div = pmic_data->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data->pwm_div_exp;

	ret = pm8xxx_pwm_config_period(pwm_dev, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm period error, ret = %d", ret);
		return -EIO;
	}
	
	/*Config period -> brightness*/
	/*Check if we have to invert*/
	if (pmic_data->invert)
		brightness = MAX_BRIGHTNESS - pmic_data->on_off_pwm;
	else
		brightness = pmic_data->on_off_pwm;
	
	duty_ms = (brightness * LED_PWM_PERIOD) / MAX_BRIGHTNESS;
	PWM_DBG("duty_ms = %d", duty_ms);
	ret = pwm_config(pwm_dev, duty_ms, LED_PWM_PERIOD);
	if (ret)
	{
		PWM_ERR("pwm_config error, ret = %d ", ret);
		return ret;
	}

//MTD-SW3-PERIPHERAL-BJ-LED_QCT_FIX-00-{
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
#ifdef Config_LED_PM8038_FIH
//Per QCT's comment:
//	PM8038 PWM channel has two flip flops for enabling the output.
//	So yes you need to enable it twice in order for it to work
/*
	PWM_DBG("pwm_config do again");

	ret = pwm_config(pwm_dev, duty_ms, LED_PWM_PERIOD);
	if (ret)
	{
		PWM_ERR("pwm_config error, ret = %d ", ret);
		return ret;
	}
*/	
#endif
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}
//MTD-SW3-PERIPHERAL-BJ-LED_QCT_FIX-00-}

#if 0
	/*Check if we have to invert*/
	if (pmic_data->invert)
		brightness = MAX_BRIGHTNESS - pmic_data->on_off_pwm;
	else
		brightness = pmic_data->on_off_pwm;

	ret = pm8xxx_pwm_config_pwm_value(pwm_dev, brightness);
	PWM_MSG("config pwm value ret = %d, PWM = %d", ret, pmic_data->on_off_pwm);
#endif

//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev, 1, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d ", ret);
	}

	/*enable lpg channel*/
	ret = pwm_enable(pwm_dev);
	if (ret)
	{
		PWM_ERR("pwm enable failed(ret = %d)", ret);
		return ret;
	}
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}

	return ret;
}

static int pmic_gpio_led_off(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;

	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev, 0, pmic_data);//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d ", ret);
	}
	/*Diable lpg channel*/
	pwm_disable(pwm_dev);
	
	return ret;
}

static int pmic_gpio_led_blinking_on(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int flags = 0;
	struct pm8xxx_pwm_period pwm_period;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data->pwm_clock;
	pwm_period.pre_div = pmic_data->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data->pwm_div_exp;

	ret = pm8xxx_pwm_config_period(pwm_dev, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm period error, ret = %d", ret);
		return -EIO;
	}

	/*Calculate Blink low and high level brightness percentage*/
	local_pwm_lut_table[0] = cal_brightness_percentage(pmic_data->blinking_pwm1); //PWM_oFF_Value
	local_pwm_lut_table[1] = cal_brightness_percentage(pmic_data->blinking_pwm2); //PWM_ON_Value

	if (pmic_data->invert)
	{
		local_pwm_lut_table[0] = 100 - local_pwm_lut_table[0];
		local_pwm_lut_table[1] = 100 - local_pwm_lut_table[1];
		PWM_MSG("Invert pwm value");
	}

	/*Setup lut*/
	flags = PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN;
	PWM_MSG("Blinking level[%d:%d], flags = 0x%X", local_pwm_lut_table[0], local_pwm_lut_table[1], flags);

	//PERI-BJ-ChangeBlinkingSequence-00+{
	//PERI-BJ-ChangeInterval_ImproveAccurate-00+{	
	//The duty_time_ms is ramp level's time, set it smaller (32) and use blinking time to pause hi & lo.
	ret = pm8xxx_pwm_lut_config(pwm_dev, 1000, local_pwm_lut_table, 32, 
								pmic_data->lut_table_start, 2, pmic_data->blinking_time1,
								pmic_data->blinking_time2, flags);

	//PERI-BJ-ChangeInterval_ImproveAccurate-00+}
	//PERI-BJ-ChangeBlinkingSequence-00+}
	if (ret)
	{
		PWM_ERR("setup lut config error! ret = %d", ret);
		return ret;
	}

	//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
		/*Configure hw related pin*/
		ret = configure_hw_pin(pwm_dev, 1, pmic_data);
		if (ret)
		{
			PWM_ERR("config IO error, ret = %d", ret);
		}
	//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}

	/*Start blinking*/
	ret = pm8xxx_pwm_lut_enable(pwm_dev, 1);
	if (ret)
	{
		PWM_ERR("enable lut channel error! ret = %d !", ret);
		return ret;
	}
	//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}
	
	return ret;
}

static int pmic_gpio_led_blinking_off(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;

	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev, 0, pmic_data); //MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	/*Diable lpg channel*/
	ret = pm8xxx_pwm_lut_enable(pwm_dev, 0);
	if (ret)
	{
		PWM_ERR("disable pwm error! ret = %d", ret);
	}

	return ret;
}

static int pmic_gpio_led_fade_on(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int flags = 0, index = 0;
	int ramp_up_percentage_size = 0;
	//int current_ramp_up_percentage = 0;
	int interval = 0;
	struct pm8xxx_pwm_period pwm_period;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data->pwm_clock;
	pwm_period.pre_div = pmic_data->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data->pwm_div_exp;

	ret = pm8xxx_pwm_config_period(pwm_dev, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm period error, ret = %d", ret);
		return ret;
	}

	if (pmic_data->fade_in_out_pwm == 0)
	{
		PWM_ERR("Please setup fade in out pwm value firstly!");
		return -EIO;
	}


//PERI-BJ-Fade_in_out_once-00+{
if (pmic_data->interval == 500 || pmic_data->interval == 1000){
	if(pmic_data->lpg_out == PM_LPG_OUT4){
		pmic_data->lut_table_start= 16;
		pmic_data->lut_table_end= 31;
	}

	if(pmic_data->lpg_out == PM_LPG_OUT5){
		pmic_data->lut_table_start= 32;
		pmic_data->lut_table_end= 47;
	}

	if(pmic_data->lpg_out == PM_LPG_OUT6){
		pmic_data->lut_table_start=48;
		pmic_data->lut_table_end=63;
	}
}else if(pmic_data->interval == 1300){
	if(pmic_data->lpg_out == PM_LPG_OUT4){
		pmic_data->lut_table_start= 1;
		pmic_data->lut_table_end= 20;
	}

	if(pmic_data->lpg_out == PM_LPG_OUT5){	
		pmic_data->lut_table_start= 21;
		pmic_data->lut_table_end= 40;
	}
	if(pmic_data->lpg_out == PM_LPG_OUT6){		
		pmic_data->lut_table_start=41;
		pmic_data->lut_table_end=60;
	}
}
else{//3000,1500,800, or others
	if(pmic_data->lpg_out == PM_LPG_OUT4){
		pmic_data->lut_table_start= 1;
		pmic_data->lut_table_end= 21;
	}	
	if(pmic_data->lpg_out == PM_LPG_OUT5){
		pmic_data->lut_table_start= 22;
		pmic_data->lut_table_end= 42;
	}

	if(pmic_data->lpg_out == PM_LPG_OUT6){
		pmic_data->lut_table_start=43;
		pmic_data->lut_table_end=63;
	}
}
//PERI-BJ-Fade_in_out_once-00+}

//MTD-SW3-PERIPHERAL-OH-LED_Porting-02+{
	/*Calculate lut table values*/
	ramp_up_percentage_size = cal_each_ramp_size(pmic_data->lut_table_end - pmic_data->lut_table_start - 1,
												 pmic_data->fade_in_out_pwm);

	if (pmic_data->invert)
	{
		local_pwm_lut_table[0] = 100;
		local_pwm_lut_table[pmic_data->lut_table_end - pmic_data->lut_table_start] = 
									100 - cal_brightness_percentage(pmic_data->fade_in_out_pwm);
	}
	else
	{
		local_pwm_lut_table[0] = 0;
		local_pwm_lut_table[pmic_data->lut_table_end - pmic_data->lut_table_start] = 
									cal_brightness_percentage(pmic_data->fade_in_out_pwm);
	}

	PWM_DBG("local_pwm_lut_table[0], value = %d", local_pwm_lut_table[0]);

	for (index = 1; index < (pmic_data->lut_table_end - pmic_data->lut_table_start); index ++)
	{
		/* assign value to loop up table for fade in/out */
		local_pwm_lut_table[index] = index * ramp_up_percentage_size;

		/* if value is out of range, re-assign max value as our PWM control*/
		if (local_pwm_lut_table[index] > 100)
			local_pwm_lut_table[index]  = 100;

		PWM_DBG("local_pwm_lut_table[%d], value = %d", index, local_pwm_lut_table[index]);

		/*Check if we need to invert*/
		if (pmic_data->invert)
		{
			local_pwm_lut_table[index] = 100 - 	local_pwm_lut_table[index];
			PWM_DBG("invert local_pwm_lut_table[%d] = %d", index, local_pwm_lut_table[index]);
		}
	}

	PWM_DBG("local_pwm_lut_table[%d], value = %d", pmic_data->lut_table_end - pmic_data->lut_table_start,
								local_pwm_lut_table[pmic_data->lut_table_end - pmic_data->lut_table_start]);
	
//MTD-SW3-PERIPHERAL-OH-LED_Porting-02+}

	/*Setup flags*/
	if (pmic_data->ramp_loop)
	{	
		flags |= PM_PWM_LUT_LOOP;
	}

	if (pmic_data->ramp_up_down)
	{
		flags |= PM_PWM_LUT_RAMP_UP;
	}

	if (pmic_data->toggle_up_down)
	{
		flags |= PM_PWM_LUT_REVERSE;
	}

	flags |= (PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN);

	PWM_DBG("PWM control flags = 0x%02X ", flags);

//PERI-BJ-Fade_in_out_once-00+{
	if (pmic_data->interval == 3000){
		interval = 128;
	}else if(pmic_data->interval == 1500 || pmic_data->interval == 1300 || pmic_data->interval == 1000){
		interval = 64;
	}else if(pmic_data->interval == 800){
		interval = 36;
	}
	else if(pmic_data->interval == 500){
		interval = 32;
	}
	else
	{
		/*Calcuate interval of fade-in or fade-out*/
		//interval value not list in QCT's spec may cause the timing not accurate
		interval = pmic_data->interval / (pmic_data->lut_table_end - pmic_data->lut_table_start + 1);
	}

	PWM_DBG("PWM control interval = %d ", interval);
//PERI-BJ-Fade_in_out_once-00+}

	/*Config lut*/
	ret = pm8xxx_pwm_lut_config(pwm_dev, 1000, local_pwm_lut_table, interval,
								pmic_data->lut_table_start, (1 + pmic_data->lut_table_end - pmic_data->lut_table_start), 
								pmic_data->blinking_time2,
								pmic_data->blinking_time1, flags); 
	if (ret)
	{
		PWM_ERR("config lut table error!, ret = %d ", ret);
		return ret;
	}

	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev, 1, pmic_data); //MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	/*Start fade in/out*/
	ret = pm8xxx_pwm_lut_enable(pwm_dev, 1);
	if (ret)
	{
		PWM_ERR("enable lut channel error! ret = %d !", ret);
		return ret;
	}

	return ret;
}

static int pmic_gpio_led_fade_off(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;

	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev, 0, pmic_data);//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	/*Diable lpg channel*/
	ret = pm8xxx_pwm_lut_enable(pwm_dev, 0);
	if (ret)
	{
		PWM_ERR("disable pwm error! ret = %d", ret);
	}

	
	return 0;
}

/*leds control function collection*/
static led_functions led_functions_collection[TYPE_MAX] = {
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
#ifdef Config_LED_PM8921_FIH
[TYPE_PMIC_GPIO] = {
		.name = "LED_PMIC_GPIO",
		.led_actions = {pmic_gpio_led_on, pmic_gpio_led_off, 
						pmic_gpio_led_blinking_on, pmic_gpio_led_blinking_off, 
						pmic_gpio_led_fade_on, pmic_gpio_led_fade_off}, 
},//PMIC GPIO

[TYPE_PMIC_MPP] = {
		.name = "LED_PMIC_MPP",
		.led_actions = {NULL, NULL, NULL, NULL, NULL, NULL}, 
},//PMIC MPP
#endif

#ifdef Config_LED_PM8038_FIH
[TYPE_PMIC_GPIO] = {
		.name = "LED_PMIC_GPIO",
		.led_actions = {NULL, NULL, NULL, NULL, NULL, NULL},
},//PMIC GPIO


[TYPE_PMIC_MPP] = {
		.name = "LED_PMIC_MPP",
		.led_actions = {pmic_gpio_led_on, pmic_gpio_led_off, 
						pmic_gpio_led_blinking_on, pmic_gpio_led_blinking_off, 
						pmic_gpio_led_fade_on, pmic_gpio_led_fade_off},
},//PMIC MPP
#endif
//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}

[TYPE_GPIO] = {
		.name = "LED_GPIO",
		.led_actions = {NULL, NULL, NULL, NULL, NULL, NULL},
}, //GPIO

[TYPE_PMIC_LPG] = {
		.name = "LED_PMIC_LPG",
		.led_actions = {pmic_gpio_led_on, pmic_gpio_led_off, 
						pmic_gpio_led_blinking_on, pmic_gpio_led_blinking_off, 
						pmic_gpio_led_fade_on, pmic_gpio_led_fade_off},
},//PMIC pure LPG contorl
};

//MTD-SW3-PERIPHERAL-BJ-LED_FADE_IN_OUT_DIFF-00*{
#if 0
int led_config_pwm_period(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	struct pm8xxx_pwm_period pwm_period;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data->pwm_clock;
	pwm_period.pre_div = pmic_data->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data->pwm_div_exp;
	ret = pm8xxx_pwm_config_period(pwm_dev, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm period error, ret = %d", ret);
		return ret;
	}

	if (pmic_data->fade_in_out_pwm == 0)
	{
		PWM_ERR("Please setup fade in out pwm value firstly!");
		return -EIO;
	}

	return ret;
}
#endif

//PERI-BJ-Implement_new_algo_for_fade_in_out-00+{
static void cal_component_val(int size, int ramp_up_percentage_size, int raw_percent, int *cX, int *cY)
{
	PWM_DBG("raw_percent = %d, size = %d, ramp_up_percentage_size = %d ", raw_percent, size, ramp_up_percentage_size);

	//special case:raw_percent - size <= 0
	if(raw_percent - size <= 0){
		*cX = raw_percent;
		*cY = size - raw_percent;
		return;
	}

	//ramp_up_percentage_size can't equal or less than 0
	if(ramp_up_percentage_size <= 0){
		PWM_ERR("ramp_up_percentage_size <= 0");
		return;
	}

	if(raw_percent == size * ramp_up_percentage_size){
		//special case:raw_percent == size * ramp_up_percentage_size

		if(ramp_up_percentage_size - 1 == 0){
			//Here, ramp_up_percentage_size - 1 == 0 is imposible
			PWM_ERR("ramp_up_percentage_size - 1 == 0");
			return;
		}

		*cY = (raw_percent - size) / (ramp_up_percentage_size - 1);
	}
	else
		*cY = (raw_percent - size) / ramp_up_percentage_size;

	*cX = size - *cY;

	return;
}
//PERI-BJ-Implement_new_algo_for_fade_in_out-00+}

void set_led_lut_table(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data, int selection)
{
//PERI-BJ-Implement_new_algo_for_fade_in_out-00*{
	int ramp_up_percentage_size = 0;
	int raw_percent = 0;
	int index = 0;
	int index_cX = 0;
	int lut_table_length = 0;

	//cX * (n * cY)
	int cX = 0;
	int cY = 0;

	PWM_DBG("cal_each_ramp_size");
	lut_table_length = pmic_data->lut_table_end - pmic_data->lut_table_start + 1;
	/*Calculate lut table values*/
	ramp_up_percentage_size = cal_each_ramp_size( lut_table_length - 2, pmic_data->fade_in_out_pwm);
	raw_percent = cal_brightness_percentage(pmic_data->fade_in_out_pwm);
	PWM_DBG("pmic_data->fade_in_out_pwm=%d, ramp_up_percentage_size = %d, raw_percent =%d", 
			pmic_data->fade_in_out_pwm, ramp_up_percentage_size, raw_percent);

	//PERI-BJ-Adjust_fade_out_color_to_let_color_as_expected-00+{
	if (raw_percent > 0 && raw_percent < (lut_table_length - 2))
		raw_percent = lut_table_length - 2;
	//PERI-BJ-Adjust_fade_out_color_to_let_color_as_expected-00+}

	cal_component_val(lut_table_length - 2, ramp_up_percentage_size, raw_percent, &cX, &cY);
	PWM_DBG("cX = %d, cY =%d", cX, cY);

#if 0
	//PERI-BJ-Improve_Fade_in_out_effect-00+{

	if(ramp_up_percentage_size <= 0 && pmic_data->fade_in_out_pwm > 0)
		ramp_up_percentage_size = 1;

	PWM_DBG("modified ramp_up_percentage_size = %d", ramp_up_percentage_size );
	//PERI-BJ-Improve_Fade_in_out_effect-00+}

#endif

	if (pmic_data->invert)
	{
		local_pwm_lut_table_diff[selection][0] = 100;
		local_pwm_lut_table_diff[selection][lut_table_length - 1] = 100 - raw_percent;
	}
	else
	{
		local_pwm_lut_table_diff[selection][0] = 0;
		local_pwm_lut_table_diff[selection][lut_table_length - 1] = raw_percent;
	}

	PWM_DBG("local_pwm_lut_table_diff[selection][0], value = %d", local_pwm_lut_table_diff[selection][0]);

#if 0
	for (index = 1; index < lut_table_length - 1; index ++)
	{
		/* assign value to loop up table for fade in/out */
		local_pwm_lut_table_diff[selection][index] = index * ramp_up_percentage_size;

		/* if value is out of range, re-assign max value as our PWM control*/
		if (local_pwm_lut_table_diff[selection][index] > 100)
			local_pwm_lut_table_diff[selection][index]	= 100;

		PWM_DBG("local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

		/*Check if we need to invert*/
		if (pmic_data->invert)
		{
			local_pwm_lut_table_diff[selection][index] = 100 -	local_pwm_lut_table_diff[selection][index];
			PWM_DBG("invert local_pwm_lut_table_diff[selection][%d] = %d", index, local_pwm_lut_table_diff[selection][index]);
		}
	}

#endif 

	if(ramp_up_percentage_size <= 0 && pmic_data->fade_in_out_pwm > 0){
		ramp_up_percentage_size = 0;
	}
	else
	{
		if(raw_percent != (lut_table_length - 2) * ramp_up_percentage_size)
			ramp_up_percentage_size+=1;
	}

	for (index = 1; index <= cY; index ++)
	{
		/* assign value to loop up table for fade in/out */
		local_pwm_lut_table_diff[selection][index] = index * ramp_up_percentage_size;

		/* if value is out of range, re-assign max value as our PWM control*/
		if (local_pwm_lut_table_diff[selection][index] > 100)
			local_pwm_lut_table_diff[selection][index]	= 100;

		PWM_DBG("local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

		/*Check if we need to invert*/
		if (pmic_data->invert)
		{
			local_pwm_lut_table_diff[selection][index] = 100 -	local_pwm_lut_table_diff[selection][index];
			PWM_DBG("invert local_pwm_lut_table_diff[selection][%d] = %d", index, local_pwm_lut_table_diff[selection][index]);
		}
	}

	for (index_cX = 1; index_cX <= cX && index <= lut_table_length - 1 ; index_cX ++, index++)
	{
		/* assign value to loop up table for fade in/out */
		local_pwm_lut_table_diff[selection][index] = cY * ramp_up_percentage_size + index_cX * ((pmic_data->fade_in_out_pwm == 0)? 0: 1);

		/* if value is out of range, re-assign max value as our PWM control*/
		if (local_pwm_lut_table_diff[selection][index] > 100)
			local_pwm_lut_table_diff[selection][index] = 100;

		PWM_DBG("local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

		/*Check if we need to invert*/
		if (pmic_data->invert)
		{
			local_pwm_lut_table_diff[selection][index] = 100 -	local_pwm_lut_table_diff[selection][index];
			PWM_DBG("invert local_pwm_lut_table_diff[selection][%d] = %d", index, local_pwm_lut_table_diff[selection][index]);
		}
	}
//PERI-BJ-Implement_new_algo_for_fade_in_out-00*}

//PERI-BJ-Improve_Fade_in_out_effect-00+{
	PWM_DBG("local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

	//make sure the last lut table item would have the max value
	if(local_pwm_lut_table_diff[selection][index] < local_pwm_lut_table_diff[selection][index-1])
		local_pwm_lut_table_diff[selection][index] = local_pwm_lut_table_diff[selection][index-1];

	PWM_DBG("modified local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);
//PERI-BJ-Improve_Fade_in_out_effect-00+}

	
}

//MTD-SW3-PERIPHERAL-BJ-LED_FADE_IN_OUT_DIFF-00*{
static int pmic_gpio_led_fade_on_diff(struct pwm_device *pwm_dev_r, struct led_pmic_data *pmic_data_r, struct pwm_device *pwm_dev_g, struct led_pmic_data *pmic_data_g, struct pwm_device *pwm_dev_b, struct led_pmic_data *pmic_data_b)
{
	int ret = 0;
	int flags = 0;
	//int current_ramp_up_percentage = 0;
	int interval_r = 0,interval_g = 0,interval_b = 0;

	PWM_DBG("pmic_gpio_led_fade_on_diff++");

/*
	You are using pm8xxx_pwm_config_period() as well as pm8xxx_pwm_lut_config().
	Whereas pm8xxx_pwm_lut_config() has the configuration for period, you don't need to use pm8xxx_pwm_config_period()
*/

//	led_config_pwm_period(pwm_dev_r,pmic_data_r);
//	led_config_pwm_period(pwm_dev_g,pmic_data_g);
//	led_config_pwm_period(pwm_dev_b,pmic_data_b);

	//To match the timing more accurate, we need to customize the settings.
	if (pmic_data_r->interval == 500 || pmic_data_r->interval == 1000){
		pmic_data_b->lut_table_start= 16;
		pmic_data_b->lut_table_end= 31;


		pmic_data_g->lut_table_start= 32;
		pmic_data_g->lut_table_end= 47;

		pmic_data_r->lut_table_start=48;
		pmic_data_r->lut_table_end=63;
	}else if(pmic_data_r->interval == 1300){
		pmic_data_b->lut_table_start= 1;
		pmic_data_b->lut_table_end= 20;
		
		pmic_data_g->lut_table_start= 21;
		pmic_data_g->lut_table_end= 40;
		
		pmic_data_r->lut_table_start=41;
		pmic_data_r->lut_table_end=60;
	}
	else{//3000,1500,800, or others
		pmic_data_b->lut_table_start= 1;
		pmic_data_b->lut_table_end= 21;
		
		
		pmic_data_g->lut_table_start= 22;
		pmic_data_g->lut_table_end= 42;
		
		pmic_data_r->lut_table_start=43;
		pmic_data_r->lut_table_end=63;
	}

	set_led_lut_table(pwm_dev_r,pmic_data_r,0);
	set_led_lut_table(pwm_dev_g,pmic_data_g,1);
	set_led_lut_table(pwm_dev_b,pmic_data_b,2);

	/*Setup flags*/
	if (pmic_data_r->ramp_loop)
	{	
		flags |= PM_PWM_LUT_LOOP;
	}

	if (pmic_data_r->ramp_up_down)
	{
		flags |= PM_PWM_LUT_RAMP_UP;
	}

	if (pmic_data_r->toggle_up_down)
	{
		flags |= PM_PWM_LUT_REVERSE;
	}

	//flags |= (PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN);

	PWM_IFO("PWM control flags = 0x%02X ", flags);


	//To match the timing more accurate, we need to customize the settings.
	if (pmic_data_r->interval == 3000){
		interval_r = 128;
		interval_g = 128;
		interval_b = 128;

	}else if(pmic_data_r->interval == 1500 || pmic_data_r->interval == 1300 || pmic_data_r->interval == 1000){
		interval_r = 64;
		interval_g = 64;
		interval_b = 64;
	}else if(pmic_data_r->interval == 800){
		interval_r = 36;
		interval_g = 36;
		interval_b = 36;
	}
	else if(pmic_data_r->interval == 500){
		interval_r = 32;
		interval_g = 32;
		interval_b = 32;
	}
	else
	{
		/*Calcuate interval of fade-in or fade-out*/
		//interval value not list in QCT's spec may cause the timing not accurate
		interval_r = pmic_data_r->interval / (pmic_data_r->lut_table_end - pmic_data_r->lut_table_start + 1);
		interval_g = pmic_data_g->interval / (pmic_data_g->lut_table_end - pmic_data_g->lut_table_start + 1);
		interval_b = pmic_data_b->interval / (pmic_data_b->lut_table_end - pmic_data_b->lut_table_start + 1);
	}
	PWM_DBG("pm8xxx_pwm_lut_config,pmic_data_r->interval=%d, pmic_data_g->interval=%d, pmic_data_b->interval=%d",pmic_data_r->interval, pmic_data_g->interval, pmic_data_b->interval);
	PWM_DBG("pm8xxx_pwm_lut_config,interval_r=%d, interval_g=%d, interval_b=%d, PM8XXX_PWM_PERIOD_MAX=%ld",interval_r, interval_g, interval_b,PM8XXX_PWM_PERIOD_MAX);

	//	PWM_DBG("pm8xxx_pwm_lut_config,interval_r=%d, interval_g=%d, interval_b=%d, PM8XXX_PWM_PERIOD_MAX=%ld",interval_r, interval_g, interval_b,PM8XXX_PWM_PERIOD_MAX);
	//	PWM_DBG("pm8xxx_pwm_lut_config,pmic_data_r->interval=%d, pmic_data_g->interval=%d, pmic_data_b->interval=%d",pmic_data_r->interval, pmic_data_g->interval, pmic_data_b->interval);
	//	PWM_DBG("pm8xxx_pwm_lut_config,flags=%d, pmic_data_r->blinking_time1=%d, pmic_data_r->blinking_time2 = %d, end-start=%d",flags,pmic_data_r->blinking_time1,pmic_data_r->blinking_time2,(1 + pmic_data_r->lut_table_end - pmic_data_r->lut_table_start));

	/*Config lut*/
	ret = pm8xxx_pwm_lut_config(pwm_dev_r, 1000, local_pwm_lut_table_diff[0], interval_r,
								pmic_data_r->lut_table_start, (1 + pmic_data_r->lut_table_end - pmic_data_r->lut_table_start), 
								pmic_data_r->blinking_time2,
								pmic_data_r->blinking_time1, flags); 
	PWM_DBG("pm8xxx_pwm_lut_config r");

	if (ret)
	{
		PWM_ERR("config lut table error!, ret = %d ", ret);
		return ret;
	}

	/*Config lut*/
	ret = pm8xxx_pwm_lut_config(pwm_dev_g, 1000, local_pwm_lut_table_diff[1], interval_g,
								pmic_data_g->lut_table_start, (1 + pmic_data_g->lut_table_end - pmic_data_g->lut_table_start), 
								pmic_data_g->blinking_time2,
								pmic_data_g->blinking_time1, flags); 
	PWM_DBG("pm8xxx_pwm_lut_config g");


	if (ret)
	{
		PWM_ERR("config lut table error!, ret = %d ", ret);
		return ret;
	}
	
	/*Config lut*/
	ret = pm8xxx_pwm_lut_config(pwm_dev_b, 1000, local_pwm_lut_table_diff[2], interval_b,
								pmic_data_b->lut_table_start, (1 + pmic_data_b->lut_table_end - pmic_data_b->lut_table_start), 
								pmic_data_b->blinking_time2,
								pmic_data_b->blinking_time1, flags); 

	PWM_DBG("pm8xxx_pwm_lut_config b");

	if (ret)
	{
		PWM_ERR("config lut table error!, ret = %d ", ret);
		return ret;
	}



	PWM_DBG("LED turn off to avoid non-smooth issue");

	//led turn off before turn on to avoid rgb nonsync issue
	ret = pmic_gpio_led_fade_off(pwm_dev_r, pmic_data_r);
	if (ret)
	{
		PWM_ERR("pmic_gpio_led_fade_off r error, ret = %d", ret);
	}
	ret = pmic_gpio_led_fade_off(pwm_dev_g, pmic_data_g);
	if (ret)
	{
		PWM_ERR("pmic_gpio_led_fade_off g error, ret = %d", ret);
	}
	ret = pmic_gpio_led_fade_off(pwm_dev_b, pmic_data_b);
	if (ret)
	{
		PWM_ERR("pmic_gpio_led_fade_off b error, ret = %d", ret);
	}


	PWM_IFO("configure_hw_pin");
	/*Configure hw related pin*/
	ret = configure_hw_pin(pwm_dev_r, 1, pmic_data_r); //MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	ret = configure_hw_pin(pwm_dev_g, 1, pmic_data_g); //MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+	
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	ret = configure_hw_pin(pwm_dev_b, 1, pmic_data_b); //MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	PWM_IFO("pm8xxx_pwm_lut_enable");

	/*Start fade in/out*/
	ret = pm8xxx_pwm_lut_enable(pwm_dev_r, 1);
	if (ret)
	{
		PWM_ERR("enable lut channel error! ret = %d !", ret);
		return ret;
	}

	ret = pm8xxx_pwm_lut_enable(pwm_dev_g, 1);
	if (ret)
	{
		PWM_ERR("enable lut channel error! ret = %d !", ret);
		return ret;
	}

	ret = pm8xxx_pwm_lut_enable(pwm_dev_b, 1);
	if (ret)
	{
		PWM_ERR("enable lut channel error! ret = %d !", ret);
		return ret;
	}

	PWM_DBG("pmic_gpio_led_fade_on_diff--");

	return ret;
}
//MTD-SW3-PERIPHERAL-BJ-LED_FADE_IN_OUT_DIFF-00*}


/*Function for led driver use to control LEDs*/
int common_control_leds(struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int index = 0;
	led_functions this_led_functions;

	//PWM_DBG("Enter");

	if (pwm_dev == NULL || pmic_data == NULL)
	{
		PWM_ERR("%s, NULL data type", __func__);
		return -EIO;
	}

	/*If hardware type is valid, determine led control funcionts*/
	if (LED_HW_PMIC_GPIO <= pmic_data->hardware && pmic_data->hardware <= LED_HW_PMIC_LPG)
	{
		this_led_functions = led_functions_collection[pmic_data->hardware];
		PWM_DBG("%s, we got led function hw type: %s", __func__, 
						this_led_functions.name);
	}
	else
	{
		PWM_ERR("%s, invalid hardware type", __func__);
		return -EIO;
	}

	/*Parse which action (fade/blinking/on)*/
	if (pmic_data->command >= SMEM_CMD_LED_ON_OFF && pmic_data->command <= SMEM_CMD_LED_FADE_IN_OUT)
	{
		if (pmic_data->control)
		{
			index = 2 * pmic_data->command;
		}
		else
		{
			index = 2 * pmic_data->command + 1;
		}
		PWM_DBG("%s, index = %d ", __func__, index);
	}

	//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+{
	#ifdef Config_LED_PM8921_FIH
	/* init LPG channel with PWM channel for LPG5~LPG7 */
		if (pmic_data->hardware == LED_HW_PMIC_LPG)
	{
		switch (pmic_data->lpg_out)
		{
			case PM_LPG_OUT5:
				led_drv_mapping_to_pwm_channel(pwm_dev, lpg_mapping_setting[LPG5_TO_PWM_CHANNEL][0], 
											 	lpg_mapping_setting[LPG5_TO_PWM_CHANNEL][1], 
											 	lpg_mapping_setting[LPG5_TO_PWM_CHANNEL][2]); 
				break;

			case PM_LPG_OUT6:
				led_drv_mapping_to_pwm_channel(pwm_dev, lpg_mapping_setting[LPG6_TO_PWM_CHANNEL][0], 
												lpg_mapping_setting[LPG6_TO_PWM_CHANNEL][1], 
												lpg_mapping_setting[LPG6_TO_PWM_CHANNEL][2]); 
				break;

			case PM_LPG_OUT7:
				led_drv_mapping_to_pwm_channel(pwm_dev, lpg_mapping_setting[LPG7_TO_PWM_CHANNEL][0], 
												lpg_mapping_setting[LPG7_TO_PWM_CHANNEL][1],
												lpg_mapping_setting[LPG7_TO_PWM_CHANNEL][2]); 
				break;

			default:
				break;
		}
	}
	#endif	

	#ifdef Config_LED_PM8038_FIH
	/* init LPG channel with PWM channel for LPG4~LPG6 */
	//PWM_IFO("%s, index = %d ", __func__, index);
	if(!index){
		if (pmic_data->hardware == LED_HW_PMIC_LPG)
		{
			//PWM_IFO("%s,LED_HW_PMIC_LPG",__func__);
		
			switch (pmic_data->lpg_out)
			{
				case PM_LPG_OUT4:
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_BLUE, SSBI_REG_ADDR_RGB_CNTL2, 0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_BLUE, SSBI_REG_ADDR_RGB_CNTL1, 0);
					//led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_BLUE, SSBI_REG_ADDR_RGB_TEST, pmic_data->current_sink);
					break;

				case PM_LPG_OUT5:
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_GREEN, SSBI_REG_ADDR_RGB_CNTL2, 0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_GREEN, SSBI_REG_ADDR_RGB_CNTL1, 0);
					//led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_GREEN, SSBI_REG_ADDR_RGB_TEST, pmic_data->current_sink);

					break;

				case PM_LPG_OUT6:
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_RED, SSBI_REG_ADDR_RGB_CNTL2, 0);
					led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_RED, SSBI_REG_ADDR_RGB_CNTL1, 0);
					//led_rgb_write(pwm_dev,PM8XXX_ID_RGB_LED_RED, SSBI_REG_ADDR_RGB_TEST, pmic_data->current_sink);

					break;

				default:
					break;
			}
		}
	}
	#endif
	
	//MTD-SW3-PERIPHERAL-BJ-LED_Porting-00+}		

	/*Check action index valid or not*/
	if (index >= LED_ACTION_MAX)
	{
		PWM_ERR("%s, out of action range, index = %d", __func__, index);
		return -EIO;
	}
	else
	{
		if (this_led_functions.led_actions[index] != NULL)
		{
			//call mapping function to control led
			ret = (*this_led_functions.led_actions[index])(pwm_dev, pmic_data);
		}
		else
		{
			PWM_ERR("the function is not support now!");
			ret = 0;
		}
	}

	return ret;	
}
EXPORT_SYMBOL_GPL(common_control_leds);

//MTD-SW3-PERIPHERAL-BJ-LED_FADE_IN_OUT_DIFF-00*{
int control_fade_nonsync_leds(struct pwm_device *pwm_dev_r, struct led_pmic_data *pmic_data_r, struct pwm_device *pwm_dev_g, struct led_pmic_data *pmic_data_g, struct pwm_device *pwm_dev_b, struct led_pmic_data *pmic_data_b)
{
	int ret = 0;
//	int index = 0;

	if (pwm_dev_r == NULL || pmic_data_r == NULL)
	{
		PWM_ERR("%s, NULL data type", __func__);
		return -EIO;
	}

	if (pwm_dev_g == NULL || pmic_data_g == NULL)
	{
		PWM_ERR("%s, NULL data type", __func__);
		return -EIO;
	}

	if (pwm_dev_b == NULL || pmic_data_b == NULL)
	{
		PWM_ERR("%s, NULL data type", __func__);
		return -EIO;
	}

	/*Parse which action (fade/blinking/on)*/
	if (pmic_data_r->control || pmic_data_g->control || pmic_data_b->control)
	{//on
		PWM_IFO("control_fade_nonsync_leds:ON");
		ret = pmic_gpio_led_fade_on_diff(pwm_dev_r, pmic_data_r,pwm_dev_g, pmic_data_g,pwm_dev_b, pmic_data_b);
		
		if (ret)
		{
			PWM_ERR("pmic_gpio_led_fade_on_diff error! ret = %d !", ret);
			return ret;
		}
	}
	else
	{//off
		PWM_IFO("control_fade_nonsync_leds:OFF");
		ret = pmic_gpio_led_fade_off(pwm_dev_r, pmic_data_r);
		if (ret)
		{
			PWM_ERR("disable r lut channel error! ret = %d !", ret);
			return ret;
		}

		ret = pmic_gpio_led_fade_off(pwm_dev_g, pmic_data_g);

		if (ret)
		{
			PWM_ERR("disable g lut channel error! ret = %d !", ret);
			return ret;
		}


		ret = pmic_gpio_led_fade_off(pwm_dev_b, pmic_data_b);
		if (ret)
		{
			PWM_ERR("disable b lut channel error! ret = %d !", ret);
			return ret;
		}

	}

	return ret;	
}
//MTD-SW3-PERIPHERAL-BJ-LED_FADE_IN_OUT_DIFF-00*}
EXPORT_SYMBOL_GPL(control_fade_nonsync_leds);

//PERI-BJ-Set_PWM_register_default-00+{
void
set_default_pwm_register(struct pwm_device *pwm_dev)
{
	//If device entering S1 boot and used led before entering OS , it will affect LED behavior(because some led register value be changed).
	//we need to set these register as QCT default value to make sure the behavior is consistent.

	led_rgb_set_bank_enable_lo_default(pwm_dev);
	led_rgb_set_ctl7_default(pwm_dev);
}

EXPORT_SYMBOL_GPL(set_default_pwm_register);
//PERI-BJ-Set_PWM_register_default-00+}


