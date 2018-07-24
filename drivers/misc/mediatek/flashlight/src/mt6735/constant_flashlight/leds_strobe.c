#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG printk
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);


static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO43 */
/* #define FLASH_GPIO_ENT GPIO1 */

#define DEV_NAME "strobe"

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);



#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
extern void flashlight_custom_set(fl_dev dev, fl_mode mode);
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end

int FL_Enable(void)
{
#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
    if(g_duty==0)
    {
        PK_DBG(" FL_Enable ,g_duty=%d,line=%d\n",g_duty,__LINE__);
        flashlight_custom_set(FL_DEV_MAIN, FL_MODE_TORCH);
    }
    else
    {
        PK_DBG(" FL_Enable ,g_duty=%d,line=%d\n",g_duty,__LINE__);
        flashlight_custom_set(FL_DEV_MAIN, FL_MODE_FLASH);
    }
    return 0;
#else
    return 0;
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end
}



int FL_Disable(void)
{
#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
    flashlight_custom_set(FL_DEV_MAIN, FL_MODE_OFF);
    return 0;
#else
    return 0;
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end

}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	g_duty = duty;
	return 0;
}




int FL_Init(void)
{
#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
    flashlight_custom_set(FL_DEV_MAIN, FL_MODE_OFF);
    return 0;
#else
    return 0;
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end

}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

//add by wuhao start++++
static struct i2c_client *KTD2685_i2c_client;

struct KTD2685_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct KTD2685_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct KTD2685_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

  int KTD2685_write_reg(u8 reg, u8 val)
{
	int ret = 0;
	struct KTD2685_chip_data *chip = i2c_get_clientdata(KTD2685_i2c_client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(KTD2685_i2c_client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

 int KTD2685_read_reg(u8 reg)
{
	int val = 0;
	struct KTD2685_chip_data *chip = i2c_get_clientdata(KTD2685_i2c_client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(KTD2685_i2c_client, reg);
	mutex_unlock(&chip->lock);


	return val;
}




static int KTD2685_chip_init(struct KTD2685_chip_data *chip)
{


	return 0;
}

static int KTD2685_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct KTD2685_chip_data *chip;
	struct KTD2685_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("KTD2685_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("KTD2685 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct KTD2685_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("KTD2685 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct KTD2685_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (KTD2685_chip_init(chip) < 0)
		goto err_chip_init;

	KTD2685_i2c_client = client;
	PK_DBG("KTD2685 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("KTD2685 probe is failed\n");
	return -ENODEV;
}

static int KTD2685_remove(struct i2c_client *client)
{
	struct KTD2685_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define KTD2685_NAME "leds-KTD2685"
static const struct i2c_device_id KTD2685_id[] = {
	{KTD2685_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id KTD2685_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver KTD2685_i2c_driver = {
	.driver = {
		   .name = KTD2685_NAME,
#ifdef CONFIG_OF
		   .of_match_table = KTD2685_of_match,
#endif
		   },
	.probe = KTD2685_probe,
	.remove = KTD2685_remove,
	.id_table = KTD2685_id,
};
static int __init KTD2685_init(void)
{
	PK_DBG("KTD2685_init\n");
	return i2c_add_driver(&KTD2685_i2c_driver);
}

static void __exit KTD2685_exit(void)
{
	i2c_del_driver(&KTD2685_i2c_driver);
}


module_init(KTD2685_init);
module_exit(KTD2685_exit);

MODULE_DESCRIPTION("Flash driver for KTD2685");
MODULE_AUTHOR("pw <hao.wu@wheatek.com>");
MODULE_LICENSE("GPL v2");


//add by wuhao end++++

















/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("WD3124 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}


EXPORT_SYMBOL(strobe_VDIrq);
