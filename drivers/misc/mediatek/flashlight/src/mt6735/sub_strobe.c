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

#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif


#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
extern void flashlight_custom_set(fl_dev dev, fl_mode mode);
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 sub_strobe_Res;

static int g_duty = -1;
static int g_timeOutTimeMs;


static struct work_struct workTimeOut;

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);


static int FL_Enable(void)
{
    printk(" FL_Enable ,line=%d\n",__LINE__);
#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
    flashlight_custom_set(FL_DEV_SUB, FL_MODE_FLASH);
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end	
    return 0;
}



static int FL_Disable(void)
{
    printk(" FL_Disable line=%d\n", __LINE__);
#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
    flashlight_custom_set(FL_DEV_SUB, FL_MODE_OFF);
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	printk(" FL_dim_duty line=%d\n", __LINE__);
	g_duty = duty;
	return 0;
}




static int FL_Init(void)
{
    printk(" FL_Init line=%d\n", __LINE__);
#if 1//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]start
    flashlight_custom_set(FL_DEV_SUB, FL_MODE_OFF);
#endif//lisong[2015-12-10][BUGID:NULL][FLASHLIGHT CUSTOM SUPPORT]end
    return 0;
}


static int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	printk("ledTimeOut_callback\n");
}



enum hrtimer_restart TimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = TimeOutCallback;
}



static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));

	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		printk("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		printk("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		printk("FLASHLIGHT_ONOFF: %d\n", (int)arg);
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
		printk(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int sub_strobe_open(void *pArg)
{
	int i4RetValue = 0;

	printk("sub_strobe_open line=%d\n", __LINE__);

	if (0 == sub_strobe_Res) {
		FL_Init();
		timerInit();
	}
	printk("sub_strobe_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (sub_strobe_Res) {
		printk(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		sub_strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	printk("sub_strobe_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int sub_strobe_release(void *pArg)
{
	printk(" sub_strobe_release\n");

	if (sub_strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		sub_strobe_Res = 0;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	printk(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}

