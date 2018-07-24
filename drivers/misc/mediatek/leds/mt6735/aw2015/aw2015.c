#include <linux/time.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <asm/irq.h>

#include <linux/slab.h>

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <linux/miscdevice.h>

//for dym create device
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/device.h>
#include <asm/uaccess.h>

//kthread
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/spinlock.h>


#define AW2015_DEBUG
#ifdef AW2015_DEBUG
#define AW2015_LOG(fmt, args...)			printk("[AW2015] " fmt, ##args)
#else
#define AW2015_LOG(fmt, args...)
#endif
#define AW2015_ERR(fmt, args...)			printk("[AW2015] " fmt, ##args)

#define AW2015_NAME     	"aw2015"
#define CONFIG_LED_CONSTANT_LIGHT

struct i2c_client *AW2015_led_client;
static const struct i2c_device_id leds_id[] = {{AW2015_NAME,0},{}};

// i2c write and read
//////////////////////////////////////////////////////
unsigned char I2C_write_reg(unsigned char addr, unsigned char reg_data)
{
	char ret = 0;
	u8 wdbuf[512];
	struct i2c_msg msg[2];
	wdbuf[0] = addr;
	wdbuf[1] = reg_data;
	
	msg[0].addr = AW2015_led_client->addr;
	msg[0].flags = 0;
	msg[0].timing = 200;
	msg[0].len = 2;
	msg[0].buf = wdbuf;
	msg[0].ext_flag = AW2015_led_client->ext_flag;

	ret = i2c_transfer(AW2015_led_client->adapter, msg, 1);
	if (ret < 0)
		AW2015_ERR("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

unsigned char I2C_read_reg(unsigned char addr)
{
	unsigned char ret = 0;
	u8 rdbuf[512];
	struct i2c_msg msg[2];
	
	rdbuf[0] = addr;

	msg[0].addr = AW2015_led_client->addr;//(AW2015_I2C_ADDR) ;
	msg[0].flags = 0;
	msg[0].timing = 200;
	msg[0].len = 1;
	msg[0].buf = rdbuf;
	msg[0].ext_flag = AW2015_led_client->ext_flag;
	
	// read addr data
	msg[1].addr = AW2015_led_client->addr;//(AW2015_I2C_ADDR) ;
	msg[1].flags = I2C_M_RD;
	msg[1].timing = 200;
	msg[1].len = 1;
	msg[1].buf = rdbuf;
	msg[1].ext_flag = AW2015_led_client->ext_flag;
	
	ret = i2c_transfer(AW2015_led_client->adapter, msg, 2);
	if (ret < 0)
		AW2015_ERR("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}



unsigned char ms2timer(unsigned int time)
{
	unsigned char i = 0;
	unsigned int ref[16] = {4, 128, 256, 384, 512, 762, 1024, 1524, 2048, 2560, 3072, 4096, 5120, 6144, 7168, 8192};
	
	for(i=0; i<15; i++)
	{
		if(time <= ref[0])
		{
			return 0;
		}
		else if(time > ref[15])
		{
			return 15;
		}
		else if((time>ref[i]) && (time<=ref[i+1]))
		{
			if((time-ref[i]) <= (ref[i+1]-time))
			{
				return i;
			}
			else
			{
				return (i+1);
			}
		}
	}
	return 0;
}	


unsigned char AW2015_LED_ON(unsigned char r, unsigned char g, unsigned char b)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x03);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	
	I2C_write_reg(0x10, r	);		// ILED1
	I2C_write_reg(0x11, g	);		// ILED2
	I2C_write_reg(0x12, b	);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	return 0;
}

unsigned char AW2015_LED_OFF(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset
	I2C_write_reg(0x07, 0x00);		// LEDEN
	return 0;
}
unsigned char AW2015_LED_Blink(unsigned char r, unsigned char g, unsigned char b, unsigned int trise_ms, unsigned int ton_ms, unsigned int tfall_ms, unsigned int toff_ms)
{
	unsigned char trise, ton, tfall, toff;
	
	trise = ms2timer(trise_ms);
	ton   = ms2timer(ton_ms);
	tfall = ms2timer(tfall_ms);
	toff  = ms2timer(toff_ms);
	
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x03);		// IMAX
	I2C_write_reg(0x04, 0x01);		// LCFG1
	I2C_write_reg(0x05, 0x01);		// LCFG2
	I2C_write_reg(0x06, 0x01);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x08);		// LEDCTR
	
	I2C_write_reg(0x10, r	);		// ILED1
	I2C_write_reg(0x11, g	);		// ILED2
	I2C_write_reg(0x12, b	);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	I2C_write_reg(0x30, (trise<<4)|ton);	// PAT_T1		Trise & Ton
	I2C_write_reg(0x31, (tfall<<4)|toff);	// PAT_T2		Tfall & Toff
	I2C_write_reg(0x32, 0x00);		// PAT_T3				Tdelay
	I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
	I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer
	
	I2C_write_reg(0x09, 0x07);		// PAT_RIN	
	return 0;
}



int red_onoff(int level)
{
	AW2015_LOG("red level =%d\n",level);
	if(level > 0)
	{
		//aw2015_blue_enable(TYPE_MIN,1,1,1);
		#ifndef CONFIG_LED_CONSTANT_LIGHT
		AW2015_LED_Blink(1, 0, 0, 20, 20, 20, 20);
		#else
		AW2015_LED_ON(1,0,0);
		#endif
	}
	else
	{
		AW2015_LED_OFF();
	}
	return 0;
}

int green_onoff(int level)
{
	AW2015_LOG("green level =%d\n",level);
	if(level > 0)
	{
		#ifndef CONFIG_LED_CONSTANT_LIGHT
		AW2015_LED_Blink(0, 1, 0, 20, 20, 20, 20);
		#else
		AW2015_LED_ON(0,1,0);
		#endif
	}
	else
	{
		AW2015_LED_OFF();
	}
	return 0;
}

int blue_onoff(int level)
{
	AW2015_LOG("blue level =%d\n",level);
	if(level > 0)
	{
		#ifndef CONFIG_LED_CONSTANT_LIGHT
		AW2015_LED_Blink(0, 0, 1, 20, 20, 20, 20);
		#else
		AW2015_LED_ON(0,0,1);
		#endif
	}
	else
	{
		AW2015_LED_OFF();
	}
	return 0;
}
/*********************************************/
ssize_t external_ic_leds_mode(int value)
{     
	if(value == 1)//red	
	{
		AW2015_LED_ON(1,0,0);
	}
	else if(value == 2)	
	{
		AW2015_LED_ON(0,1,0);
	}
	else if(value == 3)//blue
	{
		AW2015_LED_ON(0,0,1);
	}
	else if(value == 4)	
	{
		AW2015_LED_Blink(1, 0, 0, 1000, 1000, 1000, 1000);
	}
	else if(value == 5)
	{
		AW2015_LED_Blink(0, 1, 0, 1000, 1000, 1000, 1000);
	}
	else if(value == 6)
	{
		AW2015_LED_Blink(0, 0, 1, 1000, 1000, 1000, 1000);
	}
	else
	{
		AW2015_LED_OFF();
	}

	return 0;
}

static int aw2015_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int aw2015_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char reg_value;
	int err = 0;
	AW2015_LOG("i2c_Probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	AW2015_led_client = client;

	reg_value = I2C_read_reg(0x00);
	printk("AW2015 CHIPID=0x%2x\n", reg_value);
	/*if(reg_value != 0x31)
	{
		err = -ENODEV;
		goto exit_create_singlethread;
	}*/
	AW2015_LED_OFF();
	AW2015_LOG("i2c probe end\n");
	//AW2015_create_sysfs(client);	
	
	return 0;

//exit_create_singlethread:
//	AW2015_led_client = NULL;
exit_check_functionality_failed:
	return err;
}


struct of_device_id aw2015_of_match[] = {
	{ .compatible = "mediatek,LEDS", },
	{},
};

static struct i2c_driver aw2015_i2c_driver = 
{  
	.probe 	 	= aw2015_i2c_probe,
	.remove	 	= aw2015_i2c_remove,
	.suspend	= NULL,//aw2015_i2c_suspend,
	.resume	 	= NULL,//aw2015_i2c_resume,
	.id_table	= leds_id,
	.driver 	= 
	{
		.owner 		 = THIS_MODULE,
		.name			 = AW2015_NAME,
		#ifdef CONFIG_OF
		.of_match_table = aw2015_of_match,
		#endif
	},
};


static int aw2015_leds_remove(struct platform_device *pdev)
{
		i2c_del_driver(&aw2015_i2c_driver);
		return 0;
}

static void aw2015_leds_shutdown(struct platform_device *dev)
{
    //GPIOLOG("shutdown\n");
	//printk("<1> shaokai debug aw2015_leds_shutdown() \n");

	return ;
}

static int aw2015_leds_probe(struct platform_device *pdev) 
{

		if(i2c_add_driver(&aw2015_i2c_driver))
		{
			printk("[aw2015_leds_probe] add driver error\n");
			return -1;
		}
		printk("[zhaoshoufeng]aw2015_leds_probe end\n");
		return 0;
}

struct platform_device aw2015_leds_device = 
{
		.name	 = AW2015_NAME,
		//.id	 = -1,
};

static struct platform_driver aw2015_leds_driver =
{
	.driver	 = 
	{
		.name	 = AW2015_NAME,
		.owner	 = THIS_MODULE,
	},
	.probe 	 = aw2015_leds_probe,
	.remove	 = aw2015_leds_remove,
	.shutdown = aw2015_leds_shutdown
};

/* called when loaded into kernel */
static int __init aw2015_leds_init(void)
{
		int retval = -1;

		AW2015_LOG("driver init start\n");

		//i2c_register_board_info(2, &i2c_leds, 1); 

		retval = platform_device_register(&aw2015_leds_device);
		if(retval < 0)
		{
			return -2;
		}

		if(platform_driver_register(&aw2015_leds_driver) < 0)
		{
			 printk("platform register aw2015 driver failed\n");
			 return -1;
		}

		AW2015_LOG("aw2015 driver init end.\n");

		return 0;
}
 
/* should never be called */
static void __exit aw2015_leds_exit(void)
{
	platform_driver_unregister(&aw2015_leds_driver);
	platform_device_unregister(&aw2015_leds_device);
}

module_init(aw2015_leds_init);
module_exit(aw2015_leds_exit);

/*----------------------------------------------------------------------------*/

MODULE_AUTHOR("shaokai@moztek.net");
MODULE_DESCRIPTION("motek aw2015 respiration lamp ic driver");
MODULE_LICENSE("GPL");
