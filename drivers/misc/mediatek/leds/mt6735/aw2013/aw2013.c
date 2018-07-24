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
#include "aw2013.h"

#define MIN(A,B)	((A)<(B)?(A):(B))
#define MAX(A,B)	((A)>(B)?(A):(B))

#define AW2013_NAME                         "aw2013"
#define CONFIG_LED_CONSTANT_LIGHT



#define DATA_LEN                            (32)
#define I2C_TWO_MSG_ONCE
#define AW2013_I2C_ADDR                     (0x45)
#define BRIGHTNESS_LEVEL (0xff)

#define AW2013_DEBUG
#ifdef AW2013_DEBUG
#define AW2013_LOG(fmt, args...)			printk("[AW2013] " fmt, ##args)
#else
#define AW2013_LOG(fmt, args...)
#endif
#define AW2013_ERR(fmt, args...)			printk("[AW2013] " fmt, ##args)



static int aw2013_is_present  = 1;
static spinlock_t led_lock;
struct i2c_client *AW2013_led_client;
static const struct i2c_device_id leds_id[] = {{AW2013_NAME,0},{}};
static int led_power 	  	  = 1 ;


static int aw2013_i2c_master_send(struct i2c_client *client, u8 addr, char *buf ,int count);
static int aw2013_i2c_master_recv(struct i2c_client *client, u8 addr, char *buf ,int count);
static int aw2013_config_enable_all(u8 rgb);
static int aw2013_config_breath_mode(u8 led_index,u8 is_breath_mode);
static int aw2013_config_current_max(u8 led_index,u8 current_level);
static int aw2013_config_fade(u8 led_index,u8 fade_off_enable,u8 fade_on_enable);
static int aw2013_config_flash_mode_brightness(u8 led_index,u8 brightness);
static int aw2013_config_time(u8 led_index,u8 rise_time,u8 hold_time,u8 fall_time,u8 off_time,u8 delay_time,u8 repeat_time);
static int led_power_on(u8 type,u8 on);
static int led_process_effects(void);
static int aw2013_make_effect(u8 type);
/*********************************************************************************
** led control {
*********************************************************************************/
// order by priority 
typedef enum __LED_TYPE__	
{
	TYPE_MIN = 0,
	TYPE_CHARGE = TYPE_MIN,
	TYPE_CALL,
	TYPE_MSG,
	TYPE_NOTIFY,
	TYPE_STANDBY,
	TYPE_TOP= 9,
	TYPE_MAX 
}LED_TYPE;

typedef struct __LED_CFG__
{
	u8   enable; /* on-off */
	u8   breath; /* long time on or breath */
	u8   r;
	u8   r_ratio; /* current_level */
	u8   g;
	u8   g_ratio;
	u8   b;
	u8   b_ratio;
	
	u8   rise_time;
	u8   hold_time;
	u8   fall_time;
	u8   off_time;
	u8   delay_time;
	u8   period_num; /* breath times */
}LED_CFG;

static LED_CFG led_state[TYPE_MAX];

/*******************************************************************************
** led control }
*********************************************************************************/

/****************************************************************************************************
** discript:read i2c
** @client :[I]: client 
** @addr   :[I]: command 
** @buf    :[O]: recv data buffer pointer
** @count  :[I]: data size for read
*****************************************************************************************************/
static int aw2013_i2c_master_recv(struct i2c_client *client, u8 addr, char *buf ,int count)
{
	 int ret = 0, retry = 0;
	 struct i2c_adapter *adap = client->adapter;
	 struct i2c_msg msg[2]; // write then read 

	 int max_try = 3;

	 char data[DATA_LEN] = { 0 } ;
	 char data_buff[DATA_LEN] = { 0 } ;
	 int  data_len = 0 ;

	 if( aw2013_is_present == 0 ) return 0;

	//return i2c_master_recv(client,buf,count);
#if defined(I2C_TWO_MSG_ONCE)
	 // success
	 memset(data,0,sizeof(data));
	 data[0] 		  = addr ;
	 data_len = 1;

	 // send addr
	 //msg.addr = 0x8a >> 1;
	 msg[0].addr = client->addr;//(AW2013_I2C_ADDR) ;
	 msg[0].flags = 0;
	 msg[0].timing = 200;
	 msg[0].len = data_len;
	 msg[0].buf = (char *)data;
	 msg[0].ext_flag = client->ext_flag;

	 // read addr data
	 msg[1].addr = client->addr;//(AW2013_I2C_ADDR) ;
	 msg[1].flags = I2C_M_RD;
	 msg[1].timing = 200;
	 msg[1].len = data_len;
	 msg[1].buf = (char *)&data_buff[0];
	 msg[1].ext_flag = client->ext_flag;

	 retry = 0;
	 while(retry++ < max_try)
	 {
			ret = i2c_transfer(adap, msg, 2);
			printk("[aw2013_i2c_master_recv] debug i2c_transfer 2 msgs ret recv = %d",ret);
			if (ret == 2)
					break;
			udelay(100);
	 }
	 memcpy(buf,&data_buff[0],data_len); // move data

	 printk("[aw2013_i2c_master_recv] recv data = %x %x %x \n",data_buff[0],data_buff[1],data_buff[2]);
 
	 /* If everything went ok (i.e. 1 msg transmitted), return #bytes
	 transmitted, else error code. */
	 return (ret == 2) ? count : ret;
#elif defined(I2C_SEND_AND_RECV_TWO_MSG)
	 // send addr 
	 memset(data,0,sizeof(data));
	 data[0] 		  = addr ;
	 data_len = 1;

	 //msg.addr = 0x8a >> 1;
	 msg[0].addr = (AW2013_I2C_ADDR) >> 1;
	 msg[0].flags = 0;
	 msg[0].timing = 200;
	 msg[0].len = data_len;
	 msg[0].buf = (char *)data;
	 msg[0].ext_flag = client->ext_flag;

	 retry = 0;
	 while(retry++ < max_try)
	 {
			ret = i2c_transfer(adap, &msg[0], 1);
			//printk("<1> shaokai debug i2c_transfer ret recv 1= %d",ret);
			if (ret == 1)
					break;
			udelay(100);
	 }
	 
	 //udelay(100);
	 msleep(10);

	 //recv data
	 memset(data_buff,0,sizeof(data_buff));
	 data_len = (count > (DATA_LEN)) ? (DATA_LEN) : count ;

	 //msg.addr = 0x8a >> 1;
	 msg[1].addr = (AW2013_I2C_ADDR);
	 msg[1].flags = I2C_M_RD;
	 msg[1].timing = 200;
	 msg[1].len = data_len;
	 msg[1].buf = (char *)&data_buff[0];
	 msg[1].ext_flag = client->ext_flag;
	 
	 retry = 0;
	 while(retry++ < max_try)
	 {
		 ret = i2c_transfer(adap, &msg[1], 1);
		 //printk("<1> shaokai debug i2c_transfer ret recv 2= %d\n",ret);
		 if (ret == 1)
				 break;
		 udelay(100);
	 }
	 memcpy(buf,&data_buff[0],data_len); // move data

	 //printk("<1> recv data = %x %x %x \n",data_buff[0],data_buff[1],data_buff[2]);
 
	 /* If everything went ok (i.e. 1 msg transmitted), return #bytes
	 transmitted, else error code. */
	 return (ret == 1) ? count : ret;
#elif defined(I2C_SEND_WRITE_THEN_READ_ONE_MSG)
	 // write then read 
	 memset(data_buff,0,sizeof(data_buff));
	 data_buff[0]	  = addr ;
	 data_len = 1 ;//count > (DATA_LEN-1-1) ? (DATA_LEN-1-1) : count ;

	 //msg.addr = 0x8a >> 1;
	 msg[0].addr = (((AW2013_I2C_ADDR) >> 1) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
	 msg[0].flags = 0;
	 msg[0].timing = 200;
	 msg[0].len = (1<<8) | 1 ; // ((data_len << 8) | data_len)
	 msg[0].buf = (char *)data_buff;
	 msg[0].ext_flag = client->ext_flag;

	 retry = 0;
	 while(retry++ < max_try)
	 {
			ret = i2c_transfer(adap, &msg[0], 1);
			//printk("<1> shaokai debug i2c_transfer recv ret = %d\n",ret);
			if (ret == 1)
					break;
			udelay(100);
	 }
	 memcpy(buf,data_buff,data_len); // move data .

	 //printk("<1> recv data = %x %x %x \n",data_buff[0],data_buff[1],data_buff[2]);
 
	 /* If everything went ok (i.e. 1 msg transmitted), return #bytes
	 transmitted, else error code. */
	 return (ret == 1) ? count : ret;
#else
	return i2c_smbus_read_i2c_block_data(client,addr,count,buf);
#endif

}

/****************************************************************************************************
** discript:write i2c
** @client :[I]: client 
** @addr   :[I]: command 
** @buf    :[I]: send data buffer pointer
** @count  :[I]: data size for write
*****************************************************************************************************/
static int aw2013_i2c_master_send(struct i2c_client *client, u8 addr, char *buf ,int count)
{
#if 1
		//single addr and multi byte data.

		int ret = 0, retry = 0;
		struct i2c_adapter *adap = client->adapter;
		struct i2c_msg msg;

		char data_buff[DATA_LEN] = { 0 } ;
		int  data_len = 0 ;

		int max_try = 3;

		if( aw2013_is_present == 0 ) return 0;

		data_len = count > (DATA_LEN-1) ? (DATA_LEN-1) : count ;

		memset(data_buff,0,sizeof(data_buff));
		data_buff[0]	  = addr ;
		memcpy(&data_buff[1],buf,data_len);


		//msg.addr = 0x8a >> 1;
	 	msg.addr = (AW2013_I2C_ADDR);
		msg.flags = 0;
		msg.timing = 200;
		msg.len = data_len+1;
		msg.buf = (char *)data_buff;
		msg.ext_flag = client->ext_flag;

		while(retry++ < max_try)
		{
				ret = i2c_transfer(adap, &msg, 1);
				//printk("<1> shaokai debug i2c_transfer send ret = %d",ret);
				if (ret == 1)
						break;
				udelay(100);
		}

		/* If everything went ok (i.e. 1 msg transmitted), return #bytes
		transmitted, else error code. */
		return (ret == 1) ? count : ret;
#else		
		//return i2c_master_send(client,data_buff,(data_len+1));
		// multi addr and multi data
		return i2c_smbus_write_i2c_block_data(client,addr,count,buf);
#endif
}


static u8 aw2013_is_poweron = 0;  // init state : power off
/*********************************************************************************
** led function {
*********************************************************************************/
static int aw2013_config_power(u8 power_on)
{
	// ic power: 0x01
	//  R7 R6    /    R5      /   R4  R3   / R2   R1  R0
	//  X       /    HZ       /  X         /  current level
	u8 data_value = 0x00;

	if(aw2013_is_poweron == power_on)
	{
		//already power on/off
		return -1;
	}

#if 1
	// first reset 
	if(power_on > 0)
	{
		if(aw2013_is_poweron == 0)
		{
			data_value = 0x55;
			aw2013_i2c_master_send(AW2013_led_client, AW2013_RESET_REG, &data_value,1);
		}
	}
#endif

	if(power_on > 0)
	{
			data_value = 1;
			aw2013_is_poweron = 1 ;
	}
	else
	{
			data_value = 0;
			aw2013_is_poweron = 0 ;
	}

	//printk("<1> aw2013_config_power() write %d\n",data_value);
	aw2013_i2c_master_send(AW2013_led_client, 0x01, &data_value,1);

	// test read back data.
	//aw2013_i2c_master_recv(AW2013_led_client, 0x01, &data_value,1);
	//printk("<1> aw2013_config_power() read %d\n",data_value);

	return 0 ;
}
// i2c write and read
static int aw2013_config_enable_all(u8 rgb)
{
	// led enable: 0x30
	//  R7 R6    /    R5      /   R4  R3   / R2   R1  R0
	//  X       /    HZ       /  X         /  leds enable


	u8 data_value = 0x00;

	data_value = ( rgb & 0x07 );
	//data_value = 0x07;//open all led
	//data_value = 0x04;//b
	//data_value = 0x02;//g
	//data_value = 0x01;//r
	aw2013_i2c_master_send(AW2013_led_client, 0x30, &data_value,1);
	return 0;
}

static int led_process_effects(void)
{
	int index = TYPE_MIN ;
	int first_power_on = -1;
	while(index < TYPE_MAX)
	{
		if(led_state[index].enable > 0)
		{
			first_power_on = index;	
			break;
		}
		index ++ ;
	}

	if(led_power == 0)  //if logic power off already ,do nothing ......
		return -1;

	printk("shaokai:<1> led_process_effects first_power_on = %d\n",first_power_on);

	if(index == TYPE_MAX)
	{
#if 0
		aw2013_config_enable(0,0); // power off led0
		aw2013_config_enable(1,0); // power off led1
		aw2013_config_enable(2,0); // power off led2
#else
		aw2013_config_enable_all(0x00); //power off led0,led1,led2 at same time.
#endif
		aw2013_config_power(0);    // power off ic
	}
	else
	{
#if 1
		aw2013_config_power(0);    // power off ic first
		msleep(1);
#endif
		aw2013_config_power(1); // power on ic

		aw2013_make_effect(first_power_on);
	}
	return 0;
}

static int aw2013_config_breath_mode(u8 led_index,u8 is_breath_mode)
{
	// mode and current: 0x31 ~ 0x33
	//  R7      /  	R6       /    R5      /   R4        /  R3  R2 /  R1  R0
	//  X       / fade off  /  fade on   /   breath    /  XX     /  current level

	int ret = -1;
	u8 data_value = 0x00;

	switch (led_index)
	{
		case 0:
			// led0
			ret=aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED0_CFG_REG, &data_value,1);
			if(is_breath_mode == 1)
			{
				data_value |= (1 << 4);
			}
			else
			{
				data_value &= ~(1 << 4);
			}
			if(ret >= 0)
			{
				aw2013_i2c_master_send(AW2013_led_client, AW2013_LED0_CFG_REG, &data_value,1);
			}
			break;
		case 1:
			// led1
			ret = aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED1_CFG_REG, &data_value,1);
			if(is_breath_mode == 1)
			{
				data_value |= (1 << 4);
			}
			else
			{
				data_value &= ~(1 << 4);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(AW2013_led_client, AW2013_LED1_CFG_REG, &data_value,1);
			}
			break;
		case 2:
			// led2
			ret = aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED2_CFG_REG, &data_value,1);
			if(is_breath_mode == 1)
			{
				data_value |= (1 << 4);
			}
			else
			{
				data_value &= ~(1 << 4);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(AW2013_led_client, AW2013_LED2_CFG_REG, &data_value,1);
			}
			break;
		default:
			break;
	}
	return 0;
}

static int aw2013_config_current_max(u8 led_index,u8 current_level)
{
	// mode and current: 0x31 ~ 0x33
	//  R7      /  	R6       /    R5      /   R4        /  R3  R2 /  R1  R0
	//  X       / fade off  /  fade on   /   breath    /  XX     /  current level

	int ret = -1;
	u8 data_value = 0x00;

	switch (led_index)
	{
		case 0:
			// led0
			ret=aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED0_CFG_REG, &data_value,1);
			data_value &= ~0x03;
			data_value |= (current_level & 0x03);
			if(ret >= 0)
			{
				aw2013_i2c_master_send(AW2013_led_client, AW2013_LED0_CFG_REG, &data_value,1);
			}
			break;
		case 1:
			// led1
			ret = aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED1_CFG_REG, &data_value,1);
			data_value &= ~0x03;
			data_value |= (current_level & 0x03);
			if(ret >= 0)
			{
					aw2013_i2c_master_send(AW2013_led_client, AW2013_LED1_CFG_REG, &data_value,1);
			}
			break;
		case 2:
			// led2
			ret = aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED2_CFG_REG, &data_value,1);
			data_value &= ~0x03;
			data_value |= (current_level & 0x03);
			if(ret >= 0)
			{
					aw2013_i2c_master_send(AW2013_led_client, AW2013_LED2_CFG_REG, &data_value,1);
			}
			break;
		default:
			break;
	}
	return 0;
}

static int aw2013_config_fade(u8 led_index,u8 fade_off_enable,u8 fade_on_enable)
{
	// mode and current: 0x31 ~ 0x33
	//  R7      /  	R6       /    R5      /   R4        /  R3  R2 /  R1  R0
	//  X       / fade off  /  fade on   /   breath    /  XX     /  current level

	int ret = -1;
	u8 data_value = 0x00;

	switch (led_index)
	{
		case 0:
			// led0
			ret=aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED0_CFG_REG, &data_value,1);
			if(fade_off_enable)
			{
				data_value |= (1<<6);
			}
			else
			{
				data_value &= ~(1<<6);
			}
			if(fade_on_enable)
			{
				data_value |= (1<<5);
			}
			else
			{
				data_value &= ~(1<<5);
			}
			if(ret >= 0)
			{
				aw2013_i2c_master_send(AW2013_led_client, AW2013_LED0_CFG_REG, &data_value,1);
			}
			break;
		case 1:
			// led1
			ret = aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED1_CFG_REG, &data_value,1);
			if(fade_off_enable)
			{
				data_value |= (1<<6);
			}
			else
			{
				data_value &= ~(1<<6);
			}
			if(fade_on_enable)
			{
				data_value |= (1<<5);
			}
			else
			{
				data_value &= ~(1<<5);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(AW2013_led_client, AW2013_LED1_CFG_REG, &data_value,1);
			}
			break;
		case 2:
			// led2
			ret = aw2013_i2c_master_recv(AW2013_led_client, AW2013_LED2_CFG_REG, &data_value,1);
			if(fade_off_enable)
			{
				data_value |= (1<<6);
			}
			else
			{
				data_value &= ~(1<<6);
			}
			if(fade_on_enable)
			{
				data_value |= (1<<5);
			}
			else
			{
				data_value &= ~(1<<5);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(AW2013_led_client, AW2013_LED2_CFG_REG, &data_value,1);
			}
			break;
		default:
			break;
	}
	return 0;
}

static int aw2013_config_flash_mode_brightness(u8 led_index,u8 brightness)
{
	// brightness: 0x34 ~ 0x36
	// R7 R6 R5 R4 R3 R2 R1 R0
	//  0 ~ 255

	u8 data_value = 0x00;

	data_value = (brightness & 0xff);

	switch (led_index)
	{
		case 0:
			// led0
			aw2013_i2c_master_send(AW2013_led_client, AW2013_PWM0_REG, &data_value,1);
			break;
		case 1:
			// led1
			aw2013_i2c_master_send(AW2013_led_client, AW2013_PWM1_REG, &data_value,1);
			break;
		case 2:
			// led2
			aw2013_i2c_master_send(AW2013_led_client, AW2013_PWM2_REG, &data_value,1);
			break;
		default:
			break;
	}
	return 0;
}

static int aw2013_config_time(u8 led_index,u8 rise_time,u8 hold_time,u8 fall_time,u8 off_time,u8 delay_time,u8 repeat_time)
{
	// time contrl: 0x37 ~ 0x3F
	// R7 /  R6 R5 R4    / R3  / R2 R1 R0
	// X  /  rise time   / X   / hold_time
	// R7 /  R6 R5 R4    / R3  / R2 R1 R0
	// X  /  fall time   / X   / off time
	// R7 R6 R5 R4 / R3 R2 R1 R0
	// delay time  / repeat times (0 -> loop)

	u8 data_value = 0x00;
	u8 hold_time_value = ((hold_time & 0x07)>5) ? 5 : (hold_time & 0x07);  	  // level 6
	u8 delay_time_value = ((delay_time & 0x0f)>8) ? 8 : (delay_time & 0x0f);  // level 9

	switch (led_index)
	{
		case 0:
			// led0
			data_value = ((rise_time & 0x07)<<4) | hold_time_value;
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED0_TIME_REG0, &data_value,1);
			data_value = ((fall_time & 0x07)<<4) | (off_time & 0x07);
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED0_TIME_REG1, &data_value,1);
			data_value = ((delay_time_value)<<4) | (repeat_time & 0x0f);
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED0_TIME_REG2, &data_value,1);
			break;
		case 1:
			// led1
			data_value = ((rise_time & 0x07)<<4) | hold_time_value;
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED1_TIME_REG0, &data_value,1);
			data_value = ((fall_time & 0x07)<<4) | (off_time & 0x07);
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED1_TIME_REG1, &data_value,1);
			data_value = ((delay_time_value)<<4) | (repeat_time & 0x0f);
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED1_TIME_REG2, &data_value,1);
			break;
		case 2:
			// led2
			data_value = ((rise_time & 0x07)<<4) | hold_time_value;
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED2_TIME_REG0, &data_value,1);
			data_value = ((fall_time & 0x07)<<4) | (off_time & 0x07);	
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED2_TIME_REG1, &data_value,1);
			data_value = ((delay_time_value)<<4) | (repeat_time & 0x0f);
			aw2013_i2c_master_send(AW2013_led_client, AW2013_LED2_TIME_REG2, &data_value,1);
			break;
		default:
			break;
	}
	return 0;
}
/*********************************************************************************
** led function }
*********************************************************************************/

/*******************************************************************************
** ic control {
*******************************************************************************/
static int aw2013_make_effect(u8 type)
{
	u8 rgb =0;
	if(led_state[type].enable > 0)
	{
		printk("<1> aw2013_make_effect begin \n");

		if(led_state[type].breath > 0)
		{
			aw2013_config_breath_mode(0,1); 	//  led0 breath mode
			aw2013_config_breath_mode(1,1); 	//  led1 breath mode
			aw2013_config_breath_mode(2,1); 	//  led2 breath mode
		}
		else
		{
			aw2013_config_breath_mode(0,0); 	//  led0 normal mode
			aw2013_config_breath_mode(1,0); 	//  led1 normal mode
			aw2013_config_breath_mode(2,0); 	//  led2 normal mode
		}

		aw2013_config_fade(0,1,1); 			// fade off -> 1 , fade_on -> 1
		aw2013_config_fade(1,1,1); 			// fade off -> 1 , fade_on -> 1
		aw2013_config_fade(2,1,1); 			// fade off -> 1 , fade_on -> 1

		// led0
		//if(led_state[type].r_ratio > 0)
		{
			aw2013_config_current_max(0,led_state[type].r_ratio); 		//  0ma, 5ma,10ma,15ma
			aw2013_config_flash_mode_brightness(0,BRIGHTNESS_LEVEL);    //  full 
			aw2013_config_time(0,led_state[type].rise_time,led_state[type].hold_time,led_state[type].fall_time,led_state[type].off_time,led_state[type].delay_time,led_state[type].period_num);  // /-\_   led1
		}

		// led1
		//if(led_state[type].g_ratio > 0)
		{
			aw2013_config_current_max(1,led_state[type].g_ratio); 		//  0ma, 5ma,10ma,15ma
			aw2013_config_flash_mode_brightness(1,BRIGHTNESS_LEVEL);    //  full 
			aw2013_config_time(1,led_state[type].rise_time,led_state[type].hold_time,led_state[type].fall_time,led_state[type].off_time,led_state[type].delay_time,led_state[type].period_num);  // /-\_   led1
		}

		// led2
		//if(led_state[type].b_ratio > 0)
		{
			aw2013_config_current_max(2,led_state[type].b_ratio); 		//  0ma, 5ma,10ma,15ma
			aw2013_config_flash_mode_brightness(2,BRIGHTNESS_LEVEL);    //  full 
			aw2013_config_time(2,led_state[type].rise_time,led_state[type].hold_time,led_state[type].fall_time,led_state[type].off_time,led_state[type].delay_time,led_state[type].period_num);  // /-\_   led1
		}

#if 0
		// led0
		if(led_state[type].r > 0)		
		{
			aw2013_config_enable(0,1); 			//  enable
		}
		else
		{
			aw2013_config_enable(0,0); 			//  disable
		}

		// led1
		if(led_state[type].g > 0)		
		{
			aw2013_config_enable(1,1); 			//  enable
		}
		else
		{
			aw2013_config_enable(1,0); 			//  disable
		}

		// led2
		if(led_state[type].b > 0 )	
		{
			aw2013_config_enable(2,1); 			//  enable
		}
		else
		{
			aw2013_config_enable(2,0); 			//  disable
		}
#else
		rgb = (((led_state[type].b)<<2) | ((led_state[type].g)<<1) | (led_state[type].r));
		aw2013_config_enable_all(rgb); 			//  enable
#endif

		printk("<1> aw2013_make_effect end \n");
	}
	return 0 ; 
}


/*******************************************************************************
** ic control }
*******************************************************************************/


static int led_power_on(u8 type,u8 on)
{
	if( (type < TYPE_MIN) || (type >= TYPE_MAX))
	{
		printk("<1> led power on off error\n");
		return -1;
	}

	printk("<1> led power %s success.\n",(on > 0 )?"on":"off");

	spin_lock(&led_lock);
	//down(&semaphore_led_mutex);

	led_state[type].enable = on;

	//up(&semaphore_led_mutex);
	spin_unlock(&led_lock);


	led_process_effects();

	return 0;
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static void aw2013_red_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	if((type >= TYPE_MIN) && (type < TYPE_MAX))
	{
		spin_lock(&led_lock);
		led_state[type].breath = (breath>0)?1:0;
		led_state[type].r = 1;
		led_state[type].r_ratio = MAX(MIN(ratio,3),0);
		led_state[type].g = 0;
		led_state[type].g_ratio = 0;
		led_state[type].b = 0;
		led_state[type].b_ratio = 0;

		led_state[type].rise_time = AW2013_LED_TIME_0_52S; 
		led_state[type].hold_time = AW2013_LED_TIME_1_04S;
		led_state[type].fall_time = AW2013_LED_TIME_0_52S;
		led_state[type].off_time = AW2013_LED_TIME_1_04S;
		led_state[type].delay_time = AW2013_LED_DELAY_TIME_0S;
		led_state[type].period_num = AW2013_LED_REPEAT_ALWAYS; /* always */
		spin_unlock(&led_lock);
		
		enable = (enable > 0)? 1:0;
		led_power_on(type,enable); // power on / off led .

	}
}
static void aw2013_green_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	if((type >= TYPE_MIN) && (type < TYPE_MAX))
	{
		spin_lock(&led_lock);
		led_state[type].breath = (breath>0)?1:0;
		led_state[type].g = 1;
		led_state[type].g_ratio = MAX(MIN(ratio,3),0);
		led_state[type].r = 0;
		led_state[type].r_ratio = 0;
		led_state[type].b = 0;
		led_state[type].b_ratio = 0;

		led_state[type].rise_time = AW2013_LED_TIME_0_52S; 
		led_state[type].hold_time = AW2013_LED_TIME_1_04S;
		led_state[type].fall_time = AW2013_LED_TIME_0_52S;
		led_state[type].off_time = AW2013_LED_TIME_1_04S;
		led_state[type].delay_time = AW2013_LED_DELAY_TIME_0S;
		led_state[type].period_num = AW2013_LED_REPEAT_ALWAYS; /* always */
		spin_unlock(&led_lock);
		
		enable = (enable > 0)? 1:0;
		led_power_on(type,enable); // power on / off led .

	}
}

static void aw2013_blue_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	if((type >= TYPE_MIN) && (type < TYPE_MAX))
	{
		spin_lock(&led_lock);

		led_state[type].breath = (breath>0)?1:0;
		led_state[type].b = 1;
		led_state[type].b_ratio = MAX(MIN(ratio,3),0);
		led_state[type].g = 0;
		led_state[type].g_ratio = 0;
		led_state[type].r = 0;
		led_state[type].r_ratio = 0;

		led_state[type].rise_time = AW2013_LED_TIME_0_52S; 
		led_state[type].hold_time = AW2013_LED_TIME_1_04S;
		led_state[type].fall_time = AW2013_LED_TIME_0_52S;
		led_state[type].off_time = AW2013_LED_TIME_1_04S;
		led_state[type].delay_time = AW2013_LED_DELAY_TIME_0S;
		led_state[type].period_num = AW2013_LED_REPEAT_ALWAYS; /* always */

		spin_unlock(&led_lock);
		
		enable = (enable > 0)? 1:0;
		led_power_on(type,enable); // power on / off led .

	}
}

int red_onoff(int level)
{
	AW2013_LOG("red level =%d\n",level);
	if(level > 0)
	{
		aw2013_blue_enable(TYPE_MIN,1,1,1);
	}
	else
	{
		aw2013_blue_enable(TYPE_MIN,0,0,0);
	}
	return 0;
}

int green_onoff(int level)
{
	AW2013_LOG("green level =%d\n",level);
	if(level > 0)
	{
		aw2013_green_enable(TYPE_MIN,1,1,1);
	}
	else
	{
		aw2013_green_enable(TYPE_MIN,0,0,0);
	}
	return 0;
}

int blue_onoff(int level)
{
	AW2013_LOG("blue level =%d\n",level);
	if(level > 0)
	{
		aw2013_red_enable(TYPE_MIN,1,1,1);
	}
	else
	{
		aw2013_red_enable(TYPE_MIN,0,0,0);
	}
	return 0;
}

static int aw2013_init_client(struct i2c_client * client)
{

	int ret = -1;
	u8 databuf[3]= {0,0,0};	
	
	databuf[0] = 0x30;   

	ret = i2c_master_send(client, databuf, 0x1);
	if(ret <= 0)
	{
		printk("error\n");
	}

	udelay(500);

	databuf[0] = 0x0;        
	ret = i2c_master_recv(client, databuf, 0x01);
	if(ret <= 0)
	{
		printk("error\n");
	}

	if( ret == 1)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

static int aw2013_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int aw2013_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	AW2013_LOG("i2c_Probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	AW2013_led_client = client;

	err = aw2013_init_client(AW2013_led_client);
	if(err < 0)
	{
		return err;
	}
	aw2013_red_enable(TYPE_MIN,0,0,0);
	aw2013_is_present = 1;
	AW2013_LOG("i2c probe end\n");
	
	
	return 0;


exit_check_functionality_failed:
	return err;
}


struct of_device_id aw2013_of_match[] = {
	{ .compatible = "mediatek,LEDS", },
	{},
};

static struct i2c_driver aw2013_i2c_driver = 
{  
	.probe 	 	= aw2013_i2c_probe,
	.remove	 	= aw2013_i2c_remove,
	.suspend	= NULL,//aw2013_i2c_suspend,
	.resume	 	= NULL,//aw2013_i2c_resume,
	.id_table	= leds_id,
	.driver 	= 
	{
		.owner 		 = THIS_MODULE,
		.name			 = AW2013_NAME,
		#ifdef CONFIG_OF
		.of_match_table = aw2013_of_match,
		#endif
	},
};


static int aw2013_leds_remove(struct platform_device *pdev)
{
		i2c_del_driver(&aw2013_i2c_driver);
		return 0;
}

static void aw2013_leds_shutdown(struct platform_device *dev)
{
    //GPIOLOG("shutdown\n");
	//printk("<1> shaokai debug aw2013_leds_shutdown() \n");

	return ;
}

static int aw2013_leds_probe(struct platform_device *pdev) 
{

		if(i2c_add_driver(&aw2013_i2c_driver))
		{
			printk("[aw2013_leds_probe] add driver error\n");
			return -1;
		}
		printk("[zhaoshoufeng]aw2013_leds_probe end\n");
		return 0;
}

struct platform_device aw2013_leds_device = 
{
		.name	 = AW2013_NAME,
		//.id	 = -1,
};

static struct platform_driver aw2013_leds_driver =
{
	.driver	 = 
	{
		.name	 = AW2013_NAME,
		.owner	 = THIS_MODULE,
	},
	.probe 	 = aw2013_leds_probe,
	.remove	 = aw2013_leds_remove,
	.shutdown = aw2013_leds_shutdown
};

/* called when loaded into kernel */
static int __init aw2013_leds_init(void)
{
		int retval = -1;

		AW2013_LOG("driver init start\n");
		spin_lock_init(&led_lock);

		//i2c_register_board_info(2, &i2c_leds, 1); 

		retval = platform_device_register(&aw2013_leds_device);
		if(retval < 0)
		{
			return -2;
		}

		if(platform_driver_register(&aw2013_leds_driver) < 0)
		{
			 printk("platform register aw2013 driver failed\n");
			 return -1;
		}

		AW2013_LOG("aw2013 driver init end.\n");

		return 0;
}
 
/* should never be called */
static void __exit aw2013_leds_exit(void)
{
	platform_driver_unregister(&aw2013_leds_driver);
	platform_device_unregister(&aw2013_leds_device);
}

ssize_t external_ic_leds_mode(int value)
{     
	if(value == 4)//red	
	{
		aw2013_blue_enable(TYPE_MIN,1,1,AW2013_BLUE_LED_BREATH_CURRENT);
	}
	else if(value == 5)	
	{
		aw2013_green_enable(TYPE_MIN,1,1,AW2013_GREEN_LED_BREATH_CURRENT);
	}
	else if(value == 6)//blue
	{
		aw2013_red_enable(TYPE_MIN,1,1,AW2013_RED_LED_BREATH_CURRENT);
	}
	else if(value == 1)	
	{
		aw2013_blue_enable(TYPE_MIN,1,0,AW2013_BLUE_LED_CURRENT);
	}
	else if(value == 2)
	{
		aw2013_green_enable(TYPE_MIN,1,0,AW2013_GREEN_LED_CURRENT);
	}
	else if(value == 3)
	{
		aw2013_red_enable(TYPE_MIN,1,0,AW2013_RED_LED_CURRENT);
	}
	else
	{
		aw2013_red_enable(TYPE_MIN,0,0,0);
	}

	return 0;
}

module_init(aw2013_leds_init);
module_exit(aw2013_leds_exit);

/*----------------------------------------------------------------------------*/

MODULE_AUTHOR("shaokai@moztek.net");
MODULE_DESCRIPTION("motek aw2013 respiration lamp ic driver");
MODULE_LICENSE("GPL");
