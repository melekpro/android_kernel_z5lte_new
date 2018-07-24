#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/wakelock.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#include "../../../../spi/mediatek/mt6735/mt_spi.h"
#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

//#include "../fp_drv/fp_drv.h"

#include "elan_fp.h"
#include <linux/completion.h>

//#define reinit_completion(x) INIT_COMPLETION(*(x))
#define VERSION_LOG	"ELAN FINGER PRINT V1.4.4.1"

#define _ELAN_DEBUG_
#ifdef _ELAN_DEBUG_
	static int elan_debug = 1;
	#define ELAN_DEBUG(format, args ...) \
			do { \
					if (elan_debug) \
							printk("[ELAN] " format, ##args); \
			} while (0)		 
#else
         #define ELAN_DEBUG(format, args ...)
#endif

#define ELAN_FINGERPRINT_SPI 	"Elan_finger_spi"


#define GPIO_EFSA120_RST	GPIO_KPD_KROW1_PIN // change by customer

#define SPI_MAX_SPEED		3*1000*1000 // chagne by customer
#define KEY_FP_INT			KEY_POWER //KEY_WAKEUP // change by customer & framework support

static int read_all = -1; // added v1.441 : -1 = read on raw, 0 = read all, else = read "read_all" raw.
#define ELAN_CALI_TIMEOUT_MSEC	1000
struct completion cmd_done;
struct completion cmd_done_irq;

static int key_status = 0;
static struct fasync_struct *fasync_queue = NULL;
static int image_ok_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(image_waiter);
static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);
static int elan_work_flag = 0;

//#define CONFIG_PM_SLEEP
#define SPI_ALINE_BYTE 4

#define WRITE_REG_HEAD			0x80
#define READ_REG_HEAD			0x40
#define READ_SERIER_REG_HEAD	0xC0
#define INT_NORMAL_HIGH			0x40
#define START_SCAN				0x01
#define START_READ_IMAGE		0x10
#define ADDR_SIZE_WH			0x01


struct efsa120s_data  {
	struct spi_device	*spi;
	struct input_dev	*input_dev;
	/* File I/O for user-space */
	int					intr_gpio;
	int					rst_gpio;
	struct work_struct	work;	
	bool				irq_status; // 1:enable, 0:disable
	wait_queue_head_t	efsa_wait; 
	struct wake_lock	wake_lock;
	struct miscdevice	efsa120_dev; /* char device for ioctl */

	int irq;
	struct regulator *reg;
	struct pinctrl *pinctrl1;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *eint_as_int, *eint_in_low, *eint_in_float, *fp_rst_low, *fp_rst_high,*miso_pull_up,*miso_pull_disable;
};

static unsigned int IMG_WIDTH = 120;
static unsigned int IMG_HEIGHT = 120;
static unsigned int IMG_WIDTH_DEFAULT = 120;
static unsigned int IMG_HEIGHT_DEFAULT = 120;
static unsigned int IMG_SIZE = 14400;

static unsigned char * imagebuffer = NULL; // for SPI Transfer and orginal data
static unsigned char * imagebuffer_TX = NULL; // for SPI Transfer and orginal data
static unsigned char * imagebuffer_RX = NULL; // for SPI Transfer and orginal data

static unsigned char bCMD_REG = 0; // CMD = 0, REG= 1
static int Image_index = 0;

static unsigned char IOIRQ_STATUS = 0;
static unsigned int spi_speed = SPI_MAX_SPEED; // 10MHz
static unsigned short spi_cs_delay = 5; // 5us


static struct workqueue_struct *efsa120s_wq;
//static unsigned char bProbeCheck[20] = {0};	//Because MTK kernel message too much, bProbeCheck is error flag

static struct efsa120s_data *elan_fp = NULL;


static int efsa120s_spi_transfer(struct spi_device *spi, const char *txbuf, char *rxbuf, int len)
{
	struct spi_transfer t;
	struct spi_message m;
	
	memset(&t, 0, sizeof(t));
	spi_message_init(&m);
	t.tx_buf = txbuf;
	t.rx_buf = rxbuf;
	t.bits_per_word = 8;
	t.len = len;
	spi_message_add_tail(&t, &m);
	
	return spi_sync(spi, &m);
}

static int efsa120s_kmalloc_image(void)
{
	int alloc_len = 0;
	
	IMG_SIZE = (IMG_WIDTH * IMG_HEIGHT);

	//(line len + dummy(1))*line counts + cmd(1) + 4Byte alignment(MTK DMA mode)
	alloc_len = ((IMG_WIDTH*2+1)*IMG_HEIGHT + 1 + (1024-1))/ 1024 * 1024;
	imagebuffer = kmalloc(sizeof(unsigned char) * alloc_len, GFP_KERNEL); // add 2 is cmd & dummy
	if (imagebuffer == NULL)
	{
		ELAN_DEBUG("imagebuffer kmalloc error\n");
		kfree(imagebuffer);
		return -ENOMEM;
	}

	//max len + cmd(1) + dummy(1) + 4Byte alignment(MTK DMA mode)
	alloc_len = (IMG_WIDTH*2 + 2 + (SPI_ALINE_BYTE-1))/ SPI_ALINE_BYTE * SPI_ALINE_BYTE;
	imagebuffer_TX = kmalloc(sizeof(unsigned char) * alloc_len, GFP_KERNEL); // add 2 is cmd & dummy
	if (imagebuffer_TX == NULL)
	{
		ELAN_DEBUG("imagebuffer_TX kmalloc error\n");
		kfree(imagebuffer_TX);
		return -ENOMEM;
	}
	
	imagebuffer_RX = kmalloc(sizeof(unsigned char) * alloc_len, GFP_KERNEL); // add 2 is cmd & dummy
	if (imagebuffer_RX == NULL)
	{
		ELAN_DEBUG("imagebuffer_RX kmalloc error\n");
		kfree(imagebuffer_RX);
		return -ENOMEM;
	}
	
	return 0;
}

static int efsa120s_read_register(struct spi_device *spi, unsigned char *RegInfo)
{
	// [0] = length, [1] = register address, [2~length+1] = data
	char *txbuf = imagebuffer_TX;
	char *rxbuf = imagebuffer_RX;

	int i;
//	int len = RegInfo[0] + 2; //add 2016-6-14
	
	if(RegInfo[0] < 2 || bCMD_REG == 0) // read with dummy 
	{
		if(bCMD_REG == 1) // 0 == CMD, 1 == REG
    			txbuf[0] = READ_REG_HEAD + RegInfo[1]; // one byte data read (+1 = cmd)
		else if(bCMD_REG == 0) // 0 == CMD, 1 == REG
			txbuf[0] = RegInfo[1]; // one byte data read (+1 = cmd)
		efsa120s_spi_transfer(spi, txbuf, rxbuf, RegInfo[0] + 1);
		if(RegInfo[0] < 2) // read reg
		{
			RegInfo[2] = rxbuf[1];
			ELAN_DEBUG("%s() Read = 0x%02x\n", __func__, rxbuf[1]);
		}
		else // read cmd over one byte
		{
			for(i = 0; i < RegInfo[0] - 1; i++)
			{
				RegInfo[i + 2] = rxbuf[i + 2];
				ELAN_DEBUG("%s() Read CMD = 0x%02x\n", __func__, rxbuf[i + 2]);
			}
		}		
	}
	else
	{
		txbuf[0] = READ_SERIER_REG_HEAD + RegInfo[1]; // mutli-byte read (+2 = cmd & dummy)
    		efsa120s_spi_transfer(spi, txbuf, rxbuf, RegInfo[0] + 2);
		for(i = 0; i < RegInfo[0]; i++)
			RegInfo[i + 2] = rxbuf[i + 2];

		ELAN_DEBUG("%s() Read = ", __func__);

		for(i = 0; i < RegInfo[0]; i++)
			ELAN_DEBUG("0x%02x ", rxbuf[i + 2]);

		ELAN_DEBUG("\n");
	}
	return 0;
}



static int efsa120s_write_register(struct spi_device *spi, unsigned char *RegInfo)
{
	// [0] = length, [1] = register address, [2~length+1] = data
	char *txbuf = imagebuffer_TX;
	//char *rxbuf = imagebuffer_RX;//add 2016-6-14
	int i;
	
	//int len = RegInfo[0] + 1; //add 2016-6-14

	if(bCMD_REG == 1) // 0 == CMD, 1 == REG
		txbuf[0] = WRITE_REG_HEAD + RegInfo[1];
	else if(bCMD_REG == 0)
		txbuf[0] = RegInfo[1];

	for(i = 0; i < RegInfo[0]; i++)
		txbuf[i + 1] = RegInfo[i + 2];

	efsa120s_spi_transfer(spi, txbuf, NULL, RegInfo[0] + 1);

	ELAN_DEBUG("%s() ", __func__);

	for(i = 0; i < RegInfo[0]; i++)
		ELAN_DEBUG("0x%02x ", txbuf[i + 1]);

	ELAN_DEBUG("\n");
	
	return 0;

}

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION_LOG);
}

static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

static ssize_t show_int_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "[ELAN] INT gpio status\n");
}

static DEVICE_ATTR(int_value, S_IRUGO, show_int_value, NULL);

#ifdef _ELAN_DEBUG_
static ssize_t elan_debug_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(elan_debug){
		elan_debug=0;
	} else {
		elan_debug=1;
	}
	return sprintf(buf, "[ELAN] elan debug %d\n", elan_debug);
}
static DEVICE_ATTR(elan_debug, S_IRUGO, elan_debug_value, NULL);
#endif

static struct attribute *efsa120s_attributes[] = {
	&dev_attr_drv_version.attr,
	&dev_attr_int_value.attr,
#ifdef _ELAN_DEBUG_
	&dev_attr_elan_debug.attr,
#endif
	NULL
};

static struct attribute_group efsa120s_attr_group = {
	.attrs = efsa120s_attributes,
};

static DEFINE_MUTEX(efsa120s_set_gpio_mutex);
static void efsa120s_gpio_as_int(void)
{
	mutex_lock(&efsa120s_set_gpio_mutex);
	printk("[efsa120s]efsa120s_gpio_as_int\n");
	pinctrl_select_state(elan_fp->pinctrl1, elan_fp->eint_as_int);
	mutex_unlock(&efsa120s_set_gpio_mutex);
}

static void efsa120s_reset_output(int level)
{
	mutex_lock(&efsa120s_set_gpio_mutex);
	//printk("[efsa120s]efsa120s_reset_output level = %d   ,%d,   %d\n", level,elan_fp->pinctrl1,elan_fp->fp_rst_low);

	if (level)
		pinctrl_select_state(elan_fp->pinctrl1, elan_fp->fp_rst_high);
	else
		pinctrl_select_state(elan_fp->pinctrl1, elan_fp->fp_rst_low);
	mutex_unlock(&efsa120s_set_gpio_mutex);
}

static void efsa120s_reset(void)
{
	efsa120s_reset_output(0);
	mdelay(5);
	efsa120s_reset_output(1);
	mdelay(50);
}

static int efsa120s_open(struct inode *inode, struct file *filp)
{
	struct efsa120s_data *fp = container_of(filp->private_data, struct efsa120s_data, efsa120_dev);	
	filp->private_data = fp;
	ELAN_DEBUG("%s()\n", __func__);
	return 0;
}

static int efsa120s_close(struct inode *inode, struct file *filp)
{
	return 0;
}

void efsa120s_irq_disable(struct efsa120s_data *fp)
{
	if(fp->irq_status)
	{
		fp->irq_status = false;
		disable_irq(fp->irq);
	}
}

void efsa120s_irq_enable(struct efsa120s_data *fp)
{
	if(!fp->irq_status)
	{
		fp->irq_status = true;
		enable_irq(fp->irq);
	}
}

static ssize_t efsa120s_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
	struct efsa120s_data *fp = filp->private_data;
	struct spi_device *spi = fp->spi;
	int ret = 0;
	unsigned char * txbuf = NULL;
	unsigned char * rxbuf = NULL;
	int i = 0;
	int len_buf = len;

	if(user_buf[0] == 7) // enable interrupt or disable
	{
		if(len == 3)
		{
			if(user_buf[1] == 1){
				printk("Interrupt enable.\r\n");
				efsa120s_irq_enable(fp);}
			else if(user_buf[1] == 0){
				printk("Interrupt enable.\r\n");
				efsa120s_irq_disable(fp);}
			else
				printk("Interrupt enable/disable data error.\r\n");
		}
		else
			printk("Interrupt enable/disable length error.\r\n");
	}
	else if(user_buf[0] == 8) // write spi
	{
		rxbuf = kzalloc(len_buf, GFP_KERNEL);		
		if (rxbuf == NULL)
			ELAN_DEBUG("memory error.\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL){
			ELAN_DEBUG("memory error.\n");
			kfree(rxbuf);
		}
		else{
			for(i=0; i< len-2; i++)
				txbuf[i] = user_buf[i+1];
			efsa120s_spi_transfer(spi, txbuf, rxbuf, len - 2);
			for(i=0; i< len - 2; i++)
				ELAN_DEBUG("[%d]%x ", i, txbuf[i]);
			ELAN_DEBUG("\n");
			for(i=0; i< len - 2; i++)
				ELAN_DEBUG("[%d]%x ", i, rxbuf[i]);
			ELAN_DEBUG("\n");
			kfree(rxbuf);
			kfree(txbuf);
		}		
	}
	else if(user_buf[0] == 9) // read spi
	{
		rxbuf = kzalloc(len_buf, GFP_KERNEL);		
		if (rxbuf == NULL)
			ELAN_DEBUG("memory error.\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL){
			ELAN_DEBUG("memory error.\n");
			kfree(rxbuf);
		}
		else{
			txbuf[0] = user_buf[1];
			efsa120s_spi_transfer(spi, txbuf, rxbuf, user_buf[2]);
			for(i=0; i< user_buf[2]; i++)
				ELAN_DEBUG("[%d]%x ", i, txbuf[i]);
			ELAN_DEBUG("\n");
			for(i=0; i< user_buf[2]; i++)
				ELAN_DEBUG("[%d]%x ", i, rxbuf[i]);
			ELAN_DEBUG("\n");
			kfree(rxbuf);
			kfree(txbuf);
		}
	}
	else if(user_buf[0] == 10){
		IOIRQ_STATUS = user_buf[1];
		image_ok_flag = 0;
		elan_work_flag = 0;
	}
	else if(user_buf[0] == 0X10){
		read_all = user_buf[1];
	}
	else if(user_buf[0] == 13) // report power key event, if you need report any key -> input_set_capability @ probe
	{
		if(!(user_buf[1] == 0 || user_buf[1] == 1))
			return -1;

		ELAN_DEBUG("KEY_#0x%x = %x.\n", KEY_FP_INT, user_buf[1]);
		input_report_key(fp->input_dev, KEY_FP_INT, user_buf[1]); // Added for KEY Event
		input_sync(fp->input_dev);
		mdelay(1);
	}
  	return (ret==0) ? len : ret;
}



static long efsa120s_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct efsa120s_data *fp = filp->private_data;
	unsigned char buf[16];
	unsigned char *pUserBuf;
	int err = 0;
	int i = 0;
//	int j = 0; //add 2016-6-14
//	int y = 0; //add 2016-6-14
	int int_buf[8] = {0};
	struct mt_chip_conf *spi_conf_mt65xx;

	ELAN_DEBUG("%s() : cmd = [%d]\n", __func__, (int)cmd);

	switch(cmd)
	{
		case ID_IOCTL_READ_REGISTER: //2
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 1; // CMD = 0, REG= 1

			efsa120s_read_register(fp->spi, pUserBuf);
			if(copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0] + 2))
				return -1;
			ELAN_DEBUG("%s() : ID_IOCTL_READ_REGISTER\n", __func__);
			break;

		case ID_IOCTL_WRITE_REGISTER: //3
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 1; // CMD = 0, REG= 1
			
			if(copy_from_user(buf, pUserBuf, pUserBuf[0] + 2))
				return -1;	
			efsa120s_write_register(fp->spi, buf);
			ELAN_DEBUG("%s() : ID_IOCTL_WRITE_REGISTER\n", __func__);
			break;

		case ID_IOCTL_RESET: //6
			efsa120s_reset();
			ELAN_DEBUG("%s() : ID_IOCTL_RESET\n", __func__);
			break;

		case ID_IOCTL_GET_RAW_IMAGE: //10
			ELAN_DEBUG("%s() : ID_IOCTL_GET_RAW_IMAGE START\n", __func__);
			pUserBuf = (unsigned char *)arg;

			// Wait for INT IRQ Read complete.
			err = wait_for_completion_interruptible_timeout(&cmd_done,
								msecs_to_jiffies(ELAN_CALI_TIMEOUT_MSEC));
			if (err <= 0)
				ELAN_DEBUG("wait image fail\n");
			else
				ELAN_DEBUG("wait image success\n");

			Image_index = 0;		

			err = copy_to_user(pUserBuf, imagebuffer, IMG_SIZE*2);
			
			ELAN_DEBUG("%s() : ID_IOCTL_GET_RAW_IMAGE\n", __func__);
			//efsa120s_irq_enable(fp);
			break;

		case ID_IOCTL_READ_CMD: //15
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 0; // CMD = 0, REG= 1

			efsa120s_read_register(fp->spi, pUserBuf);
			if(copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0] + 2))
				return -1;
			ELAN_DEBUG("%s() : ID_IOCTL_READ_CMD\n", __func__);
			break;

		case ID_IOCTL_WRITE_CMD: //16	
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 0; // CMD = 0, REG= 1
			
			if(copy_from_user(buf, pUserBuf, pUserBuf[0] + 2))
				return -1;
			if(pUserBuf[1] == START_SCAN)
				Image_index = 0;
			reinit_completion(&cmd_done);
			efsa120s_write_register(fp->spi, buf);
			ELAN_DEBUG("%s() : ID_IOCTL_WRITE_CMD\n", __func__);
			break;

		case ID_IOCTL_IOIRQ_STATUS: //17
			pUserBuf = (unsigned char *)arg;

			if(copy_from_user(&IOIRQ_STATUS, pUserBuf, 1))
				return -1;
			ELAN_DEBUG("%s() : ID_IOCTL_IOIRQ_STATUS: IOIRQ = %x.\n", __func__, IOIRQ_STATUS);
			//if((IOIRQ_STATUS & 0xA0) || (IOIRQ_STATUS & 0X08))
				//efsa120s_irq_enable(fp);				
			break;

		case ID_IOCTL_SPI_STATUS: //18
			pUserBuf = (unsigned char *)arg;			
			
			if(pUserBuf[0] & 0x80) // Update spi parameter
			{
				spi_speed = (unsigned int) (pUserBuf[1] | (pUserBuf[2] << 8) | (pUserBuf[3] << 16) | (pUserBuf[4] << 24));
				spi_cs_delay = (unsigned int) (pUserBuf[5] | (pUserBuf[6] << 8));
				ELAN_DEBUG("%s() : ID_IOCTL_SPI_STATUS: Update SPI : Speed=%d CS_Delay=%d.\n", __func__, spi_speed, spi_cs_delay);
			}
			else
			{
				if(copy_to_user(&pUserBuf[1], &spi_speed, 4))
					return -1;
				if(copy_to_user(&pUserBuf[5], &spi_cs_delay, 2))
					return -1;
				ELAN_DEBUG("%s() : ID_IOCTL_SPI_STATUS: Read SPI : %x %x %x %x, %x %x\n", __func__, pUserBuf[1], pUserBuf[2], pUserBuf[3], pUserBuf[4], pUserBuf[5], pUserBuf[6]);
			}				
			break;

		case ID_IOCTL_POLL_INIT: //20
			reinit_completion(&cmd_done);
			reinit_completion(&cmd_done_irq);
			ELAN_DEBUG("RID_IOCTL_POLL_INIT\n");
			break;

		case ID_IOCTL_READ_ALL: //21
			pUserBuf = (unsigned char *)arg;
			read_all = (signed char) pUserBuf[0];
			ELAN_DEBUG("Read Raw = %d\n", read_all);
			break;

		case ID_IOCTL_POLL_EXIT:  //23
			ELAN_DEBUG("ID_IOCTL_POLL_EXIT\n");
			complete(&cmd_done_irq);
			complete(&cmd_done);
			break;

		case ID_IOCTL_EN_IRQ: //55
			efsa120s_irq_enable(fp);
			ELAN_DEBUG("ID_IOCTL_EN_IRQ\n");
			break;

		case ID_IOCTL_DIS_IRQ: //66
			efsa120s_irq_disable(fp);
			ELAN_DEBUG("ID_IOCTL_DIS_IRQ\n");
			break;
		case IOCTL_SPI_CONFIG:
			err = copy_from_user(int_buf, (int *)arg, 6*sizeof(int));
			if(err)
				break;
			printk("[elan]: ");
			for(i=0; i<6; i++)
				printk("%x ", int_buf[i]);
			printk("\n");
			spi_conf_mt65xx = (struct mt_chip_conf *) elan_fp->spi->controller_data;
			spi_conf_mt65xx->setuptime = int_buf[0];
			spi_conf_mt65xx->holdtime = int_buf[1];
			spi_conf_mt65xx->high_time = int_buf[2];
			spi_conf_mt65xx->low_time = int_buf[3];
			spi_conf_mt65xx->cs_idletime = int_buf[4];
			spi_conf_mt65xx->ulthgh_thrsh = int_buf[5];
			break;
		case IOCTL_READ_KEY_STATUS:
			return key_status;
		case IOCTL_WRITE_KEY_STATUS:
			key_status = arg;
			if (fasync_queue){
				kill_fasync(&fasync_queue, SIGIO, POLL_IN);
			}
			break;	
		default:
			ELAN_DEBUG("%s() : Unknown cmd\n", __func__);
			break;
	}
	return 0;
}

static unsigned int efsa120s_poll(struct file *file, poll_table *wait)
{
	//struct efsa120s_data *fp = file->private_data; //add 2016-6-14
	int mask=0; 
	poll_wait(file, &elan_poll_wq, wait);
	wait_for_completion_interruptible(&cmd_done_irq);
	if(elan_work_flag > 0){
		mask = elan_work_flag;
	}
	
	elan_work_flag = 0;
	return mask;
}

static int elan_fp_fasync(int fd, struct file * filp, int on)
{
	printk("%s enter \n",__func__);
	return fasync_helper(fd, filp, on, &fasync_queue);
}

ssize_t elan_fp_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	ssize_t ret = 0;
	
	ret = wait_event_timeout(image_waiter, image_ok_flag!= 0, 1*HZ);
	if ( ret != 0 ){
		ret = copy_to_user(buff, imagebuffer, IMG_SIZE*2);
		printk("image is readly \n");
		ret = 1;
	}
	
	return image_ok_flag;
}

static const struct file_operations efsa120s_fops = {
	.owner 			= THIS_MODULE,
	.open 			= efsa120s_open,
	.read			= elan_fp_read,
	.write 			= efsa120s_write,
	.unlocked_ioctl = efsa120s_ioctl,
	.poll			= efsa120s_poll,
	.compat_ioctl 	= efsa120s_ioctl,
	.release 		= efsa120s_close,
	.fasync = elan_fp_fasync,
};

static void efsa120s_fp_work_func(struct work_struct *work)
{
	struct efsa120s_data *fp;
	#if 0
	int i;
	int j = 0;
	int k = 0;
	#endif
	int read_all_count = 0;
	char *txbuf = imagebuffer_TX;
	char *rxbuf = imagebuffer_RX;
	int len = IMG_WIDTH*2+2;

	fp = container_of(work, struct efsa120s_data, work);

	ELAN_DEBUG("%s() IOIRQ=%x.\n", __func__, IOIRQ_STATUS);
	complete(&cmd_done_irq);

	if(IOIRQ_STATUS & 0xA0) // WOE Interrupt Enable
	{
		//efsa120s_irq_enable(fp);
		elan_work_flag = 1;
		wake_up(&elan_poll_wq);
		return;
	}

	if(IOIRQ_STATUS & 0x08) // BUFFER Interrupt Enable
	{
		txbuf[0] = START_READ_IMAGE;
		//ELAN_DEBUG("H2[%d] ", i);
		if(read_all == -1)
		{
			if(Image_index <= IMG_HEIGHT){
				efsa120s_spi_transfer(elan_fp->spi, txbuf, rxbuf, len);
				memcpy(&imagebuffer[(len-2)*Image_index], &rxbuf[2], len-2);
			}
			Image_index++;
			ELAN_DEBUG("Image_Index=%d H=%d\n", Image_index, IMG_HEIGHT);
		}
		else if(read_all == 0)
		{
			imagebuffer_TX[0] = START_READ_IMAGE;
			Image_index = 0;
			len = ((IMG_WIDTH*2+1)*IMG_HEIGHT + 1 + (1024-1))/ 1024 * 1024;
			efsa120s_spi_transfer(fp->spi, imagebuffer_TX, imagebuffer, len); // +2 is cmd & dummy
			ELAN_DEBUG("imagebuffer=%x %x %x %x\n", imagebuffer[0], imagebuffer[1], imagebuffer[2], imagebuffer[3]);
			for(Image_index = 0; Image_index < IMG_HEIGHT; Image_index++){
				if(Image_index == 0){         		
					memcpy(imagebuffer, &imagebuffer[2], IMG_WIDTH*2);
				}
				else{						
					memcpy(&imagebuffer[IMG_WIDTH*2*Image_index], \
						&imagebuffer[(IMG_WIDTH*2+1)*Image_index + 2], IMG_WIDTH*2);
				}
    		}
			Image_index = IMG_HEIGHT;
		}
		else
		{
			Image_index = 0;
			do{				
				if(Image_index <= IMG_HEIGHT){
					efsa120s_spi_transfer(elan_fp->spi, txbuf, rxbuf, len);
					memcpy(&imagebuffer[(len-2)*Image_index], &rxbuf[2], len-2);
				}
				Image_index++;
				ELAN_DEBUG("Image_Index=%d H=%d\n", Image_index, IMG_HEIGHT);
			}while(read_all_count < read_all && Image_index < IMG_HEIGHT);
		}

		if(Image_index == IMG_HEIGHT){
			ELAN_DEBUG("cmd_done finish\n");
			complete(&cmd_done);
			Image_index = 0;
			image_ok_flag = 1;
			wake_up(&image_waiter);
		}
	}
	//efsa120s_irq_enable(fp);
	return;
}

static irqreturn_t elan_fp_irq_handler(int irq, void *dev_id)
{
	wake_lock_timeout(&elan_fp->wake_lock, msecs_to_jiffies(1000));
	ELAN_DEBUG("mt_efsa120s_irq_handler\n");
	if(elan_fp==NULL)
		return -1;
	ELAN_DEBUG("%s()\n", __func__);
	queue_work(efsa120s_wq, &elan_fp->work);
	return 0;
}

static int efsa120s_setup_cdev(struct efsa120s_data *fp)
{
	
	fp->efsa120_dev.minor = MISC_DYNAMIC_MINOR;
	fp->efsa120_dev.name = "elan_fp";
	fp->efsa120_dev.fops = &efsa120s_fops;
	fp->efsa120_dev.mode = S_IFREG|S_IRWXUGO; 
	if (misc_register(&fp->efsa120_dev) < 0) {
  		ELAN_DEBUG("misc_register failed!!");
		return -1;		
	}
  	else {
		ELAN_DEBUG("misc_register finished!!");		
	}
	return 0;
}

static int efsa120s_sysfs_create(struct efsa120s_data *sysfs)
{
	struct efsa120s_data *fp = spi_get_drvdata(sysfs->spi);
	int error = 0;
	
	/* Register sysfs */
	error = sysfs_create_group(&fp->spi->dev.kobj, &efsa120s_attr_group);
	if (error) {
		ELAN_DEBUG("Failed to create sysfs attributes, err: %d\n", error);
		goto fail_un;
	}
	return 0;
fail_un:
	/* Remove sysfs */
	sysfs_remove_group(&fp->spi->dev.kobj, &efsa120s_attr_group);
		
	return error;
}

static char efsa120s_gpio_config(struct efsa120s_data *fp)
{	
	int ret = -1;

	
	struct device_node *node;
	printk("[elan]:%s enter\n", __func__);
//	u32 ints[2] = {0, 0};
	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");

	if ( node)
	{
		#if 0
		of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "fingerprint-irq");
		gpio_set_debounce(ints[0], ints[1]);
		
		printk("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		#endif
		efsa120s_gpio_as_int();
		fp->irq = irq_of_parse_and_map(node, 0);
		
		printk(" efsa120s->irq = %d\n",  fp->irq);
		if (! fp->irq)
		{
			printk("irq_of_parse_and_map fail!!\n");
			return -1;
		}
	}
	else
	{
		printk("null irq node!!\n");
		return -1;
	}
	ret = request_irq(fp->irq, elan_fp_irq_handler,IRQF_TRIGGER_RISING, "elan_fp", NULL);
	if (ret) {
		pr_err("%s : =====EINT IRQ LINE NOT AVAILABLE  %d\n", __func__,ret);
	} else {
		pr_debug("%s : =====set EINT finished, fp_irq=%d", __func__, fp->irq);
		efsa120s_irq_disable(fp);
		msleep(20);
		efsa120s_irq_enable(fp);
	}
	return 0;
}

int efsa120s_dts_init(struct efsa120s_data *pdev)
{
	int ret=0;
	struct device_node *node;
	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	if (node) {
		
		printk("[fp] mt_fp_pinctrl+++++++++++++++++\n");
#if 0		
		pdev->pins_default = pinctrl_lookup_state(pdev->pinctrl1, "fp_default");
		if (IS_ERR(pdev->pins_default)) {
			ret = PTR_ERR(pins_default);
			printk("fwq Cannot find nasri_fingerprint pinctrl default %d!\n", ret);
		}
	
#endif
		pdev->fp_rst_high = pinctrl_lookup_state(pdev->pinctrl1, "state_spi_rst_high");
		if (IS_ERR(pdev->fp_rst_high)) {
			ret = PTR_ERR(pdev->fp_rst_high);
			printk("fwq Cannot find fp pinctrl fp_rst_high!\n");
			return ret;
		}
		pdev->fp_rst_low = pinctrl_lookup_state(pdev->pinctrl1, "state_spi_rst_low");
		if (IS_ERR(pdev->fp_rst_low)) {
			ret = PTR_ERR(pdev->fp_rst_low);
			printk("fwq Cannot find fp pinctrl fp_rst_low!\n");
			return ret;
		}
		pdev->eint_as_int = pinctrl_lookup_state(pdev->pinctrl1, "state_spi_eint_init");
		if (IS_ERR(pdev->eint_as_int)) {
			ret = PTR_ERR(pdev->eint_as_int);
			printk("fwq Cannot find fp pinctrl eint_as_int!\n");
			return ret;
		}
		pdev->eint_in_low = pinctrl_lookup_state(pdev->pinctrl1, "state_spi_eint_low");
		if (IS_ERR(pdev->eint_in_low)) {
			ret = PTR_ERR(pdev->eint_in_low);
			printk("fwq Cannot find fp pinctrl eint_output_low!\n");
			return ret;
		}
#if 0
		pdev->eint_in_float = pinctrl_lookup_state(pdev->pinctrl1, "eint_in_float");
		if (IS_ERR(pdev->eint_in_float)) {
			ret = PTR_ERR(pdev->eint_in_float);
			dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl eint_output_high!\n");
			return ret;
		}
#endif
		return ret;
		printk("[FP] mt_fp_pinctrl----------\n");
	}
	return 0;
}

void efsa120s_gpio_power(int onoff)
{
	int ret;
	printk("%s onoff = %d", __func__, onoff);
	if(onoff){
		ret = regulator_enable(elan_fp->reg);	/*enable regulator*/
		if (ret)
			printk("regulator_enable() failed!\n");
	}else{
		ret = regulator_disable(elan_fp->reg);	/*disable regulator*/
		if (ret)
			printk("regulator_disable() failed!\n");
	}
}


static int efsa120s_probe(struct spi_device *spi)
{	
	struct efsa120s_data *fp = NULL;
	struct input_dev *input_dev = NULL;
	int err = 0;
	unsigned char rxbuf[8] = {0};
	unsigned char txbuf[8] = {0};
	
	ELAN_DEBUG("%s() %s\n", __func__, VERSION_LOG);
	init_completion(&cmd_done);
	init_completion(&cmd_done_irq);	

	err = spi_setup(spi);
	if(err < 0)
		ELAN_DEBUG("spi_setup fail (0x%x).\n", err);
	
	/* Allocate Device Data */
	fp = kzalloc(sizeof(struct efsa120s_data), GFP_KERNEL);
	if(!fp)
		ELAN_DEBUG("alloc efsa120s data fail.\n");

	elan_fp = fp;
	init_waitqueue_head(&fp->efsa_wait);

	/* Init Input Device */
	input_dev = input_allocate_device();
	if (!input_dev)
		ELAN_DEBUG("alloc input_dev fail.\n");

	fp->spi = spi;

	spi_set_drvdata(spi, fp);

	input_dev->name = "efsa120s";
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &spi->dev;
	input_set_drvdata(input_dev, fp);

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	input_set_capability(input_dev, EV_KEY, KEY_FP_INT); // change by customer, send key event to framework. KEY_xxx could be changed.

	fp->input_dev = input_dev;

	/* Init Sysfs */
	err = efsa120s_sysfs_create(fp);
	if(err < 0)
		ELAN_DEBUG("efsa120s sysfs fail.\n");

	wake_lock_init(&fp->wake_lock, WAKE_LOCK_SUSPEND, "fp_wake_lock");

	/* Init Char Device */
	err = efsa120s_setup_cdev(fp);
	if(err < 0)
		ELAN_DEBUG("efsa120s setup device fail.\n");
	
	/* Register Input Device */
	err = input_register_device(input_dev);
	if(err)
		ELAN_DEBUG("Unable to register input device, error: %d!\n", err);

	spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	fp->reg = regulator_get(&spi->dev, "vfingerprint");
	err = regulator_set_voltage(fp->reg, 2800000, 2800000);	/*set 2.8v*/
	if (err) {
		printk("regulator_set_voltage(%d) failed!\n", err);
		return -1;
	}
	fp->pinctrl1 = devm_pinctrl_get(&spi->dev);
	if (IS_ERR(fp->pinctrl1)) {
		err = PTR_ERR(fp->pinctrl1);
		printk("fwq Cannot find fp pinctrl1!\n");
		return err;
	}
	efsa120s_dts_init(fp);
	efsa120s_gpio_power(1);
	msleep(10);
	efsa120s_reset();
	
	/* Init EFSA120S GPIO */
	err = efsa120s_gpio_config(fp);
	if(err < 0)
		ELAN_DEBUG("GPIO request fail (%d).\n", err);

	txbuf[0] = 0xc1;
	efsa120s_spi_transfer(spi, txbuf, rxbuf, 6);
	{
		ELAN_DEBUG("efsa120s read device %02x %02x %02x %02x.\n", rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5]);
		IMG_WIDTH = (unsigned int)(rxbuf[5] - rxbuf[4] + 1);
		IMG_HEIGHT = (unsigned int)(rxbuf[3] - rxbuf[2] + 1);
		IMG_WIDTH_DEFAULT = IMG_WIDTH;
		IMG_HEIGHT_DEFAULT = IMG_HEIGHT;
		ELAN_DEBUG("efsa120s WIDTH(Y)=%d, HEIGHT(X)=%d.\n", IMG_WIDTH, IMG_HEIGHT);
	}

	/* Allocate image buffer */
	err = efsa120s_kmalloc_image();
	if(err)
		ELAN_DEBUG("Unable to kmalloc image buffer, error (%d).\n", err);

	efsa120s_wq = create_singlethread_workqueue("efsa120s_wq");

	if(!efsa120s_wq)
		ELAN_DEBUG("Work Create error!\n");
	
	INIT_WORK(&fp->work, efsa120s_fp_work_func);
	
	return 0;
}

static int efsa120s_remove(struct spi_device *spi)
{
	struct efsa120s_data *fp = spi_get_drvdata(spi);
	
	if (fp->intr_gpio)
		free_irq(fp->intr_gpio, fp);

	
	misc_deregister(&fp->efsa120_dev);
	input_free_device(fp->input_dev);
		
	kfree(fp);

	spi_set_drvdata(spi, NULL);
		
	return 0;
}

static const struct spi_device_id efp_id[] = {
	{"elan_fp96s", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, efp_id);
#ifndef CONFIG_OF
static struct of_device_id efsa120s_of_match[] = {
	{ .compatible = "mediatek,elan_fp", },
	{}
};

MODULE_DEVICE_TABLE(of, efsa120s_of_match);
#endif

static struct spi_driver efsa120s_driver = {
	.driver = {
		.name 	= "elan_fp96s",
		.owner = THIS_MODULE,
	#ifdef CONFIG_OF
	//	.of_match_table = efsa120s_of_match,
	#endif	
	},
	.probe 	= efsa120s_probe,
	.remove = efsa120s_remove,
	.id_table = efp_id,
};

static struct mt_chip_conf spi_xxxx_conf = {  
	//SPI speed
	.setuptime = 3,
	.holdtime = 3,
	.high_time = 10,
	.low_time = 10,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	
	//SPI mode
	.cpol = 0,
	.cpha = 0,
	
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	
	.tx_endian = 0,
	.rx_endian = 0,
	
	.com_mod = 1,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

static struct spi_board_info efsa120s_spi_board_info[] = {
      
	[0] = {
		.modalias               = "elan_fp96s",
		.bus_num                = 0, // change by customer
		.chip_select            = 0, // change by customer, usually = 0.
		.max_speed_hz           = SPI_MAX_SPEED,
		.mode			= SPI_MODE_0,
		.controller_data = (void*)&spi_xxxx_conf,
	},
};

static int __init efsa120s_init(void)
{
	ELAN_DEBUG("******************* efsa120s_init *******************\n");
	spi_register_board_info(efsa120s_spi_board_info, ARRAY_SIZE(efsa120s_spi_board_info));

	return spi_register_driver(&efsa120s_driver);
}

static void __exit efsa120s_exist(void)
{	
	spi_unregister_driver(&efsa120s_driver);

	if(efsa120s_wq)
		destroy_workqueue(efsa120s_wq);
}

module_init(efsa120s_init);
module_exit(efsa120s_exist);

MODULE_AUTHOR("Elan");
MODULE_DESCRIPTION("ELAN SPI FingerPrint eFSA120S driver");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");
