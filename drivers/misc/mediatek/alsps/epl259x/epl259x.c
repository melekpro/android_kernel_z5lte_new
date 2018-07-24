/* drivers/hwmon/mt6516/amit/epl259x.c - EPL259x ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "epl259x.h"
#include <alsps.h>
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif

//#include "upmu_sw.h"

#ifndef CUST_EINT_ALS_TYPE
#define CUST_EINT_ALS_TYPE  8
#endif

//#define Mineloginfo

/******************************************************************************
 * extern functions
*******************************************************************************/
#if 0
#if defined(MT6575) || defined(MT6571) || defined(MT6589)
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#else
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt_eint_set_sens(kal_uint8 eintno, kal_bool sens);
//extern void mt_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
//                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
//                                     kal_bool auto_umask);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
#endif

/*-------------------------MT6516&MT6575 define-------------------------------*/
//#define POWER_NONE_MACRO MT65XX_POWER_NONE



/******************************************************************************
 *  configuration
 ******************************************************************************/
#define LUX_PER_COUNT			400//700		// 0.7

#define COMMON_DEBUG    1
#define ALS_DEBUG       1
#define PS_DEBUG        1
#define SHOW_DBG        1

#define PS_GES          0
#define PS_DYN_K        1
#define PS_DYN_K_STR    0
#define ALS_DYN_INTT    0
#define HS_ENABLE       0

/******************************************************************************
*******************************************************************************/

#define TXBYTES 				2
#define RXBYTES					2
#define PACKAGE_SIZE 			48
#define I2C_RETRY_COUNT 		2
#define EPL_DEV_NAME   		    "EPL259x"
#define DRIVER_VERSION             "2.0.6"

struct input_dev dev;
struct hwmsen_object *ps_hw, * als_hw;
static struct epl_sensor_priv *epl_sensor_obj;
static struct epl_sensor_priv *epl259x_obj;
//static struct platform_driver epl_sensor_alsps_driver;
static struct wake_lock ps_lock;
static struct mutex sensor_mutex;
bool polling_flag = true;
bool eint_flag = true;

#if PS_DYN_K
static int dynk_polling_delay = 200;
int dynk_min_ps_raw_data = 0xffff;
int dynk_max_ir_data;

unsigned long dynk_thd_low = 0;
unsigned long dynk_thd_high = 0;

int dynk_low_offset;
int dynk_high_offset;

bool dynk_change_flag = false;
u16 dynk_change_thd_max;
u16 dynk_thd_offset;

u8 dynk_last_status = 0;

#endif

static epl_optical_sensor epl_sensor;
int i2c_max_count=8;
static bool first_als_report_flag = true;

//ps calibration file location
static const char ps_cal_file[]="/data/data/com.eminent.ps.calibration/ps.dat";

//als calibration file location
static const char als_cal_file[]="/data/data/com.eminent.ps.calibration/als.dat";

static int PS_h_offset = 3000;
static int PS_l_offset = 2000;
static int PS_MAX_XTALK = 30000;

int als_frame_time = 0;
int ps_frame_time = 0;

typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
} epl_raw_data;
static epl_raw_data	gRawData;

static int alsps_local_init(void);
static int alsps_remove(void);
static int alsps_init_flag = -1;
/*----------------------------------------------------------------------------*/
static struct alsps_init_info epl_sensor_init_info = {
		.name = EPL_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,
};

/*----------------------------------------------------------------------------*/
#define APS_TAG                 	  	"[ALS/PS] "
#define APS_FUN(f)              	  	printk( APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk( APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    	    printk( fmt, ##args)

/*----------------------------------------------------------------------------*/
static struct i2c_client *epl_sensor_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl_sensor_i2c_id[] = { {EPL_DEV_NAME, 0}, {} };
static unsigned long long int_top_time;
/* Maintain alsps cust info here */
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
	return &alsps_cust;
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl_sensor_i2c_remove(struct i2c_client *client);
static int epl_sensor_i2c_detect(struct i2c_client *client,  struct i2c_board_info *info);
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl_sensor_i2c_resume(struct i2c_client *client);
static irqreturn_t epl_sensor_eint_func(int irq, void *desc);

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);

int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
#if PS_DYN_K
void epl_sensor_restart_dynk_polling(void);
#endif
void epl_sensor_update_mode(struct i2c_client *client);
int epl_sensor_read_ps(struct i2c_client *client);
int epl_sensor_read_als_status(struct i2c_client *client);
static int ps_sensing_time(int intt, int adc, int cycle);
static int als_sensing_time(int intt, int adc, int cycle);

/*----------------------------------------------------------------------------*/
static DECLARE_WAIT_QUEUE_HEAD(wait_rsp_wq);
/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_ALS   	= 1,
    CMC_BIT_PS     	= 2,
} CMC_BIT;

typedef enum
{
    CMC_BIT_RAW   			= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
    CMC_BIT_DEF_LIGHT		= 0x4,
    CMC_BIT_TABLE			= 0x8,
} CMC_ALS_REPORT_TYPE;

/*----------------------------------------------------------------------------*/
struct epl_sensor_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl_sensor_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
#if PS_DYN_K
    struct delayed_work  dynk_thd_polling_work;
#endif

    struct input_dev *gs_input_dev;
    struct input_dev *hb_input_dev;
    struct epl_sensor_i2c_addr  addr;


    struct device_node *irq_node;	
    int irq;

    /*misc*/
    atomic_t    trace;
    atomic_t   	als_suspend;

    atomic_t    ps_suspend;

    /*data*/
    u16		    lux_per_count;
    ulong       enable;         	/*record HAL enalbe status*/
    ulong      	pending_intr;   	/*pending interrupt*/

    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif

};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,alsps", },
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver epl_sensor_i2c_driver =
{
    .probe     	= epl_sensor_i2c_probe,
    .remove     = epl_sensor_i2c_remove,
    .detect       = epl_sensor_i2c_detect,
    .suspend    = epl_sensor_i2c_suspend,
    .resume     = epl_sensor_i2c_resume,
    .id_table   = epl_sensor_i2c_id,
    .driver = {
        .name   = EPL_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = alsps_of_match,
#endif   
    },
};



/*
//====================I2C write operation===============//
*/
static int epl_sensor_I2C_Write_Cmd(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = regaddr;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n",ret);
        mdelay(10);
    }


    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}

static int epl_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = epl_sensor_I2C_Write_Cmd(client, regaddr, data, 0x02);
    return ret;
}


/*
//====================I2C read operation===============//
*/
static int epl_sensor_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount)
{
    int ret = 0;
    int retry;
    int read_count=0, rx_count=0;

    while(bytecount>0)
    {
        epl_sensor_I2C_Write_Cmd(client, regaddr+read_count, 0x00, 0x01);

        for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
        {
            rx_count = bytecount>i2c_max_count?i2c_max_count:bytecount;
            ret = i2c_master_recv(client, &gRawData.raw_bytes[read_count], rx_count);

            if (ret == rx_count)
                break;

            APS_ERR("i2c read error,RXBYTES %d\r\n",ret);
            mdelay(10);
        }

        if(retry>=I2C_RETRY_COUNT)
        {
            APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
            return -EINVAL;
        }
        bytecount-=rx_count;
        read_count+=rx_count;
    }

    return ret;
}


/*
//====================write global variable===============//
*/
static void write_global_variable(struct i2c_client *client)
{
	u8 buf;
//	 struct epl_sensor_priv *obj = epl_sensor_obj;

    //wake up chip
    buf = epl_sensor.reset | epl_sensor.power;
    epl_sensor_I2C_Write(client, 0x11, buf);

    /* read revno*/
    epl_sensor_I2C_Read(client, 0x20, 2);
    epl_sensor.revno = gRawData.raw_bytes[0] | gRawData.raw_bytes[1] << 8;

    /*chip refrash*/
    epl_sensor_I2C_Write(client, 0xfd, 0x8e);
    epl_sensor_I2C_Write(client, 0xfe, 0x22);
    epl_sensor_I2C_Write(client, 0xfe, 0x02);
    epl_sensor_I2C_Write(client, 0xfd, 0x00);

	epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);
    
	/*ps setting*/
	buf = epl_sensor.ps.integration_time | epl_sensor.ps.gain;
	epl_sensor_I2C_Write(client, 0x03, buf);

        buf = epl_sensor.ps.adc | epl_sensor.ps.cycle;
        epl_sensor_I2C_Write(client, 0x04, buf);

        buf = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
        epl_sensor_I2C_Write(client, 0x05, buf);

        buf = epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type;
        epl_sensor_I2C_Write(client, 0x06, buf);

        buf = epl_sensor.ps.compare_reset | epl_sensor.ps.lock;
        epl_sensor_I2C_Write(client, 0x1b, buf);

        epl_sensor_I2C_Write(client, 0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(client, 0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        /*als setting*/
        buf = epl_sensor.als.integration_time | epl_sensor.als.gain;
        epl_sensor_I2C_Write(client, 0x01, buf);

        buf = epl_sensor.als.adc | epl_sensor.als.cycle;
        epl_sensor_I2C_Write(client, 0x02, buf);

        buf = epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;
        epl_sensor_I2C_Write(client, 0x07, buf);

        buf = epl_sensor.als.compare_reset | epl_sensor.als.lock;
        epl_sensor_I2C_Write(client, 0x12, buf);

	set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);	

    //set mode and wait
    buf = epl_sensor.wait | epl_sensor.mode;
    epl_sensor_I2C_Write(client, 0x00, buf);

}

static int write_factory_calibration(struct epl_sensor_priv *epl_data, char* ps_data, int ps_cal_len)
{
    struct file *fp_cal;

	mm_segment_t fs;
	loff_t pos;

	APS_FUN();
    pos = 0;

	fp_cal = filp_open(ps_cal_file, O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
	if (IS_ERR(fp_cal))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}

    fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp_cal, ps_data, ps_cal_len, &pos);

    filp_close(fp_cal, NULL);

	set_fs(fs);

	return 0;
}
static bool read_factory_calibration(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
    char buffer[100]= {0};
    if(epl_sensor.ps.factory.calibration_enable && !epl_sensor.ps.factory.calibrated)
    {
		fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.ps.factory.calibration_enable =  false;
        }
        else
        {
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%u,%u,%u", &epl_sensor.ps.factory.cancelation, &epl_sensor.ps.factory.high_threshold, &epl_sensor.ps.factory.low_threshold);
            set_fs(fs);

            epl_sensor.ps.high_threshold = epl_sensor.ps.factory.high_threshold;
            epl_sensor.ps.low_threshold = epl_sensor.ps.factory.low_threshold;
            epl_sensor.ps.cancelation = epl_sensor.ps.factory.cancelation;
        }

        epl_sensor_I2C_Write(obj->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(obj->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        epl_sensor.ps.factory.calibrated = true;
    }

    if(epl_sensor.als.factory.calibration_enable && !epl_sensor.als.factory.calibrated)
    {
        fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.als.factory.calibration_enable =  false;
        }
        else
        {
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%u", &epl_sensor.als.factory.lux_per_count);
            set_fs(fs);
        }
        epl_sensor.als.factory.calibrated = true;
    }
    return true;
}

static int epl_run_ps_calibration(struct epl_sensor_priv *epl_data)
{
    struct epl_sensor_priv *epld = epl_data;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =5, i;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0, ps_cal_len = 0;
    char ps_calibration[20];


    if(PS_MAX_XTALK < 0)
    {
        APS_ERR("[%s]:Failed: PS_MAX_XTALK < 0 \r\n", __func__);
        return -EINVAL;
    }

    if(enable_ps == 0)
    {
        set_bit(CMC_BIT_PS, &epld->enable);
        epl_sensor_update_mode(epld->client);
    }

    polling_flag = false;

    for(i=0; i<count; i++)
    {
        msleep(50);
    	switch(epl_sensor.mode)
    	{
    		case EPL_MODE_PS:
    		case EPL_MODE_ALS_PS:
    		    if(enable_ps == true && polling_flag == true && eint_flag == true)
    		 	    epl_sensor_read_ps(epld->client);
    			ch1 = epl_sensor.ps.data.data;
		    break;
    	}

    	ch1_all = ch1_all + ch1;
    	if(epl_sensor.wait == EPL_WAIT_SINGLE)
    		epl_sensor_I2C_Write(epld->client,0x11, epl_sensor.power | epl_sensor.reset);
    }


    ch1 = (u16)(ch1_all/count);

    if(ch1 > PS_MAX_XTALK)
    {
        APS_ERR("[%s]:Failed: ch1 > max_xtalk(%d) \r\n", __func__, ch1);
        return -EINVAL;
    }
    else if(ch1 <= 0)
    {
        APS_ERR("[%s]:Failed: ch1 = 0\r\n", __func__);
        return -EINVAL;
    }

    ps_hthr = ch1 + PS_h_offset;
    ps_lthr = ch1 + PS_l_offset;

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d", ps_cancelation, ps_hthr, ps_lthr);

    if(write_factory_calibration(epld, ps_calibration, ps_cal_len) < 0)
    {
        APS_ERR("[%s] create file error \n", __func__);
        return -EINVAL;
    }

    epl_sensor.ps.low_threshold = ps_lthr;
	epl_sensor.ps.high_threshold = ps_hthr;
	set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

	APS_LOG("[%s]: ch1 = %d\n", __func__, ch1);

    polling_flag = true;
	return ch1;
}


static void set_als_ps_intr_type(struct i2c_client *client, bool ps_polling, bool als_polling)
{

    //set als / ps interrupt control mode and trigger type
	switch((ps_polling << 1) | als_polling)
	{
		case 0: // ps and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 1: //ps interrupt and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 2: // ps polling and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;

		case 3: //ps and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;
	}
}
/*
//====================initial global variable===============//
*/
static void initial_global_variable(struct i2c_client *client, struct epl_sensor_priv *obj)
{
    //general setting
    epl_sensor.power = EPL_POWER_ON;
    epl_sensor.reset = EPL_RESETN_RUN;
    epl_sensor.mode = EPL_MODE_IDLE;
    epl_sensor.wait = EPL_WAIT_20_MS;
    epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;

    //als setting
    epl_sensor.als.polling_mode = obj->hw->polling_mode_als;
    epl_sensor.als.integration_time = EPL_ALS_INTT_1024;
    epl_sensor.als.gain = EPL_GAIN_LOW;
    epl_sensor.als.adc = EPL_PSALS_ADC_11;
    epl_sensor.als.cycle = EPL_CYCLE_16;
    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
    epl_sensor.als.persist = EPL_PERIST_1;
    epl_sensor.als.compare_reset = EPL_CMP_RESET;
    epl_sensor.als.lock = EPL_UN_LOCK;
    epl_sensor.als.report_type = ALS_REPORT_TYPE; //CMC_BIT_DYN_INT;
    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;

    //als factory
    epl_sensor.als.factory.calibration_enable =  false;
    epl_sensor.als.factory.calibrated = false;
    epl_sensor.als.factory.lux_per_count = LUX_PER_COUNT;

    //ps setting
    epl_sensor.ps.polling_mode = obj->hw->polling_mode_ps;
    epl_sensor.ps.integration_time = obj->hw->ps_integration_time;
    epl_sensor.ps.gain = EPL_GAIN_LOW;
#if PS_DYN_K
    dynk_max_ir_data = 50000; // ps max ch0
    dynk_low_offset = 	DYNK_LOW_OFFSET;
    dynk_high_offset = DYNK_HIGH_OFFSET;
    dynk_change_thd_max = 30000; //If NEAR, change H/L
    dynk_thd_offset = 0; //500; //If NEAR, change H/L offset

#endif
    epl_sensor.ps.adc = EPL_PSALS_ADC_11;
    epl_sensor.ps.cycle = EPL_CYCLE_16;
    epl_sensor.ps.persist = EPL_PERIST_1;
    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.ps.ir_drive = EPL_IR_DRIVE_100;
    epl_sensor.ps.compare_reset = EPL_CMP_RESET;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor.ps.high_threshold = obj->hw->ps_threshold_high;
    epl_sensor.ps.low_threshold = obj->hw->ps_threshold_low;

    //ps factory
    epl_sensor.ps.factory.calibration_enable =  false;
    epl_sensor.ps.factory.calibrated = false;
    epl_sensor.ps.factory.cancelation= 0;

    //set als / ps interrupt control mode and trigger type
    set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    //write setting to sensor
    write_global_variable(client);
}

static int epl_sensor_get_als_value(struct epl_sensor_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;

    switch(epl_sensor.als.report_type)
    {
        case CMC_BIT_RAW:
            return als;
            break;

        case CMC_BIT_PRE_COUNT:
            return (als * epl_sensor.als.factory.lux_per_count)/1000;
            break;

        case CMC_BIT_TABLE:
            for(idx = 0; idx < obj->als_level_num; idx++)
            {
                if(als < obj->hw->als_level[idx])
                {
                    break;
                }
            }

            if(idx >= obj->als_value_num)
            {
                APS_ERR("exceed range\n");
                idx = obj->als_value_num - 1;
            }

            if(!invalid)
            {
                APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
                return obj->hw->als_value[idx];
            }
            else
            {
                APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
                return als;
            }
            break;
    }

    return 0;
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;


    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);

    epl_sensor_I2C_Write(client, 0x0c, low_lsb);
    epl_sensor_I2C_Write(client, 0x0d, low_msb);
    epl_sensor_I2C_Write(client, 0x0e, high_lsb);
    epl_sensor_I2C_Write(client, 0x0f, high_msb);

    return 0;
}

static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    epl_sensor_I2C_Write(client, 0x08, low_lsb);
    epl_sensor_I2C_Write(client, 0x09, low_msb);
    epl_sensor_I2C_Write(client, 0x0a, high_lsb);
    epl_sensor_I2C_Write(client, 0x0b, high_msb);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);

    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    APS_LOG("chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    APS_LOG("chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    APS_LOG("chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    APS_LOG("chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    APS_LOG("chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    APS_LOG("chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    APS_LOG("chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    APS_LOG("chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    APS_LOG("chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    APS_LOG("chip id REG 0x20 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x21 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x21));
    APS_LOG("chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    APS_LOG("chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    APS_LOG("chip id REG 0xfb value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfb));
    APS_LOG("chip id REG 0xfc value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfc));
}

/*----------------------------------------------------------------------------*/
int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");

    epl_sensor_i2c_client = client;

    APS_LOG(" I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr,__LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_get_addr(struct alsps_hw *hw, struct epl_sensor_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_power(struct alsps_hw *hw, unsigned int on)
{
//	printk("--lantao -- (epl_sensor_power) end.\n ");
}

/*----------------------------------------------------------------------------*/

int epl_sensor_read_als(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(obj->client, 0x13, 4);
    
        epl_sensor.als.data.channels[0] = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
        epl_sensor.als.data.channels[1] = (gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2];

        APS_LOG("read als channe 0 = %d\n", epl_sensor.als.data.channels[0]);
        APS_LOG("read als channe 1 = %d\n", epl_sensor.als.data.channels[1]);
    return 0;
}

int epl_sensor_read_als_status(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    u8 buf;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(obj->client, 0x12, 1);

    buf = gRawData.raw_bytes[0];

    epl_sensor.als.saturation = (buf & 0x20);
    epl_sensor.als.compare_high = (buf & 0x10);
    epl_sensor.als.compare_low = (buf & 0x08);
    epl_sensor.als.interrupt_flag = (buf & 0x04);
    epl_sensor.als.compare_reset = (buf & 0x02);
    epl_sensor.als.lock = (buf & 0x01);

#if PS_DEBUG
	APS_LOG("als: ~~~~ ALS ~~~~~ \n");
	APS_LOG("als: buf = 0x%x\n", buf);
	APS_LOG("als: sat = 0x%x\n", epl_sensor.als.saturation);
	APS_LOG("als: cmp h = 0x%x, l = %d\n", epl_sensor.als.compare_high, epl_sensor.als.compare_low);
	APS_LOG("als: int_flag = 0x%x\n",epl_sensor.als.interrupt_flag);
	APS_LOG("als: cmp_rstn = 0x%x, lock = 0x%0x\n", epl_sensor.als.compare_reset, epl_sensor.als.lock);
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_read_ps(struct i2c_client *client)
{

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(client,0x1c, 4);

    epl_sensor.ps.data.ir_data = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    epl_sensor.ps.data.data = (gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2];

	APS_LOG("[%s]: data = %d\n", __func__, epl_sensor.ps.data.data);
	APS_LOG("[%s]: ir data = %d\n", __func__, epl_sensor.ps.data.ir_data);

    if(epl_sensor.wait == EPL_WAIT_SINGLE)
	    epl_sensor_I2C_Write(client, 0x11, epl_sensor.power | epl_sensor.reset);

    return 0;
}

int epl_sensor_read_ps_status(struct i2c_client *client)
{
    u8 buf;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(client, 0x1b, 1);
    buf = gRawData.raw_bytes[0];

    epl_sensor.ps.saturation = (buf & 0x20);
    epl_sensor.ps.compare_high = (buf & 0x10);
    epl_sensor.ps.compare_low = (buf & 0x08);
    epl_sensor.ps.interrupt_flag = (buf & 0x04);
    epl_sensor.ps.compare_reset = (buf & 0x02);
    epl_sensor.ps.lock= (buf & 0x01);

#if PS_DEBUG
	APS_LOG("ps: ~~~~ PS ~~~~~ \n");
	APS_LOG("ps: buf = 0x%x\n", buf);
	APS_LOG("ps: sat = 0x%x\n", epl_sensor.ps.saturation);
	APS_LOG("ps: cmp h = 0x%x, l = 0x%x\n", epl_sensor.ps.compare_high, epl_sensor.ps.compare_low);
	APS_LOG("ps: int_flag = 0x%x\n",epl_sensor.ps.interrupt_flag);
	APS_LOG("ps: cmp_rstn = 0x%x, lock = %x\n", epl_sensor.ps.compare_reset, epl_sensor.ps.lock);
#endif
    return 0;
}

#if PS_DYN_K
void epl_sensor_reset_dynk_thd(u8 last_status, u8 now_status)
{
    if(last_status==0 && now_status==1)
    {
        if( (epl_sensor.ps.saturation == 0)&&(epl_sensor.ps.data.ir_data < dynk_max_ir_data) )

        {
            dynk_min_ps_raw_data = epl_sensor.ps.data.data;
        }
        dynk_thd_low = dynk_min_ps_raw_data + dynk_low_offset;
	    dynk_thd_high = dynk_min_ps_raw_data + dynk_high_offset;

        if(dynk_thd_low>65534)
            dynk_thd_low = 65534;
        if(dynk_thd_high>65535)
            dynk_thd_high = 65535;
#if PS_DEBUG
	    APS_LOG("[%s]:restart dynk ps raw = %d, min = %d, ir_data = %d\n", __func__, epl_sensor.ps.data.data, dynk_min_ps_raw_data, epl_sensor.ps.data.ir_data);
	    APS_LOG("[%s]:restart dynk thre_l = %lu, thre_h = %lu\n", __func__, dynk_thd_low, dynk_thd_high);
#endif
        eint_flag = false;
	    set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
        eint_flag = true;
    }
    else if(last_status==1 && now_status==0)
    {
        dynk_change_flag = true;
    }
}

void epl_sensor_restart_dynk_polling(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    cancel_delayed_work(&obj->dynk_thd_polling_work);
    schedule_delayed_work(&obj->dynk_thd_polling_work, msecs_to_jiffies(2*dynk_polling_delay));
}

void epl_sensor_dynk_thd_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    APS_LOG("[%s]:als / ps enable: %d / %d\n", __func__,enable_als, enable_ps);

    if(enable_ps == true)
    {

        if(polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
        {
            mutex_lock(&sensor_mutex);
            epl_sensor_read_ps_status(obj->client);
            epl_sensor_read_ps(obj->client);
            mutex_unlock(&sensor_mutex);
        }

        if( (dynk_min_ps_raw_data > epl_sensor.ps.data.data)
            && (epl_sensor.ps.saturation == 0)
            && (epl_sensor.ps.data.ir_data < dynk_max_ir_data) )

        {
            dynk_min_ps_raw_data = epl_sensor.ps.data.data;
            dynk_thd_low = dynk_min_ps_raw_data + dynk_low_offset;
		    dynk_thd_high = dynk_min_ps_raw_data + dynk_high_offset;

            if(dynk_thd_low>65534)
                dynk_thd_low = 65534;
            if(dynk_thd_high>65535)
                dynk_thd_high = 65535;

		    APS_LOG("[%s]:dyn ps raw = %d, min = %d, ir_data = %d\n", __func__, epl_sensor.ps.data.data, dynk_min_ps_raw_data, epl_sensor.ps.data.ir_data);
            eint_flag = false;
            mutex_lock(&sensor_mutex);
		    set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
            mutex_unlock(&sensor_mutex);
            eint_flag = true;
		    APS_LOG("[%s]:dyn k thre_l = %lu, thre_h = %lu\n", __func__, dynk_thd_low, dynk_thd_high);
        }
        else if(dynk_change_flag==true && (epl_sensor.ps.data.data>dynk_change_thd_max) && ( (epl_sensor.ps.compare_low >> 3)==0 ))
        {
            dynk_change_flag = false;
            dynk_thd_low += dynk_thd_offset;
	        dynk_thd_high += dynk_thd_offset;

	        if(dynk_thd_low>65534)
                dynk_thd_low = 65534;
            if(dynk_thd_high>65535)
                dynk_thd_high = 65535;

            eint_flag = false;
            mutex_lock(&sensor_mutex);
            set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
            mutex_unlock(&sensor_mutex);
            eint_flag = true;
            APS_LOG("[%s]: epl_sensor.ps.data.data=%d, L/H=%lu/%lu \r\n", __func__, epl_sensor.ps.data.data, dynk_thd_low, dynk_thd_high);
        }

        schedule_delayed_work(&obj->dynk_thd_polling_work, msecs_to_jiffies(dynk_polling_delay));
    }

}
#endif
/************************************************************************/
//for 3637
static int als_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_adc, als_cycle;

    als_intt = als_intt_value[intt>>2];
    als_adc = adc_value[adc>>3];
    als_cycle = cycle_value[cycle];
#if COMMON_DEBUG
    APS_LOG("ALS: INTT=%d, ADC=%d, Cycle=%d \r\n", als_intt, als_adc, als_cycle);
#endif

    sensing_us_time = (als_intt + als_adc*2*2) * 2 * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;

#if COMMON_DEBUG
    APS_LOG("[%s]: sensing=%d ms \r\n", __func__, sensing_ms_time);
#endif
    return (sensing_ms_time + 5);
}

static int ps_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int ps_intt, ps_adc, ps_cycle;

    ps_intt = ps_intt_value[intt>>2];
    ps_adc = adc_value[adc>>3];
    ps_cycle = cycle_value[cycle];
#if COMMON_DEBUG
    APS_LOG("PS: INTT=%d, ADC=%d, Cycle=%d \r\n", ps_intt, ps_adc, ps_cycle);
#endif

    sensing_us_time = (ps_intt*3 + ps_adc*2*3) * ps_cycle;
    sensing_ms_time = sensing_us_time / 1000;
#if COMMON_DEBUG
    APS_LOG("[%s]: sensing=%d ms\r\n", __func__, sensing_ms_time);
#endif


    return (sensing_ms_time + 5);
}

static int epl_sensor_get_wait_time(int ps_time, int als_time)
{
    int wait_idx = 0;
    int wait_time = 0;

    wait_time = als_time - ps_time;
    if(wait_time < 0){
        wait_time = 0;
    }
#if COMMON_DEBUG
    APS_LOG("[%s]: wait_len = %d \r\n", __func__, wait_len);
#endif
    for(wait_idx = 0; wait_idx < wait_len; wait_idx++)
	{
	    if(wait_time < wait_value[wait_idx])
	    {
	        break;
	    }
	}
	if(wait_idx >= wait_len){
        wait_idx = wait_len - 1;
	}

#if COMMON_DEBUG
	APS_LOG("[%s]: wait_idx = %d, wait = %dms \r\n", __func__, wait_idx, wait_value[wait_idx]);
#endif
	return (wait_idx << 4);
}
/************************************************************************/

void epl_sensor_update_mode(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int als_time = 0, ps_time = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
    ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);

    als_frame_time = als_time;
    ps_frame_time = ps_time;

    polling_flag = false;

	APS_LOG("mode selection =0x%x\n", enable_ps | (enable_als << 1));

	   /*PS_GES and HS_ENABLE*/

        //PS unlock and reset
        epl_sensor.ps.compare_reset = EPL_CMP_RESET;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

        //ALS unlock and reset
        epl_sensor.als.compare_reset = EPL_CMP_RESET;
    	epl_sensor.als.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

        epl_sensor.ps.interrupt_flag = EPL_INT_CLEAR;
        epl_sensor.als.interrupt_flag = EPL_INT_CLEAR;
	    //epl_sensor_I2C_Write(client, 0x11, EPL_RESETN_RESET | EPL_POWER_OFF);

        //**** mode selection ****
        switch((enable_als << 1) | enable_ps)


        {
            case 0: //disable all
                epl_sensor.mode = EPL_MODE_IDLE;
                break;

            case 1: //als = 0, ps = 1

                epl_sensor.mode = EPL_MODE_PS;

                break;

            case 2: //als = 1, ps = 0
                epl_sensor.mode = EPL_MODE_ALS;
                break;

            case 3: //als = 1, ps = 1
                epl_sensor.mode = EPL_MODE_ALS_PS;
                break;
        }

        //**** write setting ****
        // step 1. set sensor at idle mode
        // step 2. uplock and reset als / ps status
        // step 3. set sensor at operation mode
        // step 4. delay sensing time
        // step 5. unlock and run als / ps status

        epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

        // initial factory calibration variable
        read_factory_calibration();


        epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
        set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);

        epl_sensor_I2C_Write(obj->client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);

        epl_sensor_I2C_Write(obj->client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

        if(enable_als == true && enable_ps == false)
        {
            epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL | EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);
        }

        if(epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 0 && epl_sensor.ps.polling_mode == 0){
            int wait = 0;
            wait = epl_sensor_get_wait_time(ps_time, als_time);
            epl_sensor_I2C_Write(client, 0x00, wait | epl_sensor.mode);
            epl_sensor.wait = wait;
            APS_LOG("[%s]: epl_sensor.als.polling_mode=%d \r\n", __func__, epl_sensor.als.polling_mode);
        }
        else{
                epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | epl_sensor.mode);
        }

        //PS unlock and run
        epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

        //ALS unlock and run
       	epl_sensor.als.compare_reset = EPL_CMP_RUN;
    	epl_sensor.als.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

        //epl_sensor_I2C_Write(client, 0x11, epl_sensor.reset | epl_sensor.power);
#if COMMON_DEBUG
        //**** check setting ****
        if(enable_ps == 1)
        {
            APS_LOG("[%s] PS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
        }

        if(enable_als == 1 && epl_sensor.als.polling_mode == 0)
        {
            APS_LOG("[%s] ALS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
        }

    	APS_LOG("[%s] reg0x00= 0x%x\n", __func__,  epl_sensor.wait | epl_sensor.mode);
    	APS_LOG("[%s] reg0x07= 0x%x\n", __func__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    	APS_LOG("[%s] reg0x06= 0x%x\n", __func__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
    	APS_LOG("[%s] reg0x11= 0x%x\n", __func__, epl_sensor.power | epl_sensor.reset);
    	APS_LOG("[%s] reg0x12= 0x%x\n", __func__, epl_sensor.als.compare_reset | epl_sensor.als.lock);
    	APS_LOG("[%s] reg0x1b= 0x%x\n", __func__, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
#endif

        if(epl_sensor.mode == EPL_MODE_PS)
        {
            msleep(ps_time);
            APS_LOG("[%s] PS only(%dms)\r\n", __func__, ps_time);
        }
        else if (epl_sensor.mode == EPL_MODE_ALS)
        {
            msleep(als_time);
            APS_LOG("[%s] ALS only(%dms)\r\n", __func__, als_time);
        }
        else if (epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 1)
        {
            msleep(ps_time+als_time+wait_value[epl_sensor.wait>>4]);
            APS_LOG("[%s] PS+ALS(%dms)\r\n", __func__, ps_time+als_time+wait_value[epl_sensor.wait>>4]);
        }

        if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER)
        {
            //PS unlock and run
            epl_sensor.ps.compare_reset = EPL_CMP_RUN;
            epl_sensor.ps.lock = EPL_UN_LOCK;
            epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

			enable_irq(obj->irq);			
        }

        if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER)
        {
            //ALS unlock and run
           	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

			enable_irq(obj->irq);			
        }
       /*PS_GES and HS_ENABLE*/
#if PS_DYN_K
    if(enable_ps == 1)
    {
        epl_sensor_restart_dynk_polling();
    }
#endif

    polling_flag = true;

}


/*----------------------------------------------------------------------------*/
static irqreturn_t epl_sensor_eint_func(int irq, void *desc)
{
    struct epl_sensor_priv *obj = epl259x_obj;

//     printk("--lantao-- epl_sensor_eint_func.\n");

    if(!obj)
        return IRQ_HANDLED;

   int_top_time = sched_clock();
   disable_irq_nosync(epl_sensor_obj->irq);

    schedule_delayed_work(&obj->eint_work, 0);
	return IRQ_HANDLED;
	
}

static void epl_sensor_report_ps_status(void)
{
    int err;

    struct hwm_sensor_data  sensor_data;

#if PS_DEBUG
    APS_FUN();
#endif


    sensor_data.values[0] = epl_sensor.ps.compare_low >> 3;
#if PS_DEBUG
	sensor_data.values[1] = epl_sensor.ps.data.data;
#else
    sensor_data.values[1] = 0;
#endif
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    APS_LOG("[%s]: epl_sensor.ps.data.data=%d, ps_status=%d \r\n", __func__, epl_sensor.ps.data.data, sensor_data.values[0]);
	if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
	{
	    APS_ERR("get interrupt data failed\n");
	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}

}

static void epl_sensor_intr_als_report_lux(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    struct hwm_sensor_data sensor_data;

    int err;

    APS_LOG("[%s]: IDEL MODE \r\n", __func__);
    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

    epl_sensor_read_als(obj->client);

    sensor_data.values[0] = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
	sensor_data.values[1] = epl_sensor.als.data.channels[1];
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

	epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epl_sensor.als.lock = EPL_UN_LOCK;
	epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

	//set dynamic threshold
	if(epl_sensor.als.compare_high >> 4)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;

		if (epl_sensor.als.high_threshold > 60000)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;
		}
	}
	if(epl_sensor.als.compare_low>> 3)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;

		if (epl_sensor.als.high_threshold < 250)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;
		}
	}

    if(epl_sensor.als.high_threshold < epl_sensor.als.low_threshold)
	{
	    APS_LOG("[%s]:recover default setting \r\n", __FUNCTION__);
	    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
	    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;
	}

	if((err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
	{
	    APS_ERR("get interrupt data failed\n");
	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}

	//write new threshold
	set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);
    APS_LOG("[%s]: MODE=0x%x \r\n", __func__, epl_sensor.mode);
}

/*----------------------------------------------------------------------------*/
static void epl_sensor_eint_work(struct work_struct *work)
{
 //   struct hwm_sensor_data sensor_data;
    struct epl_sensor_priv *obj = epl259x_obj;

    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    eint_flag = false;
    APS_LOG("--lantao-- (epl_sensor_eint_work) start.\n");
	
    if(enable_ps && epl_sensor.ps.polling_mode == 0)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps_status(obj->client);
        mutex_unlock(&sensor_mutex);
        if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER && !obj->hw->polling_mode_ps)
        {
#if PS_DEBUG || PS_DYN_K
            mutex_lock(&sensor_mutex);
		    epl_sensor_read_ps(obj->client);
		    mutex_unlock(&sensor_mutex);
#endif
            wake_lock_timeout(&ps_lock, 2*HZ);
            epl_sensor_report_ps_status();
#if PS_DYN_K
            //epl_sensor_reset_dynk_thd(dynk_last_status, (epl_sensor.ps.compare_low >> 3));
            dynk_last_status = (epl_sensor.ps.compare_low >> 3);
#endif
            //if(polling_flag == true)
            //{
                //PS unlock and run
        		epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        		epl_sensor.ps.lock = EPL_UN_LOCK;
        		epl_sensor_I2C_Write(obj->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
            //}
        }

    }
    if(enable_als && obj->hw->polling_mode_als == 0)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_als_status(obj->client);
        mutex_unlock(&sensor_mutex);
        if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER && !obj->hw->polling_mode_als)
        {
		epl_sensor_intr_als_report_lux();
		epl_sensor.als.compare_reset = EPL_CMP_RUN;
		epl_sensor.als.lock = EPL_UN_LOCK;
		epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		}
    }

	eint_flag = true;	
	enable_irq(obj->irq);
}



/*----------------------------------------------------------------------------*/
int epl_sensor_setup_eint(struct i2c_client *client)
{
	struct epl_sensor_priv *obj = i2c_get_clientdata(client);
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};

	struct platform_device *alspsPltFmDev = get_alsps_platformdev();	
	APS_LOG("--lantao--(epl_sensor_setup_eint)  start.\n");
/*configure to GPIO function, external interrupt*/

	epl259x_obj=obj;
	
/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	}	
	pinctrl_select_state(pinctrl, pins_cfg);
/* eint request */
	if (epl_sensor_obj->irq_node) {
//		printk("--lantao--(epl_sensor_obj->irq_node) start.\n ");
		of_property_read_u32_array(epl_sensor_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		//pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG(" --lantao-- ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		epl_sensor_obj->irq = irq_of_parse_and_map(epl_sensor_obj->irq_node, 0);
		APS_LOG("epl_sensor_obj->irq = %d\n", epl_sensor_obj->irq);
		if (!epl_sensor_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		
		if (request_irq(epl_sensor_obj->irq, epl_sensor_eint_func, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq(epl_sensor_obj->irq);
	}
	else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}	
    
    return 0;
}




/*----------------------------------------------------------------------------*/
static int epl_sensor_init_client(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err=0;

    APS_LOG("I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr, __LINE__);

    /*  interrupt mode */


    APS_FUN();

    if(obj->hw->polling_mode_ps == 0)
    {
		enable_irq(obj->irq);	

        if((err = epl_sensor_setup_eint(client)))
 //       if (0)
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl_sensor interrupt setup\n");
    }


    if((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }


    /*  interrupt mode */
    //if(obj->hw->polling_mode_ps == 0)
    //     mt65xx_eint_unmask(CUST_EINT_ALS_NUM);

    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_reg(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct i2c_client *client = epl_sensor_obj->client;

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    if(epl_sensor.als.polling_mode == 0)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x08 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x08));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x09));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0A));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0B));
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x13));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x14 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x14));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x15 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x15));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x16 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x16));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x22 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x22));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x23 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x23));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFC value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFC));

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;

    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;

    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip is %s, ver is %s \n", EPL_DEV_NAME, DRIVER_VERSION);
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps polling is %d-%d\n", epl_sensor.als.polling_mode, epl_sensor.ps.polling_mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, mode = %d\n",epl_sensor.wait >> 4, epl_sensor.mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "interrupt control = %d\n", epl_sensor.interrupt_control >> 4);
    len += snprintf(buf+len, PAGE_SIZE-len, "frame time ps=%dms, als=%dms\n", ps_frame_time, als_frame_time);

    if(enable_ps)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "PS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.ps.integration_time >> 2, epl_sensor.ps.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d, ir drive = %d\n", epl_sensor.ps.adc >> 3, epl_sensor.ps.cycle, epl_sensor.ps.ir_drive);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d, int flag = %d\n", epl_sensor.ps.saturation >> 5, epl_sensor.ps.interrupt_flag >> 2);
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
#if PS_DYN_K
        len += snprintf(buf+len, PAGE_SIZE-len, "Dyn thr(L/H) = (%lu/%lu)\n", dynk_thd_low, dynk_thd_high);
#endif
        len += snprintf(buf+len, PAGE_SIZE-len, "pals data = %d, data = %d\n", epl_sensor.ps.data.ir_data, epl_sensor.ps.data.data);
    }
    if(enable_als)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "ALS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.als.integration_time >> 2, epl_sensor.als.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d\n", epl_sensor.als.adc >> 3, epl_sensor.als.cycle);
    if(epl_sensor.als.polling_mode == 0)
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch0 = %d, ch1 = %d\n", epl_sensor.als.data.channels[0], epl_sensor.als.data.channels[1]);
    }

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();

    sscanf(buf, "%hu", &mode);

    if(enable_als != mode)
    {
        if(mode)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);
        }
        epl_sensor_update_mode(obj->client);
    }

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_FUN();

    sscanf(buf, "%hu", &mode);
    if(enable_ps != mode)
    {
        if(mode)
        {
            //wake_lock(&ps_lock);
            set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
            set_psensor_intr_threshold(65534, 65535);   // dont use first ps status
            dynk_min_ps_raw_data = 0xffff;
            dynk_change_flag = false;

#endif
        }
        else
        {
            clear_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
            cancel_delayed_work(&obj->dynk_thd_polling_work);
#endif
            //wake_unlock(&ps_lock);
        }
        epl_sensor_update_mode(obj->client);
    }
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_cal_raw(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =5;
    int i;
    ssize_t len = 0;
//    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    for(i=0; i<count; i++)
    {
        msleep(50);
        switch(epl_sensor.mode)
        {
            case EPL_MODE_PS:
                ch1 = epl_sensor.ps.data.data;
                break;

            case EPL_MODE_ALS:
                if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
                    epl_sensor_read_als(obj->client);
                ch1 = epl_sensor.als.data.channels[1];
                break;
        }

        ch1_all = ch1_all + ch1;
        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(obj->client, 0x11,  epl_sensor.power | epl_sensor.reset);
    }

    ch1 = (u16)ch1_all/count;

    APS_LOG("cal_raw = %d \r\n" , ch1);

    len += snprintf(buf + len, PAGE_SIZE - len, "%d \r\n", ch1);

    return  len;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_threshold(struct device_driver *ddri,const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int low, high;
    APS_FUN();
    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    sscanf(buf, "%d,%d", &low, &high);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:

            obj->hw->ps_threshold_low = low;
            obj->hw->ps_threshold_high = high;
            epl_sensor.ps.low_threshold = low;
            epl_sensor.ps.high_threshold = high;
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
            break;

        case EPL_MODE_ALS:
            obj->hw->als_threshold_low = low;
            obj->hw->als_threshold_high = high;
            epl_sensor.als.low_threshold = low;
            epl_sensor.als.high_threshold = high;
            set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
            break;

    }

    return  count;
}

static ssize_t epl_sensor_show_threshold(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    ssize_t len = 0;

    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:

            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_low=%d \r\n", obj->hw->ps_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_high=%d \r\n", obj->hw->ps_threshold_high);
            break;

        case EPL_MODE_ALS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_low=%d \r\n", obj->hw->als_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_high=%d \r\n", obj->hw->als_threshold_high);
            break;

    }
    return  len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_wait_time(struct device_driver *ddri, const char *buf, size_t count)
{
    int val;

    sscanf(buf, "%d",&val);

    epl_sensor.wait = (val & 0xf) << 4;

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;
    APS_FUN();

    sscanf(buf, "%d", &value);

    value = value & 0x03;

	switch (epl_sensor.mode)
	{
        case EPL_MODE_PS:

            epl_sensor.ps.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
		break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
		break;

    }

	epl_sensor_update_mode(epld->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int value=0;
    APS_FUN();

    clear_bit(CMC_BIT_PS, &obj->enable);
    clear_bit(CMC_BIT_ALS, &obj->enable);

    sscanf(buf, "%d",&value);

    switch (value)
    {
        case 0:
            epl_sensor.mode = EPL_MODE_IDLE;
            break;

        case 1:
            //set_bit(CMC_BIT_ALS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS;
            break;

        case 2:
            //set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_PS;
            break;

        case 3:
            //set_bit(CMC_BIT_ALS, &obj->enable);
            //set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

    epl_sensor_update_mode(obj->client);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:

            switch(value)
            {
                case 0:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
                    break;

                case 1:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_VOLTAGE;
                    break;
            }

            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
         break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_contrl(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    uint8_t  data;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:

        switch(value)
        {
            case 0:
                epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_OFF;
                break;

            case 1:
                epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
                break;
        }

        data = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
        APS_LOG("%s: 0x05 = 0x%x\n", __FUNCTION__, data);

        epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
         break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_drive(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:

        epl_sensor.ps.ir_drive = (value & 0x03);
        epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
        break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_interrupt_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:

            if(!obj->hw->polling_mode_ps)
            {
                epl_sensor.ps.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
                APS_LOG("%s: 0x06 = 0x%x\n", __FUNCTION__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
            }
            break;

        case EPL_MODE_ALS: //als
            if(!obj->hw->polling_mode_als)
            {
                epl_sensor.als.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
                APS_LOG("%s: 0x07 = 0x%x\n", __FUNCTION__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
            }
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_integration(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:

            epl_sensor.ps.integration_time = (value & 0xf) << 2;
            epl_sensor_I2C_Write(obj->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
            epl_sensor_I2C_Read(obj->client, 0x03, 1);
            APS_LOG("%s: 0x03 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.integration_time | epl_sensor.ps.gain, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.integration_time = (value & 0xf) << 2;
            epl_sensor_I2C_Write(obj->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
            epl_sensor_I2C_Read(obj->client, 0x01, 1);
            APS_LOG("%s: 0x01 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.integration_time | epl_sensor.als.gain, gRawData.raw_bytes[0]);
            break;

    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_adc(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:

            epl_sensor.ps.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

static ssize_t epl_sensor_store_cycle(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {

        case EPL_MODE_PS:

            epl_sensor.ps.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_report_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    APS_FUN();

    sscanf(buf, "%d", &value);
    epl_sensor.als.report_type = value & 0xf;

    return count;
}


static ssize_t epl_sensor_store_ps_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    struct hwmsen_object obj_ps;

    sscanf(buf, "%d",&epld->hw->polling_mode_ps);

    hwmsen_detach(ID_PROXIMITY);

    APS_LOG("epld->hw->polling_mode_ps=%d \r\n", epld->hw->polling_mode_ps);

    epl_sensor.ps.polling_mode = epld->hw->polling_mode_ps;

    obj_ps.self = epl_sensor_obj;
    obj_ps.polling = epld->hw->polling_mode_ps;
    obj_ps.sensor_operate = epl_sensor_ps_operate;

    if(hwmsen_attach(ID_PROXIMITY, &obj_ps))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }

    epl_sensor_update_mode(epld->client);

    return count;
}

static ssize_t epl_sensor_store_als_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    struct hwmsen_object obj_ps;

    sscanf(buf, "%d",&epld->hw->polling_mode_als);

    hwmsen_detach(ID_LIGHT);

    APS_LOG("epld->hw->polling_mode_als=%d \r\n", epld->hw->polling_mode_als);
    epl_sensor.als.polling_mode = epld->hw->polling_mode_als;

    obj_ps.self = epl_sensor_obj;
    obj_ps.polling = epld->hw->polling_mode_als;
    obj_ps.sensor_operate = epl_sensor_als_operate;

    if(hwmsen_attach(ID_LIGHT, &obj_ps))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }

    epl_sensor_update_mode(epld->client);

    return count;
}
#if 0
static ssize_t epl_sensor_store_ps_w_calfile(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0;
    int ps_cal_len = 0;
    char ps_calibration[20];
	APS_FUN();

	if(!epl_sensor_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d,%d",&ps_cancelation, &ps_hthr, &ps_lthr);

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d",  ps_cancelation, ps_hthr, ps_lthr);

    write_factory_calibration(epld, ps_calibration, ps_cal_len);
	return count;
}
#endif
/*----------------------------------------------------------------------------*/

static ssize_t epl_sensor_store_unlock(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int mode;
    APS_FUN();

    sscanf(buf, "%d",&mode);

    APS_LOG("mode = %d \r\n", mode);
	switch (mode)
	{
		case 0: //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 1: //PS unlock and reset
        	epl_sensor.ps.compare_reset = EPL_CMP_RESET;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 2: //ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;

        case 3: //ALS unlock and reset
    		epl_sensor.als.compare_reset = EPL_CMP_RESET;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
        break;

		case 4: //ps+als
		    //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

			//ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;
	}
    /*double check PS or ALS lock*/


	return count;
}

static ssize_t epl_sensor_store_als_ch_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ch_sel;
    APS_FUN();

    sscanf(buf, "%d",&ch_sel);

    APS_LOG("channel selection = %d \r\n", ch_sel);
	switch (ch_sel)
	{
		case 0: //ch0
		    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_0;
		break;

		case 1: //ch1
        	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
		break;
	}
    epl_sensor_I2C_Write(epld->client,0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_update_mode(epld->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_lux_per_count(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%u",&epl_sensor.als.factory.lux_per_count);
    return count;
}

#if 0
static ssize_t epl_sensor_store_ps_cal_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    APS_FUN();

    sscanf(buf, "%s",&epl_sensor.ps.factory.calibration_enable);

    return count;
}
#endif

static ssize_t epl_sensor_store_ps_cancelation(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int cancelation;
    APS_FUN();

    sscanf(buf, "%d",&cancelation);

    epl_sensor.ps.cancelation = cancelation;

    APS_LOG("epl_sensor.ps.cancelation = %d \r\n", epl_sensor.ps.cancelation);

    epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

	return count;
}

static ssize_t epl_sensor_show_ps_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.ps.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_als_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.als.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_ps_run_cali(struct device_driver *ddri, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	ssize_t len = 0;
    int ret;

    APS_FUN();

    ret = epl_run_ps_calibration(epld);

    len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\r\n", ret);

	return len;
}

static ssize_t epl_sensor_show_pdata(struct device_driver *ddri, char *buf)
{
//      struct epl_sensor_priv *epld = epl_sensor_obj;
      ssize_t len = 0;
//      bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
      APS_FUN();

      len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.ps.data.data);
      return len;

}

static ssize_t epl_sensor_show_als_data(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    ssize_t len = 0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;
    APS_FUN();
    
        if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
        {
            mutex_lock(&sensor_mutex);
    	    epl_sensor_read_als(epld->client);
    	    mutex_unlock(&sensor_mutex);
    	}
    	len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.als.data.channels[1]);
    
    return len;

}

#if PS_DYN_K
static ssize_t epl_sensor_store_dyn_offset(struct device_driver *ddri, const char *buf, size_t count)
{
//    struct epl_sensor_priv *epld = epl_sensor_obj;
    int dyn_h,dyn_l;
    APS_FUN();

    sscanf(buf, "%d,%d",&dyn_l, &dyn_h);

    dynk_low_offset = dyn_l;
    dynk_high_offset = dyn_h;

    return count;
}

static ssize_t epl_sensor_store_dyn_thd_offset(struct device_driver *ddri, const char *buf, size_t count)
{
 //   struct epl_sensor_priv *epld = epl_sensor_obj;
    int thd_offset;
    APS_FUN();

    sscanf(buf, "%d",&thd_offset);
    dynk_thd_offset = thd_offset;

    return count;
}

static ssize_t epl_sensor_store_dyn_change_thd_max(struct device_driver *ddri, const char *buf, size_t count)
{
//    struct epl_sensor_priv *epld = epl_sensor_obj;
    int thd_max;
    APS_FUN();

    sscanf(buf, "%d",&thd_max);
    dynk_change_thd_max = thd_max;

    return count;
}

static ssize_t epl_sensor_store_dyn_max_ir_data(struct device_driver *ddri, const char *buf, size_t count)
{
 //   struct epl_sensor_priv *epld = epl_sensor_obj;
    int max_ir_data;
    APS_FUN();

    sscanf(buf, "%d",&max_ir_data);
    dynk_max_ir_data = max_ir_data;

	return count;
}

#endif

static ssize_t epl_sensor_store_reg_write(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int reg;
    int data;
    APS_FUN();

    sscanf(buf, "%x,%x",&reg, &data);

    APS_LOG("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);

    if(reg == 0x00 && ((data & 0x0f) == EPL_MODE_PS || (data & 0x0f) == EPL_MODE_ALS_PS))
    {
        set_bit(CMC_BIT_PS, &epld->enable);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &epld->enable);
    }

    epl_sensor_I2C_Write(epld->client, reg, data);

    return count;
}


static ssize_t epl_sensor_show_renvo(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    APS_FUN();
    APS_LOG("gRawData.renvo=0x%x \r\n", epl_sensor.revno);

    len += snprintf(buf+len, PAGE_SIZE-len, "%x", epl_sensor.revno);

    return len;
}

/*CTS --> S_IWUSR | S_IRUGO*/
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,					S_IWUSR | S_IRUGO, epl_sensor_show_status,  	  		NULL										);
static DRIVER_ATTR(elan_reg,    				S_IWUSR | S_IRUGO, epl_sensor_show_reg,   				NULL										);
static DRIVER_ATTR(als_enable,					S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_als_enable					);
static DRIVER_ATTR(als_report_type,				S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_als_report_type			);
static DRIVER_ATTR(als_polling_mode,			S_IWUSR | S_IRUGO, epl_sensor_show_als_polling,   		epl_sensor_store_als_polling_mode			);
static DRIVER_ATTR(als_lux_per_count,			S_IWUSR | S_IRUGO, NULL,   					 		epl_sensor_store_als_lux_per_count			);
static DRIVER_ATTR(ps_enable,					S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_ps_enable					);
static DRIVER_ATTR(ps_polling_mode,			    S_IWUSR | S_IRUGO, epl_sensor_show_ps_polling,   		epl_sensor_store_ps_polling_mode			);
static DRIVER_ATTR(ir_mode,					    S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_ir_mode					);
static DRIVER_ATTR(ir_drive,					S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_ir_drive					);
static DRIVER_ATTR(ir_on,						S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_ir_contrl					);
static DRIVER_ATTR(interrupt_type,				S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_interrupt_type				);
static DRIVER_ATTR(integration,					S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_integration				);
static DRIVER_ATTR(gain,					    S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_gain					    );
static DRIVER_ATTR(adc,					        S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_adc						);
static DRIVER_ATTR(cycle,						S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_cycle						);
static DRIVER_ATTR(mode,						S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_mode						);
static DRIVER_ATTR(wait_time,					S_IWUSR | S_IRUGO, NULL,   					 		epl_sensor_store_wait_time					);
static DRIVER_ATTR(set_threshold,     			S_IWUSR | S_IRUGO, epl_sensor_show_threshold,                epl_sensor_store_threshold			);
static DRIVER_ATTR(cal_raw, 					S_IWUSR | S_IRUGO, epl_sensor_show_cal_raw, 	  		NULL										);
static DRIVER_ATTR(unlock,				        S_IWUSR | S_IRUGO, NULL,			                    epl_sensor_store_unlock						);
static DRIVER_ATTR(als_ch,				        S_IWUSR | S_IRUGO, NULL,			                    epl_sensor_store_als_ch_sel					);
static DRIVER_ATTR(ps_cancel,				    S_IWUSR | S_IRUGO, NULL,			                    epl_sensor_store_ps_cancelation				);
static DRIVER_ATTR(run_ps_cali, 				S_IWUSR | S_IRUGO, epl_sensor_show_ps_run_cali, 	  	NULL								    	);
static DRIVER_ATTR(pdata,                       S_IWUSR | S_IRUGO, epl_sensor_show_pdata,              NULL                                        );
static DRIVER_ATTR(als_data,                    S_IWUSR | S_IRUGO, epl_sensor_show_als_data,           NULL                                        );
#if PS_DYN_K
static DRIVER_ATTR(dyn_offset,                  S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_dyn_offset                 );
static DRIVER_ATTR(dyn_thd_offset,              S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_dyn_thd_offset             );
static DRIVER_ATTR(dyn_change_max,              S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_dyn_change_thd_max         );
static DRIVER_ATTR(dyn_max_ir_data,             S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_dyn_max_ir_data            );
#endif

static DRIVER_ATTR(i2c_w,                       S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_reg_write                  );
static DRIVER_ATTR(elan_renvo,                  S_IWUSR | S_IRUGO, epl_sensor_show_renvo,              NULL                                        );


/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl_sensor_attr_list[] =
{
    &driver_attr_elan_status,
    &driver_attr_elan_reg,
    &driver_attr_als_enable,
    &driver_attr_als_report_type,
    &driver_attr_als_polling_mode,
    &driver_attr_als_lux_per_count,
    &driver_attr_ps_enable,
    &driver_attr_ps_polling_mode,
    &driver_attr_elan_renvo,

    &driver_attr_mode,
    &driver_attr_ir_mode,
    &driver_attr_ir_drive,
    &driver_attr_ir_on,
    &driver_attr_interrupt_type,
    &driver_attr_cal_raw,
    &driver_attr_set_threshold,
    &driver_attr_wait_time,
    &driver_attr_integration,
    &driver_attr_gain,
    &driver_attr_adc,
    &driver_attr_cycle,
    &driver_attr_unlock,
    &driver_attr_ps_cancel,
    &driver_attr_als_ch,
    &driver_attr_run_ps_cali,
    &driver_attr_pdata,
    &driver_attr_als_data,
#if PS_DYN_K
    &driver_attr_dyn_offset,
    &driver_attr_dyn_thd_offset,
    &driver_attr_dyn_change_max,
    &driver_attr_dyn_max_ir_data,

#endif

    &driver_attr_i2c_w,
};

/*----------------------------------------------------------------------------*/
static int epl_sensor_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, epl_sensor_attr_list[idx])))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl_sensor_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}

/*----------------------------------------------------------------------------*/
static int epl_sensor_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl_sensor_attr_list[idx]);
    }

    return err;
}


/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl_sensor_open(struct inode *inode, struct file *file)
{
    file->private_data = epl_sensor_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl_sensor_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long epl_sensor_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err = 0;
#if 1
    int ps_result;
//	int ps_cali;
	int threshold[2];
#endif
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_LOG("%s cmd = 0x%04x", __FUNCTION__, cmd);
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            if(enable_ps != enable)
            {
                if(enable)
                {
                    //wake_lock(&ps_lock);
                    set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                    set_psensor_intr_threshold(65534, 65535);   //dont use first ps status
                    dynk_min_ps_raw_data = 0xffff;
                    dynk_change_flag = false;

#endif
                }
                else
                {
                    clear_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                    cancel_delayed_work(&obj->dynk_thd_polling_work);
#endif
                    //wake_unlock(&ps_lock);
                }
                epl_sensor_update_mode(obj->client);
            }
            break;

        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA:
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }
#if !PS_DYN_K
            if(polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_ps_status(obj->client);
                mutex_unlock(&sensor_mutex);
            }
#endif
            dat = epl_sensor.ps.compare_low >> 3;

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }
#if !PS_DYN_K
            if(polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_ps(obj->client);
                mutex_unlock(&sensor_mutex);
            }
#endif
            dat = epl_sensor.ps.data.data;

            APS_LOG("ioctl ps raw value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable_als != enable)
            {
                if(enable)
                {
                    set_bit(CMC_BIT_ALS, &obj->enable);
                }
                else
                {
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }
                epl_sensor_update_mode(obj->client);
            }
            break;

        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_DATA:
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }

            if(enable_als == true && polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_als(obj->client);
                mutex_unlock(&sensor_mutex);
            }

            dat = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);

            APS_LOG("ioctl get als data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_RAW_DATA:
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }

            if(enable_als == true && polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_als(obj->client);
                mutex_unlock(&sensor_mutex);
            }
            
                dat = epl_sensor.als.data.channels[1];
                APS_LOG("ioctl get als raw data = %d\n", dat);
            
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }
            if(polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_ps(obj->client);
                mutex_unlock(&sensor_mutex);
            }


            if(epl_sensor.ps.data.data > obj->hw->ps_threshold_high)
			{
			    ps_result = 0;
			}
			else
			    ps_result = 1;

			APS_LOG("[%s] ps_result = %d \r\n", __func__, ps_result);

			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
#if 0 //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

		case ALSPS_IOCTL_CLR_CALI:
#if 0
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
#else

            APS_LOG("[%s]: ALSPS_IOCTL_CLR_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_GET_CALI:
#if 0
			ps_cali = obj->ps_cali ;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
#else
            APS_LOG("[%s]: ALSPS_IOCTL_GET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_SET_CALI:
#if 0
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
#else
            APS_LOG("[%s]: ALSPS_IOCTL_SET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
#if 0
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm
			set_psensor_threshold(obj->client);
#else
            APS_LOG("[%s] set threshold high: %d, low: %d\n", __func__, threshold[0],threshold[1]);
            obj->hw->ps_threshold_high = threshold[0];
            obj->hw->ps_threshold_low = threshold[1];
            set_psensor_intr_threshold(obj->hw->ps_threshold_low, obj->hw->ps_threshold_high);
#endif
			break;
#endif //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		case ALSPS_GET_PS_THRESHOLD_HIGH:
#if 0
			APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_high));
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_high;
            APS_LOG("[%s] get threshold high: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
#if 0
			APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_low));
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_low;
            APS_LOG("[%s] get threshold low: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		/*------------------------------------------------------------------------------------------*/
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}

/*----------------------------------------------------------------------------*/
static struct file_operations epl_sensor_fops =
{
    .owner = THIS_MODULE,
    .open = epl_sensor_open,
    .release = epl_sensor_release,
    .unlocked_ioctl = epl_sensor_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice epl_sensor_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl_sensor_fops,
};

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
//    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
#if 1
    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->als_suspend, 1);

        if(enable_ps == 1){
            atomic_set(&obj->ps_suspend, 0);
            APS_LOG("[%s]: ps enable \r\n", __func__);
        }
        else{
            atomic_set(&obj->ps_suspend, 1);
            APS_LOG("[%s]: ps disable \r\n", __func__);
            epl_sensor_update_mode(obj->client);
        }

        epl_sensor_power(obj->hw, 0);
    }
#endif
    return 0;

}

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_resume(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
//    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
#if 1
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    atomic_set(&obj->ps_suspend, 0);
    atomic_set(&obj->als_suspend, 0);

    if(enable_ps == 1)
    {
        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else
    {
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_update_mode(obj->client);
    }

    epl_sensor_power(obj->hw, 1);
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
                       void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    struct hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_LOG("epl_sensor_ps_operate command = %x\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                APS_LOG("ps enable = %d\n", value);

                if(enable_ps != value)
                {
                    if(value)
                    {
                        //wake_lock(&ps_lock);
                        set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                        set_psensor_intr_threshold(65534, 65535);   //dont use first ps status
                        dynk_min_ps_raw_data = 0xffff;
                        dynk_change_flag = false;

#endif
                    }
                    else
                    {

                        clear_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                        cancel_delayed_work(&obj->dynk_thd_polling_work);
#endif
                        //wake_unlock(&ps_lock);
                    }
                    epl_sensor_update_mode(client);
                }
            }

            break;

        case SENSOR_GET_DATA:
            APS_LOG(" get ps data !!!!!!\n");
            if((buff_out == NULL) || (size_out< sizeof(sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                APS_LOG("---SENSOR_GET_DATA---\n\n");

                if(polling_flag == true && eint_flag == true)
                {
                    mutex_lock(&sensor_mutex);

                    epl_sensor_read_ps_status(client);
                    epl_sensor_read_ps(client);

                    if(epl_sensor.wait == EPL_WAIT_SINGLE)
                        epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);

                    mutex_unlock(&sensor_mutex);
     		    }
     		    else
     		    {

                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running\r\n", __func__, polling_flag, eint_flag);

     		    }
#if PS_DYN_K
                epl_sensor_reset_dynk_thd(dynk_last_status, (epl_sensor.ps.compare_low >> 3));
                dynk_last_status = (epl_sensor.ps.compare_low >> 3);
#endif


//                sensor_data = (hwm_sensor_data*)buff_out;
		  sensor_data = buff_out;
                sensor_data->values[0] = epl_sensor.ps.compare_low >> 3;
                sensor_data->values[1] = epl_sensor.ps.data.data;
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;

            }
            break;

        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;

}

int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
                        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value = 0;
    struct hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
    APS_LOG("epl_sensor_als_operate command = %x\n",command);

    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            break;


        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                APS_LOG("als enable = %d\n", value);

                value = *(int *)buff_in;
                if(enable_als != value)
                {
                    if(value)
                    {
                        set_bit(CMC_BIT_ALS, &obj->enable);

                    }
                    else
                    {
                        clear_bit(CMC_BIT_ALS, &obj->enable);
                    }
                    epl_sensor_update_mode(client);
                }
            }
            break;


        case SENSOR_GET_DATA:
            APS_LOG("get als data !!!!!!\n");

            if((buff_out == NULL) || (size_out< sizeof(sensor_data)) || first_als_report_flag == true)

            {
                APS_ERR("get sensor data parameter error!\n");
				
                if(polling_flag == true && eint_flag == true)

                    first_als_report_flag = false;
                err = -EINVAL;
            }
            else
            {

                if(polling_flag == true && eint_flag == true)

                {
                    mutex_lock(&sensor_mutex);
                    epl_sensor_read_als(client);
                    if(epl_sensor.wait == EPL_WAIT_SINGLE)
                        epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);
                    mutex_unlock(&sensor_mutex);
                }
                else
                {

                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running !\r\n", __func__, polling_flag, eint_flag);

                }
//                sensor_data = (hwm_sensor_data*)buff_out;
		     sensor_data = buff_out;
                if(epl_sensor.als.report_type != CMC_BIT_DYN_INT){
                    sensor_data->values[0] = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                }
                sensor_data->values[1] = epl_sensor.als.data.channels[1];
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                APS_LOG("get als data->values[0] = %d\n", sensor_data->values[0]);
            }
            break;

        default:
            APS_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;

    }

    return err;

}

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, EPL_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl_sensor_priv *obj;
    struct hwmsen_object obj_ps, obj_als;
    int err = 0;
	
#ifdef  Mineloginfo
	printk("--lantao-- (epl_sensor_i2c_probe)  start.\n ");
#endif

    APS_FUN();

    epl_sensor_dumpReg(client);

    if((i2c_smbus_read_byte_data(client, 0x21)) != EPL_REVNO){ //check chip
        APS_LOG("elan ALS/PS sensor is failed. \n");
        err = -1;
        goto exit;
    }

    client->timing = 400;

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl_sensor_obj = obj;
    obj->hw = hw;

    epl_sensor_get_addr(obj->hw, &obj->addr);
    APS_ERR("--lantao-- (epl_sensor_get_addr)  addr is 0x%x!\n", obj->addr.write_addr);

    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));

    INIT_DELAYED_WORK(&obj->eint_work, epl_sensor_eint_work);
#if PS_DYN_K
    INIT_DELAYED_WORK(&obj->dynk_thd_polling_work, epl_sensor_dynk_thd_polling_work);
#endif

    obj->client = client;

    mutex_init(&sensor_mutex);
    wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");

    obj->enable = 0;
    obj->pending_intr = 0;

    epl_sensor_i2c_client = client;

    //initial global variable and write to senosr
    initial_global_variable(client, obj);

    if((err = epl_sensor_init_client(client)))
    {
        goto exit_init_failed;
    }
	APS_LOG("cm36652_init_client() OK!\n");

    if((err = misc_register(&epl_sensor_device)))
    {
        APS_ERR("epl_sensor_device register failed\n");
        goto exit_misc_device_register_failed;
    }
	APS_LOG("cm36652_device misc_register OK!\n");

    if((err = epl_sensor_create_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
	APS_LOG("--lantao --  (epl_sensor_create_attr) OK!\n");

    obj_ps.self = epl_sensor_obj;

    ps_hw = &obj_ps;

    if( obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_ps.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_ps.sensor_operate = epl_sensor_ps_operate;

    if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }
    obj_als.self = epl_sensor_obj;

    als_hw = &obj_als;

    if( obj->hw->polling_mode_als)
    {
        obj_als.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_als.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_als.sensor_operate = epl_sensor_als_operate;
    APS_LOG("als polling mode\n");

    if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
        epl_sensor_setup_eint(client);

	alsps_init_flag = 0;
    APS_LOG("%s: OK\n", __FUNCTION__);
#ifdef  Mineloginfo
	printk("--lantao-- (epl_sensor_i2c_probe)  end.\n ");
#endif
    return 0;

exit_create_attr_failed:
    misc_deregister(&epl_sensor_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
//	exit_kfree:
    kfree(obj);
exit:
    epl_sensor_i2c_client = NULL;

    APS_ERR("%s: err = %d\n", __FUNCTION__, err);
    alsps_init_flag = -1;
    return err;

}

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_remove(struct i2c_client *client)
{
    int err;

    if((err = epl_sensor_delete_attr(&epl_sensor_i2c_driver.driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }

    if((err = misc_deregister(&epl_sensor_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl_sensor_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}

static int alsps_local_init(void)
{
#ifdef  Mineloginfo
	printk("--lantao-- (alsps_local_init)  start.\n ");
#endif
	epl_sensor_power(hw, 1);
#ifdef  Mineloginfo
	printk("--lantao-- (i2c_add_driver)  start.\n ");
#endif
	if(i2c_add_driver(&epl_sensor_i2c_driver)){
#ifdef  Mineloginfo
	printk("--lantao-- (i2c_add_driver)  error.\n ");
#endif		
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == alsps_init_flag)
	   return -1;	
	return 0;
}

static int alsps_remove(void)
{
    APS_FUN();
    epl_sensor_power(hw, 0);
    APS_ERR("epl_sensor remove \n");
    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}




/*----------------------------------------------------------------------------*/
static int __init epl_sensor_init(void)
{
    const char *name = "mediatek,epl259x";
    APS_FUN();

#ifdef  Mineloginfo
	printk("--lantao-- (epl_sensor_init)  start.\n ");
#endif
	hw = get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("get dts info fail\n");
	else
		alsps_driver_add(&epl_sensor_init_info);	
    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit epl_sensor_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(epl_sensor_init);
module_exit(epl_sensor_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL259x ALPsr driver");
MODULE_LICENSE("GPL");

