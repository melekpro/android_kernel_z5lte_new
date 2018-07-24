#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "ncp1851.h"



/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define NCP1851_SLAVE_ADDR_WRITE   0x6C
#define NCP1851_SLAVE_ADDR_READ	   0x6D

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id ncp1851_i2c_id[] = {{"ncp1851",0},{}};   

kal_bool chargin_hw_init_done = KAL_FALSE;
static int ncp1851_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id ncp1851_of_match[] = {
	{.compatible = "mediatek,SWITHING_CHARGER",},
	{},
};

MODULE_DEVICE_TABLE(of, ncp1854_of_match);
#endif

static struct i2c_driver ncp1851_driver = {
    .driver = {
        .name    = "ncp1851",
#ifdef CONFIG_OF
        .of_match_table = ncp1851_of_match,
#endif            
    },
    .probe       = ncp1851_driver_probe,
    .id_table    = ncp1851_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
#define ncp1851_REG_NUM 19

unsigned char ncp1851_reg[ncp1851_REG_NUM] = {0};

static DEFINE_MUTEX(ncp1851_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write ncp1851]
  *
  *********************************************************/
int ncp1851_read_byte(unsigned char cmd, unsigned char *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&ncp1851_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;
		
        mutex_unlock(&ncp1851_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    //new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
	
    mutex_unlock(&ncp1851_i2c_access);    
    return 1;
}

int ncp1851_write_byte(unsigned char cmd, unsigned char writeData)
{
    char    write_data[2] = {0};
    int    ret=0;

    mutex_lock(&ncp1851_i2c_access);

    write_data[0] = cmd;
    write_data[1] = writeData;

    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
	
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0)
	{
        new_client->ext_flag=0;    
        mutex_unlock(&ncp1851_i2c_access);
        return 0;
    }

    new_client->ext_flag=0;    
    mutex_unlock(&ncp1851_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int ncp1851_read_interface (unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{   
    unsigned char ncp1851_reg = 0;
    int ret = 0;

    printk("--------------------------------------------------\n");

    ret = ncp1851_read_byte(RegNum, &ncp1851_reg);
    printk("[ncp1851_read_interface] Reg[%x]=0x%x\n", RegNum, ncp1851_reg);

    ncp1851_reg &= (MASK << SHIFT);
    *val = (ncp1851_reg >> SHIFT);    
    printk("[ncp1851_read_interface] Val=0x%x\n", *val);

    return ret;
}

unsigned int ncp1851_config_interface (unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char ncp1851_reg = 0;
    int ret = 0;

    printk("--------------------------------------------------\n");

    ret = ncp1851_read_byte(RegNum, &ncp1851_reg);
    //printk("[ncp1851_config_interface] Reg[%x]=0x%x\n", RegNum, ncp1851_reg);

    ncp1851_reg &= ~(MASK << SHIFT);
    ncp1851_reg |= (val << SHIFT);

    ret = ncp1851_write_byte(RegNum, ncp1851_reg);
    //printk("[ncp18516_config_interface] Write Reg[%x]=0x%x\n", RegNum, ncp1851_reg);

    // Check
    //ncp1851_read_byte(RegNum, &ncp1851_reg);
    //printk("[ncp1851_config_interface] Check Reg[%x]=0x%x\n", RegNum, ncp1851_reg);

    return ret;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
//CON0
unsigned int ncp1851_get_chip_status(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON0),
							    (unsigned char*)(&val),
							    (unsigned char)(CON0_STATE_MASK),
							    (unsigned char)(CON0_STATE_SHIFT)
							    );
    return val;
}

unsigned int ncp1851_get_batfet(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON0),
	        					      (unsigned char*)(&val),
							      (unsigned char)(CON0_BATFET_MASK),
							      (unsigned char)(CON0_BATFET_SHIFT)
							      );
    return val;
}

unsigned int ncp1851_get_ntc(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON0),
	        					      (unsigned char*)(&val),
							      (unsigned char)(CON0_NTC_MASK),
							      (unsigned char)(CON0_NTC_SHIFT)
							      );
    return val;
}

unsigned int ncp1851_get_statint(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON0),
	        					      (unsigned char*)(&val),
							      (unsigned char)(CON0_STATINT_MASK),
							      (unsigned char)(CON0_STATINT_SHIFT)
							      );
    return val;
}

unsigned int ncp1851_get_faultint(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON0),
	        					      (unsigned char*)(&val),
							      (unsigned char)(CON0_FAULTINT_MASK),
							      (unsigned char)(CON0_FAULTINT_SHIFT)
							      );
    return val;
}

//CON1
void ncp1851_set_reset(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON1),
								(unsigned char)(val),
								(unsigned char)(CON1_REG_RST_MASK),
								(unsigned char)(CON1_REG_RST_SHIFT)
								);
}

void ncp1851_set_chg_en(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON1),
    								(unsigned char)(val),
    								(unsigned char)(CON1_CHG_EN_MASK),
    								(unsigned char)(CON1_CHG_EN_SHIFT)
    								);
}

void ncp1851_set_otg_en(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON1),
                                                   (unsigned char)(val),
                                                   (unsigned char)(CON1_OTG_EN_MASK),
                                                   (unsigned char)(CON1_OTG_EN_SHIFT)
                                                   );
}

unsigned int ncp1851_get_otg_en(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON1),
	        					      (unsigned char*)(&val),
							      (unsigned char)(CON1_OTG_EN_MASK),
							      (unsigned char)(CON1_OTG_EN_SHIFT)
							      );
    return val;
}

void ncp1851_set_ntc_en(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON1),
    								(unsigned char)(val),
    								(unsigned char)(CON1_NTC_EN_MASK),
    								(unsigned char)(CON1_NTC_EN_SHIFT)
    								);
}

void ncp1851_set_tj_warn_opt(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON1),
                                                   (unsigned char)(val),
                                                   (unsigned char)(CON1_TJ_WARN_OPT_MASK),
                                                   (unsigned char)(CON1_TJ_WARN_OPT_SHIFT)
                                                   );
}

void ncp1851_set_jeita_opt(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON1),
                                                   (unsigned char)(val),
                                                   (unsigned char)(CON1_JEITA_OPT_MASK),
                                                   (unsigned char)(CON1_JEITA_OPT_SHIFT)
                                                   );
}

void ncp1851_set_tchg_rst(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface(	(unsigned char)(NCP1851_CON1),
								(unsigned char)(val),
								(unsigned char)(CON1_TCHG_RST_MASK),
								(unsigned char)(CON1_TCHG_RST_SHIFT)
								);
}

void ncp1851_set_int_mask(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON1),
                                                   (unsigned char)(val),
                                                   (unsigned char)(CON1_INT_MASK_MASK),
                                                   (unsigned char)(CON1_INT_MASK_SHIFT)
                                                   );
}

//CON2
void ncp1851_set_wdto_dis(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_WDTO_DIS_MASK),
								(unsigned char)(CON2_WDTO_DIS_SHIFT)
								);
}

void ncp1851_set_chgto_dis(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_CHGTO_DIS_MASK),
								(unsigned char)(CON2_CHGTO_DIS_SHIFT)
								);
}

void ncp1851_set_pwr_path(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_PWR_PATH_MASK),
								(unsigned char)(CON2_PWR_PATH_SHIFT)
								);
}

void ncp1851_set_trans_en(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_TRANS_EN_MASK),
								(unsigned char)(CON2_TRANS_EN_SHIFT)
								);
}

void ncp1851_set_factory_mode(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_FCTRY_MOD_MASK),
								(unsigned char)(CON2_FCTRY_MOD_SHIFT)
								);
}

void ncp1851_set_iinset_pin_en(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_IINSET_PIN_EN_MASK),
								(unsigned char)(CON2_IINSET_PIN_EN_SHIFT)
								);
}

void ncp1851_set_iinlim_en(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_IINLIM_EN_MASK),
								(unsigned char)(CON2_IINLIM_EN_SHIFT)
								);
}

void ncp1851_set_aicl_en(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON2),
								(unsigned char)(val),
								(unsigned char)(CON2_AICL_EN_MASK),
								(unsigned char)(CON2_AICL_EN_SHIFT)
								);
}

#if 0//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]start
//CON6
unsigned int ncp1851_get_vbusilim(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON6),
	        					      (unsigned char*)(&val),
							      (unsigned char)(CON6_VBUSILIM_MASK),
							      (unsigned char)(CON6_VBUSILIM_SHIFT)
							      );
    return val;
}
#endif//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]end

//CON8
unsigned int ncp1851_get_vfet_ok(void)
{
    unsigned int ret=0;
    unsigned int val=0;

    ret=ncp1851_read_interface((unsigned char)(NCP1851_CON8),
	        					      (unsigned char*)(&val),
							      (unsigned char)(CON8_VFET_OK_MASK),
							      (unsigned char)(CON8_VFET_OK_SHIFT)
							      );
    return val;
}

#if 0//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]start
//CON13
void ncp1851_set_vbusilim_mask(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON13),
								(unsigned char)(val),
								(unsigned char)(CON13_VBUSILIM_MASK),
								(unsigned char)(CON13_VBUSILIM_SHIFT)
								);
}
#endif//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]end

//CON14
void ncp1851_set_ctrl_vbat(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON14),
								(unsigned char)(val),
								(unsigned char)(CON14_CTRL_VBAT_MASK),
								(unsigned char)(CON14_CTRL_VBAT_SHIFT)
								);
}

//CON15
void ncp1851_set_ieoc(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON15),
								(unsigned char)(val),
								(unsigned char)(CON15_IEOC_MASK),
								(unsigned char)(CON15_IEOC_SHIFT)
								);
}

void ncp1851_set_ichg(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON15),
								(unsigned char)(val),
								(unsigned char)(CON15_ICHG_MASK),
								(unsigned char)(CON15_ICHG_SHIFT)
								);
}
unsigned int  ncp1851_get_ichg(void)
{
    unsigned int ret=0;
    unsigned int val=0;

	ret = ncp1851_read_interface((unsigned char)NCP1851_CON15, 
									(unsigned char*)&val, 
									(unsigned char)CON15_ICHG_MASK, 
									(unsigned char)CON15_ICHG_SHIFT);						    
    return val;
}

//CON16
void ncp1851_set_iweak(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON16),
								(unsigned char)(val),
								(unsigned char)(CON16_IWEAK_MASK),
								(unsigned char)(CON16_IWEAK_SHIFT)
								);
}

void ncp1851_set_ctrl_vfet(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON16),
								(unsigned char)(val),
								(unsigned char)(CON16_CTRL_VFET_MASK),
								(unsigned char)(CON16_CTRL_VFET_SHIFT)
								);
}

void ncp1851_set_iinlim(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON16),
								(unsigned char)(val),
								(unsigned char)(CON16_IINLIM_MASK),
								(unsigned char)(CON16_IINLIM_SHIFT)
								);
}

//CON17
void ncp1851_set_vchred(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON17),
								(unsigned char)(val),
								(unsigned char)(CON17_VCHRED_MASK),
								(unsigned char)(CON17_VCHRED_SHIFT)
								);
}

void ncp1851_set_ichred(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON17),
								(unsigned char)(val),
								(unsigned char)(CON17_ICHRED_MASK),
								(unsigned char)(CON17_ICHRED_SHIFT)
								);
}

//CON18
void ncp1851_set_batcold(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON18),
								(unsigned char)(val),
								(unsigned char)(CON18_BATCOLD_MASK),
								(unsigned char)(CON18_BATCOLD_SHIFT)
								);
}

void ncp1851_set_bathot(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON18),
								(unsigned char)(val),
								(unsigned char)(CON18_BATHOT_MASK),
								(unsigned char)(CON18_BATHOT_SHIFT)
								);
}

void ncp1851_set_batchilly(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON18),
								(unsigned char)(val),
								(unsigned char)(CON18_BATCHIL_MASK),
								(unsigned char)(CON18_BATCHIL_SHIFT)
								);
}

void ncp1851_set_batwarm(unsigned int val)
{
    unsigned int ret=0;

    ret=ncp1851_config_interface((unsigned char)(NCP1851_CON18),
								(unsigned char)(val),
								(unsigned char)(CON18_BATWARM_MASK),
								(unsigned char)(CON18_BATWARM_SHIFT)
								);
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void ncp1851_dump_register(void)
{
    int i=0;
    for (i=0;i<ncp1851_REG_NUM;i++)
    {
        if((i == 3) || (i == 4) || (i == 5) || (i == 6)) //do not dump read clear status register
            continue;
        if((i == 10) || (i == 11) || (i == 12) || (i == 13)) //do not dump interrupt mask bit register
            continue;		
        ncp1851_read_byte(i, &ncp1851_reg[i]);
        printk("[ncp1851_dump_register] Reg[0x%X]=0x%X\n", i, ncp1851_reg[i]);
    }
}

void ncp1851_read_register(int i)
{
    ncp1851_read_byte(i, &ncp1851_reg[i]);
    printk("[ncp1851_read_register] Reg[0x%X]=0x%X\n", i, ncp1851_reg[i]);
}

static void ncp1851_parse_customer_setting(void)
{
#ifdef CONFIG_OF
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_drvvbus_init;
	struct pinctrl_state *pinctrl_drvvbus_low;

	/* check customer setting */
	pinctrl = devm_pinctrl_get(&new_client->dev);
	if (IS_ERR(pinctrl)) {
		battery_log(BAT_LOG_CRTI, "[%s]Cannot find drvvbus pinctrl, err=%d\n",
			__func__, (int)PTR_ERR(pinctrl));
		return;
	}

	pinctrl_drvvbus_init = pinctrl_lookup_state(pinctrl, "drvvbus_init");
	if (IS_ERR(pinctrl_drvvbus_init)) {
		battery_log(BAT_LOG_CRTI, "[%s]Cannot find drvvbus_init state, err=%d\n",
			__func__, (int)PTR_ERR(pinctrl_drvvbus_init));
		return;
	}

	pinctrl_drvvbus_low = pinctrl_lookup_state(pinctrl, "drvvbus_low");
	if (IS_ERR(pinctrl_drvvbus_low)) {
		battery_log(BAT_LOG_CRTI, "[%s]Cannot find drvvbus_low state, err=%d\n",
			__func__, (int)PTR_ERR(pinctrl_drvvbus_low));
		return;
	}

	pinctrl_select_state(pinctrl, pinctrl_drvvbus_init);
	pinctrl_select_state(pinctrl, pinctrl_drvvbus_low);
	devm_pinctrl_put(pinctrl);
	battery_log(BAT_LOG_FULL, "[%s]pinctrl_select_state success\n", __func__);
#endif
}

static int ncp1851_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err=0;

    printk("[ncp1851_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
         err = -ENOMEM;
        goto exit;
    }
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;

    chargin_hw_init_done = KAL_TRUE;

    ncp1851_parse_customer_setting();
    //---------------------
    
    return 0;

exit:
    return err;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_ncp1851=0;
static ssize_t show_ncp1851_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    printk("[show_ncp1851_access] 0x%x\n", g_reg_value_ncp1851);
    return sprintf(buf, "%u\n", g_reg_value_ncp1851);
}
static ssize_t store_ncp1851_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;

    printk("[store_ncp1851_access] \n");

    if(buf != NULL && size != 0)
    {
        printk("[store_ncp1851_access] buf is %s and size is %ld \n",buf,size);
        reg_address = simple_strtoul(buf,&pvalue,16);

        if(size > 3)
        {
            reg_value = simple_strtoul((pvalue+1),NULL,16);
            printk("[store_ncp1851_access] write ncp1851 reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=ncp1851_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {
            ret=ncp1851_read_interface(reg_address, &g_reg_value_ncp1851, 0xFF, 0x0);
            printk("[store_ncp1851_access] read ncp1851 reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_ncp1851);
            printk("[store_ncp1851_access] Please use \"cat ncp1851_access\" to get value\r\n");
        }
    }
    return size;
}
static DEVICE_ATTR(ncp1851_access, 0664, show_ncp1851_access, store_ncp1851_access); //664

static int ncp1851_user_space_probe(struct platform_device *dev)
{
    int ret_device_file = 0;

    printk("******** ncp1851_user_space_probe!! ********\n" );

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ncp1851_access);

    return 0;
}

struct platform_device ncp1851_user_space_device = {
    .name   = "ncp1851-user",
    .id	    = -1,
};

static struct platform_driver ncp1851_user_space_driver = {
    .probe		= ncp1851_user_space_probe,
    .driver     = {
        .name = "ncp1851-user",
    },
};

#if 0//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]start
static DECLARE_WAIT_QUEUE_HEAD(ncp1851_int_status_wq);
static atomic_t ncp1851_int_status_flag = ATOMIC_INIT(0);

static void ncp1851_int_status_handler(void)
{
    atomic_set(&ncp1851_int_status_flag, 1);
    wake_up(&ncp1851_int_status_wq);
} 

static int ncp1851_int_status_kthread(void *x)
{
    while(1)
    {
        wait_event(ncp1851_int_status_wq, atomic_read(&ncp1851_int_status_flag));
        atomic_set(&ncp1851_int_status_flag, 0);

        printk("%s!!\n",__func__);

        if(1 == ncp1851_get_faultint())
        {
            if(1 == ncp1851_get_vbusilim())
            {
                ncp1851_set_otg_en(0);
                msleep(5);
                ncp1851_set_otg_en(1);
                ncp1851_set_vbusilim_mask(0x0);
                ncp1851_set_int_mask(0x0);
            }
        }
        
        mt_eint_unmask(CUST_EINT_CHR_STAT_NUM);
    }
}


static void ncp1851_int_setup(void)
{
    mt_set_gpio_mode(GPIO_EINT_CHG_STAT_PIN, GPIO_EINT_CHG_STAT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_EINT_CHG_STAT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_EINT_CHG_STAT_PIN, GPIO_PULL_DISABLE);
    mt_eint_registration(CUST_EINT_CHR_STAT_NUM, CUST_EINT_CHR_STAT_TYPE, ncp1851_int_status_handler, 0);
}

void ncp1851_int_enable(int enable)
{
    printk("%s!! enable :%d\n",__func__,enable);
    
    if(enable){
        ncp1851_set_vbusilim_mask(0x0);
        ncp1851_set_int_mask(0x0);
        mt_eint_unmask(CUST_EINT_CHR_STAT_NUM);
    }
    else{
        ncp1851_set_vbusilim_mask(0x1);
        ncp1851_set_int_mask(0x1);
        mt_eint_mask(CUST_EINT_CHR_STAT_NUM);
    }
}

#endif//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]end

//#ifndef NCP1851_BUSNUM
//#define NCP1851_BUSNUM 1
//#endif

//static struct i2c_board_info __initdata i2c_ncp1851 = { I2C_BOARD_INFO("ncp1851", (0x6c>>1))};

static int __init ncp1851_init(void)
{
    int ret=0;

    printk("[ncp1851_init] init start\n");

//    i2c_register_board_info(NCP1851_BUSNUM, &i2c_ncp1851, 1);

    if(i2c_add_driver(&ncp1851_driver)!=0)
    {
        printk("[ncp1851_init] failed to register ncp1851 i2c driver.\n");
    }
    else
    {
        printk("[ncp1851_init] Success to register ncp1851 i2c driver.\n");
    }

    // ncp1851 user space access interface
    ret = platform_device_register(&ncp1851_user_space_device);
    if (ret) {
        printk("****[ncp1851_init] Unable to device register(%d)\n", ret);
        return ret;
    }
    ret = platform_driver_register(&ncp1851_user_space_driver);
    if (ret) {
        printk("****[ncp1851_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
#if 0//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]start
    kthread_run(ncp1851_int_status_kthread, NULL, "ncp1851_int_status_kthread"); 
    ncp1851_int_setup();
#endif//lisong 2014-8-19 [NO BUGID][fix bug:ncp1851 otg vbus overload]end
    return 0;
}

static void __exit ncp1851_exit(void)
{
    i2c_del_driver(&ncp1851_driver);
}

module_init(ncp1851_init);
module_exit(ncp1851_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C ncp1851 Driver");
MODULE_AUTHOR("YT Lee<yt.lee@mediatek.com>");

