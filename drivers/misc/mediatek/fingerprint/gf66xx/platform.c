#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <mach/irqs.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
//#include <mach/mt_gpio.h>
#include <linux/delay.h>
//#include <mach/eint.h>
//#include <cust_gpio_usage.h>
//#include <cust_eint.h>
#include <linux/io.h>          
#include "gf66xx-spi.h"
#include <linux/gpio.h>
//#include <mach/mt_pm_ldo.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
extern struct platform_device *fingerprint_device;
struct pinctrl *pinctrl_spi;
struct pinctrl_state *pins_spi_default;
struct pinctrl_state *pins_spi_cs_set, *pins_spi_clk_set; 
struct pinctrl_state *pins_spi_miso_set, *pins_spi_mosi_set;
struct pinctrl_state *pins_spi_int_init, *pins_spi_int_low, *pins_spi_int_high;
struct pinctrl_state *pins_spi_rst_high, *pins_spi_rst_low;
static struct of_device_id fingerprint_of_match[] = {
	{ .compatible = "mediatek,fingerprint",},
	{},
};
int gf66xx_parse_dts(struct gf66xx_dev *gf66xx_dev)
{
    int retval = 0;
    return retval;
}

struct regulator *reg = NULL;
int gf66xx_power_on(struct gf66xx_dev *gf66xx_dev,int power_on)
{
	int ret = 0;
	printk("...power_on = %d %s %d\n",power_on,__func__,__LINE__);
	if(power_on)
	{
		//2.8V
		reg = regulator_get(&(fingerprint_device->dev),"vfingerprint");
		ret = regulator_set_voltage(reg, 2800000, 2800000);
		if(ret)
			printk("...regulator_set_voltage failed!\n");
		ret = regulator_enable(reg);	/*enable regulator*/
		if (ret)
			printk("...regulator enable\n");
		msleep(100);

	}
	else
	{
		if(reg != NULL)
		{
			regulator_disable(reg);
		}
		else
			printk("...reg is NULL !!! something error???\n");
	}
	return 0;
}


#define FINGER_PRINT_EINT_STRING "FINGERPRINT-eint"
int gf66xx_irq_setup(struct gf66xx_dev *gf66xx_dev, void (*irq_handler)(void )) //need change yfpan
{
    struct device_node *node = NULL;
	int ret = 0;
	unsigned int fingerprint_irq = 0;
	u32 ints[2] = { 0, 0 };
	
	pinctrl_select_state(pinctrl_spi, pins_spi_int_init);
	
	node = of_find_matching_node(node, fingerprint_of_match);
	msleep(20);

	if (node) 
	{
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		fingerprint_irq = irq_of_parse_and_map(node, 0);
	
		msleep(30);
		printk("...fingerprint_irq = %d\n",fingerprint_irq);	
		gf66xx_dev->spi->irq = fingerprint_irq;	
		ret = request_irq(fingerprint_irq, (irq_handler_t)irq_handler, IRQF_TRIGGER_RISING,FINGER_PRINT_EINT_STRING, NULL);
		if (ret > 0)
		{
			ret = -1;
		}
		
	} 
	else
	{
		ret = -1;
	}
	printk("....%s %d ret = %d\n",__func__,__LINE__,ret);

	return ret; 
}

static int spi_get_gpio_info(struct platform_device *pdev)
{
	int ret;

	/*SPI_DBG("spi_get_gpio_info pin ctrl===================\n");*/
	pinctrl_spi = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl_spi)) {
		ret = PTR_ERR(pinctrl_spi);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl1!\n");
		return ret;
	}
	pins_spi_default = pinctrl_lookup_state(pinctrl_spi, "default");
	if (IS_ERR(pins_spi_default)) {
		ret = PTR_ERR(pins_spi_default);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_default!\n");
		//return ret;
	}
	pins_spi_cs_set = pinctrl_lookup_state(pinctrl_spi, "state_spi_cs_init");
	if (IS_ERR(pins_spi_cs_set)) {
		ret = PTR_ERR(pins_spi_cs_set);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_cs_set!\n");
		return ret;
	}

	pins_spi_clk_set = pinctrl_lookup_state(pinctrl_spi, "state_spi_ck_init");
	if (IS_ERR(pins_spi_clk_set)) {
		ret = PTR_ERR(pins_spi_clk_set);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_clk_set!\n");
		return ret;
	}

	pins_spi_miso_set = pinctrl_lookup_state(pinctrl_spi, "state_spi_miso_init");
	if (IS_ERR(pins_spi_miso_set)) {
		ret = PTR_ERR(pins_spi_miso_set);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_miso_set!\n");
		return ret;
	}

	pins_spi_mosi_set = pinctrl_lookup_state(pinctrl_spi, "state_spi_mosi_init");
	if (IS_ERR(pins_spi_mosi_set)) {
		ret = PTR_ERR(pins_spi_mosi_set);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_mosi_set!\n");
		return ret;
	}
	//int
	pins_spi_int_init = pinctrl_lookup_state(pinctrl_spi, "state_spi_eint_init");
	if (IS_ERR(pins_spi_int_init)) {
		ret = PTR_ERR(pins_spi_int_init);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_int_init!\n");
		return ret;
	}
	
	pins_spi_int_low = pinctrl_lookup_state(pinctrl_spi, "state_spi_eint_low");
	if (IS_ERR(pins_spi_int_low)) {
		ret = PTR_ERR(pins_spi_int_low);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_int_low!\n");
		return ret;
	}
	pins_spi_int_high = pinctrl_lookup_state(pinctrl_spi, "state_spi_eint_high");
	if (IS_ERR(pins_spi_int_high)) {
		ret = PTR_ERR(pins_spi_int_high);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_int_high!\n");
		return ret;
	}
	//rst
	pins_spi_rst_low = pinctrl_lookup_state(pinctrl_spi, "state_spi_rst_low");
	if (IS_ERR(pins_spi_rst_low)) {
		ret = PTR_ERR(pins_spi_rst_low);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_rst_low!\n");
		return ret;
	}
	pins_spi_rst_high = pinctrl_lookup_state(pinctrl_spi, "state_spi_rst_high");
	if (IS_ERR(pins_spi_rst_high)) {
		ret = PTR_ERR(pins_spi_rst_high);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl pins_spi_rst_high!\n");
		return ret;
	}
	return 0;
}

void gf66xx_spi_pins_config(void)
{
	spi_get_gpio_info(fingerprint_device);
	pinctrl_select_state(pinctrl_spi, pins_spi_cs_set);
	pinctrl_select_state(pinctrl_spi, pins_spi_clk_set);
	pinctrl_select_state(pinctrl_spi, pins_spi_miso_set);
	pinctrl_select_state(pinctrl_spi, pins_spi_mosi_set);
    msleep(1);
}

int gf66xx_irq_open(int irq_no)
{
	enable_irq(irq_no);   
    return 0;
}
int gf66xx_irq_close(int irq_no)
{
	disable_irq(irq_no);  
	return 0;
}

int gf66xx_irq_release(struct gf66xx_dev *gf66xx_dev)
{
    int retval = 0;
	#if 0
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }

//	mt_eint_mask(gf66xx_dev->spi->irq);   
	mt_set_gpio_pull_enable(gf66xx_dev->irq_gpio, 0);
	mt_set_gpio_pull_select(gf66xx_dev->irq_gpio,  0);
	retval = mt_set_gpio_mode(gf66xx_dev->irq_gpio, GPIO_SPI_EINT_PIN_M_GPIO); //set to eint MODE for enable eint function
    if(retval < 0) {
        pr_info("Failed to set IRQ pin as GPIO. retval = %d\n", retval);
    } else {
	    pr_info("SPI GPIO EINT PIN mode:num:%lx, %d, dir:%d,pullen:%d,pullup%d\n",gf66xx_dev->irq_gpio,
				mt_get_gpio_mode(gf66xx_dev->irq_gpio),mt_get_gpio_dir(gf66xx_dev->irq_gpio),
				mt_get_gpio_pull_enable(gf66xx_dev->irq_gpio),mt_get_gpio_pull_select(gf66xx_dev->irq_gpio));    
    }
	#endif
    return retval;
}
int gf66xx_hw_reset(struct gf66xx_dev *gf66xx_dev, unsigned int delay_ms) 
{
    if(gf66xx_dev == NULL) {
        pr_info("%s. input buffer is NULL.\n", __func__);
        return -1;
    }
	pinctrl_select_state(pinctrl_spi, pins_spi_rst_high);
	pinctrl_select_state(pinctrl_spi, pins_spi_rst_low);
	mdelay(5);
	pinctrl_select_state(pinctrl_spi, pins_spi_rst_high);
    msleep(delay_ms);
    return 0;
}




