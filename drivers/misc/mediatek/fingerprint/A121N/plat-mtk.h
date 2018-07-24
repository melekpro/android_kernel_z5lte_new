/* MicroArray Fingerprint
 * plat-mtk.h
 * date: 2015-08-20
 * version: v2.0
 * Author: czl
 */

#ifndef PLAT_MTK_H
#define PLAT_MTK_H

#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/of_irq.h>
#include "mt_spi.h"

extern struct spi_device *fingerprint_spi;

extern int ma_finger_get_gpio_info(struct platform_device *pdev);
extern int ma_finger_set_reset(int cmd);
extern int ma_finger_set_eint(int cmd);
//extern int mas_probe(struct spi_device *spi);
extern int mas_probe_init(struct spi_device *spi);
//extern int mas_remove(struct spi_device *spi);
extern int mas_plat_probe(struct platform_device *pdev);
extern int mas_plat_remove(struct platform_device *pdev);

extern struct pinctrl *ma_finger_pinctrl;

#ifdef CONFIG_OF
static struct of_device_id sof_match[] = {
	{ .compatible = "mediatek,ma_finger", },
	{}
};
MODULE_DEVICE_TABLE(of, sof_match);
#endif

#if 0
struct spi_device_id sdev_id = {"madev", 0};
struct spi_driver sdrv = {
	.driver = {
		.name =	"madev",
		.owner = THIS_MODULE,
	},
	.probe = mas_probe,
	.remove = mas_remove,
	.id_table = &sdev_id,
};
#endif

static struct platform_driver spdrv = {
	.probe	  = mas_plat_probe,
	.remove	 = mas_plat_remove,
	.driver = {
		.name  = "madev",
		.owner = THIS_MODULE,			
#ifdef CONFIG_OF
		.of_match_table = sof_match,
#endif
	}
};

struct mt_chip_conf smt_conf = {
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 10, // 10--6m 15--4m 20--3m 30--2m [ 60--1m 120--0.5m  300--0.2m]
	.low_time = 10,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 5,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};


#if 0
struct spi_board_info smt_info[] __initdata = {
	[0] = {
		.modalias = "madev",
		.max_speed_hz = SPI_SPEED,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &smt_conf
	}
};
#endif

void plat_set_gpio(struct platform_device *plat) {		
	ma_finger_get_gpio_info(plat);
	ma_finger_set_reset(1);
	ma_finger_set_eint(1);
}

void plat_free_gpio(void) {
    devm_pinctrl_put(ma_finger_pinctrl);//free resource
}
int plat_register_driver(void) {
	int ret;

	printd("%s: start\n", __func__);
    ret = platform_driver_register(&spdrv);
#if 0	
	spi_register_board_info(smt_info, ARRAY_SIZE(smt_info));
	ret = spi_register_driver(&sdrv);
	if(ret==0) {
		//
	}
#endif
    fingerprint_spi->controller_data = &smt_conf;
	ret = mas_probe_init(fingerprint_spi);

	printd("%s: end.\n", __func__);

	return ret;
}

void plat_unregister_driver(void) {
	//spi_unregister_driver(&sdrv);
}

/* MTK电源开关
 * @power 1:开，0：关
 * @return 0成功，-1失败
 */
int plat_power(int power) {
	int ret = 0;

	// 仅PMU管理电源
//	if(power) {
//		ret = hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3000, MODE_NAME);
//	} else {
//		ret = hwPowerDown(MT6323_POWER_LDO_VGP1, MODE_NAME);
//	}
	return (ret==1)? 0: -1;
}

void plat_set_speed(int speed) {
	int val, num;

	num = speed/1000000;
	num = num>0? num: 1;
	val = 60/num;
	smt_conf.high_time = val;
	smt_conf.low_time = val;
	smt_conf.cs_idletime = val;	
}

void plat_tansfer(struct spi_device *spi, int len) {
	static int mode = -1;
	int tmp = len>32? DMA_TRANSFER: FIFO_TRANSFER;

	//printd("%s: start\n", __func__);

	if(tmp!=mode) {
		struct mt_chip_conf *conf = (struct mt_chip_conf *) spi->controller_data;
		conf->com_mod = tmp;
		spi_setup(spi);
		mode = tmp;
	}

	//printd("%s: end.\n", __func__);
}

void plat_enable_irq(struct spi_device *spi, u8 flag) {
	static int state = -1;

	//printd("%s: start\n", __func__);

	if (state != flag) {
		if (flag) {
			printd("%s: enable_irq.\n", __func__);
			enable_irq(spi->irq);
		} else {
			printd("%s: disable_irq.\n", __func__);
			disable_irq_nosync(spi->irq);
		}
		state = flag;
	} 

	//printd("%s: end.\n", __func__);
}
/*
static struct of_device_id fingerprint_of_match[] = {
	{ .compatible = "mediatek,ma_finger", },
	{},
};
*/

int plat_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
        const char *name, void *dev) {	
	struct device_node *node = NULL;
	const char*tname = "mediatek,ma_finger";
	node = of_find_compatible_node(NULL, NULL, "mediatek,ma_finger");
	//node = of_find_matching_node(node, fingerprint_of_match);
	if (node) {
    	irq = irq_of_parse_and_map(node, 0);
	}
	return request_irq(irq, handler, flags, tname, dev);
}

#endif



