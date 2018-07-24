#include <linux/spi/spi.h>
#include <linux/module.h>

#define SPI_DEV_NAME "spidev"
struct spi_device *fingerprint_spi;
struct spi_device_id sdev_id = {"spidev", 0};

static int  platform_probe(struct spi_device *spi)
{
	fingerprint_spi = spi;
	return 0;
}

static int  platform_remove(struct spi_device *spi)
{
    return 0;
}

static struct spi_driver platform_spi_driver = {
	.driver = {
		.name =		SPI_DEV_NAME,
		.owner =	THIS_MODULE,
	},
	.probe =	platform_probe,
	.remove =	platform_remove,
	.id_table = &sdev_id,
};

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
		.modalias=SPI_DEV_NAME,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
	},
};
	
static int __init platform_fingerprint_init(void)
{
    int status = 0;

	spi_register_board_info(spi_board_devs,ARRAY_SIZE(spi_board_devs));

    status = spi_register_driver(&platform_spi_driver);
    if (status < 0) {
		printk("Failed to register SPI driver.");
    }
	
	printk("%s %d\n",__FUNCTION__,__LINE__);
	
    return status;
}
module_init(platform_fingerprint_init);

static void __exit platform_fingerprint_exit(void)
{
    spi_unregister_driver(&platform_spi_driver);
}
module_exit(platform_fingerprint_exit);


MODULE_AUTHOR("wheatek.com");
MODULE_DESCRIPTION("for microarray fprint driver");
MODULE_LICENSE("GPL");