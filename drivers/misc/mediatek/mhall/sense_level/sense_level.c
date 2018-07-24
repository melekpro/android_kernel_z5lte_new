#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/kthread.h>
#include <linux/input.h>

#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <sensors_io.h>

#define KEY_mhall_COVER_OPEN			KEY_F1
#define KEY_mhall_COVER_CLOSE			KEY_F2
#ifdef RGK_mhall_AK8789_SUPPORT
#define MHALL_DEVICE_NAME				"AK8789"
#elif defined(RGK_mhall_OCH165_SUPPORT)
#define MHALL_DEVICE_NAME				"OCH165"
#else
#define MHALL_DEVICE_NAME				"UNKNOWN"
#endif

#define MHALL_DEBUG
#ifdef MHALL_DEBUG
#define MHALL_LOG(fmt, args...)			printk("[MHALL-" MHALL_DEVICE_NAME "] " fmt, ##args)
#else
#define MHALL_LOG(fmt, args...)
#endif

#define MHALL_NAME						"mhall"


static int mhall_flag = 0;
static int mhall_wakeup_flag = 0;
static int mhall_gpio_state = 1;
static int mhall_eint_pin;
static int mhall_irq;

static struct device_node *mhall_node;
static struct input_dev *mhall_input_dev;
static struct pinctrl *pinctrl;
static struct pinctrl_state *pin_init;

static DECLARE_WAIT_QUEUE_HEAD(mhall_thread_wq);

extern unsigned int mt_gpio_to_irq(unsigned int gpio);


#ifdef CONFIG_OF
static const struct of_device_id mhall_of_match[] = {
    { .compatible = "mediatek, mhall", },
    {},
};
#endif

MODULE_DEVICE_TABLE(of, mhall_of_match);

static int mhall_open(struct inode *inode, struct file *file)
{
    return 0; 
}

static int mhall_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long mhall_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    MHALL_LOG("mhall_ioctl\n");
    switch (cmd) 
    {
        case MHALL_IOCTL_GET_STATUS:              
            if (copy_to_user((void __user *)arg, &mhall_flag, sizeof(mhall_flag)))
            {
                MHALL_LOG("mhall_ioctl: copy_to_user failed\n");
                return -EFAULT;
            }
            break;
        default:
            break;
    }
    return 0;
}

static struct file_operations mhall_fops = {
    .owner           = THIS_MODULE,
    .open            = mhall_open,
    .release         = mhall_release,
    .unlocked_ioctl  = mhall_ioctl,
};

static struct miscdevice mhall_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "mhall",
    .fops = &mhall_fops,
};

static ssize_t mhall_show_status(struct device* dev, struct device_attribute *attr, char *buf)
{
    ssize_t res;

    res = snprintf(buf, PAGE_SIZE, "%d\n", mhall_flag);
    return res;    
}

static ssize_t mhall_store_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static DEVICE_ATTR(status, S_IWUSR | S_IRUGO, mhall_show_status, mhall_store_status);


static irqreturn_t mhall_eint_interrupt_handler(int irq, void *desc)
{
    MHALL_LOG("eint interrupt\n");

    mhall_wakeup_flag=1;
    wake_up_interruptible(&mhall_thread_wq);
    disable_irq_nosync(mhall_irq);

    return IRQ_HANDLED;
}


static int mhall_thread_kthread(void *x)
{
	while(1)
	{
		wait_event_interruptible(mhall_thread_wq, mhall_wakeup_flag!=0);
		mhall_wakeup_flag=0;

		mhall_gpio_state = gpio_get_value(mhall_eint_pin);
		MHALL_LOG("gpio value=%d\n", mhall_gpio_state);
		if(mhall_gpio_state == 1) 
		{
			input_report_key(mhall_input_dev, KEY_mhall_COVER_OPEN, 1);
			input_sync(mhall_input_dev);
			msleep(10);
			input_report_key(mhall_input_dev, KEY_mhall_COVER_OPEN, 0);
			input_sync(mhall_input_dev);
			
			mhall_flag = 0;

            irq_set_irq_type(mhall_irq, IRQF_TRIGGER_LOW);

			enable_irq(mhall_irq);

            MHALL_LOG("cover open\n");
		} 
		else 
		{
			input_report_key(mhall_input_dev, KEY_mhall_COVER_CLOSE, 1);
			input_sync(mhall_input_dev);
			msleep(10);
			input_report_key(mhall_input_dev, KEY_mhall_COVER_CLOSE, 0);
			input_sync(mhall_input_dev);

            mhall_flag = 1;

            irq_set_irq_type(mhall_irq, IRQF_TRIGGER_HIGH);
			
			enable_irq(mhall_irq);
			
			MHALL_LOG("cover close\n");
		}
	}
    
	return 0;
}

static int mhall_probe(struct platform_device *pdev) 
{
    int ret;
    u32 ints[2] = { 0, 0 };

    MHALL_LOG("detecetd hall success\n");

    mhall_input_dev = input_allocate_device();
    if(!mhall_input_dev) 
    {
        MHALL_LOG("alloc input device failed\n");
        return -ENOMEM;
    }

    set_bit(EV_KEY, mhall_input_dev->evbit);
    set_bit(KEY_mhall_COVER_OPEN, mhall_input_dev->keybit);
    set_bit(KEY_mhall_COVER_CLOSE, mhall_input_dev->keybit);

    mhall_input_dev->id.bustype = BUS_HOST;
    mhall_input_dev->name = MHALL_NAME;
    ret = input_register_device(mhall_input_dev);
    if(ret) {
        MHALL_LOG("input device register failed\n");
        return ret;
    }
    MHALL_LOG("input register success\n");
    ret = misc_register(&mhall_misc_device);
    if(ret) {
        MHALL_LOG("misc device register failed\n");
        return ret;
    }
    MHALL_LOG("misc device register success\n");
    ret = device_create_file(mhall_misc_device.this_device,&dev_attr_status);
    if(ret) 
    {
        MHALL_LOG("dev attr status register failed\n");
        return ret;
    }
    MHALL_LOG("dev attr status register success\n");

    if (IS_ERR(kthread_run(mhall_thread_kthread, NULL, "mhall_thread_kthread"))) 
    {
        MHALL_LOG("kthread create failed\n");
        return -1;
    }
    MHALL_LOG("kthread create success\n");

    pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pinctrl)) 
    {
        ret = PTR_ERR(pinctrl);
        MHALL_LOG("Cannot find mhall pinctrl!\n");
        return ret;
    }

    pin_init = pinctrl_lookup_state(pinctrl, "state_eint_as_int");
    if (IS_ERR(pin_init)) {
        ret = PTR_ERR(pin_init);
        MHALL_LOG("Cannot find mhall pin_init!\n");
        return ret;
    }

    pinctrl_select_state(pinctrl, pin_init);

    MHALL_LOG("set mhall pin_init success!\n");


    mhall_node = of_find_compatible_node(NULL, NULL, "mediatek,mhall");
    if (mhall_node){
        of_property_read_u32_array(mhall_node, "debounce", ints, ARRAY_SIZE(ints));

        MHALL_LOG("ints[0]=%d,ints[1]=%d\n",ints[0],ints[1]);

        mhall_eint_pin = ints[0];
        gpio_set_debounce(mhall_eint_pin, ints[1]);
        mhall_irq = irq_of_parse_and_map(mhall_node, 0);

        mhall_gpio_state = gpio_get_value(mhall_eint_pin);
        if(mhall_gpio_state)
        {
            mhall_flag = 0;
            ret = request_irq(mhall_irq, mhall_eint_interrupt_handler, IRQF_TRIGGER_LOW, "mhall", NULL); 
            MHALL_LOG("cover init state:open\n");
        }
        else
        {
            mhall_flag = 1;
            ret = request_irq(mhall_irq, mhall_eint_interrupt_handler, IRQF_TRIGGER_HIGH, "mhall", NULL);
            MHALL_LOG("cover init state:close\n");
        }

        if (ret != 0) {
			MHALL_LOG("mhall set irq failed!!\n");
		} else {
			MHALL_LOG("mhall set irq success!!\n");
		}
    }
    else 
    {
        MHALL_LOG("find mhall node failed\n");
    }

    return 0;
}


static int mhall_remove(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver mhall_driver =
{
    .probe      = mhall_probe,
    .remove     = mhall_remove,    
    .driver     = 
    {
        .name = "mhall",
#ifdef CONFIG_OF
        .of_match_table = mhall_of_match,
#endif
    }
};

static int __init mhall_init(void)
{
    if(platform_driver_register(&mhall_driver))
    {
        MHALL_LOG("failed to register driver");
        return -ENODEV;
    }
    return 0;
}


static void __exit mhall_exit(void)
{
    platform_driver_unregister(&mhall_driver);
    return ;
}

module_init(mhall_init);
module_exit(mhall_exit);

MODULE_AUTHOR("juwei.yang@wheatek.com");
MODULE_DESCRIPTION("mhall driver for level sense chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("mhall sense level");
