/*----------------------------------------------------------------
 * Author : Yuvraj Saxena (imyuvraj@github.com)
 * Email : theyuvrajsaxena@gmail.com
 * Note : (Don't remove this block of code from here!)
 * Neither you and nor your any friends or family members will
 * change credits from here! you know the consequences of removing!
 *---------------------------------------------------------------*/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/upmu_hw.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
//#include <string.h>
#else
/* #include <mach/mt_pm_ldo.h> */	/* hwPowerOn */
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>

#include <mt-plat/mt_gpio.h>
#endif

/*  ---------------------------------------------------------------------------
    Local Constants
    --------------------------------------------------------------------------- */

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF

#define LCM_ID 0x91

/* ---------------------------------------------------------------------------
   Local Variables
   --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)  (lcm_util.set_gpio_out((n), (v)))
#define read_reg_v2(cmd, buffer, buffer_size)  lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

/* ---------------------------------------------------------------------------
   Local Functions
   --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xbf, 3, {0x91,0x61,0xf2}},
{REGFLAG_DELAY, 1, {}},
{0xb3, 2, {0x00,0x4c}},
{REGFLAG_DELAY, 1, {}},
{0xb4, 2, {0x00,0x4c}},
{REGFLAG_DELAY, 1, {}},
{0xb8, 6, {0x00,0xbf,0x01,0x00,0xbf,0x01}},
{REGFLAG_DELAY, 1, {}},
{0xba, 3, {0x3e,0x23,0x00}},
{REGFLAG_DELAY, 1, {}},
{0xc3, 1, {0x04}},
{REGFLAG_DELAY, 1, {}},
{0xc4, 2, {0x30,0x6a}},
{REGFLAG_DELAY, 1, {}},
{0xc7, 9, {0x00,0x01,0x31,0x0a,0x6a,0x2c,0x0f,0xa5,0xa5}},
{REGFLAG_DELAY, 1, {}},
{0xc8, 38, {0x7f,0x68,0x55,0x47,0x40,0x30,0x33,0x1d,0x37,0x37,0x37,0x53,0x40,0x45,0x35,0x2b,0x1b,0x0b,0x00,0x7f,0x68,0x55,0x47,0x40,0x30,0x33,0x1d,0x37,0x37,0x37,0x53,0x40,0x45,0x35,0x2b,0x1b,0x0b,0x00}},
{REGFLAG_DELAY, 1, {}},
{0xd4, 16, {0x1f,0x1f,0x1f,0x03,0x01,0x05,0x07,0x09,0x0b,0x11,0x13,0x1f,0x1f,0x1f,0x1f,0x1f}},
{REGFLAG_DELAY, 1, {}},
{0xd5, 16, {0x1f,0x1f,0x1f,0x02,0x00,0x04,0x06,0x08,0x0a,0x10,0x12,0x1f,0x1f,0x1f,0x1f,0x1f}},
{REGFLAG_DELAY, 1, {}},
{0xd6, 16, {0x1f,0x1f,0x1f,0x10,0x12,0x04,0x0a,0x08,0x06,0x02,0x00,0x1f,0x1f,0x1f,0x1f,0x1f}},
{REGFLAG_DELAY, 1, {}},
{0xd7, 16, {0x1f,0x1f,0x1f,0x11,0x13,0x05,0x0b,0x09,0x07,0x03,0x01,0x1f,0x1f,0x1f,0x1f,0x1f}},
{REGFLAG_DELAY, 1, {}},
{0xd8, 20, {0x20,0x00,0x00,0x30,0x03,0x30,0x01,0x02,0x30,0x01,0x02,0x06,0x70,0x73,0x5d,0x72,0x06,0x38,0x70,0x08}},
{REGFLAG_DELAY, 1, {}},
{0xd9, 19, {0x00,0x0a,0x0a,0x88,0x00,0x00,0x06,0x7b,0x00,0x80,0x00,0x3b,0x33,0x1f,0x00,0x00,0x00,0x06,0x70}},
{REGFLAG_DELAY, 1, {}},
{0xbe, 1, {0x01}},
{0xcc, 10, {0x34,0x20,0x38,0x60,0x11,0x91,0x00,0x40,0x00,0x00}},
{0xbe, 1, {0x00}},
{0x11, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 1, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
{0x28, 0, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x10, 0, {0x00}},
{REGFLAG_DELAY, 120, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {

        unsigned int cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }

}

/* ---------------------------------------------------------------------------
   LCM Driver Implementations
   --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = 2;
        params->dsi.LANE_NUM = 2;
        params->dsi.data_format.color_order = 0;
        params->dsi.data_format.trans_seq = 0;
        params->dsi.data_format.padding = 0;
        params->dsi.data_format.format = 2;
        params->dsi.packet_size = 256;
        params->dsi.intermediat_buffer_num = 2;
        params->dsi.PS = 2;
        params->width = 480;
        params->dsi.word_count = 1440;
        params->height = 854;
        params->dsi.vertical_sync_active = 4;
        params->dsi.mode = 1;
        params->dsi.vertical_backporch = 6;
        params->dsi.vertical_frontporch = 6;
        params->dsi.vertical_active_line = 854;
        params->dsi.horizontal_sync_active = 10;
        params->dsi.horizontal_backporch = 10;
        params->dsi.horizontal_frontporch = 10;
        params->dsi.lcm_esd_check_table[0].cmd = 10;
        params->dsi.horizontal_active_pixel = 480;
        params->dsi.PLL_CLOCK = 155;
        params->dsi.ssc_disable = 1;
        params->dsi.esd_check_enable = 1;
        params->dsi.customization_esd_check_enable = 1;
        params->dsi.lcm_esd_check_table[0].count = 1;
        params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
        params->dsi.noncont_clock = 1;
}

static void lcm_init(void)
{
	MDELAY(100);
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(30);
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	MDELAY(100);
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(30);
	SET_RESET_PIN(1);
        MDELAY(120);
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(20);
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

/* ---------------------------------------------------------------------------
   Get LCM Driver Hooks
   --------------------------------------------------------------------------- */

LCM_DRIVER JD9161_HONGZHAN_TN_HFW50194QRA_IVO_DSI_2_VDO_FWVGA_lcm_drv =
{
        .name		= "JD9161_HONGZHAN_TN_HFW50194QRA_IVO_DSI_2_VDO_FWVGA",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
