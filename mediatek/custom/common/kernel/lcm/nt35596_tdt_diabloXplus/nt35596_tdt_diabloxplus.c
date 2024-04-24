

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID_NT35596 (0x96)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)	        							(lcm_util.set_gpio_out((n), (v)))
#define LCD_LDO_ENN_GPIO_PIN          							(GPIO_LCD_ENN_PIN)//168
#define LCD_LDO_ENP_GPIO_PIN          							(GPIO_LCD_ENP_PIN)//mocku=20;proto=83


#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY 									0X101  //0xFE
#define REGFLAG_END_OF_TABLE 							0X100 //0xFD // END OF REGISTERS MARKER


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define dsi_lcm_set_gpio_out(pin, out)										lcm_util.set_gpio_out(pin, out)
#define dsi_lcm_set_gpio_mode(pin, mode)									lcm_util.set_gpio_mode(pin, mode)
#define dsi_lcm_set_gpio_dir(pin, dir)										lcm_util.set_gpio_dir(pin, dir)
#define dsi_lcm_set_gpio_pull_enable(pin, en)								lcm_util.set_gpio_pull_enable(pin, en)

#define   LCM_DSI_CMD_MODE							0

static bool lcm_is_init = false;

struct LCM_setting_table {
unsigned char cmd;
unsigned char count;
unsigned char para_list[64];
};

void TC358768_DCS_write_1A_1P(unsigned char cmd, unsigned char para)
{
	unsigned int data_array[16];
	//unsigned char buffer;

#if 0//ndef BUILD_LK

	do {
		data_array[0] =(0x00001500 | (para<<24) | (cmd<<16));
		dsi_set_cmdq(data_array, 1, 1);

		if (cmd == 0xFF)
			break;

		read_reg_v2(cmd, &buffer, 1);

		if(buffer != para)
			printk("%s, data_array = 0x%08x, (cmd, para, back) = (0x%02x, 0x%02x, 0x%02x)\n", __func__, data_array[0], cmd, para, buffer);	

		MDELAY(1);

	} while (buffer != para);

#else

	data_array[0] =(0x00001500 | (para<<24) | (cmd<<16));
	dsi_set_cmdq(data_array, 1, 1);

	//MDELAY(1);

#endif

}

void TC358768_DCS_write_1A_0P(unsigned char cmd)
{
    unsigned int data_array[16];
    
    data_array[0]=(0x00000500 | (cmd<<16));
    dsi_set_cmdq(data_array, 1, 1);

}																																									

static struct LCM_setting_table lcm_initialization_ret[] = {
    /*Note :
    Data ID will depends on the following rule.
    count of parameters > 1	=> Data ID = 0x39
    count of parameters = 1	=> Data ID = 0x15
    count of parameters = 0	=> Data ID = 0x05
    Structure Format :
    {DCS command, count of parameters, {parameter list}}
    {REGFLAG_DELAY, milliseconds of time, {}},
    ...
    Setting ending by predefined flag
    {REGFLAG_END_OF_TABLE, 0x00, {}}
    */




    //NT35596 + LGD6" Panel Initial Code

    //CMD2 Page 0
    {0xFF,1,{0x01}},
    {REGFLAG_DELAY, 1, {0}},

    {0xFB,1,{0x01}},



    //ccmon

    //Power-Related

    {0x00,1,{0x01}},
    {0x01,1,{0x55}},
    {0x02,1,{0x45}},
    {0x03,1,{0x55}},
    {0x05,1,{0x50}},

    //VGH/VGL w/Clamp +/- 9V
    {0x14,1,{0x9E}},
    {0x07,1,{0xA8}},
    {0x08,1,{0x0C}},

    //Gamma Voltage +/-4.35V
    {0x0B,1,{0x96}},
    {0x0C,1,{0x96}},

    //VGHO/VGLO Disable
    {0x0E,1,{0x00}},
    {0x0F,1,{0x00}},

    {0x11,1,{0x27}},
    {0x12,1,{0x27}},
    {0x13,1,{0x03}},
    {0x06,1,{0x0A}},
    {0x15,1,{0x99}},
    {0x16,1,{0x99}},
    {0x1B,1,{0x39}},
    {0x1C,1,{0x39}},
    {0x1D,1,{0x47}},

    //ID code
    {0x44,1,{0xC1}},
    {0x45,1,{0x86}},
    {0x46,1,{0xC4}},

    //Gate EQ
    {0x58,1,{0x05}},
    {0x59,1,{0x05}},
    {0x5A,1,{0x05}},
    {0x5B,1,{0x05}},
    {0x5C,1,{0x00}},
    {0x5D,1,{0x00}},
    {0x5E,1,{0x00}},
    {0x5F,1,{0x00}},


    //ISOP P/N (Normal Mode)
    {0x6D,1,{0x44}},


    //Gamma R +/-

    {0x75,1,{0x00}},
    {0x76,1,{0x00}},
    {0x77,1,{0x00}},
    {0x78,1,{0x22}},
    {0x79,1,{0x00}},
    {0x7A,1,{0x46}},
    {0x7B,1,{0x00}},
    {0x7C,1,{0x5C}},
    {0x7D,1,{0x00}},
    {0x7E,1,{0x76}},
    {0x7F,1,{0x00}},
    {0x80,1,{0x8D}},
    {0x81,1,{0x00}},
    {0x82,1,{0xA6}},
    {0x83,1,{0x00}},
    {0x84,1,{0xB8}},
    {0x85,1,{0x00}},
    {0x86,1,{0xC7}},
    {0x87,1,{0x00}},
    {0x88,1,{0xF6}},
    {0x89,1,{0x01}},
    {0x8A,1,{0x1D}},
    {0x8B,1,{0x01}},
    {0x8C,1,{0x54}},
    {0x8D,1,{0x01}},
    {0x8E,1,{0x81}},
    {0x8F,1,{0x01}},
    {0x90,1,{0xCB}},
    {0x91,1,{0x02}},
    {0x92,1,{0x05}},
    {0x93,1,{0x02}},
    {0x94,1,{0x07}},
    {0x95,1,{0x02}},
    {0x96,1,{0x47}},
    {0x97,1,{0x02}},
    {0x98,1,{0x82}},
    {0x99,1,{0x02}},
    {0x9A,1,{0xAB}},
    {0x9B,1,{0x02}},
    {0x9C,1,{0xDC}},
    {0x9D,1,{0x03}},
    {0x9E,1,{0x01}},
    {0x9F,1,{0x03}},
    {0xA0,1,{0x3A}},
    {0xA2,1,{0x03}},
    {0xA3,1,{0x56}},
    {0xA4,1,{0x03}},
    {0xA5,1,{0x6D}},
    {0xA6,1,{0x03}},
    {0xA7,1,{0x89}},
    {0xA9,1,{0x03}},
    {0xAA,1,{0xA3}},
    {0xAB,1,{0x03}},
    {0xAC,1,{0xC9}},
    {0xAD,1,{0x03}},
    {0xAE,1,{0xDD}},
    {0xAF,1,{0x03}},
    {0xB0,1,{0xF5}},
    {0xB1,1,{0x03}},
    {0xB2,1,{0xFF}},
    {0xB3,1,{0x00}},
    {0xB4,1,{0x00}},
    {0xB5,1,{0x00}},
    {0xB6,1,{0x22}},
    {0xB7,1,{0x00}},
    {0xB8,1,{0x46}},
    {0xB9,1,{0x00}},
    {0xBA,1,{0x5C}},
    {0xBB,1,{0x00}},
    {0xBC,1,{0x76}},
    {0xBD,1,{0x00}},
    {0xBE,1,{0x8D}},
    {0xBF,1,{0x00}},
    {0xC0,1,{0xA6}},
    {0xC1,1,{0x00}},
    {0xC2,1,{0xB8}},
    {0xC3,1,{0x00}},
    {0xC4,1,{0xC7}},
    {0xC5,1,{0x00}},
    {0xC6,1,{0xF6}},
    {0xC7,1,{0x01}},
    {0xC8,1,{0x1D}},
    {0xC9,1,{0x01}},
    {0xCA,1,{0x54}},
    {0xCB,1,{0x01}},
    {0xCC,1,{0x81}},
    {0xCD,1,{0x01}},
    {0xCE,1,{0xCB}},
    {0xCF,1,{0x02}},
    {0xD0,1,{0x05}},
    {0xD1,1,{0x02}},
    {0xD2,1,{0x07}},
    {0xD3,1,{0x02}},
    {0xD4,1,{0x47}},
    {0xD5,1,{0x02}},
    {0xD6,1,{0x82}},
    {0xD7,1,{0x02}},
    {0xD8,1,{0xAB}},
    {0xD9,1,{0x02}},
    {0xDA,1,{0xDC}},
    {0xDB,1,{0x03}},
    {0xDC,1,{0x01}},
    {0xDD,1,{0x03}},
    {0xDE,1,{0x3A}},
    {0xDF,1,{0x03}},
    {0xE0,1,{0x56}},
    {0xE1,1,{0x03}},
    {0xE2,1,{0x6D}},
    {0xE3,1,{0x03}},
    {0xE4,1,{0x89}},
    {0xE5,1,{0x03}},
    {0xE6,1,{0xA3}},
    {0xE7,1,{0x03}},
    {0xE8,1,{0xC9}},
    {0xE9,1,{0x03}},
    {0xEA,1,{0xDD}},
    {0xEB,1,{0x03}},
    {0xEC,1,{0xF5}},
    {0xED,1,{0x03}},
    {0xEE,1,{0xFF}},


    //Gamma G+

    {0xEF,1,{0x00}},
    {0xF0,1,{0x00}},
    {0xF1,1,{0x00}},
    {0xF2,1,{0x22}},
    {0xF3,1,{0x00}},
    {0xF4,1,{0x46}},
    {0xF5,1,{0x00}},
    {0xF6,1,{0x5C}},
    {0xF7,1,{0x00}},
    {0xF8,1,{0x76}},
    {0xF9,1,{0x00}},
    {0xFA,1,{0x8D}},

    //ccmoff
    //ccmrun

    //CMD2 Page 1
    {0xFF,1,{0x02}},
    {REGFLAG_DELAY, 1, {0}},

    {0xFB,1,{0x01}},

    //ccmon
    //Gamma G+/-

    {0x00,1,{0x00}},
    {0x01,1,{0xA6}},
    {0x02,1,{0x00}},
    {0x03,1,{0xB8}},
    {0x04,1,{0x00}},
    {0x05,1,{0xC7}},
    {0x06,1,{0x00}},
    {0x07,1,{0xF6}},
    {0x08,1,{0x01}},
    {0x09,1,{0x1D}},
    {0x0A,1,{0x01}},
    {0x0B,1,{0x54}},
    {0x0C,1,{0x01}},
    {0x0D,1,{0x81}},
    {0x0E,1,{0x01}},
    {0x0F,1,{0xCB}},
    {0x10,1,{0x02}},
    {0x11,1,{0x05}},
    {0x12,1,{0x02}},
    {0x13,1,{0x07}},
    {0x14,1,{0x02}},
    {0x15,1,{0x47}},
    {0x16,1,{0x02}},
    {0x17,1,{0x82}},
    {0x18,1,{0x02}},
    {0x19,1,{0xAB}},
    {0x1A,1,{0x02}},
    {0x1B,1,{0xDC}},
    {0x1C,1,{0x03}},
    {0x1D,1,{0x01}},
    {0x1E,1,{0x03}},
    {0x1F,1,{0x3A}},
    {0x20,1,{0x03}},
    {0x21,1,{0x56}},
    {0x22,1,{0x03}},
    {0x23,1,{0x6D}},
    {0x24,1,{0x03}},
    {0x25,1,{0x89}},
    {0x26,1,{0x03}},
    {0x27,1,{0xA3}},
    {0x28,1,{0x03}},
    {0x29,1,{0xC9}},
    {0x2A,1,{0x03}},
    {0x2B,1,{0xDD}},
    {0x2D,1,{0x03}},
    {0x2F,1,{0xF5}},
    {0x30,1,{0x03}},
    {0x31,1,{0xFF}},
    {0x32,1,{0x00}},
    {0x33,1,{0x00}},
    {0x34,1,{0x00}},
    {0x35,1,{0x22}},
    {0x36,1,{0x00}},
    {0x37,1,{0x46}},
    {0x38,1,{0x00}},
    {0x39,1,{0x5C}},
    {0x3A,1,{0x00}},
    {0x3B,1,{0x76}},
    {0x3D,1,{0x00}},
    {0x3F,1,{0x8D}},
    {0x40,1,{0x00}},
    {0x41,1,{0xA6}},
    {0x42,1,{0x00}},
    {0x43,1,{0xB8}},
    {0x44,1,{0x00}},
    {0x45,1,{0xC7}},
    {0x46,1,{0x00}},
    {0x47,1,{0xF6}},
    {0x48,1,{0x01}},
    {0x49,1,{0x1D}},
    {0x4A,1,{0x01}},
    {0x4B,1,{0x54}},
    {0x4C,1,{0x01}},
    {0x4D,1,{0x81}},
    {0x4E,1,{0x01}},
    {0x4F,1,{0xCB}},
    {0x50,1,{0x02}},
    {0x51,1,{0x05}},
    {0x52,1,{0x02}},
    {0x53,1,{0x07}},
    {0x54,1,{0x02}},
    {0x55,1,{0x47}},
    {0x56,1,{0x02}},
    {0x58,1,{0x82}},
    {0x59,1,{0x02}},
    {0x5A,1,{0xAB}},
    {0x5B,1,{0x02}},
    {0x5C,1,{0xDC}},
    {0x5D,1,{0x03}},
    {0x5E,1,{0x01}},
    {0x5F,1,{0x03}},
    {0x60,1,{0x3A}},
    {0x61,1,{0x03}},
    {0x62,1,{0x56}},
    {0x63,1,{0x03}},
    {0x64,1,{0x6D}},
    {0x65,1,{0x03}},
    {0x66,1,{0x89}},
    {0x67,1,{0x03}},
    {0x68,1,{0xA3}},
    {0x69,1,{0x03}},
    {0x6A,1,{0xC9}},
    {0x6B,1,{0x03}},
    {0x6C,1,{0xDD}},
    {0x6D,1,{0x03}},
    {0x6E,1,{0xF5}},
    {0x6F,1,{0x03}},
    {0x70,1,{0xFF}},

    //Gamma B+/-

    {0x71,1,{0x00}},
    {0x72,1,{0x00}},
    {0x73,1,{0x00}},
    {0x74,1,{0x22}},
    {0x75,1,{0x00}},
    {0x76,1,{0x46}},
    {0x77,1,{0x00}},
    {0x78,1,{0x5C}},
    {0x79,1,{0x00}},
    {0x7A,1,{0x76}},
    {0x7B,1,{0x00}},
    {0x7C,1,{0x8D}},
    {0x7D,1,{0x00}},
    {0x7E,1,{0xA6}},
    {0x7F,1,{0x00}},
    {0x80,1,{0xB8}},
    {0x81,1,{0x00}},
    {0x82,1,{0xC7}},
    {0x83,1,{0x00}},
    {0x84,1,{0xF6}},
    {0x85,1,{0x01}},
    {0x86,1,{0x1D}},
    {0x87,1,{0x01}},
    {0x88,1,{0x54}},
    {0x89,1,{0x01}},
    {0x8A,1,{0x81}},
    {0x8B,1,{0x01}},
    {0x8C,1,{0xCB}},
    {0x8D,1,{0x02}},
    {0x8E,1,{0x05}},
    {0x8F,1,{0x02}},
    {0x90,1,{0x07}},
    {0x91,1,{0x02}},
    {0x92,1,{0x47}},
    {0x93,1,{0x02}},
    {0x94,1,{0x82}},
    {0x95,1,{0x02}},
    {0x96,1,{0xAB}},
    {0x97,1,{0x02}},
    {0x98,1,{0xDC}},
    {0x99,1,{0x03}},
    {0x9A,1,{0x01}},
    {0x9B,1,{0x03}},
    {0x9C,1,{0x3A}},
    {0x9D,1,{0x03}},
    {0x9E,1,{0x56}},
    {0x9F,1,{0x03}},
    {0xA0,1,{0x6D}},
    {0xA2,1,{0x03}},
    {0xA3,1,{0x89}},
    {0xA4,1,{0x03}},
    {0xA5,1,{0xA3}},
    {0xA6,1,{0x03}},
    {0xA7,1,{0xC9}},
    {0xA9,1,{0x03}},
    {0xAA,1,{0xDD}},
    {0xAB,1,{0x03}},
    {0xAC,1,{0xF5}},
    {0xAD,1,{0x03}},
    {0xAE,1,{0xFF}},
    {0xAF,1,{0x00}},
    {0xB0,1,{0x00}},
    {0xB1,1,{0x00}},
    {0xB2,1,{0x22}},
    {0xB3,1,{0x00}},
    {0xB4,1,{0x46}},
    {0xB5,1,{0x00}},
    {0xB6,1,{0x5C}},
    {0xB7,1,{0x00}},
    {0xB8,1,{0x76}},
    {0xB9,1,{0x00}},
    {0xBA,1,{0x8D}},
    {0xBB,1,{0x00}},
    {0xBC,1,{0xA6}},
    {0xBD,1,{0x00}},
    {0xBE,1,{0xB8}},
    {0xBF,1,{0x00}},
    {0xC0,1,{0xC7}},
    {0xC1,1,{0x00}},
    {0xC2,1,{0xF6}},
    {0xC3,1,{0x01}},
    {0xC4,1,{0x1D}},
    {0xC5,1,{0x01}},
    {0xC6,1,{0x54}},
    {0xC7,1,{0x01}},
    {0xC8,1,{0x81}},
    {0xC9,1,{0x01}},
    {0xCA,1,{0xCB}},
    {0xCB,1,{0x02}},
    {0xCC,1,{0x05}},
    {0xCD,1,{0x02}},
    {0xCE,1,{0x07}},
    {0xCF,1,{0x02}},
    {0xD0,1,{0x47}},
    {0xD1,1,{0x02}},
    {0xD2,1,{0x82}},
    {0xD3,1,{0x02}},
    {0xD4,1,{0xAB}},
    {0xD5,1,{0x02}},
    {0xD6,1,{0xDC}},
    {0xD7,1,{0x03}},
    {0xD8,1,{0x01}},
    {0xD9,1,{0x03}},
    {0xDA,1,{0x3A}},
    {0xDB,1,{0x03}},
    {0xDC,1,{0x56}},
    {0xDD,1,{0x03}},
    {0xDE,1,{0x6D}},
    {0xDF,1,{0x03}},
    {0xE0,1,{0x89}},
    {0xE1,1,{0x03}},
    {0xE2,1,{0xA3}},
    {0xE3,1,{0x03}},
    {0xE4,1,{0xC9}},
    {0xE5,1,{0x03}},
    {0xE6,1,{0xDD}},
    {0xE7,1,{0x03}},
    {0xE8,1,{0xF5}},
    {0xE9,1,{0x03}},
    {0xEA,1,{0xFF}},

    {0xEB,1,{0x30}},
    {0xEC,1,{0x17}},
    {0xED,1,{0x20}},
    {0xEE,1,{0x0F}},
    {0xEF,1,{0x1F}},
    {0xF0,1,{0x0F}},
    {0xF1,1,{0x0F}},
    {0xF2,1,{0x07}},

    //ccmoff
    //ccmrun


    //CMD2 Page 4
    {0xFF,1,{0x05}},
    {REGFLAG_DELAY, 1, {0}},

    {0xFB,1,{0x01}},

    //ccmon
    //LTPS related

    //CGOUT Mapping
    {0x00,1,{0x0F}},
    {0x01,1,{0x00}},
    {0x02,1,{0x00}},
    {0x03,1,{0x00}},
    {0x04,1,{0x0B}},
    {0x05,1,{0x0C}},
    {0x06,1,{0x00}},
    {0x07,1,{0x00}},
    {0x08,1,{0x00}},
    {0x09,1,{0x00}},
    {0x0A,1,{0x03}},
    {0x0B,1,{0x04}},
    {0x0C,1,{0x01}},
    {0x0D,1,{0x13}},
    {0x0E,1,{0x15}},
    {0x0F,1,{0x17}},
    {0x10,1,{0x0F}},
    {0x11,1,{0x00}},
    {0x12,1,{0x00}},
    {0x13,1,{0x00}},
    {0x14,1,{0x0B}},
    {0x15,1,{0x0C}},
    {0x16,1,{0x00}},
    {0x17,1,{0x00}},
    {0x18,1,{0x00}},
    {0x19,1,{0x00}},
    {0x1A,1,{0x03}},
    {0x1B,1,{0x04}},
    {0x1C,1,{0x01}},
    {0x1D,1,{0x13}},
    {0x1E,1,{0x15}},
    {0x1F,1,{0x17}},

    //STV                            
    {0x20,1,{0x09}},
    {0x21,1,{0x01}},
    {0x22,1,{0x00}},
    {0x23,1,{0x00}},
    {0x24,1,{0x00}},
    {0x25,1,{0x6D}},


    //U2D/D2U                              
    {0x29,1,{0x58}},
    {0x2A,1,{0x16}},
    {0x2B,1,{0x0A}},

    //GCK1/2                            
    {0x2F,1,{0x02}},
    {0x30,1,{0x04}},
    {0x31,1,{0x49}},
    {0x32,1,{0x23}},
    {0x33,1,{0x01}},
    {0x34,1,{0x00}},
    {0x35,1,{0x69}},
    {0x36,1,{0x00}},
    {0x37,1,{0x2D}},
    {0x38,1,{0x08}},


    //APO                           
    {0x5B,1,{0x00}},
    {0x5F,1,{0x75}},
    {0x63,1,{0x00}},
    {0x67,1,{0x04}},

    //Resolution                                                              
    {0x90,1,{0x00}},

    //MUX                                                          
    {0x74,1,{0x10}},
    {0x75,1,{0x19}},
    {0x76,1,{0x06}},
    {0x77,1,{0x03}},
    {0x78,1,{0x00}},
    {0x79,1,{0x00}},
    {0x7B,1,{0x80}},
    {0x7C,1,{0xD8}},
    {0x7D,1,{0x60}},
    {0x7E,1,{0x10}},
    {0x7F,1,{0x19}},
    {0x80,1,{0x00}},
    {0x81,1,{0x06}},
    {0x82,1,{0x03}},
    {0x83,1,{0x00}},
    {0x84,1,{0x03}},
    {0x85,1,{0x07}},

    //MUX and LTF setting                              
    {0x86,1,{0x1B}},
    {0x87,1,{0x39}},
    {0x88,1,{0x1B}},
    {0x89,1,{0x39}},
    {0x8A,1,{0x33}},
    {0xB5,1,{0x20}},
    {0x8C,1,{0x01}},

    //RTN,1,{BP,1,{PF  (Normal/Idle Mode)                           
    {0x91,1,{0x4C}},
    {0x92,1,{0x79}},
    {0x93,1,{0x04}},
    {0x94,1,{0x04}},
    {0x95,1,{0xE4}},

    {0x98,1,{0x00}},
    {0x99,1,{0x33}},
    {0x9B,1,{0x0F}},
    {0xA4,1,{0x0F}},
    {0x9D,1,{0xB0}},


    //GPIO output                            
    {0xC4,1,{0x24}},
    {0xC5,1,{0x30}},
    {0xC6,1,{0x00}},

    //ccmoff
    //ccmrun


    //PWM Frequency
    {0xFF,1,{0x23}},
    {REGFLAG_DELAY, 1, {0}},
    {0x08,1,{0x04}},
    {0xFB,1,{0x01}},

    //FRM
    //{0xFF,1,{0x05}},
    //{REGFLAG_DELAY, 1, {0}},
    //{0xEC,1,{0x01}},


    //CMD1
    {0xFF,1,{0x00}},
    {REGFLAG_DELAY, 1, {0}},

    {0xD3,1,{0x04}},
    {0xD4,1,{0x04}},

    {0xFF,1,{0x10}},
    {REGFLAG_DELAY, 1, {0}},
    {0xFB,1,{0x01}},
    {0xBB,1,{0x03}},

    {0x11, 	0,	{0x00}},
    {REGFLAG_DELAY, 120, {0}},
    {0x29, 	0,	{0x00}},
    //{REGFLAG_DELAY, 10, {0}},

    {REGFLAG_END_OF_TABLE, 0x00, {0}}

};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for(i = 0; i < count; i++) {

		unsigned cmd;
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

static void lcm_update(unsigned int x, unsigned int y,
unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
static void lcm_get_params(LCM_PARAMS *params)
{

    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED; //LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE
#endif

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_LSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_MSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting		
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.word_count=1080*3;	


    params->dsi.vertical_sync_active				= 3; //2//1
    params->dsi.vertical_backporch					= 10; //28;//30,18; //10 //1//2
    params->dsi.vertical_frontporch					= 6; //30;//20; //30//6//4
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 8; //4
    params->dsi.horizontal_backporch				= 80;//50 //118
    params->dsi.horizontal_frontporch				= 80;//50//118
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.pll_select=0;	 // 1  //0: MIPI_PLL; 1: LVDS_PLL
    // Bit rate calculation
    //1 Every lane speed
    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =16;  //16-58hz,14-52hz,11-38hz  //0X12  // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	


}

typedef unsigned char u8;
static void init_lcm_registers(void)
{

	u8 i,j;
	
	#if 0
	u8 gamma2[]={
	0x00,0x00,
	0x00,0x10,
	0x00,0x24,
	0x00,0x38,
	0x00,0x4B,
	0x00,0x5C,
	0x00,0x68,
	0x00,0x75,
	0x00,0x83,
	0x00,0xAC,
	0x00,0xD0,
	0x01,0x0A,
	0x01,0x3C,
	0x01,0x90,
	0x01,0xD9,
	0x01,0xDB,
	0x02,0x1D,
	0x02,0x6D,
	0x02,0x9F,
	0x02,0xF5,
	0x03,0x1F,
	0x03,0x58,
	0x03,0x64,
	0x03,0x75,
	0x03,0x89,
	0x03,0x99,
	0x03,0xAC,
	0x03,0xC3,
	0x03,0xDD,
	0x03,0xE8,
	};
	
	#else
	u8 gamma2[]={		//gamma2.2 setting
	0x00,0x00,  
	0x00,0x0A,  
	0x00,0x1E,  
	0x00,0x2D,  
	0x00,0x3C,  
	0x00,0x4B,  
	0x00,0x59,  
	0x00,0x64,  
	0x00,0x74,  
	0x00,0x96,  
	0x00,0xB4,  
	0x00,0xEB,  
	0x01,0x21,  
	0x01,0x7C,  
	0x01,0xCB,  
	0x01,0xCD,  
	0x02,0x16,  
	0x02,0x6D,  
	0x02,0x9C,  
	0x02,0xEC,  
	0x03,0x1E,  
	0x03,0x58,  
	0x03,0x64,  
	0x03,0x75,
	0x03,0x7F,  
	0x03,0x93,  
	0x03,0xA7,  
	0x03,0xC0,  
	0x03,0xD9,  
	0x03,0xE8,
	};
	#endif
	
    //NT35596 + LGD6" Panel Initial Code
    TC358768_DCS_write_1A_1P(0xFF,0xEE);
    TC358768_DCS_write_1A_1P(0xFB,0x01);
    TC358768_DCS_write_1A_1P(0x18,0x40);
    MDELAY(10);
    TC358768_DCS_write_1A_1P(0x18,0x00);
    MDELAY(20);
    //CMD2 Page 0
    TC358768_DCS_write_1A_1P(0xFF,0x01);
    MDELAY(1);

    TC358768_DCS_write_1A_1P(0xFB,0x01);
    //ccmon

    //Power-Related

    TC358768_DCS_write_1A_1P(0x00,0x01);
    TC358768_DCS_write_1A_1P(0x01,0x55);
    TC358768_DCS_write_1A_1P(0x02,0x45);
    TC358768_DCS_write_1A_1P(0x03,0x55);
    TC358768_DCS_write_1A_1P(0x05,0x50);

    //VGH/VGL w/Clamp +/- 10V
    TC358768_DCS_write_1A_1P(0x14,0xA8);
    TC358768_DCS_write_1A_1P(0x07,0xB2);
    TC358768_DCS_write_1A_1P(0x08,0x0C);

    //Gamma Voltage +/-4.21V
    TC358768_DCS_write_1A_1P(0x0B,0x88);
    TC358768_DCS_write_1A_1P(0x0C,0x88);

    //VGHO/VGLO Enable +/- 10V
    TC358768_DCS_write_1A_1P(0x0E,0xBF); //BF
    TC358768_DCS_write_1A_1P(0x0F,0xC2); //C2

    TC358768_DCS_write_1A_1P(0x11,0x1F);
    TC358768_DCS_write_1A_1P(0x12,0x1F);
    TC358768_DCS_write_1A_1P(0x13,0x03);
    TC358768_DCS_write_1A_1P(0x06,0x0A);
    TC358768_DCS_write_1A_1P(0x15,0x95);
    TC358768_DCS_write_1A_1P(0x16,0x95);
 //   TC358768_DCS_write_1A_1P(0x1B,0x39);
 //   TC358768_DCS_write_1A_1P(0x1C,0x39);
    TC358768_DCS_write_1A_1P(0x1D,0x47);

    //ID code
    //TC358768_DCS_write_1A_1P(0x44,0xC1);
    //TC358768_DCS_write_1A_1P(0x45,0x86);
    //TC358768_DCS_write_1A_1P(0x46,0xC4);

    //Gate EQ
    TC358768_DCS_write_1A_1P(0x58,0x05);
    TC358768_DCS_write_1A_1P(0x59,0x05);
    TC358768_DCS_write_1A_1P(0x5A,0x05);
    TC358768_DCS_write_1A_1P(0x5B,0x05);
    TC358768_DCS_write_1A_1P(0x5C,0x00);
    TC358768_DCS_write_1A_1P(0x5D,0x00);
    TC358768_DCS_write_1A_1P(0x5E,0x00);
    TC358768_DCS_write_1A_1P(0x5F,0x00);


    //ISOP P/N (Normal Mode)
    TC358768_DCS_write_1A_1P(0x6D,0x44);



TC358768_DCS_write_1A_1P(0xFF,0x01);

//R(+) MCR cmd
TC358768_DCS_write_1A_1P(0x75,0x00);
TC358768_DCS_write_1A_1P(0x76,0x0A);
TC358768_DCS_write_1A_1P(0x77,0x00);
TC358768_DCS_write_1A_1P(0x78,0x3D);
TC358768_DCS_write_1A_1P(0x79,0x00);
TC358768_DCS_write_1A_1P(0x7A,0x68);
TC358768_DCS_write_1A_1P(0x7B,0x00);
TC358768_DCS_write_1A_1P(0x7C,0x84);
TC358768_DCS_write_1A_1P(0x7D,0x00);
TC358768_DCS_write_1A_1P(0x7E,0x9A);
TC358768_DCS_write_1A_1P(0x7F,0x00);
TC358768_DCS_write_1A_1P(0x80,0xAD);
TC358768_DCS_write_1A_1P(0x81,0x00);
TC358768_DCS_write_1A_1P(0x82,0xBD);
TC358768_DCS_write_1A_1P(0x83,0x00);
TC358768_DCS_write_1A_1P(0x84,0xCA);
TC358768_DCS_write_1A_1P(0x85,0x00);
TC358768_DCS_write_1A_1P(0x86,0xD7);
TC358768_DCS_write_1A_1P(0x87,0x01);
TC358768_DCS_write_1A_1P(0x88,0x02);
TC358768_DCS_write_1A_1P(0x89,0x01);
TC358768_DCS_write_1A_1P(0x8A,0x26);
TC358768_DCS_write_1A_1P(0x8B,0x01);
TC358768_DCS_write_1A_1P(0x8C,0x5E);
TC358768_DCS_write_1A_1P(0x8D,0x01);
TC358768_DCS_write_1A_1P(0x8E,0x8A);
TC358768_DCS_write_1A_1P(0x8F,0x01);
TC358768_DCS_write_1A_1P(0x90,0xD1);
TC358768_DCS_write_1A_1P(0x91,0x02);
TC358768_DCS_write_1A_1P(0x92,0x0B);
TC358768_DCS_write_1A_1P(0x93,0x02);
TC358768_DCS_write_1A_1P(0x94,0x0D);
TC358768_DCS_write_1A_1P(0x95,0x02);
TC358768_DCS_write_1A_1P(0x96,0x43);
TC358768_DCS_write_1A_1P(0x97,0x02);
TC358768_DCS_write_1A_1P(0x98,0x7F);
TC358768_DCS_write_1A_1P(0x99,0x02);
TC358768_DCS_write_1A_1P(0x9A,0xA5);
TC358768_DCS_write_1A_1P(0x9B,0x02);
TC358768_DCS_write_1A_1P(0x9C,0xD7);
TC358768_DCS_write_1A_1P(0x9D,0x02);
TC358768_DCS_write_1A_1P(0x9E,0xF9);
TC358768_DCS_write_1A_1P(0x9F,0x03);
TC358768_DCS_write_1A_1P(0xA0,0x26);
TC358768_DCS_write_1A_1P(0xA2,0x03);
TC358768_DCS_write_1A_1P(0xA3,0x33);
TC358768_DCS_write_1A_1P(0xA4,0x03);
TC358768_DCS_write_1A_1P(0xA5,0x43);
TC358768_DCS_write_1A_1P(0xA6,0x03);
TC358768_DCS_write_1A_1P(0xA7,0x54);
TC358768_DCS_write_1A_1P(0xA9,0x03);
TC358768_DCS_write_1A_1P(0xAA,0x68);
TC358768_DCS_write_1A_1P(0xAB,0x03);
TC358768_DCS_write_1A_1P(0xAC,0x7F);
TC358768_DCS_write_1A_1P(0xAD,0x03);
TC358768_DCS_write_1A_1P(0xAE,0x9A);
TC358768_DCS_write_1A_1P(0xAF,0x03);
TC358768_DCS_write_1A_1P(0xB0,0xB8);
TC358768_DCS_write_1A_1P(0xB1,0x03);
TC358768_DCS_write_1A_1P(0xB2,0xBE);
//R(-) MCR cmd
TC358768_DCS_write_1A_1P(0xB3,0x00);
TC358768_DCS_write_1A_1P(0xB4,0x0A);
TC358768_DCS_write_1A_1P(0xB5,0x00);
TC358768_DCS_write_1A_1P(0xB6,0x3D);
TC358768_DCS_write_1A_1P(0xB7,0x00);
TC358768_DCS_write_1A_1P(0xB8,0x68);
TC358768_DCS_write_1A_1P(0xB9,0x00);
TC358768_DCS_write_1A_1P(0xBA,0x84);
TC358768_DCS_write_1A_1P(0xBB,0x00);
TC358768_DCS_write_1A_1P(0xBC,0x9A);
TC358768_DCS_write_1A_1P(0xBD,0x00);
TC358768_DCS_write_1A_1P(0xBE,0xAD);
TC358768_DCS_write_1A_1P(0xBF,0x00);
TC358768_DCS_write_1A_1P(0xC0,0xBD);
TC358768_DCS_write_1A_1P(0xC1,0x00);
TC358768_DCS_write_1A_1P(0xC2,0xCA);
TC358768_DCS_write_1A_1P(0xC3,0x00);
TC358768_DCS_write_1A_1P(0xC4,0xD7);
TC358768_DCS_write_1A_1P(0xC5,0x01);
TC358768_DCS_write_1A_1P(0xC6,0x02);
TC358768_DCS_write_1A_1P(0xC7,0x01);
TC358768_DCS_write_1A_1P(0xC8,0x26);
TC358768_DCS_write_1A_1P(0xC9,0x01);
TC358768_DCS_write_1A_1P(0xCA,0x5E);
TC358768_DCS_write_1A_1P(0xCB,0x01);
TC358768_DCS_write_1A_1P(0xCC,0x8A);
TC358768_DCS_write_1A_1P(0xCD,0x01);
TC358768_DCS_write_1A_1P(0xCE,0xD1);
TC358768_DCS_write_1A_1P(0xCF,0x02);
TC358768_DCS_write_1A_1P(0xD0,0x0B);
TC358768_DCS_write_1A_1P(0xD1,0x02);
TC358768_DCS_write_1A_1P(0xD2,0x0D);
TC358768_DCS_write_1A_1P(0xD3,0x02);
TC358768_DCS_write_1A_1P(0xD4,0x43);
TC358768_DCS_write_1A_1P(0xD5,0x02);
TC358768_DCS_write_1A_1P(0xD6,0x7F);
TC358768_DCS_write_1A_1P(0xD7,0x02);
TC358768_DCS_write_1A_1P(0xD8,0xA5);
TC358768_DCS_write_1A_1P(0xD9,0x02);
TC358768_DCS_write_1A_1P(0xDA,0xD7);
TC358768_DCS_write_1A_1P(0xDB,0x02);
TC358768_DCS_write_1A_1P(0xDC,0xF9);
TC358768_DCS_write_1A_1P(0xDD,0x03);
TC358768_DCS_write_1A_1P(0xDE,0x26);
TC358768_DCS_write_1A_1P(0xDF,0x03);
TC358768_DCS_write_1A_1P(0xE0,0x33);
TC358768_DCS_write_1A_1P(0xE1,0x03);
TC358768_DCS_write_1A_1P(0xE2,0x43);
TC358768_DCS_write_1A_1P(0xE3,0x03);
TC358768_DCS_write_1A_1P(0xE4,0x54);
TC358768_DCS_write_1A_1P(0xE5,0x03);
TC358768_DCS_write_1A_1P(0xE6,0x68);
TC358768_DCS_write_1A_1P(0xE7,0x03);
TC358768_DCS_write_1A_1P(0xE8,0x7F);
TC358768_DCS_write_1A_1P(0xE9,0x03);
TC358768_DCS_write_1A_1P(0xEA,0x9A);
TC358768_DCS_write_1A_1P(0xEB,0x03);
TC358768_DCS_write_1A_1P(0xEC,0xB8);
TC358768_DCS_write_1A_1P(0xED,0x03);
TC358768_DCS_write_1A_1P(0xEE,0xBE);
//G(+) MCR cmd
TC358768_DCS_write_1A_1P(0xEF,0x00);
TC358768_DCS_write_1A_1P(0xF0,0x0A);
TC358768_DCS_write_1A_1P(0xF1,0x00);
TC358768_DCS_write_1A_1P(0xF2,0x3D);
TC358768_DCS_write_1A_1P(0xF3,0x00);
TC358768_DCS_write_1A_1P(0xF4,0x68);
TC358768_DCS_write_1A_1P(0xF5,0x00);
TC358768_DCS_write_1A_1P(0xF6,0x84);
TC358768_DCS_write_1A_1P(0xF7,0x00);
TC358768_DCS_write_1A_1P(0xF8,0x9A);
TC358768_DCS_write_1A_1P(0xF9,0x00);
TC358768_DCS_write_1A_1P(0xFA,0xAD);
//page selection cmd start
TC358768_DCS_write_1A_1P(0xFF, 0x02);

TC358768_DCS_write_1A_1P(0xFB, 0x01);
//page selection cmd end
TC358768_DCS_write_1A_1P(0x00,0x00);
TC358768_DCS_write_1A_1P(0x01,0xBD);
TC358768_DCS_write_1A_1P(0x02,0x00);
TC358768_DCS_write_1A_1P(0x03,0xCA);
TC358768_DCS_write_1A_1P(0x04,0x00);
TC358768_DCS_write_1A_1P(0x05,0xD7);
TC358768_DCS_write_1A_1P(0x06,0x01);
TC358768_DCS_write_1A_1P(0x07,0x02);
TC358768_DCS_write_1A_1P(0x08,0x01);
TC358768_DCS_write_1A_1P(0x09,0x26);
TC358768_DCS_write_1A_1P(0x0A,0x01);
TC358768_DCS_write_1A_1P(0x0B,0x5E);
TC358768_DCS_write_1A_1P(0x0C,0x01);
TC358768_DCS_write_1A_1P(0x0D,0x8A);
TC358768_DCS_write_1A_1P(0x0E,0x01);
TC358768_DCS_write_1A_1P(0x0F,0xD1);
TC358768_DCS_write_1A_1P(0x10,0x02);
TC358768_DCS_write_1A_1P(0x11,0x0B);
TC358768_DCS_write_1A_1P(0x12,0x02);
TC358768_DCS_write_1A_1P(0x13,0x0D);
TC358768_DCS_write_1A_1P(0x14,0x02);
TC358768_DCS_write_1A_1P(0x15,0x43);
TC358768_DCS_write_1A_1P(0x16,0x02);
TC358768_DCS_write_1A_1P(0x17,0x7F);
TC358768_DCS_write_1A_1P(0x18,0x02);
TC358768_DCS_write_1A_1P(0x19,0xA5);
TC358768_DCS_write_1A_1P(0x1A,0x02);
TC358768_DCS_write_1A_1P(0x1B,0xD7);
TC358768_DCS_write_1A_1P(0x1C,0x02);
TC358768_DCS_write_1A_1P(0x1D,0xF9);
TC358768_DCS_write_1A_1P(0x1E,0x03);
TC358768_DCS_write_1A_1P(0x1F,0x26);
TC358768_DCS_write_1A_1P(0x20,0x03);
TC358768_DCS_write_1A_1P(0x21,0x33);
TC358768_DCS_write_1A_1P(0x22,0x03);
TC358768_DCS_write_1A_1P(0x23,0x43);
TC358768_DCS_write_1A_1P(0x24,0x03);
TC358768_DCS_write_1A_1P(0x25,0x54);
TC358768_DCS_write_1A_1P(0x26,0x03);
TC358768_DCS_write_1A_1P(0x27,0x68);
TC358768_DCS_write_1A_1P(0x28,0x03);
TC358768_DCS_write_1A_1P(0x29,0x7F);
TC358768_DCS_write_1A_1P(0x2A,0x03);
TC358768_DCS_write_1A_1P(0x2B,0x9A);
TC358768_DCS_write_1A_1P(0x2D,0x03);
TC358768_DCS_write_1A_1P(0x2F,0xB8);
TC358768_DCS_write_1A_1P(0x30,0x03);
TC358768_DCS_write_1A_1P(0x31,0xBE);
//G(-) MCR cmd
TC358768_DCS_write_1A_1P(0x32,0x00);
TC358768_DCS_write_1A_1P(0x33,0x0A);
TC358768_DCS_write_1A_1P(0x34,0x00);
TC358768_DCS_write_1A_1P(0x35,0x3D);
TC358768_DCS_write_1A_1P(0x36,0x00);
TC358768_DCS_write_1A_1P(0x37,0x68);
TC358768_DCS_write_1A_1P(0x38,0x00);
TC358768_DCS_write_1A_1P(0x39,0x84);
TC358768_DCS_write_1A_1P(0x3A,0x00);
TC358768_DCS_write_1A_1P(0x3B,0x9A);
TC358768_DCS_write_1A_1P(0x3D,0x00);
TC358768_DCS_write_1A_1P(0x3F,0xAD);
TC358768_DCS_write_1A_1P(0x40,0x00);
TC358768_DCS_write_1A_1P(0x41,0xBD);
TC358768_DCS_write_1A_1P(0x42,0x00);
TC358768_DCS_write_1A_1P(0x43,0xCA);
TC358768_DCS_write_1A_1P(0x44,0x00);
TC358768_DCS_write_1A_1P(0x45,0xD7);
TC358768_DCS_write_1A_1P(0x46,0x01);
TC358768_DCS_write_1A_1P(0x47,0x02);
TC358768_DCS_write_1A_1P(0x48,0x01);
TC358768_DCS_write_1A_1P(0x49,0x26);
TC358768_DCS_write_1A_1P(0x4A,0x01);
TC358768_DCS_write_1A_1P(0x4B,0x5E);
TC358768_DCS_write_1A_1P(0x4C,0x01);
TC358768_DCS_write_1A_1P(0x4D,0x8A);
TC358768_DCS_write_1A_1P(0x4E,0x01);
TC358768_DCS_write_1A_1P(0x4F,0xD1);
TC358768_DCS_write_1A_1P(0x50,0x02);
TC358768_DCS_write_1A_1P(0x51,0x0B);
TC358768_DCS_write_1A_1P(0x52,0x02);
TC358768_DCS_write_1A_1P(0x53,0x0D);
TC358768_DCS_write_1A_1P(0x54,0x02);
TC358768_DCS_write_1A_1P(0x55,0x43);
TC358768_DCS_write_1A_1P(0x56,0x02);
TC358768_DCS_write_1A_1P(0x58,0x7F);
TC358768_DCS_write_1A_1P(0x59,0x02);
TC358768_DCS_write_1A_1P(0x5A,0xA5);
TC358768_DCS_write_1A_1P(0x5B,0x02);
TC358768_DCS_write_1A_1P(0x5C,0xD7);
TC358768_DCS_write_1A_1P(0x5D,0x02);
TC358768_DCS_write_1A_1P(0x5E,0xF9);
TC358768_DCS_write_1A_1P(0x5F,0x03);
TC358768_DCS_write_1A_1P(0x60,0x26);
TC358768_DCS_write_1A_1P(0x61,0x03);
TC358768_DCS_write_1A_1P(0x62,0x33);
TC358768_DCS_write_1A_1P(0x63,0x03);
TC358768_DCS_write_1A_1P(0x64,0x43);
TC358768_DCS_write_1A_1P(0x65,0x03);
TC358768_DCS_write_1A_1P(0x66,0x54);
TC358768_DCS_write_1A_1P(0x67,0x03);
TC358768_DCS_write_1A_1P(0x68,0x68);
TC358768_DCS_write_1A_1P(0x69,0x03);
TC358768_DCS_write_1A_1P(0x6A,0x7F);
TC358768_DCS_write_1A_1P(0x6B,0x03);
TC358768_DCS_write_1A_1P(0x6C,0x9A);
TC358768_DCS_write_1A_1P(0x6D,0x03);
TC358768_DCS_write_1A_1P(0x6E,0xB8);
TC358768_DCS_write_1A_1P(0x6F,0x03);
TC358768_DCS_write_1A_1P(0x70,0xBE);
//B(+) MCR cmd
TC358768_DCS_write_1A_1P(0x71,0x00);
TC358768_DCS_write_1A_1P(0x72,0x0A);
TC358768_DCS_write_1A_1P(0x73,0x00);
TC358768_DCS_write_1A_1P(0x74,0x3D);
TC358768_DCS_write_1A_1P(0x75,0x00);
TC358768_DCS_write_1A_1P(0x76,0x68);
TC358768_DCS_write_1A_1P(0x77,0x00);
TC358768_DCS_write_1A_1P(0x78,0x84);
TC358768_DCS_write_1A_1P(0x79,0x00);
TC358768_DCS_write_1A_1P(0x7A,0x9A);
TC358768_DCS_write_1A_1P(0x7B,0x00);
TC358768_DCS_write_1A_1P(0x7C,0xAD);
TC358768_DCS_write_1A_1P(0x7D,0x00);
TC358768_DCS_write_1A_1P(0x7E,0xBD);
TC358768_DCS_write_1A_1P(0x7F,0x00);
TC358768_DCS_write_1A_1P(0x80,0xCA);
TC358768_DCS_write_1A_1P(0x81,0x00);
TC358768_DCS_write_1A_1P(0x82,0xD7);
TC358768_DCS_write_1A_1P(0x83,0x01);
TC358768_DCS_write_1A_1P(0x84,0x02);
TC358768_DCS_write_1A_1P(0x85,0x01);
TC358768_DCS_write_1A_1P(0x86,0x26);
TC358768_DCS_write_1A_1P(0x87,0x01);
TC358768_DCS_write_1A_1P(0x88,0x5E);
TC358768_DCS_write_1A_1P(0x89,0x01);
TC358768_DCS_write_1A_1P(0x8A,0x8A);
TC358768_DCS_write_1A_1P(0x8B,0x01);
TC358768_DCS_write_1A_1P(0x8C,0xD1);
TC358768_DCS_write_1A_1P(0x8D,0x02);
TC358768_DCS_write_1A_1P(0x8E,0x0B);
TC358768_DCS_write_1A_1P(0x8F,0x02);
TC358768_DCS_write_1A_1P(0x90,0x0D);
TC358768_DCS_write_1A_1P(0x91,0x02);
TC358768_DCS_write_1A_1P(0x92,0x43);
TC358768_DCS_write_1A_1P(0x93,0x02);
TC358768_DCS_write_1A_1P(0x94,0x7F);
TC358768_DCS_write_1A_1P(0x95,0x02);
TC358768_DCS_write_1A_1P(0x96,0xA5);
TC358768_DCS_write_1A_1P(0x97,0x02);
TC358768_DCS_write_1A_1P(0x98,0xD7);
TC358768_DCS_write_1A_1P(0x99,0x02);
TC358768_DCS_write_1A_1P(0x9A,0xF9);
TC358768_DCS_write_1A_1P(0x9B,0x03);
TC358768_DCS_write_1A_1P(0x9C,0x26);
TC358768_DCS_write_1A_1P(0x9D,0x03);
TC358768_DCS_write_1A_1P(0x9E,0x33);
TC358768_DCS_write_1A_1P(0x9F,0x03);
TC358768_DCS_write_1A_1P(0xA0,0x43);
TC358768_DCS_write_1A_1P(0xA2,0x03);
TC358768_DCS_write_1A_1P(0xA3,0x54);
TC358768_DCS_write_1A_1P(0xA4,0x03);
TC358768_DCS_write_1A_1P(0xA5,0x68);
TC358768_DCS_write_1A_1P(0xA6,0x03);
TC358768_DCS_write_1A_1P(0xA7,0x7F);
TC358768_DCS_write_1A_1P(0xA9,0x03);
TC358768_DCS_write_1A_1P(0xAA,0x9A);
TC358768_DCS_write_1A_1P(0xAB,0x03);
TC358768_DCS_write_1A_1P(0xAC,0xB8);
TC358768_DCS_write_1A_1P(0xAD,0x03);
TC358768_DCS_write_1A_1P(0xAE,0xBE);
//B(-) MCR cmd
TC358768_DCS_write_1A_1P(0xAF,0x00);
TC358768_DCS_write_1A_1P(0xB0,0x0A);
TC358768_DCS_write_1A_1P(0xB1,0x00);
TC358768_DCS_write_1A_1P(0xB2,0x3D);
TC358768_DCS_write_1A_1P(0xB3,0x00);
TC358768_DCS_write_1A_1P(0xB4,0x68);
TC358768_DCS_write_1A_1P(0xB5,0x00);
TC358768_DCS_write_1A_1P(0xB6,0x84);
TC358768_DCS_write_1A_1P(0xB7,0x00);
TC358768_DCS_write_1A_1P(0xB8,0x9A);
TC358768_DCS_write_1A_1P(0xB9,0x00);
TC358768_DCS_write_1A_1P(0xBA,0xAD);
TC358768_DCS_write_1A_1P(0xBB,0x00);
TC358768_DCS_write_1A_1P(0xBC,0xBD);
TC358768_DCS_write_1A_1P(0xBD,0x00);
TC358768_DCS_write_1A_1P(0xBE,0xCA);
TC358768_DCS_write_1A_1P(0xBF,0x00);
TC358768_DCS_write_1A_1P(0xC0,0xD7);
TC358768_DCS_write_1A_1P(0xC1,0x01);
TC358768_DCS_write_1A_1P(0xC2,0x02);
TC358768_DCS_write_1A_1P(0xC3,0x01);
TC358768_DCS_write_1A_1P(0xC4,0x26);
TC358768_DCS_write_1A_1P(0xC5,0x01);
TC358768_DCS_write_1A_1P(0xC6,0x5E);
TC358768_DCS_write_1A_1P(0xC7,0x01);
TC358768_DCS_write_1A_1P(0xC8,0x8A);
TC358768_DCS_write_1A_1P(0xC9,0x01);
TC358768_DCS_write_1A_1P(0xCA,0xD1);
TC358768_DCS_write_1A_1P(0xCB,0x02);
TC358768_DCS_write_1A_1P(0xCC,0x0B);
TC358768_DCS_write_1A_1P(0xCD,0x02);
TC358768_DCS_write_1A_1P(0xCE,0x0D);
TC358768_DCS_write_1A_1P(0xCF,0x02);
TC358768_DCS_write_1A_1P(0xD0,0x43);
TC358768_DCS_write_1A_1P(0xD1,0x02);
TC358768_DCS_write_1A_1P(0xD2,0x7F);
TC358768_DCS_write_1A_1P(0xD3,0x02);
TC358768_DCS_write_1A_1P(0xD4,0xA5);
TC358768_DCS_write_1A_1P(0xD5,0x02);
TC358768_DCS_write_1A_1P(0xD6,0xD7);
TC358768_DCS_write_1A_1P(0xD7,0x02);
TC358768_DCS_write_1A_1P(0xD8,0xF9);
TC358768_DCS_write_1A_1P(0xD9,0x03);
TC358768_DCS_write_1A_1P(0xDA,0x26);
TC358768_DCS_write_1A_1P(0xDB,0x03);
TC358768_DCS_write_1A_1P(0xDC,0x33);
TC358768_DCS_write_1A_1P(0xDD,0x03);
TC358768_DCS_write_1A_1P(0xDE,0x43);
TC358768_DCS_write_1A_1P(0xDF,0x03);
TC358768_DCS_write_1A_1P(0xE0,0x54);
TC358768_DCS_write_1A_1P(0xE1,0x03);
TC358768_DCS_write_1A_1P(0xE2,0x68);
TC358768_DCS_write_1A_1P(0xE3,0x03);
TC358768_DCS_write_1A_1P(0xE4,0x7F);
TC358768_DCS_write_1A_1P(0xE5,0x03);
TC358768_DCS_write_1A_1P(0xE6,0x9A);
TC358768_DCS_write_1A_1P(0xE7,0x03);
TC358768_DCS_write_1A_1P(0xE8,0xB8);
TC358768_DCS_write_1A_1P(0xE9,0x03);
TC358768_DCS_write_1A_1P(0xEA,0xBE);


    TC358768_DCS_write_1A_1P(0xEB,0x30);
    TC358768_DCS_write_1A_1P(0xEC,0x17);
    TC358768_DCS_write_1A_1P(0xED,0x20);
    TC358768_DCS_write_1A_1P(0xEE,0x0F);
    TC358768_DCS_write_1A_1P(0xEF,0x1F);
    TC358768_DCS_write_1A_1P(0xF0,0x0F);
    TC358768_DCS_write_1A_1P(0xF1,0x0F);
    TC358768_DCS_write_1A_1P(0xF2,0x07);

    //ccmoff
    //ccmrun


    //CMD2 Page 4
    TC358768_DCS_write_1A_1P(0xFF,0x05);
    MDELAY(1);

    TC358768_DCS_write_1A_1P(0xFB,0x01);


    //ccmon
    //LTPS related

    //CGOUT Mapping
    TC358768_DCS_write_1A_1P(0x00,0x0F);
    TC358768_DCS_write_1A_1P(0x01,0x00);
    TC358768_DCS_write_1A_1P(0x02,0x00);
    TC358768_DCS_write_1A_1P(0x03,0x00);
    TC358768_DCS_write_1A_1P(0x04,0x0B);
    TC358768_DCS_write_1A_1P(0x05,0x0C);
    TC358768_DCS_write_1A_1P(0x06,0x00);
    TC358768_DCS_write_1A_1P(0x07,0x00);
    TC358768_DCS_write_1A_1P(0x08,0x00);
    TC358768_DCS_write_1A_1P(0x09,0x00);
    TC358768_DCS_write_1A_1P(0x0A,0x03);
    TC358768_DCS_write_1A_1P(0x0B,0x04);
    TC358768_DCS_write_1A_1P(0x0C,0x01);
    TC358768_DCS_write_1A_1P(0x0D,0x13);
    TC358768_DCS_write_1A_1P(0x0E,0x15);
    TC358768_DCS_write_1A_1P(0x0F,0x17);
    TC358768_DCS_write_1A_1P(0x10,0x0F);
    TC358768_DCS_write_1A_1P(0x11,0x00);
    TC358768_DCS_write_1A_1P(0x12,0x00);
    TC358768_DCS_write_1A_1P(0x13,0x00);
    TC358768_DCS_write_1A_1P(0x14,0x0B);
    TC358768_DCS_write_1A_1P(0x15,0x0C);
    TC358768_DCS_write_1A_1P(0x16,0x00);
    TC358768_DCS_write_1A_1P(0x17,0x00);
    TC358768_DCS_write_1A_1P(0x18,0x00);
    TC358768_DCS_write_1A_1P(0x19,0x00);
    TC358768_DCS_write_1A_1P(0x1A,0x03);
    TC358768_DCS_write_1A_1P(0x1B,0x04);
    TC358768_DCS_write_1A_1P(0x1C,0x01);
    TC358768_DCS_write_1A_1P(0x1D,0x13);
    TC358768_DCS_write_1A_1P(0x1E,0x15);
    TC358768_DCS_write_1A_1P(0x1F,0x17);

    //STV                            
    TC358768_DCS_write_1A_1P(0x20,0x09);
    TC358768_DCS_write_1A_1P(0x21,0x01);
    TC358768_DCS_write_1A_1P(0x22,0x00);
    TC358768_DCS_write_1A_1P(0x23,0x00);
    TC358768_DCS_write_1A_1P(0x24,0x00);
//Angus modified
    //TC358768_DCS_write_1A_1P(0x25,0x6D);
    TC358768_DCS_write_1A_1P(0x25,0xED);


    //U2D/D2U                              
    TC358768_DCS_write_1A_1P(0x29,0x58);
    TC358768_DCS_write_1A_1P(0x2A,0x16);
//Angus modified
    //TC358768_DCS_write_1A_1P(0x2B,0x0A);
    TC358768_DCS_write_1A_1P(0x2B,0x05);

    //GCK1/2                            
    TC358768_DCS_write_1A_1P(0x2F,0x02);
    TC358768_DCS_write_1A_1P(0x30,0x04);
    TC358768_DCS_write_1A_1P(0x31,0x49);
    TC358768_DCS_write_1A_1P(0x32,0x23);
    TC358768_DCS_write_1A_1P(0x33,0x01);
    TC358768_DCS_write_1A_1P(0x34,0x00);
    TC358768_DCS_write_1A_1P(0x35,0x69);
    TC358768_DCS_write_1A_1P(0x36,0x00);
    TC358768_DCS_write_1A_1P(0x37,0x2D);
//Angus modified
    //TC358768_DCS_write_1A_1P(0x38,0x08);
    TC358768_DCS_write_1A_1P(0x38,0x18);


    //APO                           
    TC358768_DCS_write_1A_1P(0x5B,0x00);
    TC358768_DCS_write_1A_1P(0x5F,0x75);
    TC358768_DCS_write_1A_1P(0x63,0x00);
    TC358768_DCS_write_1A_1P(0x67,0x04);
//Angus add
    TC358768_DCS_write_1A_1P(0x6C,0x00);

    //Resolution                                                              
    TC358768_DCS_write_1A_1P(0x90,0x00);

    //MUX                                                          
    TC358768_DCS_write_1A_1P(0x74,0x10);
    TC358768_DCS_write_1A_1P(0x75,0x19);
    TC358768_DCS_write_1A_1P(0x76,0x06);
    TC358768_DCS_write_1A_1P(0x77,0x03);
    TC358768_DCS_write_1A_1P(0x78,0x00);
    TC358768_DCS_write_1A_1P(0x79,0x00);
    TC358768_DCS_write_1A_1P(0x7B,0x80);
    TC358768_DCS_write_1A_1P(0x7C,0xD8);
    TC358768_DCS_write_1A_1P(0x7D,0x60);
    TC358768_DCS_write_1A_1P(0x7E,0x10);
    TC358768_DCS_write_1A_1P(0x7F,0x19);
    TC358768_DCS_write_1A_1P(0x80,0x00);
    TC358768_DCS_write_1A_1P(0x81,0x06);
    TC358768_DCS_write_1A_1P(0x82,0x03);
    TC358768_DCS_write_1A_1P(0x83,0x00);
    TC358768_DCS_write_1A_1P(0x84,0x03);
    TC358768_DCS_write_1A_1P(0x85,0x07);

    //MUX and LTF setting                              
    TC358768_DCS_write_1A_1P(0x86,0x1B);
    TC358768_DCS_write_1A_1P(0x87,0x39);
    TC358768_DCS_write_1A_1P(0x88,0x1B);
    TC358768_DCS_write_1A_1P(0x89,0x39);
    TC358768_DCS_write_1A_1P(0x8A,0x33);
    //TC358768_DCS_write_1A_1P(0xB5,0x20);//remove by ydyu, for FAE suggestion
    TC358768_DCS_write_1A_1P(0x8C,0x01);

    //RTN,BP,PF  (Normal/Idle Mode)                           
    TC358768_DCS_write_1A_1P(0x91,0x4C);
    TC358768_DCS_write_1A_1P(0x92,0x79);
    TC358768_DCS_write_1A_1P(0x93,0x04);
    TC358768_DCS_write_1A_1P(0x94,0x04);
    TC358768_DCS_write_1A_1P(0x95,0xE4);

    TC358768_DCS_write_1A_1P(0x98,0x00);
    TC358768_DCS_write_1A_1P(0x99,0x33);
    TC358768_DCS_write_1A_1P(0x9B,0x0F);
    TC358768_DCS_write_1A_1P(0xA4,0x0F);
    TC358768_DCS_write_1A_1P(0x9D,0xB0); //modified for screen flip 20130904
    //TC358768_DCS_write_1A_1P(0x9D,0xB6);


    //GPIO output                            
    TC358768_DCS_write_1A_1P(0xC4,0x24);
    TC358768_DCS_write_1A_1P(0xC5,0x30);
    TC358768_DCS_write_1A_1P(0xC6,0x00);

    //ccmoff
    //ccmrun


    //PWM Frequency
    TC358768_DCS_write_1A_1P(0xFF,0x23);
    MDELAY(1);
    TC358768_DCS_write_1A_1P(0x08,0x04);
    TC358768_DCS_write_1A_1P(0xFB,0x01);

    //FRM
    //TC358768_DCS_write_1A_1P(0xFF,0x05);
    //MDELAY(1);
    //TC358768_DCS_write_1A_1P(0xEC,0x01);


    //CMD1
    TC358768_DCS_write_1A_1P(0xFF,0x00);
    MDELAY(1);
    TC358768_DCS_write_1A_1P(0xFB,0x01);



///===============================================//

	//TC358768_DCS_write_1A_1P(0xD3,0x1E);  //04
    //TC358768_DCS_write_1A_1P(0xD4,0x1E);
  
    //TC358768_DCS_write_1A_1P(0xFF,0x05);
   // MDELAY(1);
    //TC358768_DCS_write_1A_1P(0xEC,0x01);

//================================================//


	//TC358768_DCS_write_1A_1P(0xD3,0x06); //modified for porch setting 20130905 
	//TC358768_DCS_write_1A_1P(0xD4,0x04);
        TC358768_DCS_write_1A_1P(0xD3,0x0C); //modified for porch setting 20130905 
	TC358768_DCS_write_1A_1P(0xD4,0x06);
									  
	TC358768_DCS_write_1A_1P(0xFF,0x10);
	TC358768_DCS_write_1A_1P(0xFB,0x01);
	TC358768_DCS_write_1A_1P(0xBB,0x03);


									 
        TC358768_DCS_write_1A_1P(0x35,0x00);

//    TC358768_DCS_write_1A_1P(0x3a,0x66);  
    TC358768_DCS_write_1A_0P(0x11);
    MDELAY(120);
    TC358768_DCS_write_1A_0P(0x29);
    MDELAY(20);
    TC358768_DCS_write_1A_1P(0xFF,0x00);
    TC358768_DCS_write_1A_1P(0xD3,0x0C); //modified for porch setting 20130905  //0c//02
    TC358768_DCS_write_1A_1P(0xD4,0x06); //06//04

}
static unsigned int lcd_id;
static void lcm_init(void)
{
#ifdef BUILD_LK
    printf("%s, %d, nt365596 id = 0x%x\n",__FUNCTION__,__LINE__,lcd_id);
#endif

#if 0
    SET_GPIO_OUT(LCD_LDO_ENP_GPIO_PIN , 1);//power on +5
	//MDELAY(2);
	MDELAY(10);
	SET_GPIO_OUT(LCD_LDO_ENN_GPIO_PIN , 1);//power on -5
	MDELAY(100);
    
       SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(5);  // 2
	SET_RESET_PIN(1);
	MDELAY(120);  //10
#endif
   //push_table(lcm_initialization_ret, sizeof(lcm_initialization_ret)/sizeof(struct LCM_setting_table), 1); 
   lcm_util.set_gpio_mode(GPIO112, GPIO_MODE_00);
   lcm_util.set_gpio_dir(GPIO112, GPIO_DIR_OUT); 
   lcm_util.set_gpio_pull_enable(GPIO112, GPIO_PULL_DISABLE); 
   
    SET_GPIO_OUT(LCD_LDO_ENP_GPIO_PIN , 1);//power on +5
    MDELAY(10);
    SET_GPIO_OUT(LCD_LDO_ENN_GPIO_PIN , 1);//power on -5
    MDELAY(150);

    lcm_util.set_gpio_out(GPIO112 , 1);
    MDELAY(15);//5
    lcm_util.set_gpio_out(GPIO112 , 0); //for GPIO reset type  reset low 
    MDELAY(15);//5
    lcm_util.set_gpio_out(GPIO112 , 1);
    MDELAY(15);//5
    lcm_util.set_gpio_out(GPIO112 , 0); //for GPIO reset type  reset low 
    MDELAY(15);//5
    lcm_util.set_gpio_out(GPIO112 , 1);
    MDELAY(120); //20

	
	init_lcm_registers();
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
    
	data_array[0]=0x00530500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);
	
	data_array[0]=0x00280500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x00100500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);//delay more for 3 frames time  17*3=54ms

	SET_RESET_PIN(1);//Reset Low
	MDELAY(10);
	SET_RESET_PIN(0);//Reset Low
	MDELAY(10);
	SET_RESET_PIN(1);//Reset Low
	MDELAY(120);
}

static void lcm_resume(void)
{
	unsigned int data_array[16];
	
	lcm_init();
#if 0    
	data_array[0]=0x00110500;
	dsi_set_cmdq(&data_array,1,1);
	MDELAY(120);
	data_array[0]=0x00290500;
	dsi_set_cmdq(&data_array,1,1);
	MDELAY(20);
#endif
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static int dummy_delay = 0;
static unsigned int lcm_esd_check(void)
{  
    //#ifndef BUILD_LK
    unsigned int  data_array[16];
    unsigned char buffer_0a = 0;
    unsigned char buffer_0b = 0;
    unsigned char buffer_0d = 0;
    unsigned char buffer_0e = 0;
    unsigned int retval = 0;
    
    dummy_delay ++;

    if (dummy_delay >=10000)
        dummy_delay = 0;
    
    if(dummy_delay %2 == 0)
    {    
        //printk("%s return 1\n",__FUNCTION__);

	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0a,&buffer_0a, 1);

	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0b,&buffer_0b, 1);

	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0d,&buffer_0d, 1);

	    data_array[0] = 0x00013700;
	    dsi_set_cmdq(data_array, 1, 1);
	    read_reg_v2(0x0e,&buffer_0e, 1);
        
            #ifdef BUILD_LK
            printf("build lk lcm_esd_check lcm 0x0A is %x-----------------\n", buffer_0a);
	    printf("lcm_esd_check lcm 0x0B is %x-----------------\n", buffer_0b);
	    printf("lcm_esd_check lcm 0x0D is %x-----------------\n", buffer_0d);
	    printf("lcm_esd_check lcm 0x0E is %x-----------------\n", buffer_0e);
            #else
            printk("not build lk lcm_esd_check lcm 0x0A is %x-----------------\n", buffer_0a);
	    printk("lcm_esd_check lcm 0x0B is %x-----------------\n", buffer_0b);
	    printk("lcm_esd_check lcm 0x0D is %x-----------------\n", buffer_0d);
	    printk("lcm_esd_check lcm 0x0E is %x-----------------\n", buffer_0e);
            #endif
	
	    if ((buffer_0a==0x9C)&&(buffer_0b==0x00)&&(buffer_0d==0x00)){
		    //printk("diablox_lcd lcm_esd_check done\n");
		    retval = 0;
	    }else{
		    //printk("diablox_lcd lcm_esd_check return true\n");
		    retval = 1;
	    }
    }

	return retval;
    //#endif
}

static unsigned int lcm_esd_recover(void)
{
    //printk("%s \n",__FUNCTION__);
    
    //lcm_resume();
    lcm_init();

    return 1;
}



static unsigned int lcm_compare_id(void)
{
    unsigned int id=0;
    unsigned char buffer[2];
    unsigned int array[16];  
    int i;
   // lcm_util.set_gpio_mode(GPIO131, GPIO_MODE_00);
   // lcm_util.set_gpio_dir(GPIO131, GPIO_DIR_OUT); 
    //lcm_util.set_gpio_pull_enable(GPIO131, GPIO_PULL_DISABLE); 
    lcm_util.set_gpio_mode(GPIO112, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO112, GPIO_DIR_OUT); 
    lcm_util.set_gpio_pull_enable(GPIO112, GPIO_PULL_DISABLE); 

    lcm_util.set_gpio_mode(LCD_LDO_ENP_GPIO_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(LCD_LDO_ENP_GPIO_PIN, GPIO_DIR_OUT); 
    lcm_util.set_gpio_pull_enable(LCD_LDO_ENP_GPIO_PIN, GPIO_PULL_DISABLE); 

    lcm_util.set_gpio_mode(LCD_LDO_ENN_GPIO_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(LCD_LDO_ENN_GPIO_PIN, GPIO_DIR_OUT); 
    lcm_util.set_gpio_pull_enable(LCD_LDO_ENN_GPIO_PIN, GPIO_PULL_DISABLE); 

    //lcm_util.set_gpio_out(GPIO131 , 0); //for GPIO reset type  reset low 
    lcm_util.set_gpio_out(GPIO112 , 0); //for GPIO reset type  reset low 

    MDELAY(50);

    SET_GPIO_OUT(LCD_LDO_ENP_GPIO_PIN , 1);//power on +5
    //MDELAY(2);
    MDELAY(10);
    SET_GPIO_OUT(LCD_LDO_ENN_GPIO_PIN , 1);//power on -5
    MDELAY(100);

   // lcm_util.set_gpio_out(GPIO131 , 1);	//for LCD idle current= 3.8mA PR454011
    lcm_util.set_gpio_out(GPIO112 , 1);

    MDELAY(50);
    //CMD1
    TC358768_DCS_write_1A_1P(0xFF,0x00);
    MDELAY(5);

    for(i=0;i<10;i++)
    {
        array[0] = 0x00013700;// read id return two byte,version and id
        dsi_set_cmdq(array, 1, 1);

        read_reg_v2(0xF4, buffer, 1);
        MDELAY(20);
        lcd_id = buffer[0];
        if (lcd_id == LCM_ID_NT35596)
           break;
    }

    if(lcd_id == LCM_ID_NT35596)
    	return 1;
    else
        return 0;


}

LCM_DRIVER nt35596_diabloxplus_hd_tdt_drv = 
{
    .name			= "nt35596_diabloxplus_hd_tdt",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    .esd_check		= lcm_esd_check,
    .esd_recover	= lcm_esd_recover,
};
