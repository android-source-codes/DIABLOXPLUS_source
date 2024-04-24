

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

#define LCM_ID_NT35695 (0xF5)

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

void TC358768_DCS_write_1P(unsigned char cmd, unsigned char para)
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

void TC358768_DCS_write_0P(unsigned char cmd)
{
    unsigned int data_array[16];
    
    data_array[0]=(0x00000500 | (cmd<<16));
    dsi_set_cmdq(data_array, 1, 1);

}																																									

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


    params->dsi.vertical_sync_active				= 2;
    params->dsi.vertical_backporch					= 2; //28;//30,18;
    params->dsi.vertical_frontporch					= 10; //30;//20;
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 4;
    params->dsi.horizontal_backporch				= 50;//118
    params->dsi.horizontal_frontporch				= 50;//118
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.pll_select=0;	 // 1  //0: MIPI_PLL; 1: LVDS_PLL
    // Bit rate calculation
    //1 Every lane speed
    //params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    //params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
    //params->dsi.fbk_div =16;  //16-58hz,14-52hz,11-38hz  //0X12  // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
    params->dsi.PLL_CLOCK = 442; //clock set 292MHz, so bit rate 292*2MHz
    params->dsi.ssc_disable=1; //1:disable ssc , 0:enable ssc

}


static void init_lcm_registers(void)
{
	
    //NT35695 + LGD6" Panel Initial Code

    TC358768_DCS_write_1P(0xFF,0xE0); //CMD page select
    MDELAY(1);

    TC358768_DCS_write_1P(0xFB,0x01); //NON-RELOAD CMD

    //Power-Related
    TC358768_DCS_write_1P(0xFF,0x20);
    TC358768_DCS_write_1P(0x00,0x01);
    TC358768_DCS_write_1P(0x01,0x55);
    TC358768_DCS_write_1P(0x02,0x45);
    TC358768_DCS_write_1P(0x03,0x55);
    TC358768_DCS_write_1P(0x05,0x50);

    TC358768_DCS_write_1P(0x06, 0x4A);
    TC358768_DCS_write_1P(0x07, 0x24);
    TC358768_DCS_write_1P(0x08, 0x0C);
    TC358768_DCS_write_1P(0x0B, 0x87);
    TC358768_DCS_write_1P(0x0C, 0x87);
    TC358768_DCS_write_1P(0x0E, 0xB0);
    TC358768_DCS_write_1P(0x0F, 0xB3);
    TC358768_DCS_write_1P(0x11, 0x10);
    TC358768_DCS_write_1P(0x12, 0x10);
    TC358768_DCS_write_1P(0x13, 0x03);
    TC358768_DCS_write_1P(0x14, 0x4A);
    TC358768_DCS_write_1P(0x15, 0x12);
    TC358768_DCS_write_1P(0x16, 0x12);

   
    TC358768_DCS_write_1P(0x58,0x82);
    TC358768_DCS_write_1P(0x59,0x00);
    TC358768_DCS_write_1P(0x5A,0x02);
    TC358768_DCS_write_1P(0x5B,0x00);
    TC358768_DCS_write_1P(0x5C,0x82);
    TC358768_DCS_write_1P(0x5D,0x80);
    TC358768_DCS_write_1P(0x5E,0x02);
    TC358768_DCS_write_1P(0x5F,0x00);


    TC358768_DCS_write_1P(0x30, 0x01);
    TC358768_DCS_write_1P(0x72, 0x31);
    TC358768_DCS_write_1P(0xFB, 0x01);
    TC358768_DCS_write_1P(0xFF, 0x24);
    TC358768_DCS_write_1P(0x00, 0x01);
    TC358768_DCS_write_1P(0x01, 0x0B);
    TC358768_DCS_write_1P(0x02, 0x0C);
    TC358768_DCS_write_1P(0x03, 0x89);
    TC358768_DCS_write_1P(0x04, 0x8A);
    TC358768_DCS_write_1P(0x05, 0x0F);
    TC358768_DCS_write_1P(0x06, 0x10);
    TC358768_DCS_write_1P(0x07, 0x10);
    TC358768_DCS_write_1P(0x08, 0x1C);
    TC358768_DCS_write_1P(0x09, 0x00);
    TC358768_DCS_write_1P(0x0A, 0x00);
    TC358768_DCS_write_1P(0x0B, 0x00);
    TC358768_DCS_write_1P(0x0C, 0x00);
    TC358768_DCS_write_1P(0x0D, 0x13);
    TC358768_DCS_write_1P(0x0E, 0x15);
    TC358768_DCS_write_1P(0x0F, 0x17);

    TC358768_DCS_write_1P(0x10,0x01);
    TC358768_DCS_write_1P(0x11,0x0B);
    TC358768_DCS_write_1P(0x12,0x0C);
    TC358768_DCS_write_1P(0x13,0x89);
    TC358768_DCS_write_1P(0x14,0x8A);
    TC358768_DCS_write_1P(0x15,0x0F);
    TC358768_DCS_write_1P(0x16,0x10);
    TC358768_DCS_write_1P(0x17,0x10);
    TC358768_DCS_write_1P(0x18,0x1C);
    TC358768_DCS_write_1P(0x19,0x00);
    TC358768_DCS_write_1P(0x1A,0x00);
    TC358768_DCS_write_1P(0x1B,0x00);
    TC358768_DCS_write_1P(0x1C,0x00);
    TC358768_DCS_write_1P(0x1D,0x13);
    TC358768_DCS_write_1P(0x1E,0x15);
    TC358768_DCS_write_1P(0x1F,0x17);

    //STV                            
    TC358768_DCS_write_1P(0x20,0x00);
    TC358768_DCS_write_1P(0x21,0x01);
    TC358768_DCS_write_1P(0x22,0x00);
    TC358768_DCS_write_1P(0x23,0x40);
    TC358768_DCS_write_1P(0x24,0x40);
    TC358768_DCS_write_1P(0x25,0x6D);                           
    TC358768_DCS_write_1P(0x26,0x40);
    TC358768_DCS_write_1P(0x27,0x40);


    TC358768_DCS_write_1P(0xBD, 0x20);
    TC358768_DCS_write_1P(0xB6, 0x21);
    TC358768_DCS_write_1P(0xB7, 0x22);
    TC358768_DCS_write_1P(0xB8, 0x07);
    TC358768_DCS_write_1P(0xB9, 0x07);
    TC358768_DCS_write_1P(0xC1, 0x6D);
    TC358768_DCS_write_1P(0xBE, 0x07);
    TC358768_DCS_write_1P(0xBF, 0x07);
    TC358768_DCS_write_1P(0x29, 0xD8);
    TC358768_DCS_write_1P(0x2A, 0x2A);

    TC358768_DCS_write_1P(0x4B, 0x03);
    TC358768_DCS_write_1P(0x4C, 0x11);
    TC358768_DCS_write_1P(0x4D, 0x10);
    TC358768_DCS_write_1P(0x4E, 0x01);
    TC358768_DCS_write_1P(0x4F, 0x01);
    TC358768_DCS_write_1P(0x50, 0x10);
    TC358768_DCS_write_1P(0x51, 0x00);
    TC358768_DCS_write_1P(0x52, 0x80);
    TC358768_DCS_write_1P(0x53, 0x00);
    TC358768_DCS_write_1P(0x56, 0x00);
    TC358768_DCS_write_1P(0x54, 0x07);
    TC358768_DCS_write_1P(0x58, 0x07);
    TC358768_DCS_write_1P(0x55, 0x25);
    TC358768_DCS_write_1P(0x5B, 0x43);
    TC358768_DCS_write_1P(0x5C, 0x00);
    TC358768_DCS_write_1P(0x5F, 0x73);


    TC358768_DCS_write_1P(0x60, 0x73);
    TC358768_DCS_write_1P(0x63, 0x22);
    TC358768_DCS_write_1P(0x64, 0x00);
    TC358768_DCS_write_1P(0x67, 0x08);
    TC358768_DCS_write_1P(0x68, 0x04);
    TC358768_DCS_write_1P(0x7A, 0x80);
    TC358768_DCS_write_1P(0x7B, 0x91);
    TC358768_DCS_write_1P(0x7C, 0xD8);
    TC358768_DCS_write_1P(0x7D, 0x60);
    TC358768_DCS_write_1P(0x93, 0x06);
    TC358768_DCS_write_1P(0x94, 0x06);

    TC358768_DCS_write_1P(0xB3, 0xC0);
    TC358768_DCS_write_1P(0xB4, 0x00);
    TC358768_DCS_write_1P(0xB5, 0x00);
    TC358768_DCS_write_1P(0x78, 0x00);
    TC358768_DCS_write_1P(0x79, 0x00);
    TC358768_DCS_write_1P(0x80, 0x00);
    TC358768_DCS_write_1P(0x83, 0x00);
    TC358768_DCS_write_1P(0x8A, 0x00);
    TC358768_DCS_write_1P(0x9B, 0x0F);


    TC358768_DCS_write_1P(0xC2, 0x00);
    TC358768_DCS_write_1P(0xE3, 0x00);
    TC358768_DCS_write_1P(0xC5, 0xA0);
    TC358768_DCS_write_1P(0xC6, 0x09);
    TC358768_DCS_write_1P(0xFB, 0x01);
    TC358768_DCS_write_1P(0xEC, 0x00);
    TC358768_DCS_write_1P(0xFF, 0x10);

    TC358768_DCS_write_1P(0x3B, 0x03);
    //TC358768_DCS_write_1P(0x35, 0x00);
    TC358768_DCS_write_1P(0xBB, 0x03);

    TC358768_DCS_write_0P(0x11); // Standby out
    MDELAY(120);
    TC358768_DCS_write_0P(0x29);
    MDELAY(40);

}
static unsigned int lcd_id;
static void lcm_init(void)
{
#ifdef BUILD_LK
    printf("%s, %d, nt35695 id = 0x%x\n",__FUNCTION__,__LINE__,lcd_id);
#endif

#if 1
    lcm_util.set_gpio_mode(GPIO112, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO112, GPIO_DIR_OUT); 
    lcm_util.set_gpio_pull_enable(GPIO112, GPIO_PULL_DISABLE); 

    lcm_util.set_gpio_mode(LCD_LDO_ENP_GPIO_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(LCD_LDO_ENP_GPIO_PIN, GPIO_DIR_OUT); 
    lcm_util.set_gpio_pull_enable(LCD_LDO_ENP_GPIO_PIN, GPIO_PULL_DISABLE); 

    lcm_util.set_gpio_mode(LCD_LDO_ENN_GPIO_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(LCD_LDO_ENN_GPIO_PIN, GPIO_DIR_OUT); 
    lcm_util.set_gpio_pull_enable(LCD_LDO_ENN_GPIO_PIN, GPIO_PULL_DISABLE); 

#endif

#if 0
    SET_GPIO_OUT(LCD_LDO_ENP_GPIO_PIN , 1);//power on +5
	MDELAY(2);
	//MDELAY(10);
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

    lcm_util.set_gpio_out(GPIO112 , 0); //for GPIO reset type  reset low 

    MDELAY(50);

    SET_GPIO_OUT(LCD_LDO_ENP_GPIO_PIN , 1);//power on +5
    MDELAY(2);
    SET_GPIO_OUT(LCD_LDO_ENN_GPIO_PIN , 1);//power on -5
    MDELAY(100);

    lcm_util.set_gpio_out(GPIO112 , 1);

    MDELAY(120);
	
	
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
	MDELAY(20);

	data_array[0] = 0x00100500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);//delay more for 3 frames time  17*3=54ms

        data_array[0] = 0x00220500; 
	dsi_set_cmdq(data_array, 1, 1);


	lcm_util.set_gpio_out(GPIO112 , 0);
	MDELAY(10);
	
	MDELAY(1);
	SET_GPIO_OUT(LCD_LDO_ENN_GPIO_PIN , 0);
	MDELAY(2);
	SET_GPIO_OUT(LCD_LDO_ENP_GPIO_PIN , 0);
	
}

static void lcm_resume(void)
{
	unsigned int data_array[16];

        lcm_util.set_gpio_out(GPIO112 , 0); //for GPIO reset type  reset low 	

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
    TC358768_DCS_write_1P(0xFF,0x00);
    MDELAY(5);

    for(i=0;i<10;i++)
    {
        array[0] = 0x00013700;// read id return two byte,version and id
        dsi_set_cmdq(array, 1, 1);

        read_reg_v2(0xF4, buffer, 1);
        MDELAY(20);
        lcd_id = buffer[0];
        if (lcd_id == LCM_ID_NT35695)
           break;
    }

    if(lcd_id == LCM_ID_NT35695)
    	return 1;
    else
        return 0;


}

LCM_DRIVER nt35695_diabloX_hd_drv = 
{
    .name			= "nt35695_diabloX_hd",
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
