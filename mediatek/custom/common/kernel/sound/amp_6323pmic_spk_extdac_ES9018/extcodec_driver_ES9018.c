#include <asm/uaccess.h>
#include <linux/xlog.h>
#include <linux/i2c.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
//#include <platform/mt_reg_base.h>
#include <linux/delay.h>
 
#include "external_codec_driver.h"
 
 
//#define MT6592_I2SOUT_GPIO
//#define CS4398_GPIO_CONTROL
//#define TPA6141_GPIO_CONTROL
//#define NB3N501_GPIO_CONTROL
 
#define GPIO_BASE (0x10005000)
 
 
static bool ecodec_log_on = true;
 
#define ECODEC_PRINTK(fmt, arg...) \
    do { \
        if (ecodec_log_on) xlog_printk(ANDROID_LOG_INFO,"ECODEC", "%s() "fmt"\n", __func__,##arg); \
    }while (0)
 
 
#define ECODEC_I2C_CHANNEL     (2)        //I2CL: SDA2,SCL2
 
// device address,  write: 10011000, read: 10011001
#define ECODEC_SLAVE_ADDR_PIO2 0x48
#define ECODEC_SLAVE_ADDR_PIO3 0x49
 
#define ECODEC_I2C_DEVNAME "ES9018"

 
static unsigned int default_reg[][2]={
     0,0,
     1,0x8c,
     2,0x18,
     3,0x10,
     4,0,
     5,0x7f,
     6,0x42,
     7,0x80,
     8,0x10,
     9,0x22,
     10,0x2d,
     11,0x2,
     12,0x5a,
     13,0,
     14,0x8a,
     15,0,
     16,0,
     17,0xff,
     18,0xff,
     19,0xff,
     20,0x21,
     21,0,
     22,0,
     23,0,
     24,0,
     25,0xea
};
// I2C variable
static struct i2c_client *new_client = NULL;

//#define DISABLE_HIFI_TMPLY //Added by jrd.lipeng: p-sensor conflicts with hifi, so disable hifi temporarily.

static int g_i2c_probe_ret = 0;
 
// new I2C register method
static const struct i2c_device_id ecodec_i2c_id[] = {{ECODEC_I2C_DEVNAME, 0}, {}};
 
//function declration
static int ecodec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ecodec_i2c_remove(struct i2c_client *client);
//i2c driver
struct i2c_driver ecodec_i2c_driver =
{
    .probe = ecodec_i2c_probe,
    .remove = ecodec_i2c_remove,
    .driver = {
        .name = ECODEC_I2C_DEVNAME,
    },
    .id_table = ecodec_i2c_id,
};
 
// function implementation
//read one register
ssize_t static ecodec_read_byte(u8 addr, u8 *returnData)
{
    char        cmd_buf[1] = {0x00};
    char        readData = 0;
    int   ret = 0;
 
    if (!new_client)
    {
        ECODEC_PRINTK("ecodec_read_byte I2C client not initialized!!");
        return -1;
    }
    cmd_buf[0] = addr;
    ret = i2c_master_send(new_client, &cmd_buf[0], 1);
    if (ret < 0)
    {
        ECODEC_PRINTK("ecodec_read_byte read sends command error!!");
        return -1;
    }
    ret = i2c_master_recv(new_client, &readData, 1);
    if (ret < 0)
    {
        ECODEC_PRINTK("ecodec_read_byte reads recv data error!!");
        return -1;
    }
    *returnData = readData;
    //ECODEC_PRINTK("addr 0x%x data 0x%x", addr, readData);
    return 0;
}
 
//read one register
static u8   I2CRead(u8 addr)
{
    u8 regvalue = 0;
    ecodec_read_byte(addr, &regvalue);
    return regvalue;
}
 
// write register
static ssize_t  I2CWrite(u8 addr, u8 writeData)
{
    char      write_data[2] = {0};
    int  ret = 0;
 
    if (!new_client)
    {
        ECODEC_PRINTK("I2C client not initialized!!");
        return -1;
    }
    write_data[0] = addr;         // ex. 0x01
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0)
    {
        ECODEC_PRINTK("write sends command error!!");
        return -1;
    }
    //ECODEC_PRINTK("addr 0x%x data 0x%x", addr, writeData);
    return 0;
}
 
static void  ecodec_set_hw_parameters(DIGITAL_INTERFACE_FORMAT dif)
{
    //MCLKDIV, SpeedMode, format
    volatile u8 reg;
    ECODEC_PRINTK("ecodec_set_hw_parameters (+) dif=%d", dif);
         int i;
        
         for(i=0;i<sizeof(default_reg)/(2*sizeof(int));i++){
                   I2CWrite(default_reg[i][0], default_reg[i][1]);
         }
        
#if 0
    reg = I2CRead(CS4398_MODE);
    reg &= ~CS4398_FUNCMODE_MASK;
    reg |= (0x00 & CS4398_FUNCMODE_MASK);
    I2CWrite(CS4398_MODE, reg); //functional mode (1:0), 00: Single-speed mode
 
    reg = I2CRead(CS4398_MISC1);
    reg &= ~CS4398_MCLKDIV_MASK;
    reg |= (0x00 & CS4398_MCLKDIV_MASK);
    I2CWrite(CS4398_MISC1, reg); //MCLKDIV2/MCLKDIV3 (4:3), 00: MCLKDIV2 disable, MCLKDIV3 disable
 
    reg = I2CRead(CS4398_MODE);
    reg &= ~CS4398_DIF_MASK;
    switch (dif)
    {
        case DIF_LEFT_JUSTIFIED:
            reg |= (CS4398_DIF_LJ & CS4398_DIF_MASK);
            I2CWrite(CS4398_MODE, reg);
            break;
        case DIF_I2S:
            reg |= (CS4398_DIF_I2S & CS4398_DIF_MASK);
            I2CWrite(CS4398_MODE, reg); //Digital interface format (6:4), 001: I2S mode(up to 24bit)
            break;
        case DIF_RIGHT_JUSTIFIED_16BIT:
            reg |= (CS4398_DIF_RJ_16 & CS4398_DIF_MASK);
            I2CWrite(CS4398_MODE, reg);
            break;
        case DIF_RIGHT_JUSTIFIED_24BIT:
            reg |= (CS4398_DIF_RJ_24 & CS4398_DIF_MASK);
            I2CWrite(CS4398_MODE, reg);
            break;
        case DIF_RIGHT_JUSTIFIED_20BIT:
            reg |= (CS4398_DIF_RJ_20 & CS4398_DIF_MASK);
            I2CWrite(CS4398_MODE, reg);
            break;
        case DIF_RIGHT_JUSTIFIED_18BIT:
            reg |= (CS4398_DIF_RJ_18 & CS4398_DIF_MASK);
            I2CWrite(CS4398_MODE, reg);
            break;
        default:
            ECODEC_PRINTK("digital interface format=%d error!", dif);
            break;
    }
 
    reg = I2CRead(CS4398_MUTE);
    reg &= ~CS4398_PAMUTE_MASK;
    reg |= (0x00 & CS4398_PAMUTE_MASK);
    I2CWrite(CS4398_MUTE, reg); //PAMUTE (7),  0: disable PCM AutoMute
#endif
#if 0
    reg = I2CRead(CS4398_MIXING);
    reg &= ~CS4398_VOLBEQA_MASK;
    reg |= (0x80 & CS4398_VOLBEQA_MASK);
    I2CWrite(CS4398_MIXING, reg); //VOLB=A (7),  1: set Vol B = A
#endif
    //ECODEC_PRINTK("ecodec_set_hw_parameters (-)");
}
 
static void  ecodec_set_control_port(u8 enable)
{
    //CPEN
    volatile u8 reg;
    ECODEC_PRINTK("ecodec_set_control_port (+), enable=%d ", enable);
    //ECODEC_PRINTK("ecodec_set_control_port(-)");
}
 
static void  ecodec_set_power(u8 enable)
{
    //PDN
    volatile u8 reg;
    ECODEC_PRINTK("ecodec_set_power(+) enable=%d ", enable);
    //ECODEC_PRINTK("ecodec_set_power(-)");
}
 
static void  ecodec_mute(u8 enable)
{
    //MUTE
    volatile u8 reg;
    ECODEC_PRINTK("ecodec_mute CS4398_MUTE(+), enable=%d ", enable);
    //ECODEC_PRINTK("ecodec_mute CS4398_MUTE(-)");
}
 
static void  ecodec_set_volume(u8 leftright, u8 gain)
{
    //Volume
    // 0: 0dB, 6: -3dB, ..., 255: -127.5dB
    ECODEC_PRINTK("ecodec_set_volume leftright=%d, gain=%d ", leftright, gain);
}
 
void ecodec_dump_register(void)
{
    int i = 0;

    ECODEC_PRINTK("ecodec_dump_register");
    for(i = 0; i <= ES9018_REG_END; i++) {
        ECODEC_PRINTK("ES9018_REG#%d = 0x%x ", i, I2CRead(i));
    }
}
 
u8 ExtCodec_ReadReg(u8 addr)
{
    if(addr < ES9018_REG_FIRST || addr > ES9018_REG_END) {
        ECODEC_PRINTK("[%s]wrong addr: %u", __func__, addr);
        return -1;
    }

    return I2CRead(addr);
}

int ExtCodec_WriteReg(u8 addr, u8 val)
{
    if(addr < ES9018_REG_FIRST || addr > ES9018_REG_END) {
        ECODEC_PRINTK("[%s]wrong addr: %u", __func__, addr);
        return -1;
    }

    return I2CWrite(addr, val);
} 
 
 
static void ecodec_suspend(void)
{
    ECODEC_PRINTK("ecodec_suspend");
    //ecodec_resetRegister();
}
 
static void ecodec_resume(void)
{
    ECODEC_PRINTK("ecodec_resume");
}
 
static int ecodec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = -1;
    int i = 0;
    volatile u8 reg;
    new_client = client;
    //new_client->timing = 400;
    ECODEC_PRINTK("client timing=%dK ", new_client->timing);

    for(i = 0; i < 5; i++) {
        ret = ecodec_read_byte(64, &reg) ;
        if(ret == 0) {
            break;
        }
        msleep(5);
    }

#if 1//Added by jrd.lipeng for compatibility of hifi chip revision V & revision W
    if((reg >> 5) & 0x1) {//revision V
        default_reg[25][1] = 0xf0;
    } else {//revision W
        default_reg[25][1] = 0xea;
    }
#endif

    reg = (reg >> 2) & 0x7;
    ECODEC_PRINTK("(%d)Find ChipID 0x%x  !!", i, reg);

    //when probe done, switch off the chip
    mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN, GPIO_OUT_ZERO); // RST tied low
    msleep(100);
    if(ret < 0) {
        //hwPowerDown(MT6323_POWER_LDO_VGP1, "Audio");
        mt_set_gpio_mode(GPIO_HIFI_VCCA_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_out(GPIO_HIFI_VCCA_EN_PIN, GPIO_OUT_ZERO);//VCCA, 3,3v
        mt_set_gpio_mode(GPIO_HIFI_AVCC_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_out(GPIO_HIFI_AVCC_EN_PIN, GPIO_OUT_ZERO);//AVCC_R & AVCC_L, 3.3v
        mt_set_gpio_mode(GPIO_HIFI_DVCC_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_out(GPIO_HIFI_DVCC_EN_PIN, GPIO_OUT_ZERO);//DVCC, 1.8v
    }
    cust_extHPAmp_gpio_off();

    g_i2c_probe_ret = ret;
    
    return ret;
}
 
static int ecodec_i2c_remove(struct i2c_client *client)
{
    ECODEC_PRINTK("ecodec_i2c_remove");
    new_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&ecodec_i2c_driver);
    return 0;
}

static int ecodec_check_register(unsigned short addr)
{
    int i = 0;
    struct i2c_adapter *adapter = NULL;
    struct i2c_client * client = NULL;
    struct i2c_board_info c = {I2C_BOARD_INFO(ECODEC_I2C_DEVNAME, addr)};
    
    adapter = i2c_get_adapter(ECODEC_I2C_CHANNEL);
    if (adapter) {
        client = i2c_new_device(adapter, &c);
        if(g_i2c_probe_ret < 0 || client == NULL) {
            ECODEC_PRINTK("Can not register HIFI through addr: 0x%x\n", c.addr);
            return -1;
        } else {
            ECODEC_PRINTK("HIFI registered with addr 0x%x successed.\n", c.addr);
            return 0;
        }
        i2c_put_adapter(adapter);
    } else {
        ECODEC_PRINTK("Can not get adapter %d\n", ECODEC_I2C_CHANNEL);
        return -1;
    }
}

static int ecodec_register(void)
{
#ifdef DISABLE_HIFI_TMPLY
    cust_extcodec_gpio_off();
    return -1;
#endif
    ECODEC_PRINTK("ecodec_register");
    
    //hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "Audio");
    /* ES9018K2M needs 4 LDOs: three 3.3v for VCCA, AVCC_R, AVCC_L &  one 1.8v for DVCC. 
     * These 4 LDOs are provided by 4 voltage regulator IC.*/
    mt_set_gpio_mode(GPIO_HIFI_DVCC_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_HIFI_DVCC_EN_PIN, GPIO_OUT_ONE);//DVCC, 1.8v
    msleep(1);
    mt_set_gpio_mode(GPIO_HIFI_VCCA_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_HIFI_VCCA_EN_PIN, GPIO_OUT_ONE);//VCCA, 3,3v
    mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN, GPIO_MODE_00);
    msleep(150);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN, GPIO_OUT_ONE); // RST tied high
    msleep(100);

    if (i2c_add_driver(&ecodec_i2c_driver))
    {
        ECODEC_PRINTK("fail to add device into i2c");
        return -1;
    }
#if 1//Added by jrd.lipeng 
    /* 1. i2c address conflict issue of p-sensor & hifi; 
     * 2.the compatibility of different hifi i2c addresses on PIO2 & PIO3 */
    int level = 0;

    /*Judge the HW board version through the input level of GPIO8: high >= PIO3; low: < PIO3*/
    mt_set_gpio_mode(GPIO_HW_VER_CHECK_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_HW_VER_CHECK_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_HW_VER_CHECK_PIN, GPIO_PULL_DISABLE);

    level = mt_get_gpio_in(GPIO_HW_VER_CHECK_PIN);
    ECODEC_PRINTK("GPIO_HW_VER_CHECK_PIN(0x%x) input: %d", GPIO_HW_VER_CHECK_PIN, level);
    if(level == GPIO_IN_ZERO) {//HW version < PIO3
        if(ecodec_check_register(ECODEC_SLAVE_ADDR_PIO2)) {
            if(ecodec_check_register(ECODEC_SLAVE_ADDR_PIO3)) {
                goto err;
            }
        } 
    } else {//HW version >= PIO3
        if(ecodec_check_register(ECODEC_SLAVE_ADDR_PIO3)) {
            if(ecodec_check_register(ECODEC_SLAVE_ADDR_PIO2)) {
                goto err;
            }
        } 
    }
#endif

    return 0;
err:
    mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN, GPIO_OUT_ZERO); // RST tied low

    //hwPowerDown(MT6323_POWER_LDO_VGP1, "Audio");
    mt_set_gpio_mode(GPIO_HIFI_VCCA_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_HIFI_VCCA_EN_PIN, GPIO_OUT_ZERO);//VCCA, 3,3v
    mt_set_gpio_mode(GPIO_HIFI_AVCC_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_HIFI_AVCC_EN_PIN, GPIO_OUT_ZERO);//AVCC_R & AVCC_L, 3.3v
    mt_set_gpio_mode(GPIO_HIFI_DVCC_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_HIFI_DVCC_EN_PIN, GPIO_OUT_ZERO);//DVCC, 1.8v
    
    cust_extHPAmp_gpio_off();
    return -1;
}
#if 0
static ssize_t ecodec_resetRegister(void)
{
    // set registers to default value
    ECODEC_PRINTK("ecodec_resetRegister");
    I2CWrite(CS4398_MODE, 0x00);
    I2CWrite(CS4398_MIXING, 0x09);
    I2CWrite(CS4398_MUTE, 0xC0);
    I2CWrite(CS4398_VOLA, 0x18); //default -12dB
    I2CWrite(CS4398_VOLB, 0x18); //default -12dB
    I2CWrite(CS4398_RAMP, 0xb0);
    I2CWrite(CS4398_MISC1, 0x80);
    I2CWrite(CS4398_MISC2, 0x08);
    return 0;
}
#endif
 
 
void ExtCodec_Init()
{
    ECODEC_PRINTK("ExtCodec_Init ");
}
 
void ExtCodec_PowerOn(void)
{
    ecodec_set_control_port(1);
    ecodec_set_hw_parameters(DIF_I2S); //Need not power enable(PDN) when R/W register, onlt need turn on CPEN
    ecodec_set_volume(0, 14); //default -7dB
    ecodec_set_volume(1, 14);
    ecodec_set_power(1);
    //ExtCodec_DumpReg();
}
 
void ExtCodec_PowerOff(void)
{
    ecodec_set_power(0);
}
 
bool ExtCodec_Register(void)
{
    int ret = -1;
    ECODEC_PRINTK("ExtCodec_Register ");
    ret = ecodec_register();
    return ret == 0 ? true : false;
}
 
void ExtCodec_Mute(void)
{
    ecodec_mute(1);
}
 
void ExtCodec_SetGain(u8 leftright, u8 gain)
{
    ecodec_set_volume(leftright, gain);
}
 
void ExtCodec_DumpReg(void)
{
    ecodec_dump_register();
}
 
int ExternalCodec(void)
{
    return 1;
}
 
void ExtCodecDevice_Suspend(void)
{
    ecodec_suspend();
}
 
void ExtCodecDevice_Resume(void)
{
    ecodec_resume();
}
 
void cust_extcodec_gpio_on()
{
    ECODEC_PRINTK("Set GPIO for ES9018 ON");
    mt_set_gpio_mode(GPIO_HIFI_AVCC_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_HIFI_AVCC_EN_PIN, GPIO_OUT_ONE);//AVCC_R & AVCC_L, 3.3v
    msleep(50);
    mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN, GPIO_OUT_ONE); // RST tied high
    msleep(100);//delay 100ms before writing hifi registers.
}
 
void cust_extcodec_gpio_off()
{
    ECODEC_PRINTK("Set GPIO for ES9018 OFF "); 
#if 0//lipeng debug
    I2CWrite(14, 0x0);
    ECODEC_PRINTK("reg14: %u", I2CRead(14));
    msleep(100);
#endif
    mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN, GPIO_OUT_ZERO); // RST tied low

    msleep(50);
    mt_set_gpio_mode(GPIO_HIFI_AVCC_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_HIFI_AVCC_EN_PIN, GPIO_OUT_ZERO);//AVCC_R & AVCC_L, 3.3v
}
#if 1//lipeng debug
static unsigned int switch_delay = 1;//unit: ms
void jrd_set_switch_delay(unsigned int ms)
{
    ECODEC_PRINTK("pre-switch_delay: %u, cur-switch_delay: %u", switch_delay, ms);
    switch_delay = ms;
}
#endif
 
#define MAX_TRY (30)//average value is about 12
void cust_extHPAmp_gpio_on()
{
    //external HPAmp, gain set to 0dB, AMP set to enable
#ifdef TPA6141_GPIO_CONTROL // 6592 GPIO setting
    ECODEC_PRINTK("Set GPIO for TPA6141 ON  ");
    mt_set_gpio_mode(GPIO_AUD_EXTHP_GAIN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTHP_GAIN_PIN, GPIO_OUT_ZERO); // fixed at 0dB
    mt_set_gpio_mode(GPIO_AUD_EXTHP_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTHP_EN_PIN, GPIO_OUT_ONE); // enable HP amp
#else
    ECODEC_PRINTK("Set GPIO for MAX97220 ON ");
    mt_set_gpio_mode(GPIO_AUD_EXTHP_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTHP_EN_PIN, GPIO_OUT_ONE); // enable HP amp
#endif
#if 1//lipeng debug
    int times = 0;
    while((I2CRead(64) & (1 << 0)) == 0 && times < MAX_TRY) {
        times++;
        msleep(20);
    }
    ECODEC_PRINTK("The Jitter Eliminator is locked. Reg64: 0x%x. times: %d", I2CRead(64), times);
#endif
    msleep(switch_delay);//lipeng debug

    /*GPIO_SWITCH1_1V8_PIN: HIGH <-> PMIC; LOW <-> HIFI*/
    mt_set_gpio_mode(GPIO_SWITCH1_1V8_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_SWITCH1_1V8_PIN, GPIO_OUT_ZERO);
}
 
void cust_extHPAmp_gpio_off()
{
    /*GPIO_SWITCH1_1V8_PIN: HIGH <-> PMIC; LOW <-> HIFI*/
    mt_set_gpio_mode(GPIO_SWITCH1_1V8_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_SWITCH1_1V8_PIN, GPIO_OUT_ONE);

    //external HPAmp, AMP set to disable, gain set to 0dB
#ifdef TPA6141_GPIO_CONTROL // 6592 GPIO setting
    ECODEC_PRINTK("Set GPIO for TPA6141 OFF ");
    mt_set_gpio_mode(GPIO_AUD_EXTHP_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTHP_EN_PIN, GPIO_OUT_ZERO); // disable HP amp
#else
    ECODEC_PRINTK("Set GPIO for MAX97220 OFF ");
    mt_set_gpio_mode(GPIO_AUD_EXTHP_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTHP_EN_PIN, GPIO_OUT_ZERO); // disable HP amp
#endif
}
 
void cust_extPLL_gpio_config()
{
    //external PLL, set S0/S1
#ifdef NB3N501_GPIO_CONTROL // 6592 GPIO setting
    ECODEC_PRINTK("Set GPIO for NB3N501 ");
    mt_set_gpio_mode(GPIO_AUD_EXTPLL_S0_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTPLL_S0_PIN, GPIO_OUT_ONE); // S0 = 1
    mt_set_gpio_mode(GPIO_AUD_EXTPLL_S1_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTPLL_S1_PIN, GPIO_OUT_ONE);   // S1 = 1
#endif
 
}

//added jrd.lipeng for 13TAH01 EN50332
void cust_extcodec_params_config(int select)
{
    switch(select) {
        case 0://lower volume for global
            default_reg[20][1] = 0x21;
            break;
        case 1://higher volume for china
            default_reg[20][1] = 0x3f;
            break;
        default:
            default_reg[20][1] = 0x21;
            break;
    }
}

