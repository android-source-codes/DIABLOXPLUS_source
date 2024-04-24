
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include "cust_gpio_usage.h"

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#elif defined(MT6577)
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#else
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/version.h>


#include <linux/watchdog.h>
#include <mach/ext_wd_drv.h>
#include <mach/mt_wdt.h>

#include "cust_gpio_usage.h"

#include "zinitix_touch.h"
#include "zinitix_touch_zxt_firmware.h"


#define TPD_RES_MAX_X		1080
#define TPD_RES_MAX_Y		1920


#if TOUCH_ONESHOT_UPGRADE
//#include "zinitix_touch_zxt_firmware.h"
//#include "zinitix_touch_zxt_firmware_taiqi.h"
//#include "zinitix_touch_zxt_firmware_saihua.h"

#if 0
#define TAIQI_MODULE      0x5449 //TI = m_firmware_data_01 
#define SAIHUA_MODULE        0x5341 //SA = m_firmware_data_02

#define TSP_TYPE_COUNT	2
const u8 *m_pFirmware [TSP_TYPE_COUNT] = {(u8*)m_firmware_data_01,(u8*)m_firmware_data_02,};
u8 m_FirmwareIdx=0;
u16 tsm_module_id = 0;
#else
#define TSP_TYPE_COUNT	1
const u8 *m_pFirmware [TSP_TYPE_COUNT] = {(u8*)m_firmware_data,};
u8 m_FirmwareIdx=0;
#endif
#endif

extern struct tpd_device *tpd;

struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static void tpd_eint_interrupt_handler(void);

#define TP_SYSFS_SUPPORT
int TP_sysfs_init(void);
void TP_sysfs_exit(void);
static int CTP_sysfs_init(void);
static void CTP_sysfs_exit(void);
int ts_set_touchmode(u16 value);
int get_raw_data(u8 *buff, int skip_cnt);
static bool ts_get_raw_data();

extern int get_charger_detect_status(void);

extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
								  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
								  kal_bool auto_umask);


static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag = 0;
static volatile int tpd_halt = 0;

#define TPD_OK 0
#define zinitix_printk printk
#ifdef MT6575
#else
#define TPD_HIGH_SPEED_DMA
#endif
#ifdef ESD_CHECK	
static struct delayed_work ctp_read_id_work;
static struct workqueue_struct * ctp_read_id_workqueue = NULL;
#endif
#if ZINITIX_ESD_TIMER_INTERVAL
static struct workqueue_struct *esd_work_struct;
static void ts_work_queue();
static DECLARE_WORK(esd_reset_work,ts_work_queue);
u8 use_esd_timer;
struct timer_list esd_timeout_tmr;
static void ts_esd_timer_start(u16 sec, struct tpd_device *tpd);
static void ts_esd_timer_stop(struct tpd_device *tpd);
static void ts_esd_timeout_handler(unsigned long data);
#endif

//=============================================================================

//#defing TPD_CLODE_POWER_IN_SLEEP

//register define
#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT 3

#ifdef TPD_HAVE_BUTTON
#if	1
#ifdef TPD_HAVE_BUTTON
#define TPD_KEY_COUNT 3
//#define TPD_KEYS        {KEY_MENU, KEY_HOMEPAGE,KEY_BACK }
#define TPD_KEYS        {KEY_BACK, KEY_HOMEPAGE,KEY_MENU }
#define TPD_KEYS_DIM            {{40,1000,80,50},{120,1000,80,50},{200,1000,80,50}}


static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;

#endif

#else
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

/* Button Enum */
u8 button[TPD_KEY_COUNT] = {ICON_BUTTON_UNCHANGE,};
#endif

#define	TOUCH_MODE	0

#define DELAY_FOR_TRANSCATION 50
#define DELAY_FOR_POST_TRANSCATION 10

#define    MAX_SUPPORTED_FINGER_NUM    5

//#define TPD_RES_MAX_X		720		
//#define TPD_RES_MAX_Y		1280

#define TOUCH_MAX_X 20
#define TOUCH_MAX_Y 11
#define TOUCH_SDND_MODE     6
#define TOUCH_PPDND_MODE    11

#define FIRMWARE_SIZE       64*1024
struct zinitix_sys_fw_info 
{
    int file_err_num;
    u8 local_buf[FIRMWARE_SIZE];
};

static u8 finger_status[MAX_SUPPORTED_FINGER_NUM] = {0};

struct zinitix_sys_fw_info *zinitix_sys_fw_info = NULL;
static int sys_err_num = 0;

//u16 zinitix_raw_data[RAWDATA_SIZE]= {0};
static u16 m_nTouchMode = 0;
static u16 m_nDndData[TOUCH_MAX_X*TOUCH_MAX_Y] = {0};
static u16 m_nFrequency;
static u16 m_nShift;

unsigned int max_raw_data = 0;
unsigned int min_raw_data = 0;

static int min_node_val = 0;
static int max_node_val = 0;

static int raw_data_result = 0;                                                                                                                                                                                

static int diff_flag = 0;
static int v_diff_result = 0;
static int h_diff_result = 0;

u16  ic_int_mask = 0;
struct _ts_zinitix_coord {
    u16 x;
    u16 y;
    u8 width;
    u8 sub_status;
};

struct _ts_zinitix_point_info {
    u16 status;
    u8 finger_cnt;
    u8 time_stamp;
    struct _ts_zinitix_coord    coord[MAX_SUPPORTED_FINGER_NUM];
};


static struct _ts_zinitix_point_info reported_touch_info;
static struct _ts_zinitix_point_info zinitix_i2c_touch_info;
static int tpd_resume(struct i2c_client *client);

struct _ts_zinitix_point_info touch_info;

struct touch_info {
  int y[3];
  int x[3];
  int p[3];
  int count;
};

static const struct i2c_device_id tpd_id[] = {{TPD_DEVICE,0},{}};

static struct i2c_board_info __initdata zinitix_i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (0x20>>1))};// 0x20

static struct i2c_driver tpd_i2c_driver = {
.driver = {
 .name = TPD_DEVICE,
// .owner = THIS_MODULE,
},
.probe = tpd_probe,
.remove = __devexit_p(tpd_remove),
.id_table = tpd_id,
.detect = tpd_detect,
//.address_data = &addr_data,
};


#ifdef TPD_HIGH_SPEED_DMA
#define TC_SECTOR_SZ		8
#define	CTP_I2C_DMA_BUF_SIZE	(4096)
static u8 *CTPI2CDMABuf_va = NULL;
static u32 CTPI2CDMABuf_pa = NULL;
#endif

//#ifdef ZINITIX_ESD_TIMER_INTERVAL
//decalare
static int zinitix_resume_proc(void);
static void   tpd_clear_report_data(void);

//#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36))
static int ts_misc_fops_ioctl(struct inode *inode, struct file *filp,
        unsigned int cmd, unsigned long arg);
#else
static long ts_misc_fops_ioctl(struct file *filp,
        unsigned int cmd, unsigned long arg);
#endif
static int ts_misc_fops_open(struct inode *inode, struct file *filp);
static int ts_misc_fops_close(struct inode *inode, struct file *filp);

static const struct file_operations ts_misc_fops = {
    .owner = THIS_MODULE,
    .open = ts_misc_fops_open,
    .release = ts_misc_fops_close,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36))
    .ioctl = ts_misc_fops_ioctl,
#else
    .unlocked_ioctl = ts_misc_fops_ioctl,
#endif
};
static struct miscdevice touch_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "zinitix_touch_misc",
    .fops = &ts_misc_fops,
};

#define TSP_CMD_STR_LEN 32
#define TSP_CMD_RESULT_STR_LEN 512
#define TSP_CMD_PARAM_NUM 8
#define TSP_BUF_SIZE 1024

#define TOUCH_IOCTL_BASE    0xbc
#define TOUCH_IOCTL_GET_DEBUGMSG_STATE      _IOW(TOUCH_IOCTL_BASE, 0, int)
#define TOUCH_IOCTL_SET_DEBUGMSG_STATE      _IOW(TOUCH_IOCTL_BASE, 1, int)
#define TOUCH_IOCTL_GET_CHIP_REVISION       _IOW(TOUCH_IOCTL_BASE, 2, int)
#define TOUCH_IOCTL_GET_FW_VERSION      _IOW(TOUCH_IOCTL_BASE, 3, int)
#define TOUCH_IOCTL_GET_REG_DATA_VERSION    _IOW(TOUCH_IOCTL_BASE, 4, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_SIZE     _IOW(TOUCH_IOCTL_BASE, 5, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_DATA     _IOW(TOUCH_IOCTL_BASE, 6, int)
#define TOUCH_IOCTL_START_UPGRADE       _IOW(TOUCH_IOCTL_BASE, 7, int)
#define TOUCH_IOCTL_GET_X_NODE_NUM      _IOW(TOUCH_IOCTL_BASE, 8, int)
#define TOUCH_IOCTL_GET_Y_NODE_NUM      _IOW(TOUCH_IOCTL_BASE, 9, int)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM      _IOW(TOUCH_IOCTL_BASE, 10, int)
#define TOUCH_IOCTL_SET_RAW_DATA_MODE       _IOW(TOUCH_IOCTL_BASE, 11, int)
#define TOUCH_IOCTL_GET_RAW_DATA        _IOW(TOUCH_IOCTL_BASE, 12, int)
#define TOUCH_IOCTL_GET_X_RESOLUTION        _IOW(TOUCH_IOCTL_BASE, 13, int)
#define TOUCH_IOCTL_GET_Y_RESOLUTION        _IOW(TOUCH_IOCTL_BASE, 14, int)
#define TOUCH_IOCTL_HW_CALIBRAION       _IOW(TOUCH_IOCTL_BASE, 15, int)
#define TOUCH_IOCTL_GET_REG         _IOW(TOUCH_IOCTL_BASE, 16, int)
#define TOUCH_IOCTL_SET_REG         _IOW(TOUCH_IOCTL_BASE, 17, int)
#define TOUCH_IOCTL_SEND_SAVE_STATUS        _IOW(TOUCH_IOCTL_BASE, 18, int)
#define TOUCH_IOCTL_DONOT_TOUCH_EVENT       _IOW(TOUCH_IOCTL_BASE, 19, int)

struct _ts_capa_info {
    u16 signature;
    u16 ic_revision;
    u16 firmware_version;
    u16 firmware_minor_version;
    u16 reg_data_version;
    u16 x_resolution;
    u16 y_resolution;
    u32 ic_fw_size;
    u32 MaxX;
    u32 MaxY;
    u32 MinX;
    u32 MinY;
    u32 Orientation;
    u8 gesture_support;
    u16 multi_fingers;
    u16 button_num;
    u16 ic_int_mask;
    u16 x_node_num;
    u16 y_node_num;
    u16 total_node_num;
    u16 hw_id;
    u16 afe_frequency;  
    u8  i2s_checksum;

};

struct zinitix_touch_dev {
    struct input_dev *input_dev;
    struct task_struct *task;
    wait_queue_head_t   wait;
#if !USE_THREADED_IRQ   
    struct work_struct  work;
#endif
    struct work_struct  tmr_work;
    struct i2c_client *client;
    struct semaphore update_lock;
    u32 i2c_dev_addr;
    struct _ts_capa_info cap_info;
    char    phys[32];

    struct _ts_zinitix_point_info touch_info;
    struct _ts_zinitix_point_info reported_touch_info;
    u16 icon_event_reg; 
    u16 event_type;
    u32 int_gpio_num;
    u32 irq;
    u8 button[MAX_SUPPORTED_BUTTON_NUM];
    u8 work_proceedure;
    struct semaphore work_proceedure_lock;
    u8 use_esd_timer;

    u16 debug_reg[8];   // for debug
    bool in_esd_timer;
    struct timer_list esd_timeout_tmr;

#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    struct semaphore    raw_data_lock;
    u16 touch_mode;
   // s16 * cur_data;
    u16 cur_data[TOUCH_MAX_X*TOUCH_MAX_Y];
    u8 update;
#if SEC_TSP_FACTORY_TEST
    s16 *normal_data;
    s16 *delta_data;    
    u16 *dnd_data;      

    struct list_head    cmd_list_head;
    u8  cmd_state;
    char    cmd[TSP_CMD_STR_LEN];
    int     cmd_param[TSP_CMD_PARAM_NUM];
    char    cmd_result[TSP_CMD_RESULT_STR_LEN];
    struct mutex    cmd_lock;
    bool    cmd_is_running;
#endif      

};
struct zinitix_touch_dev *misc_touch_dev;

//define sub functions
//=====================================================================================

#ifdef TPD_HIGH_SPEED_DMA
#define I2C_RETRY_CNT 5

static int zinitix_i2c_DMAread(struct i2c_client *client, U16 addr, U16 len, U8 *rxbuf)
{
    int retry,i;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG,
			.flags = 0,
			.len = 2,
			.buf = &addr,
		},
		
		{
			.addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG,
			.flags = I2C_M_RD,
			.len = len,
			.buf = CTPI2CDMABuf_pa,
		}
	};
	for (retry = 0; retry < I2C_RETRY_CNT; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(5);
	}
	if (retry == I2C_RETRY_CNT) {
		printk(KERN_ERR "i2c_read_block retry over %d\n",I2C_RETRY_CNT);
		return -EIO;
	}
	
	for(i=0;i<len;i++)
            rxbuf[i]=CTPI2CDMABuf_va[i];
    
	client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);
	return 0;
}

static int zinitix_i2c_DMA_write_ii(struct i2c_client *client,  u16 addr, U8 *txbuf, U16 tlen)
{
    int retry,i;
    
    u8 reg_high = (addr >> 8) & 0xff;
    u8 reg_low = addr & 0xff;
    
    int writelen = TC_SECTOR_SZ + 2;
    int ret = 0;
    
    retry = 3;

     //address
    CTPI2CDMABuf_va[0] = reg_low;              
    CTPI2CDMABuf_va[1] = reg_high;      
    
   if(tlen!=0) {
        for(i = 0 ; i < tlen; i++){
            CTPI2CDMABuf_va[i+2] = txbuf[i];
        }
        client->ext_flag = client->ext_flag | I2C_DMA_FLAG | I2C_ENEXT_FLAG;

        while (retry--) {            
            if((ret=i2c_master_send(client,(unsigned char *) CTPI2CDMABuf_pa,writelen ))!=writelen){
                printk(KERN_ERR "DMA_write_addr: 0x04%x\n" ,addr);  
                mdelay(5);
            }else{
                break;
            }
         }
   }
   
   client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG)& (~I2C_ENEXT_FLAG);
   return ret; 
}

static int ts_i2c_write_dma(struct i2c_client *client, u8 *buf, u32 len)
{
	int i = 0, err = 0;

	//TPD_DMESG("%s, %d, len=%d\n", __FUNCTION__, __LINE__, len);
	for(i = 0 ; i < len; i++)
	{
		CTPI2CDMABuf_va[i] = buf[i];
	}
	
	if(len <= 8)
	{
		client->addr = client->addr & I2C_MASK_FLAG;
		return i2c_master_send(client, buf, len);
	}
	else
	{
		//printk("CTPI2CDMABuf_va=0x%x, CTPI2CDMABuf_pa=0x%x\n", CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		err = i2c_master_send(client, (char*)CTPI2CDMABuf_pa, len);
		client->addr = client->addr & I2C_MASK_FLAG;
		return err;
	}    
}

static int ts_i2c_read_dma(struct i2c_client *client, u8 *buf, u32 len)
{
	int i = 0, err = 0;
	char *ptr;

	//TPD_DMESG("%s, %d, len=%d\n", __FUNCTION__, __LINE__, len);
	if(len < 8)
	{
		client->addr = client->addr & I2C_MASK_FLAG;
		return i2c_master_recv(client, buf, len);
	}
	else
	{
		//printk("CTPI2CDMABuf_va=0x%x, CTPI2CDMABuf_pa=0x%x\n", CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		err = i2c_master_recv(client, (char*)CTPI2CDMABuf_pa, len);
		client->addr = client->addr & I2C_MASK_FLAG;
		if(err < 0)
		{
			return err;
		}
		
		for(i = 0; i < len; i++)
		{
			buf[i] = CTPI2CDMABuf_va[i];
		}
		return err;
	}
}
#endif

s32 ts_read_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;

	//TPD_DMESG("%s, %d, reg=0x%x, length=%d\n", __FUNCTION__, __LINE__, reg, length);
	/* select register*/
	ret = i2c_master_send(client , (u8*)&reg , 2);
	if (ret < 0)
		return ret;
	/* for setup tx transaction.*/
	udelay(DELAY_FOR_TRANSCATION);

#ifndef TPD_HIGH_SPEED_DMA
	ret = i2c_master_recv(client , values , length);
#else
	ret = ts_i2c_read_dma(client , values , length);
#endif
	//TPD_DMESG("%s, %d, ret=%d\n", __FUNCTION__, __LINE__, ret);
	if (ret < 0)
		return ret;
	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

s32 ts_write_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;
	u8 pkt[4];

	//TPD_DMESG("%s, %d, reg=0x%x, length=%d\n", __FUNCTION__, __LINE__, reg, length);
	pkt[0] = (reg)&0xff;
	pkt[1] = (reg >>8)&0xff;
	pkt[2] = values[0];
	pkt[3] = values[1];
#ifndef TPD_HIGH_SPEED_DMA
	ret = i2c_master_send(client, pkt, length+2);
#else
	ret = ts_i2c_write_dma(client, pkt, length+2);
#endif
	//TPD_DMESG("%s, %d, ret=%d\n", __FUNCTION__, __LINE__, ret);
	udelay(DELAY_FOR_POST_TRANSCATION);
	return ret;
}

s32 ts_write_cmd(struct i2c_client *client, u16 reg)
{
	//TPD_DMESG("%s, %d, reg=0x%x\n", __FUNCTION__, __LINE__, reg);
	if(i2c_master_send(client, (u8 *)&reg, 2) < 0)
		return -1;
	return 0;
}

s32 ts_write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	//TPD_DMESG("%s, %d, reg=0x%x, value=%d\n", __FUNCTION__, __LINE__, reg, value);
	if(ts_write_data(client, reg, (u8 *)&value, 2) < 0)
		return -1;
	return 0;
}

s32 ts_read_raw_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;

	//TPD_DMESG("%s, %d, reg=0x%x, length=%d\n", __FUNCTION__, __LINE__, reg, length); 
	if(i2c_master_send(client, (u8 *)&reg, 2) < 0)
		return -1;
	mdelay(5); 	   // for setup tx transaction
#ifndef TPD_HIGH_SPEED_DMA
	ret = i2c_master_recv(client , values , length);
#else
	ret = ts_i2c_read_dma(client , values , length);
#endif
	//if((ret = i2c_master_recv(client , values , length)) < 0)
	if(ret < 0)
		return ret;
	 udelay(DELAY_FOR_POST_TRANSCATION);	
	 return length;
 }
 
#if ZINITIX_ESD_TIMER_INTERVAL
static void zinitix_touch_tmr_work(struct tpd_device *tpd)
{
    printk(KERN_ERR "tmr queue work ++\r\n");
    if (tpd == NULL) {
        printk(KERN_ERR "tpd dev == NULL ?\r\n");
    	goto fail_time_out_init;
    }

    printk(KERN_ERR "error. timeout occured. maybe ts device dead. so power off/on reset & reinit.\r\n");
  
    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
    mdelay(CHIP_OFF_DELAY);
//    hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_3300,"TP"); 
//    mdelay(CHIP_ON_DELAY);
#if 0       
    ts_power_sequence(i2c_client);
    if (ts_init_touch() < 0)
#endif
    if(zinitix_resume_proc() < 0)
		goto fail_time_out_init;    
    
    printk(KERN_ERR "clear all reported points\r\n");
    tpd_clear_report_data();
	
    printk(KERN_ERR "tmr queue work ----\r\n");
    return;
fail_time_out_init:
    printk(KERN_ERR "tmr work : restart error\r\n");
	ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);

    return;
}

static void ts_esd_timer_start(u16 sec, struct tpd_device *tpd)
{
    if(tpd == NULL)    return;
    if (esd_timeout_tmr.data != NULL){
        printk(KERN_ERR "del_timer\n"); 
        del_timer(&esd_timeout_tmr);   

    }
    init_timer(&(esd_timeout_tmr));
    esd_timeout_tmr.data = (unsigned long)(tpd);
    esd_timeout_tmr.function = ts_esd_timeout_handler;
    esd_timeout_tmr.expires = jiffies + HZ*sec;    
    add_timer(&esd_timeout_tmr);        
}

static void ts_esd_timer_stop(struct tpd_device *tpd)
{
//    printk(KERN_ERR "ts_esd_timer_stop\n");
    if(tpd == NULL)	return;
    if (esd_timeout_tmr.data != NULL)
    del_timer(&esd_timeout_tmr);
    esd_timeout_tmr.data = NULL;
}

u8 esd_count = 0;
static void ts_esd_timeout_handler(unsigned long data)
{
    struct tpd_device *tpd = (struct tpd_device *)data;
    if(tpd == NULL)	return;
    if(esd_count == 1) return;
    esd_timeout_tmr.data = NULL;        
  
    printk(KERN_ERR "esd timer expired. Go to tmr work.\n");
    
    //wake up
    if (queue_work(esd_work_struct, &esd_reset_work) < 0){
        printk(KERN_ERR "Add the esd_reset_work failed!\n");
    }
}

static void ts_work_queue()
{
    printk(KERN_ERR "ts_work_queue\n");
    //sys_err_num+= 100;
 //   ZINITIX_LED_OFF;  
    zinitix_touch_tmr_work(tpd);   
}

#endif

#if 1 
u16 chip_code_info = 0xffff;
static bool ts_power_sequence(struct i2c_client *client)
{
	int retry = 0;
	u16 chip_code;
	
	retry_power_sequence:	
	if (ts_write_reg(i2c_client, 0xc000, 0x0001) != I2C_SUCCESS){
		printk("power sequence error (vendor cmd enable)\n");
		goto fail_power_sequence;
	}
	udelay(10);
	if (ts_read_data(i2c_client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		printk("fail to read chip code\n");
		goto fail_power_sequence;
	}
    chip_code_info = chip_code;

  zinitix_printk("chip code = 0x%x\n", chip_code);

	udelay(10);	
	if (ts_write_cmd(i2c_client, 0xc004) != I2C_SUCCESS){
		printk("power sequence error (intn clear)\n");
		goto fail_power_sequence;
	}
	udelay(10);
	if (ts_write_reg(i2c_client, 0xc002, 0x0001) != I2C_SUCCESS){
		printk("power sequence error (nvm init)\n");
		goto fail_power_sequence;
	}
	mdelay(2);
	if (ts_write_reg(i2c_client, 0xc001, 0x0001) != I2C_SUCCESS){
		printk("power sequence error (program start)\n");
		goto fail_power_sequence;
	}
	msleep(FIRMWARE_ON_DELAY);	//wait for checksum cal
	return false;//modify by shj for zinitix
	
	fail_power_sequence:
	if(retry++ < 3) {
		msleep(CHIP_ON_DELAY);
        printk("retry = %d\n", retry);		
        mtk_wdt_restart(1);
        mtk_wdt_restart(0);
        wdt_dump_reg();
		goto retry_power_sequence;
	}
	return true;
}
#else
static int  ts_power_sequence(struct i2c_client *client)
{
    int retry = 0; 
    u16 chip_code;

    printk(KERN_ERR "tpd:ts_power_sequence\n"); 
retry_power_sequence:   
    if (ts_write_reg(i2c_client, 0xc000, 0x0001) != I2C_SUCCESS){
        printk(KERN_ERR "power sequence error (vendor cmd enable)\n");
        goto fail_power_sequence;
    }    
    udelay(10);
    if (ts_read_data(i2c_client, 0xcc00, (u8 *)&chip_code, 2) < 0) { 
        printk(KERN_ERR "fail to read chip code\n");
        goto fail_power_sequence;
    }    
    printk(KERN_ERR "chip code = 0x%x\n", chip_code);
//    zinitix_ic_chip_code = chip_code;
    udelay(10); 
    if (ts_write_cmd(i2c_client, 0xc004) != I2C_SUCCESS){
        printk(KERN_ERR "power sequence error (intn clear)\n");
        goto fail_power_sequence;
    }    
    udelay(10);
    if (ts_write_reg(i2c_client, 0xc002, 0x0001) != I2C_SUCCESS){
        printk(KERN_ERR "power sequence error (nvm init)\n");
        goto fail_power_sequence;
    }    
    mdelay(10);
    if (ts_write_reg(i2c_client, 0xc001, 0x0001) != I2C_SUCCESS){
        printk(KERN_ERR "power sequence error (program start)\n");
        goto fail_power_sequence;
    }    
    msleep(FIRMWARE_ON_DELAY);  //wait for checksum cal
    printk(KERN_ERR "tpd:ts_power_sequence_END\n");            
    return 0;

fail_power_sequence:
    if(retry++ < 3) { 
        msleep(CHIP_ON_DELAY);
        printk(KERN_ERR "retry = %d\n", retry);
//        mtk_wdt_restart(WK_WDT_EXT_TYPE);//kick external WDT 
//        mtk_wdt_restart(WK_WDT_LOC_TYPE);
        goto retry_power_sequence;
    }    
    return -1;
}


#endif
#if TOUCH_ONESHOT_UPGRADE
#if 0
static int  ts_check_tsm_module_id(void) {   
     if(tsm_module_id == TAIQI_MODULE) { 
            printk(KERN_ERR "[tpd] this module is taiqi\n");
            m_FirmwareIdx = 0;
     } else  if(tsm_module_id == SAIHUA_MODULE) {
            printk(KERN_ERR "[tpd] this module is saihua\n");
            m_FirmwareIdx = 1;
     }else{
            printk(KERN_ERR "[tpd] module ID error!\n");
            //check_module_id_lcd();
            return  1;
     }
     return 0;
}
#endif
static s32 ts_read_firmware_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	return ts_read_data(client, reg, values, length);
}

static bool ts_check_need_upgrade(u16 curVersion, u16 curMinorVersion, u16 curRegVersion)
{
	u16	newVersion;
	u16	newMinorVersion;	
	u16	newRegVersion;
	u16	newChipCode;
	u16	newHWID;
	u8 *firmware_data;
	int ret  = 0;
	
	//ret = ts_check_tsm_module_id(); 
	
#if 0
	if(ret == 1){
	    printk(KERN_ERR "[tpd] TP was unnormal, so upgrade without check version.\n");
	    return true;
	}
#endif 

	firmware_data = (u8*)m_pFirmware[m_FirmwareIdx];
	
	curVersion = curVersion&0xff;
	newVersion = (u16) (firmware_data[52] | (firmware_data[53]<<8));
	newVersion = newVersion&0xff;
	newMinorVersion = (u16) (firmware_data[56] | (firmware_data[57]<<8));
	newRegVersion = (u16) (firmware_data[60] | (firmware_data[61]<<8));
	newChipCode = (u16) (firmware_data[64] | (firmware_data[65]<<8));
	
	printk("cur version = 0x%x, new version = 0x%x\n",
		curVersion, newVersion);
	if (curVersion < newVersion)
		return true;
	else if (curVersion > newVersion)
		return false;
	
	/*
	printk("cur minor version = 0x%x, new minor version = 0x%x\n",
			curMinorVersion, newMinorVersion);
	if (curMinorVersion < newMinorVersion)
		return true;
	else if (curMinorVersion > newMinorVersion)
		return false;
	*/
	
	printk("cur reg data version = 0x%x, new reg data version = 0x%x\n",
			curRegVersion, newRegVersion);
	if (curRegVersion < newRegVersion)
		return true;
	
	return false;
}

static bool ts_hw_calibration(void)
{
	u16	chip_eeprom_info;
	int time_out = 0;
	
	if (ts_write_reg(i2c_client,
	ZINITIX_TOUCH_MODE, 0x07) != I2C_SUCCESS)
	return false;
#if ZINITIX_ESD_TIMER_INTERVAL
    if (use_esd_timer) {
        ts_esd_timer_stop(tpd);
        printk(KERN_ERR "tpd_suspend:ts_esd_timer_stop\n");
    }
#endif
    mdelay(10);
	ts_write_cmd(i2c_client,	ZINITIX_CLEAR_INT_STATUS_CMD);
	mdelay(10);
	ts_write_cmd(i2c_client,	ZINITIX_CLEAR_INT_STATUS_CMD);	
	mdelay(50);
	ts_write_cmd(i2c_client,	ZINITIX_CLEAR_INT_STATUS_CMD);	
	mdelay(10);
	if (ts_write_cmd(i2c_client,
	ZINITIX_CALIBRATE_CMD) != I2C_SUCCESS)
	return false;
	if (ts_write_cmd(i2c_client,
	ZINITIX_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
	return false;
	mdelay(10);
	ts_write_cmd(i2c_client,	ZINITIX_CLEAR_INT_STATUS_CMD);
	
	/* wait for h/w calibration*/
	do {
		mdelay(500);
		ts_write_cmd(i2c_client,
				ZINITIX_CLEAR_INT_STATUS_CMD);			
		if (ts_read_data(i2c_client,
			ZINITIX_EEPROM_INFO_REG,
			(u8 *)&chip_eeprom_info, 2) < 0)
        {
#if ZINITIX_ESD_TIMER_INTERVAL     
            if (use_esd_timer) {
                ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
                printk(KERN_ERR "tpd_touchinfo:esd tmr start\n");
            }     
#endif
            return false;
        }
        printk("touch eeprom info = 0x%04X\r\n",
			chip_eeprom_info);
		if (!zinitix_bit_test(chip_eeprom_info, 0))
			break;
		if(time_out++ == 4){
			ts_write_cmd(i2c_client,	ZINITIX_CALIBRATE_CMD);
			mdelay(10);
			ts_write_cmd(i2c_client,
				ZINITIX_CLEAR_INT_STATUS_CMD);						
			zinitix_printk("h/w calibration retry timeout.\n");
		}
		if(time_out++ > 10){
			zinitix_printk("[error] h/w calibration timeout.\n");
			break;						
		}
	} while (1);
	
	if (ts_write_reg(i2c_client,
	ZINITIX_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
    {
#if ZINITIX_ESD_TIMER_INTERVAL     
        if (use_esd_timer) {
            ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
            printk(KERN_ERR "tpd_touchinfo:esd tmr start\n");
        }     
#endif
        return false;
    }

	if (ic_int_mask != 0)
	if (ts_write_reg(i2c_client,
		ZINITIX_INT_ENABLE_FLAG,
		ic_int_mask)
		!= I2C_SUCCESS)
    {
#if ZINITIX_ESD_TIMER_INTERVAL     
        if (use_esd_timer) {
            ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
            printk(KERN_ERR "tpd_touchinfo:esd tmr start\n");
        }     
#endif
		return false;
    }
	
	ts_write_reg(i2c_client, 0xc003, 0x0001);
	ts_write_reg(i2c_client, 0xc104, 0x0001);
	udelay(100);
	if (ts_write_cmd(i2c_client,
	ZINITIX_SAVE_CALIBRATION_CMD) != I2C_SUCCESS)
	return false;
	mdelay(1000);	
	ts_write_reg(i2c_client, 0xc003, 0x0000);
	ts_write_reg(i2c_client, 0xc104, 0x0000);
#if ZINITIX_ESD_TIMER_INTERVAL     
        if (use_esd_timer) {
            ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
            printk(KERN_ERR "tpd_touchinfo:esd tmr start\n");
        }     
#endif
	return true;				
	
}


static u8 ts_upgrade_firmware(const u8 *firmware_data, u32 size)
{
	u16 flash_addr;
	u8 *verify_data;
	int retry_cnt = 0;
	int i;
	int page_sz = 64;
	u16 chip_code;
	
	
	verify_data = kzalloc(size, GFP_KERNEL);
	if (verify_data == NULL) {
	printk(KERN_ERR "cannot alloc verify buffer\n");
	return false;
	}
#if ZINITIX_ESD_TIMER_INTERVAL
    esd_count = 1;
#endif

	retry_upgrade:
	//ts_power_control(touch_dev, POWER_OFF);
	hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
//	hwPowerDown(MT6323_POWER_LDO_VGP2,"TP"); 
	mdelay(50);
	hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_2800,"TP"); 
//	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, "TP");
	mdelay(30);

    msleep(1);

    printk("set tpd reset pin to high\n");
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

    msleep(CHIP_ON_DELAY);

	if (ts_write_reg(i2c_client, 0xc000, 0x0001) != I2C_SUCCESS){
	printk("power sequence error (vendor cmd enable)\n");
	goto fail_upgrade;
	}
	udelay(10);
	if (ts_read_data(i2c_client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
	printk("fail to read chip code\n");
	goto fail_upgrade;
	}
	printk("chip code = 0x%x\n", chip_code);
	if((chip_code == 0xf400)||(chip_code == 0xe240)) {
	//		touch_dev->cap_info.is_zmt200 = 0;
	page_sz = 128;
	} else {
	//		touch_dev->cap_info.is_zmt200 = 1;
	page_sz = 64;		
	}
	
	udelay(10);	
	if (ts_write_cmd(i2c_client, 0xc004) != I2C_SUCCESS){
	printk("power sequence error (intn clear)\n");
	goto fail_upgrade;
	}
	udelay(10);
	if (ts_write_reg(i2c_client, 0xc002, 0x0001) != I2C_SUCCESS){
	printk("power sequence error (nvm init)\n");
	goto fail_upgrade;
	}
	mdelay(10);
	printk("init flash\n");
	if (ts_write_reg(i2c_client, 0xc003, 0x0001) != I2C_SUCCESS){
	printk("fail to write nvm vpp on\n");
	goto fail_upgrade;
	}
	mdelay(1);
	if (ts_write_reg(i2c_client, 0xc104, 0x0001) != I2C_SUCCESS){
	printk("fail to write nvm wp disable\n");
	goto fail_upgrade;
	}
	
	if (ts_write_cmd(i2c_client, ZINITIX_INIT_FLASH) != I2C_SUCCESS) {
		printk("failed to init flash\n");
		goto fail_upgrade;
	}
	
	//add for fw upgrade 2013-11-25
	mtk_wdt_restart(1);
	mtk_wdt_restart(0);
    wdt_dump_reg();
	
  udelay(10);
  size = 32*1024;

	printk("writing firmware data\n");
	for (flash_addr = 0; flash_addr < size; ) {
		//printk(KERN_ERR  "Addr:0x%04x\n", flash_addr);
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
		//printk("write :addr=%04x, len=%d\n",	flash_addr, TC_SECTOR_SZ);
		if (zinitix_i2c_DMA_write_ii(i2c_client,
			ZINITIX_WRITE_FLASH,
			(u8 *)&firmware_data[flash_addr],TC_SECTOR_SZ) < 0) {
			printk(KERN_INFO"error : write zinitix tc firmare\n");
			goto fail_upgrade;
		}
		flash_addr += TC_SECTOR_SZ;
		udelay(100);
	}
	//		if(touch_dev->cap_info.is_zmt200 == 0)
	mdelay(30);	//for fuzing delay
	//		else
	//			mdelay(15);
	}
	mdelay(100);
	if (ts_write_reg(i2c_client, 0xc003, 0x0000) != I2C_SUCCESS){
	printk("nvm write vpp off\n");
	goto fail_upgrade;
	}
	
	if (ts_write_reg(i2c_client, 0xc104, 0x0000) != I2C_SUCCESS){
		printk("nvm wp enable\n");
		goto fail_upgrade;
	}
	
	printk("init flash\n");
	if (ts_write_cmd(i2c_client, ZINITIX_INIT_FLASH) != I2C_SUCCESS) {
	printk("failed to init flash\n");
	goto fail_upgrade;
	}
	
	
	//add for fw upgrade 2013-11-25
	mtk_wdt_restart(1);
	mtk_wdt_restart(0);
    wdt_dump_reg();
	#if 1
	printk("read firmware data\n");
	for (flash_addr = 0; flash_addr < size; ) {
	for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
	//printk("read :addr=%04x, len=%d\n", flash_addr, TC_SECTOR_SZ);
	if (/*zinitix_i2c_DMAread*/ts_read_firmware_data(i2c_client,
	ZINITIX_READ_FLASH,
	(u8*)&verify_data[flash_addr], TC_SECTOR_SZ) < 0) {
	printk("error : read zinitix tc firmare\n");
	goto fail_upgrade;
	}
	flash_addr += TC_SECTOR_SZ;
	}
	}
       #endif
	/* verify */
	printk("verify firmware data\n");
	if (memcmp((u8 *)&firmware_data[0], (u8 *)&verify_data[0], size) == 0) 
		{
		printk("upgrade finished\n");
		kfree(verify_data);
		//ts_power_control(touch_dev, POWER_OFF);
		hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
		//hwPowerDown(MT6323_POWER_LDO_VGP2,"TP"); 
		//ts_power_control(touch_dev, POWER_ON_SEQUENCE);
		mdelay(50);
		hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_2800,"TP"); 
		//hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, "TP");
		mdelay(30);
		ts_power_sequence(i2c_client);
#if ZINITIX_ESD_TIMER_INTERVAL     
        esd_count = 0;
#endif
		return true;
	}
	
	
	fail_upgrade:
	//ts_power_control(touch_dev, POWER_OFF);
	hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
	if (retry_cnt++ < ZINITIX_INIT_RETRY_CNT) {
	printk("upgrade fail : so retry... (%d)\n", retry_cnt);
	goto retry_upgrade;		
	}
	
	if (verify_data != NULL)
	kfree(verify_data);
	
	printk("upgrade fail..\n");
#if ZINITIX_ESD_TIMER_INTERVAL     
        esd_count = 0;
#endif
	return false;
	
	
}
#endif

static void   tpd_clear_report_data(void)
{
 int i;
 u8 reported = false;
 
#ifdef TPD_HAVE_BUTTON
 for (i = 0; i < TPD_KEY_COUNT; i++) {
	 if (button[i] == ICON_BUTTON_DOWN) {
		 button[i] = ICON_BUTTON_UP;
		 input_report_key(tpd->dev, tpd_keys_local[i], 0);
		 reported = true;
		 TPD_DMESG(TPD_DEVICE "button up = %d \r\n", i);
	 }
 }
#endif    

 for (i = 0; i < MAX_SUPPORTED_FINGER_NUM; i++) {		 
	 if (zinitix_bit_test(reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST)) {
		 input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, i); 		   
		 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, (u32)0);
		 input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, (u32)0);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_X, reported_touch_info.coord[i].x);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, reported_touch_info.coord[i].y);  
		 input_mt_sync(tpd->dev);
		 reported = true;
	 }
	 reported_touch_info.coord[i].sub_status = 0;
 }
 
 if (reported == true) {
	 input_report_key(tpd->dev, BTN_TOUCH, 0);
	 input_sync(tpd->dev);
 }
}

static int tpd_touchinfo(void)
{

 int i = 0;    
 u8 reported = false;	 
 u32 x, y;
 u32 w;
	 u8 palm = 0;
 
#ifdef TPD_HAVE_BUTTON
 u16 icon_event_reg;				
#endif

	ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);

#if USE_WAKEUP_GESTURE
	 if (zinitix_bit_test(touch_info.status, BIT_WAKEUP)) 
		 {
		 /*wake up*/
//		 printk("zinitix gesture wakeup\r\n");
		 }
#else
	 if(tpd_halt) {
		 TPD_DMESG("zinitix suspend halt status...\n");
		 return false;
	 }
#endif
//for  Debugging Tool add by eric

    if (misc_touch_dev->touch_mode != TOUCH_POINT_MODE)
	{
//        printk("tpd_touchinfo other mode\r\n");
		if(misc_touch_dev->update == 0) {
  //             printk("tpd_touchinfo begin get raw data update=0\r\n");
		    if (ts_get_raw_data() == false)
		        return false;
		} 
               return true;
	}
//eric end
 if (ts_read_data(i2c_client, ZINITIX_POINT_STATUS_REG, (u8 *)&(touch_info), sizeof(struct _ts_zinitix_point_info)) < 0) {
	 printk("tpd_touchinfo : I2C Error");
	 return false;
	}
	
#if ZINITIX_ESD_TIMER_INTERVAL
    if (use_esd_timer) {
		ts_esd_timer_stop(tpd);
//		printk("tpd_touchinfo:esd timer stop\r\n");
	}
    
  	/* invalid : maybe periodical repeated int. */
  if (touch_info.status == 0x0) {
//      printk("periodical_interrupt\r\n");    
      ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
      tpd_clear_report_data();
      goto continue_get_touchinfo_data;
  }
#endif
 

cont_tpd_touchinfo:
 
#ifdef TPD_HAVE_BUTTON    
 if (zinitix_bit_test(touch_info.status, BIT_ICON_EVENT)) {
	 udelay(20);
	 if (ts_read_data(i2c_client, ZINITIX_ICON_STATUS_REG, (u8 *)(&icon_event_reg), 2) < 0) {
		 printk("error read icon info using i2c.\n");
		 return false;
	 }
	 //return true;
 }
#endif
	
	//ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);	
	
	/* invalid : maybe periodical repeated int. */
	if (touch_info.status == 0x0)	//  return false;    
      goto continue_get_touchinfo_data;

//	printk("jiang touch_info.status=0x%x\n", touch_info.status);


#ifdef TPD_HAVE_BUTTON
 if (zinitix_bit_test(touch_info.status, BIT_ICON_EVENT)) {
	 for (i = 0; i < TPD_KEY_COUNT; i++) {
		 if (zinitix_bit_test(icon_event_reg, (BIT_O_ICON0_DOWN+i))) {
			 button[i] = ICON_BUTTON_DOWN;
			 input_report_key(tpd->dev, tpd_keys_local[i], 1);
			 reported = true;
//			 TPD_DMESG(TPD_DEVICE "button down = %d \r\n", i);
		 }
	 }

	 for (i = 0; i < TPD_KEY_COUNT; i++) {
		 if (zinitix_bit_test(icon_event_reg, (BIT_O_ICON0_UP+i))) {
			 button[i] = ICON_BUTTON_UP;
			 input_report_key(tpd->dev, tpd_keys_local[i], 0);
			 reported = true;
//			 TPD_DMESG(TPD_DEVICE "button up = %d \r\n", i);
		 }
	 }
 }
#endif
#if 0
	//Large touch enable
	if(zinitix_bit_test(touch_info.status, BIT_PALM)){
		printk("palm report\r\n");
		TPD_DMESG(TPD_DEVICE "BIT_PALM \r\n");
			 
		input_report_key(tpd->dev, KEY_F5, 1);
		input_sync(tpd->dev);
		palm = 1;
	}
	else if(zinitix_bit_test(touch_info.status, BIT_PALM_REJECT)){
		printk("palm reject\r\n");
			 TPD_DMESG(TPD_DEVICE "BIT_PALM_REJECT \r\n");
			 
		input_report_key(tpd->dev, KEY_F5, 1);
		input_sync(tpd->dev);
		palm = 1;		
	}
	else {
		printk("large touch release\r\n");			 
		input_report_key(tpd->dev, KEY_F5, 0);
		input_sync(tpd->dev);
		palm = 0;		
	}
	//end 2013-11-06
#endif
	/* if button press or up event occured... */
	if (reported == true || !zinitix_bit_test(touch_info.status, BIT_PT_EXIST)) {
	 for (i = 0; i < MAX_SUPPORTED_FINGER_NUM; i++)    {			
		 if (zinitix_bit_test(reported_touch_info.coord[i].sub_status, SUB_BIT_EXIST)) {
//			 TPD_DMESG(TPD_DEVICE "finger [%02d] up \r\n", i);
			 input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, i); 		   
			 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, (u32)0);
			 input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, (u32)0);
			 input_report_abs(tpd->dev, ABS_MT_POSITION_X, reported_touch_info.coord[i].x);
			 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, reported_touch_info.coord[i].y);  
			 input_mt_sync(tpd->dev);
		 }			  
	 }
	 input_sync(tpd->dev);
	 input_report_key(tpd->dev, BTN_TOUCH, 0);
	 memset(&reported_touch_info, 0x0, sizeof(struct _ts_zinitix_point_info));
     input_mt_sync(tpd->dev);
	 input_sync(tpd->dev);
	 if(reported == true)	 udelay(100);
      goto continue_get_touchinfo_data;
	// return true;
	}
	
    input_report_key(tpd->dev, BTN_TOUCH, 1);
 	for (i = 0; i < MAX_SUPPORTED_FINGER_NUM; i++) {		 

	 if (zinitix_bit_test(touch_info.coord[i].sub_status, SUB_BIT_DOWN)
		 || zinitix_bit_test(touch_info.coord[i].sub_status, SUB_BIT_MOVE)
		 || zinitix_bit_test(touch_info.coord[i].sub_status, SUB_BIT_EXIST)) {

		 x = touch_info.coord[i].x;
		 y = touch_info.coord[i].y;
		 w = touch_info.coord[i].width;
		 
//		 TPD_DMESG(TPD_DEVICE "finger [%02d] x = %d, y = %d \r\n", i, x, y);
		 if (w == 0)
			 w = 1;
		 input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, i);
		 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, (u32)w);
		 input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, (u32)w);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);			  
		 input_mt_sync(tpd->dev);
	 } else if (zinitix_bit_test(touch_info.coord[i].sub_status, SUB_BIT_UP)) {
//		 TPD_DMESG(TPD_DEVICE "finger [%02d] up \r\n", i);
		 memset(&touch_info.coord[i], 0x0, sizeof(struct _ts_zinitix_coord));						 
		 input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, i); 		   
		 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, (u32)0);
		 input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, (u32)0);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_X, reported_touch_info.coord[i].x);
		 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, reported_touch_info.coord[i].y); 		   
		 input_mt_sync(tpd->dev);
	 } else
		 memset(&touch_info.coord[i], 0x0, sizeof(struct _ts_zinitix_coord));

 }
 memcpy((char *)&reported_touch_info, (char *)&touch_info, sizeof(struct _ts_zinitix_point_info));
 //input_report_key(tpd->dev, BTN_TOUCH, 1);
 input_sync(tpd->dev);
 
continue_get_touchinfo_data:
    
#if ZINITIX_ESD_TIMER_INTERVAL     
    if (use_esd_timer) {
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
//		printk(KERN_ERR "tpd_touchinfo:esd tmr start\n");
	}     
#endif 

 return true;

}

static int touch_event_handler(void *unused)
{	  
 
 struct sched_param param = {.sched_priority = RTPM_PRIO_TPD};
 sched_setscheduler(current, SCHED_RR, &param);
 
 static u8 tp_status = 0; 
 static u8 charge_info = 0;

 do
 {
	 //mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 	   
	 set_current_state(TASK_INTERRUPTIBLE); 
	 wait_event_interruptible(waiter, tpd_flag!=0); 	   
	 tpd_flag = 0;		  
	 set_current_state (TASK_RUNNING);	  

     /*   modify for charge in/out status PR705169
      * */
     if (ts_read_data(i2c_client, 0x0116, &charge_info, 1) < 0) { 
         printk("read 0x0116 charging status I2C Error\n");
     }    
     else{
 //        printk("read 0x0116 charging status tp_status = %d,charge_info = %d\n",tp_status,charge_info);

         if(0 == get_charger_detect_status()){
             tp_status = 0;
             if (tp_status != charge_info)
                 ts_write_reg(i2c_client ,0x0116,0);// no charge
         }
         else{
             tp_status = 1;
             if (tp_status != charge_info)
                 ts_write_reg(i2c_client ,0x0116,1);// charge in
         }
     }

	 if (tpd_touchinfo()) { 			   
		 TPD_DEBUG_SET_TIME;			
	 }
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 }while(!kthread_should_stop());

 return 0;
}


static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
{
   printk("mtk_tpd: enter bt4xx tpd_detect\n");
 strcpy(info->type, TPD_DEVICE);	
  return 0;
}

static void tpd_eint_interrupt_handler(void)
{
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
  //printk("TPD interrup has been triggered\n");    
  TPD_DEBUG_PRINT_INT;
  tpd_flag = 1;
  wake_up_interruptible(&waiter);
}

static bool ts_init_touch(void)
{
 u16 reg_mask = 0x880f;	 //0x880f
 
 //u16 reg_mask = 0x883f;//esd
 int i; 
   u16 ic_revision;
   u16 firmware_version;
u16 minor_firmware_version;	
u16 reg_data_version;
u16 chip_eeprom_info;
#if  1//USE_CHECKSUM	
u16 chip_check_sum;
u8 checksum_err;
  u32 ic_fw_size;
  
#endif	
	int retry_cnt = 0;

 ic_int_mask = reg_mask;
 
 memset(&reported_touch_info, 0x0, sizeof(struct _ts_zinitix_point_info));

retry_init:

    for(i = 0; i < ZINITIX_INIT_RETRY_CNT; i++) {       
        if (ts_read_data(i2c_client,
                ZINITIX_EEPROM_INFO_REG,
                (u8 *)&chip_eeprom_info, 2) < 0) {
            printk("fail to read eeprom info(%d)\r\n", i);
            mdelay(10);
            continue;
        } else
            break;
    }
    if (i == ZINITIX_INIT_RETRY_CNT) {  
        goto fail_init;
    }
#if 1//USE_CHECKSUM        
    printk("check checksum\r\n");            

    checksum_err = 0;

    for (i = 0; i < ZINITIX_INIT_RETRY_CNT; i++) {
        if (ts_read_data(i2c_client,
            ZINITIX_CHECKSUM_RESULT, (u8 *)&chip_check_sum, 2) < 0) {
            mdelay(10);
            continue;
        }

        printk("0x%04X\r\n", chip_check_sum);

        if(chip_check_sum == 0x55aa)                
            break;
        else {
            if(chip_eeprom_info == 1 && chip_check_sum == 1) {
                printk("it might fail to boot.\n");     
                goto fail_init;
            }
            checksum_err = 1;
            break;
        }
    }
        
    if (i == ZINITIX_INIT_RETRY_CNT || checksum_err) {
        printk("fail to check firmware data\r\n");
        if(checksum_err == 1 && retry_cnt < ZINITIX_INIT_RETRY_CNT)
            retry_cnt = ZINITIX_INIT_RETRY_CNT;
        goto fail_init;
    }

#endif


 
 printk("send reset command\r\n");	
 for (i = 0; i < ZINITIX_INIT_RETRY_CNT; i++) {
	 if (ts_write_cmd(i2c_client ,ZINITIX_SWRESET_CMD) == 0)
		 break;
	 mdelay(10);
 }

 if (i == ZINITIX_INIT_RETRY_CNT) {
	 printk("fail to send reset command\r\n");
	 goto fail_init;
 }

  // read inform..
  if (ts_read_data(i2c_client, ZINITIX_FIRMWARE_VERSION,
      (u8 *)&firmware_version, 2) < 0)
      goto fail_init;
printk("bt4xx touch reg data version = %d\r\n",
		firmware_version);

  if (ts_read_data(i2c_client, ZINITIX_DATA_VERSION_REG,
      (u8 *)&reg_data_version, 2) < 0)
      goto fail_init;

  if (ts_read_data(i2c_client, ZINITIX_CHIP_REVISION,
      (u8 *)&ic_revision, 2) < 0) 
      goto fail_init;
  #if 0
    if (ts_read_data(i2c_client, 0X001E, (u8 *)&tsm_module_id, 2) < 0) {
        printk(KERN_ERR "read tsm_module_id error!\n");
        goto fail_init;
    }       
    printk(KERN_ERR "TSP MODULE ID = 0x%02x (0x5449 = TAIQI, 0x5341 = SAIHUA)\n",
		tsm_module_id);
	#endif
 //set touch_mode
 if (ts_write_reg(i2c_client, ZINITIX_TOUCH_MODE, TOUCH_MODE) != 0)
	 goto fail_init;

 if (ts_write_reg(i2c_client ,ZINITIX_SUPPORTED_FINGER_NUM,(u16)MAX_SUPPORTED_FINGER_NUM) != 0)
	 goto fail_init;
#if 1//USE_CHECKSUM	

   ic_fw_size = 32*1024;
#endif

#if TOUCH_ONESHOT_UPGRADE

if (ts_read_data(i2c_client,
	ZINITIX_DATA_VERSION_REG,
	(u8 *)&reg_data_version, 2) < 0)
	goto fail_init;
	
printk("touch reg data version = %d\r\n",
	reg_data_version);

if (ts_check_need_upgrade(firmware_version, minor_firmware_version,
	reg_data_version) == true) {
	printk("start upgrade firmware\n");
	
	if(ts_upgrade_firmware(m_pFirmware[m_FirmwareIdx],
		ic_fw_size) == false)
		goto fail_init;

	if(ts_hw_calibration() == false)
		goto fail_init;

	/* disable chip interrupt */
	if (ts_write_reg(i2c_client,
		ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
		goto fail_init;

	/* get chip firmware version */
	if (ts_read_data(i2c_client,
		ZINITIX_FIRMWARE_VERSION,
		(u8 *)&firmware_version, 2) < 0)
		goto fail_init;
	printk("touch chip firmware version = %x\r\n",
		firmware_version);

	if (ts_read_data(i2c_client,
		ZINITIX_MINOR_FW_VERSION,
		(u8 *)&minor_firmware_version, 2) < 0)
		goto fail_init;
	printk("touch chip firmware version = %x\r\n",
		minor_firmware_version);
}
#endif

	/* initialize */
	if (ts_write_reg(i2c_client ,ZINITIX_X_RESOLUTION,(u16)(TPD_RES_MAX_X)) != 0)
		goto fail_init;

	if (ts_write_reg(i2c_client ,ZINITIX_Y_RESOLUTION,(u16)(TPD_RES_MAX_Y)) != 0)
		goto fail_init;
		
	if (ts_read_data(i2c_client,
		ZINITIX_DATA_VERSION_REG,
		(u8 *)&reg_data_version, 2) < 0)
		goto fail_init;
	printk("bt4xx touch reg data version = %d\r\n",
		reg_data_version);
	
	if (ts_read_data(i2c_client,
		ZINITIX_EEPROM_INFO_REG,
		(u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;
		printk("bt4xx touch eeprom info = 0x%04X\r\n", chip_eeprom_info);
	 
	  if (zinitix_bit_test(chip_eeprom_info, 0)) {
	  	
			#if TOUCH_ONESHOT_UPGRADE
			if(ts_hw_calibration() == false)
				goto fail_init;
			#endif				
			
	    if(ts_write_reg(i2c_client, ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
	    		goto fail_init;
	   }
	
	  if (ts_read_data(i2c_client,ZINITIX_EEPROM_INFO_REG,(u8 *)&chip_eeprom_info, 2) < 0)
	 goto fail_init;
		printk("bt4xx touch eeprom info = 0x%X\r\n", chip_eeprom_info);
	
	   if (ts_write_cmd(i2c_client, ZINITIX_CALIBRATE_CMD) != I2C_SUCCESS)
		 goto fail_init;
	
	   if (ts_write_reg(i2c_client, ZINITIX_INT_ENABLE_FLAG, reg_mask) !=0)
		 goto fail_init;
	   printk("bt4xx touch ZINITIX_INT_ENABLE_FLAG = 0x%04X\r\n", reg_mask);
	 /* read garbage data */
	 for (i = 0; i < 10; i++) {
		 ts_write_cmd(i2c_client , ZINITIX_CLEAR_INT_STATUS_CMD);
		 udelay(10);
	 }
#if 1//add by shj

    printk("tpd_sys ic_revision:%x,firmware_version:%x,minor_firmware_version:%x,reg_data_version:%x \n",ic_revision,firmware_version,minor_firmware_version,reg_data_version);
misc_touch_dev->cap_info.ic_revision = (u16)ic_revision;
misc_touch_dev->cap_info.firmware_version = (u16)firmware_version;
misc_touch_dev->cap_info.firmware_minor_version = (u16)minor_firmware_version;
misc_touch_dev->cap_info.reg_data_version = (u16)reg_data_version;


/* initialize */
misc_touch_dev->cap_info.x_resolution = TPD_RES_MAX_X;
misc_touch_dev->cap_info.y_resolution = TPD_RES_MAX_Y;

misc_touch_dev->cap_info.MinX = (u32)0;
misc_touch_dev->cap_info.MinY = (u32)0;
misc_touch_dev->cap_info.MaxX = (u32)misc_touch_dev->cap_info.x_resolution;
misc_touch_dev->cap_info.MaxY = (u32)misc_touch_dev->cap_info.y_resolution;
misc_touch_dev->cap_info.multi_fingers = MAX_SUPPORTED_FINGER_NUM;
misc_touch_dev->cap_info.ic_fw_size = 32*1024;
//add by eric
//for  Debugging Tool
    if (ts_read_data(misc_touch_dev->client,
        ZINITIX_HW_ID,
        (u8 *)&misc_touch_dev->cap_info.hw_id, 2) < 0)
	{
	 printk("read ZINITIX_HW_IDERROR\r\n");
	}
    
        printk("touch chip hw id = 0x%04x\r\n",
            misc_touch_dev->cap_info.hw_id);
    
    if (ts_read_data(misc_touch_dev->client,
        ZINITIX_TOTAL_NUMBER_OF_X,
        (u8 *)&misc_touch_dev->cap_info.x_node_num, 2) < 0)
       {
 	printk("read ZINITIX_x_node_num ERROR\r\n");
       }
        printk("touch chip x node num = %d\r\n",
        misc_touch_dev->cap_info.x_node_num);
    if (ts_read_data(misc_touch_dev->client,
        ZINITIX_TOTAL_NUMBER_OF_Y,
        (u8 *)&misc_touch_dev->cap_info.y_node_num, 2) < 0)
     {
	printk("read ZINITIX_y_node_num ERROR\r\n");
      }
        printk("touch chip y node num = %d\r\n",
        misc_touch_dev->cap_info.y_node_num);

    misc_touch_dev->cap_info.total_node_num =
        misc_touch_dev->cap_info.x_node_num*misc_touch_dev->cap_info.y_node_num;
    printk("touch chip total node num = %d\r\n",
        misc_touch_dev->cap_info.total_node_num);

if (ts_read_data(misc_touch_dev->client,
            ZINITIX_AFE_FREQUENCY,
            (u8 *)&misc_touch_dev->cap_info.afe_frequency, 2) < 0)
{
  printk("read ZINITIX_AFE_FREQUENCY ERROR\r\n");
}
           

#endif

#if  ZINITIX_ESD_TIMER_INTERVAL  

     if (ts_write_reg(i2c_client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 
            ZINITIX_SCAN_RATE_HZ*ZINITIX_ESD_TIMER_INTERVAL) != I2C_SUCCESS)
        goto fail_init;

     if (use_esd_timer) {
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
		printk(KERN_ERR "tpd:esd timer start\r\n");
	 }
#endif
	 printk("successfully initialized\r\n");
	 return true;
	
	fail_init:
        printk("enter fail_init\r\n");
        mtk_wdt_restart(1);
	mtk_wdt_restart(0);
    wdt_dump_reg();
    if (++retry_cnt <= ZINITIX_INIT_RETRY_CNT) {
        
	hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
	mdelay(50);
	hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
	msleep(1);
	printk("set tpd reset pin to high\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(10);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(CHIP_ON_DELAY);  
	ts_power_sequence(i2c_client);
        printk("retry to initiallize(retry cnt = %d)\r\n",
            retry_cnt);
        goto    retry_init;
    } else if(retry_cnt == ZINITIX_INIT_RETRY_CNT+1) {
        //if(touch_dev->signature == 0xf400 || touch_dev->signature == 0xe400)
           // touch_dev->cap_info.ic_fw_size = 32*1024;
       // else
           // touch_dev->cap_info.ic_fw_size = 24*1024;
        printk("retry to initiallize(retry cnt = %d)\r\n", retry_cnt);

        if((chip_code_info == 0xf400) || (chip_code_info == 0xe240))
        {        
        
        if(ts_upgrade_firmware(m_pFirmware[m_FirmwareIdx],
		ic_fw_size) == false)
	{
	  printk("retry upgrade fail\r\n");
          return false;
	}
        }
        else 
            return false;


	if(ts_hw_calibration() == false)
	{
	  printk("retry calibration fail\r\n");
          return false;
	}
        goto retry_init;
    
    }

	 return false;
}

u8 fw_thread = 0;
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
    int retval = TPD_OK;
    int reset_count = 0;
    int ret1;

#ifdef ESD_CHECK	
	int ret;
#endif

	char data;
	i2c_client = client;
    i2c_client->addr=0x20;
#if ZINITIX_ESD_TIMER_INTERVAL
    use_esd_timer = 0;

    if (ZINITIX_ESD_TIMER_INTERVAL) {
		use_esd_timer = 1;
    }
#endif
  printk("mtk_tpd: enter bt4xx tpd_i2c_probe\n");
	printk("bt4xx  tpd_probe\n");

    misc_touch_dev = kzalloc(sizeof(struct zinitix_touch_dev), GFP_KERNEL);
    if (!misc_touch_dev) {
        printk("unabled to allocate touch data \n");
    }

    misc_touch_dev->client = client;
    misc_touch_dev->client->addr=0x20;
    i2c_set_clientdata(client, misc_touch_dev);
//eric add start
/* init touch mode */
    misc_touch_dev->touch_mode = TOUCH_POINT_MODE;
    misc_touch_dev->update ==2;//not use state
#if 0
misc_touch_dev->cur_data = kzalloc(misc_touch_dev->cap_info.total_node_num*2 + sizeof(struct _ts_zinitix_point_info), GFP_KERNEL);
    if(misc_touch_dev->cur_data == NULL) {   
        printk("unabled to allocate cur data\r\n");
    }
#endif
//eric end
#ifdef TPD_HIGH_SPEED_DMA
	CTPI2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, CTP_I2C_DMA_BUF_SIZE, &CTPI2CDMABuf_pa, GFP_KERNEL);
	if(!CTPI2CDMABuf_va)
	{
		TPD_DMESG(TPD_DEVICE "dma_alloc_coherent error\n");
	}
	TPD_DMESG("CTPI2CDMABuf_va=0x%x, CTPI2CDMABuf_pa=0x%x\n", CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
#endif

reset_proc:    

#ifdef GPIO_CTP_EN_PIN
  mt_set_gpio_mode(GPIO_ETP_EN_PIN, GPIO_ETP_EN_PIN_M_GPIO);
  mt_set_gpio_dir(GPIO_ETP_EN_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_ETP_EN_PIN, GPIO_OUT_ONE); 
#else
	//hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "TP");
#endif
hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
//hwPowerDown(MT6323_POWER_LDO_VGP2,"TP"); 
	//mdelay(50);
mdelay(CHIP_OFF_DELAY+20);
hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
//hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, "TP");

  msleep(1);

  printk("set tpd reset pin to high\n");
  mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
  mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
  msleep(10);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
 
  msleep(CHIP_ON_DELAY+20);  
#if 0
  ts_power_sequence(i2c_client); 
      
  retval = ts_init_touch();

#ifdef TPD_RESET_ISSUE_WORKAROUND
  if(reset_count < TPD_MAX_RESET_COUNT && retval == false) {
    reset_count++;
    goto reset_proc;            
  }
#endif

    if(retval == false)
    	return -1;
#endif

if (true == ts_power_sequence(i2c_client))
    	return -1;

zinitix_resume_proc();

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 0);	
//	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1); 
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1); 
		
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
        printk("error : not cmpatible i2c function \n");
        return -1;
    }
	
    tpd_load_status = 1;
    
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread)) {
        retval = false;
        TPD_DMESG(TPD_DEVICE "failed to create touch event thread: %d\n", retval);
		return -1;
    }

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
    //---------------------------------------------------------------
#if ZINITIX_ESD_TIMER_INTERVAL
    esd_work_struct = create_singlethread_workqueue("ESD_WORK_QUEUE");
    if (esd_work_struct ==NULL){
        ;//goto probe_fail;
    }
#endif
  //---------------------------------------------------------------

  printk("bt4xx tpd_probe end\n ");

#ifdef TP_SYSFS_SUPPORT
    TP_sysfs_init();
    CTP_sysfs_init();
#endif

	//zinitix_resume_proc();

  ret1 = misc_register(&touch_misc_device);
  if (ret1)
      printk("Fail to register touch misc device.\n");

  TPD_DMESG(TPD_DEVICE "Zinitix bt4xx Touch Panel Device Probe %s\n", (retval == false) ? "FAIL" : "PASS");
  fw_thread = 1;
  return 0;
 
}

void upgradefw(void)
{
    do{

        hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
        //mdelay(50);
        mdelay(CHIP_OFF_DELAY+20);
        hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");

        msleep(1);

        printk("[zinitix] set tpd reset pin to high\n");
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
        msleep(10);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

        msleep(CHIP_ON_DELAY+20);  
        ts_power_sequence(i2c_client); 

        if(true == ts_init_touch()) {
			zinitix_resume_proc();
			break;
        }

    }while (1);
    printk("[zinitix] upgradefw success end\n");
}

struct task_struct *upgrade_fw_thread = NULL;
void upgrade_zinitix_fw_thread(void)
{
    if (fw_thread == 0)return;
    upgrade_fw_thread = kthread_run(upgradefw, 0, "upgrade tp fw");
    if (IS_ERR(upgrade_fw_thread)) {
        printk("[zinitix] failed to create upgrade fw thread\n");
    }
}
EXPORT_SYMBOL(upgrade_zinitix_fw_thread);


static int __devexit tpd_remove(struct i2c_client *client)
{
 printk("mtk_tpd: enter bt4xx tpd_remove\n");
 
#ifdef TPD_HIGH_SPEED_DMA
	if(CTPI2CDMABuf_va)
	{
		dma_free_coherent(NULL, CTP_I2C_DMA_BUF_SIZE, CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		CTPI2CDMABuf_va = NULL;
		CTPI2CDMABuf_pa = NULL;
	}
#endif

       misc_deregister(&touch_misc_device);

#ifdef ESD_CHECK	
destroy_workqueue(ctp_read_id_workqueue);
#endif	
 return 0;
}

static int tpd_local_init(void)
{
printk("mtk_tpd: enter bt4xx tpd_local_init\n");

TPD_DMESG("Focaltech bt4xx I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

if(i2c_add_driver(&tpd_i2c_driver)!=0)
{
TPD_DMESG(" bt4xx unable to add i2c driver.\n");
TP_sysfs_exit();
CTP_sysfs_exit();
	return -1;
}
#ifdef TPD_HAVE_BUTTON     
  tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   

		TPD_DMESG("end bt4xx %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }
 /*********************************************************************/
 
 static int zinitix_resume_proc()
 {
	 int i = 0;
	 int pwd_ret;
     u16 reg_mask = 0x880f;
	// u16 reg_mask = 0x883f;
 
	 tpd_clear_report_data();
hwPowerDown(MT6323_POWER_LDO_VGP1,"TP"); 
//hwPowerDown(MT6323_POWER_LDO_VGP2,"TP"); 
mdelay(CHIP_OFF_DELAY+20);
hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
//hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, "TP");
  msleep(1);
  mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
  mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
  msleep(10);
  mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
 
  msleep(CHIP_ON_DELAY+20);  
	 
	 pwd_ret = ts_power_sequence(i2c_client);
	 if (pwd_ret < 0){
		  printk(KERN_ERR "mtk_tpd: pwd_ret: %d\n", pwd_ret);
	 }
	 
	  for (i = 0; i < 10; i++) {
		  if (ts_write_cmd(i2c_client ,ZINITIX_SWRESET_CMD) == 0)
			  break;
		  mdelay(10);
		printk(KERN_ERR "send cmd times %d\n\n", i);
	  }
 
	  if (i == 10) {
		 printk("fail to send reset command\r\n");
		 goto fail_init;
	  }
	 
	  //set touch_mode
	  if (ts_write_reg(i2c_client, ZINITIX_TOUCH_MODE, TOUCH_MODE) != 0)
		  goto fail_init;
 
	  if (ts_write_reg(i2c_client ,ZINITIX_SUPPORTED_FINGER_NUM,(u16)MAX_SUPPORTED_FINGER_NUM) != 0)
		  goto fail_init;
 
	  //hardware calibration
 // 	if(ts_hw_calibration() == false)
 // 	  goto fail_init;
	 
	 /* initialize */
	 if (ts_write_reg(i2c_client ,ZINITIX_X_RESOLUTION,(u16)(TPD_RES_MAX_X)) != 0)
		 goto fail_init;
 
	 if (ts_write_reg(i2c_client ,ZINITIX_Y_RESOLUTION,(u16)(TPD_RES_MAX_Y)) != 0)
		 goto fail_init;
 
	  if (ts_write_reg(i2c_client, ZINITIX_INT_ENABLE_FLAG, reg_mask) !=0)
		  goto fail_init;
	  printk("bt4xx touch ZINITIX_INT_ENABLE_FLAG = 0x%04X\r\n", reg_mask);
 
	  
	  /* read garbage data */
	  for (i = 0; i < 10; i++) {
		  ts_write_cmd(i2c_client , ZINITIX_CLEAR_INT_STATUS_CMD);
		  udelay(10);
	  }
	  
#if  ZINITIX_ESD_TIMER_INTERVAL  
 // 	if (ts_write_reg(i2c_client, 0x011D,0x6000) != I2C_SUCCESS)
 // 	   goto fail_init;
	 
	  if (ts_write_reg(i2c_client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 
			 ZINITIX_SCAN_RATE_HZ*ZINITIX_ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		 goto fail_init;
   
	 if (use_esd_timer) {
		 ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
		 printk(KERN_ERR "zinitix_resume_proc:esd timer start\r\n");
	 }
#endif
 
	  return 0;
  
  fail_init:
	  return -1;
				  
 }

static int tpd_resume(struct i2c_client *client)
{
	int i, ret ;
	
	TPD_DMESG(TPD_DEVICE "TPD wake up\n");
	//#ifdef TDP_CLOSE_POWER_IN_SLEEP
    #if !USE_WAKEUP_GESTURE
	//hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "TP");
	// hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_1800, "TP");

	ret = zinitix_resume_proc();
	if(ret < 0){
	printk("tpd_resume : zinitix_resume_proc error\n");
	}
	#else
	
	ts_write_cmd(i2c_client, ZINITIX_WAKEUP_CMD);
	msleep(1);
	ts_write_cmd(i2c_client ,ZINITIX_SWRESET_CMD);
	msleep(20);    
	for(i=0; i<10; i++)
	{
	     ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
	     mdelay(10);
	}
	
    #if  ZINITIX_ESD_TIMER_INTERVAL      	   
    		if (ts_write_reg(i2c_client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 
    			   ZINITIX_SCAN_RATE_HZ*ZINITIX_ESD_TIMER_INTERVAL) != I2C_SUCCESS)
    		   goto fail_init;
    	   
    	   if (use_esd_timer) {
    		   ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
    		   printk(KERN_ERR "zinitix_resume_proc:esd timer start\r\n");
    	   }
    #endif
	#endif
	
	tpd_clear_report_data();    
	tpd_halt = 0;
	
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	// mt_eint_unmask(CUST_EINT_ETP_NUM); 
	TPD_DMESG(TPD_DEVICE "TPD wake up done\n");
	return;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{

  TPD_DMESG(TPD_DEVICE "TPD enter sleep\n");
	tpd_halt = 1;
	
#if !USE_WAKEUP_GESTURE
  mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
    tpd_clear_report_data();
#if ZINITIX_ESD_TIMER_INTERVAL
    if (use_esd_timer) {
		ts_esd_timer_stop(tpd);
		printk(KERN_ERR "tpd_suspend:ts_esd_timer_stop\n");
	}
#endif
        ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
        ts_write_cmd(i2c_client, ZINITIX_SLEEP_CMD);
		
#if !USE_WAKEUP_GESTURE
	    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
#endif

  TPD_DMESG(TPD_DEVICE "TPD enter sleep done\n");
  return;
} 


 /*********************************************************************/
/*                              ADD SYSFS                                                                           */
/*********************************************************************/
#ifdef TP_SYSFS_SUPPORT



static ssize_t TP_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
    char *s = buf;
    int ret;
    u16 firmware_version;
    u16 minor_firmware_version;	
    u16 reg_data_version;
    u16 ic_vender;
    u16 module_id;
    u16 hw_id;
    u16 chip_id;
    
    U8 val[2];

    //power on TP
    zinitix_resume_proc();
    s += sprintf(s, "IC_VENDOR:   Zinitix\n");
    
    ret = zinitix_i2c_DMAread(i2c_client, 0x001e,2, (u8 *)&(val[0]));
    s += sprintf(s, "Module_ID:   0x%04x \n", (val[0]|(val[1]<<8)));

    
#ifdef FW_FOR_GLOBAL
    s += sprintf(s, "Global_FW_Version \n");
#else
    s += sprintf(s, "China_FW_Version \n");
#endif




    ret = zinitix_i2c_DMAread(i2c_client, ZINITIX_FIRMWARE_VERSION,2, (u8 *)&(val[0]));
    s += sprintf(s, "FW_VERSION:   0x%04x \n", (val[0]|(val[1]<<8)));

    ret = zinitix_i2c_DMAread(i2c_client, ZINITIX_MINOR_FW_VERSION,2, (u8 *)&(val[0]));
    s += sprintf(s, "MINOR_FW_VERSION:   0x%04x \n", (val[0]|(val[1]<<8)));

    ret = zinitix_i2c_DMAread(i2c_client, ZINITIX_DATA_VERSION_REG,2, (u8 *)&(val[0]));
    s += sprintf(s, "Register_Version:   0x%04x \n", (val[0]|(val[1]<<8)));

//    ret = zinitix_i2c_DMAread(i2c_client, ZINITIX_IC_VENDOR_ID,2, (u8 *)&(val[0]));
//    s += sprintf(s, "IC_VENDOR_ID:   0x%04x\n", (val[0]|(val[1]<<8)));

    ret = zinitix_i2c_DMAread(i2c_client, ZINITIX_HW_ID,2, (u8 *)&(val[0]));
    s += sprintf(s, "HW_ID:   0x%04x\n", (val[0]|(val[1]<<8)));

    ret = zinitix_i2c_DMAread(i2c_client, ZINITIX_CHIP_REVISION,2, (u8 *)&(val[0]));
    s += sprintf(s, "CHIP_ID:   0x%04x\n", (val[0]|(val[1]<<8)));

    ret = zinitix_i2c_DMAread(i2c_client, ZINITIX_AFE_FREQUENCY,2, (u8 *)&(val[0]));   
    s += sprintf(s, "Rate:  %d\n", (val[0]|(val[1]<<8)));  
    
    ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
    ts_write_cmd(i2c_client, ZINITIX_SLEEP_CMD);
    



    return (s - buf);
}

static ssize_t TP_value_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{

    return n;
}

static ssize_t get_driver_version(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
    int count = 0;
    return count;
}

static int download_fw_online(struct i2c_client *client, const char *file_name)
{
    size_t size = 0;
    u16 reg_mask = 0x880f;//0x800f;
    int i  = 0;
    struct file *fp = NULL;
    struct inode *inode = NULL;
    mm_segment_t fs;
    loff_t pos = 0;
    off_t fsize; 
    unsigned long magic; 	

	printk(KERN_ERR "tpd_sys:download_fw_online!\n");
	
	//STEP 1: open the file
#if 1
	fp = filp_open(file_name, O_RDONLY , 0777);
	
	if (IS_ERR(fp))
	{
		printk(KERN_ERR "tpd_sys:open_firmware_file_failed!\n");
		return -1;
	}
	inode=fp->f_dentry->d_inode;
	fsize=inode->i_size;
//	magic=inode->i_sb->s_magic; 

	fs = get_fs();
//	set_fs(KERNEL_DS);
	set_fs(get_ds());
	//STEP 2:read the file to buffer	
	size = fp->f_op->read(fp,zinitix_sys_fw_info->local_buf,fsize,&(fp->f_pos));
	if (size < 0)
	{
		printk(KERN_ERR "tpd_sys:read_firmware_file_failed!\n");
		return -2;		
	}
#endif

#if 0
	printk(KERN_ERR "*****tpd_sys:fsize %d****\n", fsize);
	printk(KERN_ERR "*****tpd_sys:size %d****\n", size);
	printk(KERN_ERR "*****tpd_sys:MELFAS_binary_length %d****\n", MELFAS_binary_length);
#endif 

	filp_close(fp, NULL);
	set_fs(fs);

    printk(KERN_ERR "ZINITIX_CMD:start upgrade\n");

    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
    
    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
    mdelay(50);
    hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
    //    hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_3300, "TP");    
    msleep(CHIP_ON_DELAY);  

    printk("set tpd reset pin to high\n");
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(CHIP_ON_DELAY);  

    if(ts_power_sequence(i2c_client) != 0){
         printk(KERN_ERR "error:ts_power_sequence\n");
        return -1;
    }
    
     for (i = 0; i < 10; i++) {
    	 if (ts_write_cmd(i2c_client ,ZINITIX_SWRESET_CMD) == 0)
    		 break;
    	 mdelay(10);
          printk(KERN_ERR "send cmd times %d\n\n", i);
     }
     //set touch_mode
    if(ts_write_reg(i2c_client, ZINITIX_TOUCH_MODE, TOUCH_MODE)!= 0){
         printk(KERN_ERR "error:ZINITIX_TOUCH_MODE\n"); 
         return -1;
    }
    if(ts_write_reg(i2c_client ,ZINITIX_SUPPORTED_FINGER_NUM,(u16)MAX_SUPPORTED_FINGER_NUM) !=0){
        printk(KERN_ERR "error:ZINITIX_SUPPORTED_FINGER_NUM\n"); 
        return -1;
    }
    
    ts_upgrade_firmware(&zinitix_sys_fw_info->local_buf[0],32*1024);
    
    if(ts_hw_calibration() == false)
       return -1;
    
   /* disable chip interrupt */
    if (ts_write_reg(i2c_client,ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
       printk(KERN_ERR "error:ZINITIX_INT_ENABLE_FLAG\n"); 
    mdelay(10);
            
    /* initialize */
    if (ts_write_reg(i2c_client ,ZINITIX_X_RESOLUTION,(u16)(TPD_RES_MAX_X)) != 0)
        printk(KERN_ERR "error:ZINITIX_X_RESOLUTION\n"); 
    if (ts_write_reg(i2c_client ,ZINITIX_Y_RESOLUTION,(u16)(TPD_RES_MAX_Y)) != 0)
        printk(KERN_ERR "error:ZINITIX_Y_RESOLUTION\n"); 
     if (ts_write_reg(i2c_client, ZINITIX_INT_ENABLE_FLAG, reg_mask) !=0)
        printk("bt4xx touch ZINITIX_INT_ENABLE_FLAG = 0x%04X\r\n", reg_mask);

     /* read garbage data */
     for (i = 0; i < 10; i++) {
    	 ts_write_cmd(i2c_client , ZINITIX_CLEAR_INT_STATUS_CMD);
    	 udelay(10);
     }
     

    ts_write_cmd(i2c_client, ZINITIX_SLEEP_CMD);
    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");

    //mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 

	return 0;

    
}

static ssize_t update_fw_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
    int count = 0;
    int i = 0;
    //    count+= snprintf(buf, PAGE_SIZE, "sys_err_num: %d\n",sys_err_num);
    for (i = 0; i< MAX_SUPPORTED_FINGER_NUM; i++){
        count+= snprintf(buf + count, PAGE_SIZE - count,  "finger_status_%d: %d,sub_status:%d\n",i,
                finger_status[i],zinitix_i2c_touch_info.coord[i].sub_status);
    }

    return count;
}

static ssize_t update_fw_store(struct device *dev,
        struct device_attribute *attr, const char *buf,size_t count)
{
    int ret = 0;
    const char *file_name = buf;

    sys_err_num = 0;

    printk(KERN_ERR "tpd_sys:Start_update_firmware_online...\n");

    if (sscanf(buf, "%s", file_name) < 0){
        printk("tpd_sys:input_error!\n");
        return -EINVAL;
    }
	printk(KERN_ERR "tpd_sys:local_buf:*%s*\n",file_name);
	sys_err_num ++;     // 1
	
	sys_err_num++;	    // 2
	printk(KERN_ERR "tpd_sys:Enter_online_mode!\n");
	
#if 1	
	zinitix_sys_fw_info = (struct zinitix_sys_fw_info *) kzalloc(sizeof(struct zinitix_sys_fw_info),GFP_KERNEL);
	if (zinitix_sys_fw_info == NULL)
	{
		printk(KERN_ERR "******tpd_sys:malloc_failed!****\n");
		return count;
	}

	ret = download_fw_online(i2c_client,file_name);	
	if (ret < 0)
	{
		printk(KERN_ERR "tpd_sys:download_fw_online failed!\n");
	}
	kfree(zinitix_sys_fw_info);
	printk(KERN_ERR "******tpd_sys:free!******");
#endif				
	return count;

}

/******************************Raw_data_check*******************************/
#if 1
static void run_reference_read_PPDND(void) //DND
{			  
    unsigned int min, max;
    int min_node, max_node;
    int i,j;
	int mode_retry_cnt = 5;

    int ppdata_h_diff_buf;
    int ppdata_v_diff_buf;

    int h_diff_spec = 0;
    int v_diff_spec = 0;
    int tsp_status_result = 1; //Good 1 , NG 0
    int ppdata_min_spec =0;
    int ppdata_max_spec =0;
    int button_min_spec = 0;
    

    h_diff_spec = 1500;  //truly spec
    v_diff_spec = 2000;

    ppdata_min_spec = 6000;
    ppdata_max_spec = 14000;

    ts_set_touchmode(TOUCH_PPDND_MODE);
    get_raw_data((u8 *)m_nDndData, 10);
    msleep(5);
    get_raw_data((u8 *)m_nDndData, 10);
    ts_set_touchmode(TOUCH_POINT_MODE);
	msleep(5);   
    for(i=0;i<mode_retry_cnt;i++) {
		ts_set_touchmode(TOUCH_POINT_MODE);
	}


    min = m_nDndData[0];
    max = m_nDndData[0];
    min_node = 0;
    max_node = 0;

///////////// test of show ppdata /////////////////
	for(i=0; i< TOUCH_MAX_X; i++)
	{
		for(j=0; j< TOUCH_MAX_Y; j++)
		{
			printk("[TSP] PP DATA : %d ", m_nDndData[j+i*TOUCH_MAX_Y]);

			if(m_nDndData[j+i*TOUCH_MAX_Y] < min)
			{
				if((i < TOUCH_MAX_X-1)) {
					min = m_nDndData[j+i*TOUCH_MAX_Y];
                    min_node = j+i*TOUCH_MAX_Y;
                }
				else
			    {
			//		if(j==3||j==14) 
			//			min = m_nDndData[j+i*TOUCH_MAX_Y];
			    }
			}

			if(m_nDndData[j+i*TOUCH_MAX_Y] > max)
			{
				if((i < TOUCH_MAX_X-1)) {
					max = m_nDndData[j+i*TOUCH_MAX_Y];
                    max_node = j+i*TOUCH_MAX_Y;
				}
                else
			    {
				//	if(j==3||j==14)
				//		max = m_nDndData[j+i*TOUCH_MAX_Y];
				}
			}

		}
	printk("\n");
	}

	printk(KERN_ERR " PP data raw_min= %u, raw_max= %u", min, max);
	
	//data
	min_raw_data = min;
	max_raw_data = max;
	//node
	min_node_val = min_node;
	max_node_val = max_node;

    if (min <= ppdata_min_spec) //truly min spec
        printk(KERN_ERR "This node is spec out min node : %d", min_node+1);  

    if (max >= ppdata_max_spec) //truly max spec 
        printk(KERN_ERR "This node is spec out max node : %d", max_node+1); 
 
    if (min < ppdata_min_spec || max > ppdata_max_spec) 
        tsp_status_result = 0; //MG

    //H-diff check
    for(i=0; i< ((TOUCH_MAX_X-1)*TOUCH_MAX_Y-1); i++) {
        if(i%(TOUCH_MAX_Y-1) != 0) {
            ppdata_h_diff_buf = m_nDndData[i+1] - m_nDndData[i];
            if (m_nDndData[i] > m_nDndData[i+1]) 
                ppdata_h_diff_buf = m_nDndData[i] - m_nDndData[i+1];     

            if (ppdata_h_diff_buf > h_diff_spec) {
                printk("This node is H-DIFF spec out max node : %d", i+1); 
                 tsp_status_result = 0;
                diff_flag = 1;
                h_diff_result =  ppdata_h_diff_buf;
            }
        }
    }

    //V-diff check
    for(i=0; i< (TOUCH_MAX_X*TOUCH_MAX_Y-(2*TOUCH_MAX_Y)); i++) {

        ppdata_v_diff_buf = m_nDndData[i+TOUCH_MAX_Y] - m_nDndData[i];
        if (m_nDndData[i] > m_nDndData[i+1]) 
            ppdata_v_diff_buf = m_nDndData[i] - m_nDndData[i+TOUCH_MAX_Y];     

        if (ppdata_v_diff_buf > v_diff_spec) {
            printk("This node is V-DIFF spec out max node : %d", i+TOUCH_MAX_Y); 
            tsp_status_result = 0; //MG
             diff_flag = 2;
            v_diff_result =  ppdata_v_diff_buf;           
        }    
    }

    //button check
    if (m_nDndData[(TOUCH_MAX_X-1)*TOUCH_MAX_Y+4] < button_min_spec) {
            printk("back key button is spec out"); 
            tsp_status_result = 0; //MG
            diff_flag = 3;
    }        
    if (m_nDndData[(TOUCH_MAX_X-1)*TOUCH_MAX_Y+8] < button_min_spec) {
            printk("home key button is spec out");
            tsp_status_result = 0; //MG
            diff_flag = 4;
    }      
    if (m_nDndData[(TOUCH_MAX_X-1)*TOUCH_MAX_Y+13] < button_min_spec) {
            printk("menu key button is spec out");
            tsp_status_result = 0; //MG
            diff_flag = 5;
    }
    
    //result
    if (tsp_status_result == 0) 
        printk(KERN_ERR "This tsp spec is failed");
    
	raw_data_result = tsp_status_result;
    
    ///////////// test of show ppdata /////////////////
	
}
int ts_set_touchmode(u16 value)
{
	int i;
	u16 N_cnt;
	u16 U_cnt;
	u16 shift_n;

	printk(KERN_ERR "[zinitix_touch] ts_set_touchmode.\n");
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
     
	if(value == TOUCH_PPDND_MODE) {
		N_cnt = 18;
		U_cnt = 20;
		
		if (ts_write_reg(i2c_client, ZINITIX_DND_N_COUNT, N_cnt)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITIX_DND_N_COUNT %d.\n", N_cnt);
		if (ts_write_reg(i2c_client, ZINITIX_DND_U_COUNT, U_cnt)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITIX_DND_U_COUNT %d.\n", U_cnt);
		if (ts_write_reg(i2c_client, ZINITIX_AFE_FREQUENCY, 91)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITIX_AFE_FREQUENCY DND.\n");
	}

	if(value == TOUCH_SDND_MODE) {
		N_cnt = 12;
		U_cnt = 2;
		shift_n = 3;
		
		if (ts_write_reg(i2c_client, ZINITIX_DND_N_COUNT, N_cnt)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITIX_DND_N_COUNT %d.\n", N_cnt);
		if (ts_write_reg(i2c_client, ZINITIX_DND_U_COUNT, U_cnt)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITIX_DND_U_COUNT %d.\n", U_cnt);
		if (ts_write_reg(i2c_client, ZINITIX_AFE_FREQUENCY, 64)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITIX_AFE_FREQUENCY SDND.\n");
		if (ts_write_reg(i2c_client, ZINITIX_N_SHIFT_COUNT, shift_n)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITIX_N_SHIFT_COUNT.\n");
	}
	else if((m_nTouchMode == TOUCH_PPDND_MODE)||(m_nTouchMode == TOUCH_SDND_MODE))
	{
		
		if (ts_write_reg(i2c_client, ZINITIX_AFE_FREQUENCY, m_nFrequency)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to reset ZINITIX_AFE_FREQUENCY.\n");
		if (ts_write_reg(i2c_client, ZINITIX_DND_U_COUNT, 2)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to reset ZINITIX_DND_U_COUNT.\n");
		if (ts_write_reg(i2c_client, ZINITIX_N_SHIFT_COUNT, m_nShift)!=I2C_SUCCESS)
			printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to reset ZINITIX_N_SHIFT_COUNT.\n");
	}


	m_nTouchMode = value;

	printk(KERN_ERR "[zinitix_touch] tsp_set_testmode, touchkey_testmode = %d\n", m_nTouchMode);

	if (ts_write_reg(i2c_client, ZINITIX_TOUCH_MODE, m_nTouchMode)!=I2C_SUCCESS)
		printk(KERN_ERR "[zinitix_touch] TEST Mode : Fail to set ZINITX_TOUCH_MODE %d.\n", m_nTouchMode);


	// clear garbage data
	for(i=0; i < 10; i++) {
		mdelay(20);
		ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
	}

	
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
	return 1;
}

int get_raw_data(u8 *buff, int skip_cnt)
{
	u32 total_node = TOUCH_MAX_X*TOUCH_MAX_Y;
	int i;
	int count = 0;

	printk(KERN_ERR "[zinitix_touch] Get_raw_data");
	u32 sz, sz2;
    u32 rawdata_addr = ZINITIX_RAWDATA_REG;

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	for(i=0; i<skip_cnt; i++) 
	{
#if 0
//======================================
//You Must Interrupt Pin Check
		while (1 == mt_get_gpio_in(GPIO_CTP_EINT_PIN)){
			msleep(1);
			count ++;
			printk (KERN_ERR "[tpd] a:1 == mt_get_gpio_in(GPIO_CTP_EINT_PIN) :%d\n", count);
		}
#endif
		ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
		msleep(1);
	}
//======================================
//You Must Interrupt Pin Check
#if 0
	while (1 == mt_get_gpio_in(GPIO_CTP_EINT_PIN)){
		msleep(1);
		count ++;
		printk (KERN_ERR "[tpd] b:1 == mt_get_gpio_in(GPIO_CTP_EINT_PIN) :%d\n", count);
	}
#endif
//static int zinitix_i2c_DMAread(struct i2c_client *client, U16 addr, U16 len, U8 *rxbuf);
//(zinitix_i2c_DMAread(i2c_client,ZINITIX_READ_FLASH,RD_TC_SECTOR_SZ,(u8*)&verify_data[flash_addr]) < 0)

	sz = total_node*2;
	printk (KERN_ERR "[tpd] read raw data...\n");
	msleep(5);

	printk (KERN_ERR "step_1\n");
    for(i=0; i<(sz/64); i++) {   		//64*16 byte   
		printk (KERN_ERR "step_1:%d\n",count++); 
	//	if (zinitix_i2c_DMAread(i2c_client, rawdata_addr + i, 64, (u8 *)&m_nDndData[i*32]) < 0) {
		if (ts_read_data(misc_touch_dev->client, rawdata_addr + i, (u8 *)&m_nDndData[i*32], 64) < 0) {
			printk(KERN_ERR "error_1 : read zinitix tc raw data\n");			
			return false;
		}
	}

#if 1
	sz2 = sz - (i*64);
	if(sz2 > 0) {						//56 byte
		printk (KERN_ERR "step_2:%d\n",count++); 
	//    if (zinitix_i2c_DMAread(i2c_client,rawdata_addr + i, sz2, (u8 *)&m_nDndData[i*32]) < 0) {
	    if (ts_read_data(misc_touch_dev->client,rawdata_addr + i, (char *)&m_nDndData[i*32], sz2) < 0) {
			printk(KERN_ERR "error_2 : read zinitix tc raw data\n");		
			return false;
		}
	}
#endif

//    m_nDndData = m_nppdata;

	ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);	

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);	
	return true;
}

/******************************************************************************/

static ssize_t get_rawdata_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    int j = 0;
    int retry_after_show = 0;

    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
    mdelay(50);
    zinitix_resume_proc();
    
	// Stop the timer
#if ZINITIX_ESD_TIMER_INTERVAL
    if (use_esd_timer) {
		ts_esd_timer_stop(tpd);
		printk(KERN_ERR "tpd_suspend:ts_esd_timer_stop\n");
	}	
#endif
#if 1	
	run_reference_read_PPDND();
#if 0
	count += snprintf(buf + count, PAGE_SIZE - count, "zinitix_raw_data:\n Y/X ");
	//show the x number
	for (i = 0; i < TOUCH_MAX_Y; i++){
		count += snprintf(buf + count, PAGE_SIZE - count, " [%02d] ", i+1);
	}

	count += snprintf(buf + count, PAGE_SIZE - count, "\n[01] ");
	for(i = 0; i < RAWDATA_SIZE; i++){
		//show the Y number
		if(i%18 == 0 && i != 0){
			j++;
			count += snprintf(buf + count, PAGE_SIZE - count, "\n[%02d] ",j+1);
		}
		count += snprintf(buf + count, PAGE_SIZE - count, "%05d ",m_nDndData[i]);
	}
#endif

#endif
	
for(i=0;i < 20; i++){
   for(j=0;j < 11; j++)
   {
	count += snprintf(buf + count, PAGE_SIZE - count, " %d",m_nDndData[i*11 + j]);
   }
	count += snprintf(buf + count, PAGE_SIZE - count, "\n");
}
//  if (min_raw_data <= 9795) //truly min spec
	count += snprintf(buf + count, PAGE_SIZE - count, "\nMin node: %d\n",min_node_val+1);
	count += snprintf(buf + count, PAGE_SIZE - count, "Min value: %05d\n",min_raw_data);

//  if (max_raw_data >= 27437) //truly max spec 
	count += snprintf(buf + count, PAGE_SIZE - count, "Max node: %d\n",max_node_val+1);
	count += snprintf(buf + count, PAGE_SIZE - count, "Max value: %05d\n",max_raw_data);
	//start the timer
#if ZINITIX_ESD_TIMER_INTERVAL
	if (use_esd_timer) {
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
		printk(KERN_ERR "tpd:esd timer start\r\n");
	}
#endif
      if (diff_flag == 0){
            count += snprintf(buf + count, PAGE_SIZE - count, "Diff_value was OK!\n");
      }else if (diff_flag == 1){
            count += snprintf(buf + count, PAGE_SIZE - count, "H_Diff_value was NOK: %d\n",h_diff_result);
      }else if (diff_flag == 2){
            count += snprintf(buf + count, PAGE_SIZE - count, "V_Diff_value was NOK:%d\n",v_diff_result);
      }else{
            count += snprintf(buf + count, PAGE_SIZE - count, "key_value was NOK!\n");
      }
        
//	count += snprintf(buf + count, PAGE_SIZE - count, "raw_data_result: %d \n",raw_data_result);
    	count +=snprintf(buf + count, PAGE_SIZE - count,"\n%s\n", 
             raw_data_result ? "==OK==" : "==FAIL==");

	ts_set_touchmode(TOUCH_POINT_MODE);
    
    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
    mdelay(50);
    for(i=0;i<3;i++) {
        retry_after_show = zinitix_resume_proc();
        if(retry_after_show == 0) break;
    }    
    return count;
}
#endif


static DEVICE_ATTR(TP_DEBUG, 0644,  TP_value_show, TP_value_store);  
static DEVICE_ATTR(DRIVER_VERSION, 0644 ,  get_driver_version, NULL); 
static DEVICE_ATTR(update_fw, 0644, update_fw_show, update_fw_store); 
static DEVICE_ATTR(raw_data, 0644, get_rawdata_show, NULL); 			//add for check the rowdata

static struct attribute *TP_sysfs_attrs[] = {
	&dev_attr_TP_DEBUG.attr,
	&dev_attr_DRIVER_VERSION.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_raw_data.attr,
	NULL,
};

static struct attribute_group TP_attr_group = {
        .attrs = TP_sysfs_attrs,
};


static struct kobject *TP_ctrl_kobj = NULL;
int TP_sysfs_init(void)
{ 
	TP_ctrl_kobj = kobject_create_and_add("tp-info", NULL);
	if (!TP_ctrl_kobj)
		return -ENOMEM;

	return sysfs_create_group(TP_ctrl_kobj, &TP_attr_group);
}
//remove sysfs
void TP_sysfs_exit(void)
{
	sysfs_remove_group(TP_ctrl_kobj, &TP_attr_group);

	kobject_put(TP_ctrl_kobj);
}

static ssize_t rd_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int count = 0;
    int i = 0;
    int j = 0;
    int retry_after_show = 0;

    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
    mdelay(50);
    zinitix_resume_proc();

    // Stop the timer
#if ZINITIX_ESD_TIMER_INTERVAL
    if (use_esd_timer) {
        ts_esd_timer_stop(tpd);
        printk(KERN_ERR "tpd_suspend:ts_esd_timer_stop\n");
    }	
#endif
#if 1	
    run_reference_read_PPDND();
#if 0
    count += snprintf(buf + count, PAGE_SIZE - count, "zinitix_raw_data:\n Y/X ");
    /*
#define TOUCH_MAX_X	30      
#define	TOUCH_MAX_Y	18
*/
    //show the x number
    for (i = 0; i < TOUCH_MAX_Y; i++){
        count += snprintf(buf + count, PAGE_SIZE - count, " [%02d] ", i+1);
    }

    count += snprintf(buf + count, PAGE_SIZE - count, "\n[01] ");
    for(i = 0; i < RAWDATA_SIZE; i++){
        //show the Y number
        if(i%18 == 0 && i != 0){
            j++;
            count += snprintf(buf + count, PAGE_SIZE - count, "\n[%02d] ",j+1);
        }
        count += snprintf(buf + count, PAGE_SIZE - count, "%05d ",m_nDndData[i]);
    }
#endif

#endif

    for(i=0;i < 20; i++){
        for(j=0;j < 11; j++)
        {
            printk(" %d",m_nDndData[i*11 + j]);
        }
        printk("\n");
    }
    //  if (min_raw_data <= 9795) //truly min spec
    printk("\nMin node: %d\n",min_node_val+1);
    printk("Min value: %05d\n",min_raw_data);

    //  if (max_raw_data >= 27437) //truly max spec 
    printk("Max node: %d\n",max_node_val+1);
    printk("Max value: %05d\n",max_raw_data);
    //start the timer
#if ZINITIX_ESD_TIMER_INTERVAL
    if (use_esd_timer) {
        ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, tpd);
        printk(KERN_ERR "tpd:esd timer start\r\n");
    }
#endif
    /*
       static int diff_flag = 0;
       static int v_diff_result = 0;
       static int h_diff_result = 0;
       */
    if (diff_flag == 0){
        printk("Diff_value was OK!\n");
    }else if (diff_flag == 1){
        printk("H_Diff_value was NOK: %d\n",h_diff_result);
    }else if (diff_flag == 2){
        printk("V_Diff_value was NOK:%d\n",v_diff_result);
    }else{
        printk("key_value was NOK!\n");
    }

    //	count += snprintf(buf + count, PAGE_SIZE - count, "raw_data_result: %d \n",raw_data_result);
    count +=snprintf(buf + count, PAGE_SIZE - count,"%s", 
            raw_data_result ? "PASS" : "FAIL");

    ts_set_touchmode(TOUCH_POINT_MODE);

    hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
    mdelay(50);
    for(i=0;i<3;i++) {
        retry_after_show = zinitix_resume_proc();
        if(retry_after_show == 0) break;
    }    
    return count;
}

static ssize_t rd_result_store(struct device *dev,
        struct device_attribute *attr, const char *buf,size_t count)
{
    return -EPERM;
}

static DEVICE_ATTR(rawdata, 0644, get_rawdata_show, NULL);
static DEVICE_ATTR(rd_result, 0644, rd_result_show, rd_result_store);
static DEVICE_ATTR(tp_info, 0644,  TP_value_show, TP_value_store);   

static struct attribute *CTP_sysfs_attrs[] = {
    &dev_attr_rawdata.attr,
    &dev_attr_rd_result.attr,
    &dev_attr_tp_info.attr,
    NULL,
};
static struct attribute_group CTP_attr_group = {
    .attrs = CTP_sysfs_attrs,
};

//add sysfs
//struct kobject *CTP_ctrl_kobj;
static struct kobject *CTP_ctrl_kobj = NULL;
static int CTP_sysfs_init(void)
{ 
    CTP_ctrl_kobj = kobject_create_and_add("CTP", NULL);
    if (!CTP_ctrl_kobj)
        return -ENOMEM;

    return sysfs_create_group(CTP_ctrl_kobj, &CTP_attr_group);
}
//remove sysfs
static void CTP_sysfs_exit(void)
{
    sysfs_remove_group(CTP_ctrl_kobj, &CTP_attr_group);
    kobject_put(CTP_ctrl_kobj);
}
#endif


static bool ts_set_touchmode_apk(u16 value){
    int i;
printk("ts_set_touchmode apk mode:%d\r\n",value);
mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    tpd_halt=1;
    if(value == TOUCH_DND_MODE) {
        if (ts_write_reg(misc_touch_dev->client, ZINITIX_DND_N_COUNT, SEC_DND_N_COUNT)!=I2C_SUCCESS)
            printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_DND_N_COUNT %d.\r\n", SEC_DND_N_COUNT);
        if (ts_write_reg(misc_touch_dev->client, ZINITIX_AFE_FREQUENCY, SEC_DND_FREQUENCY)!=I2C_SUCCESS)
            printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_AFE_FREQUENCY %d.\r\n", SEC_DND_FREQUENCY);
    } else if(misc_touch_dev->touch_mode == TOUCH_DND_MODE)
    {
       printk("ts_set_touchmode ZINITIX_AFE_FREQUENCY %d.\r\n", misc_touch_dev->cap_info.afe_frequency);
        if (ts_write_reg(misc_touch_dev->client, ZINITIX_AFE_FREQUENCY, 
            misc_touch_dev->cap_info.afe_frequency)!=I2C_SUCCESS)
            printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_AFE_FREQUENCY %d.\r\n", misc_touch_dev->cap_info.afe_frequency);
    }
        

    if(value == TOUCH_SEC_NORMAL_MODE)
        misc_touch_dev->touch_mode = TOUCH_POINT_MODE;
    else
        misc_touch_dev->touch_mode = value;

    printk(KERN_INFO "[zinitix_touch] tsp_set_testmode, touchkey_testmode = %d\r\n", misc_touch_dev->touch_mode);

    if(misc_touch_dev->touch_mode != TOUCH_POINT_MODE) {
        if (ts_write_reg(misc_touch_dev->client,
            ZINITIX_DELAY_RAW_FOR_HOST,
            RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
            zinitix_printk("Fail to set ZINITIX_DELAY_RAW_FOR_HOST.\r\n");

    } 

    if (ts_write_reg(misc_touch_dev->client, ZINITIX_TOUCH_MODE, misc_touch_dev->touch_mode)!=I2C_SUCCESS)
        printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITX_TOUCH_MODE %d.\r\n", misc_touch_dev->touch_mode);


    // clear garbage data
    for(i=0; i < 10; i++) {
        mdelay(20);
        ts_write_cmd(misc_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
       }
     mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
     tpd_halt=0;

    return 1;         
}


static bool ts_get_raw_data()
{
    u32 total_node = TOUCH_MAX_X*TOUCH_MAX_Y;//misc_touch_dev->cap_info.total_node_num;
    u32 sz,sz2;
    int count=0;
    u32 rawdata_addr=ZINITIX_RAWDATA_REG;
    int i;
    printk("ts_get_raw_data: total node :%d\r\n",total_node);
        sz = total_node*2;
	//printk (KERN_ERR "[tpd] read raw data...\n");
	msleep(1);
     mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    tpd_halt=1;
    ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
	msleep(1);
	printk (KERN_ERR "ts_get_raw_data read raw data step_1\n");
    for(i=0; i<(sz/64); i++) {   		//64*16 byte   
		printk (KERN_ERR "ts_get_raw_data step_1:%d\n",count++); 
		if (ts_read_data(misc_touch_dev->client, rawdata_addr + i, (u8 *)&misc_touch_dev->cur_data[i*32], 64) < 0) {
			printk(KERN_ERR "error_1 : read zinitix tc raw data\n");			
			return false;
		}
	}
printk (KERN_ERR "[tpd] read raw data step_2\n");

#if 1
	sz2 = sz - (i*64);
	if(sz2 > 0) {						//56 byte
		printk (KERN_ERR "step_2:%d\n",count++); 
	    if (ts_read_data(misc_touch_dev->client,rawdata_addr + i, (char *)&misc_touch_dev->cur_data[i*32], sz2) < 0) {
			printk(KERN_ERR "error_2 : read zinitix tc raw data\n");		
			return false;
		}
	}
#endif




      misc_touch_dev->update = 1;
      ts_write_cmd(i2c_client, ZINITIX_CLEAR_INT_STATUS_CMD);
     mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
     tpd_halt=0;
printk (KERN_ERR "ts_get_raw_data end\n");
  // ts_set_touchmode_apk(TOUCH_POINT_MODE);
    return true;
}


static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
        return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
        return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36))
static int ts_misc_fops_ioctl(struct inode *inode,
        struct file *filp, unsigned int cmd,
        unsigned long arg)
#else
static long ts_misc_fops_ioctl(struct file *filp,
        unsigned int cmd, unsigned long arg)
#endif
{

    void __user *argp = (void __user *)arg;
    struct _raw_ioctl raw_ioctl;
    u8 *u8Data;
    int ret = 0;
    size_t sz = 0;
    u16 version;
    u16 mode;
    u16 reg_mask = 0x880f;//0x800f;
    u16 i;

    struct _reg_ioctl reg_ioctl;
    u16 val;
    int nval = 0;

    printk("tpd_sys enter ts_misc_fops_ioctl ==================\n");

    if (misc_touch_dev == NULL)
        return -1;

    /* zinitix_debug_msg("cmd = %d, argp = 0x%x\n", cmd, (int)argp); */

    printk("tpd_sys enter ts_misc_fops_ioctl cmd is %x\n",cmd);
    switch (cmd) {

        case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
    printk("tpd_sys TOUCH_IOCTL_GET_DEBUGMSG_STATE:%x \n",TOUCH_IOCTL_GET_DEBUGMSG_STATE);
                break;

        case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
    printk("tpd_sys TOUCH_IOCTL_SET_DEBUGMSG_STATE:%x \n",TOUCH_IOCTL_SET_DEBUGMSG_STATE);
            break;

        case TOUCH_IOCTL_GET_CHIP_REVISION:
    printk("tpd_sys TOUCH_IOCTL_GET_CHIP_REVISION:%x \n",TOUCH_IOCTL_GET_CHIP_REVISION);
            ret = misc_touch_dev->cap_info.ic_revision;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

        case TOUCH_IOCTL_GET_FW_VERSION:
    printk("tpd_sys TOUCH_IOCTL_GET_FW_VERSION:%x \n",TOUCH_IOCTL_GET_FW_VERSION);
            ret = misc_touch_dev->cap_info.firmware_version;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

       case TOUCH_IOCTL_GET_REG_DATA_VERSION:
    printk("tpd_sys TOUCH_IOCTL_GET_REG_DATA_VERSION:%x \n",TOUCH_IOCTL_GET_REG_DATA_VERSION);
                ret = misc_touch_dev->cap_info.reg_data_version;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

        case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
    printk("tpd_sys TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:%x \n",TOUCH_IOCTL_VARIFY_UPGRADE_SIZE);
            if (copy_from_user(&sz, argp, sizeof(size_t)))
                return -1;

            printk(KERN_INFO "firmware size = %d\r\n", sz);
            if (misc_touch_dev->cap_info.ic_fw_size != sz) {
                printk(KERN_INFO "firmware size error\r\n");
                return -1;
            }
            break;

        case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
    printk("tpd_sys TOUCH_IOCTL_VARIFY_UPGRADE_DATA:%x \n",TOUCH_IOCTL_VARIFY_UPGRADE_DATA);
            if (copy_from_user(m_pFirmware[0],
                        argp, misc_touch_dev->cap_info.ic_fw_size))
                return -1;

            version = (u16) (m_pFirmware[0][52] | (m_pFirmware[0][53]<<8));     

            printk(KERN_INFO "firmware version = %x\r\n", version);

            if (copy_to_user(argp, &version, sizeof(version)))
                return -1;
            break;

        case TOUCH_IOCTL_START_UPGRADE:
    printk("tpd_sys TOUCH_IOCTL_START_UPGRADE:%x \n",TOUCH_IOCTL_START_UPGRADE);
            ret = ts_upgrade_firmware(m_pFirmware[m_FirmwareIdx],32*1024);
            if(ts_hw_calibration() == false)
                return -1;

            /* disable chip interrupt */
            if (ts_write_reg(i2c_client,ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
                printk(KERN_ERR "error:ZINITIX_INT_ENABLE_FLAG\n"); 
            mdelay(10);

            /* initialize */
            if (ts_write_reg(i2c_client ,ZINITIX_X_RESOLUTION,(u16)(TPD_RES_MAX_X)) != 0)
                printk(KERN_ERR "error:ZINITIX_X_RESOLUTION\n"); 
            if (ts_write_reg(i2c_client ,ZINITIX_Y_RESOLUTION,(u16)(TPD_RES_MAX_Y)) != 0)
                printk(KERN_ERR "error:ZINITIX_Y_RESOLUTION\n"); 
            if (ts_write_reg(i2c_client, ZINITIX_INT_ENABLE_FLAG, reg_mask) !=0)
                printk("bt4xx touch ZINITIX_INT_ENABLE_FLAG = 0x%04X\r\n", reg_mask);

            /* read garbage data */
            for (i = 0; i < 10; i++) {
                ts_write_cmd(i2c_client , ZINITIX_CLEAR_INT_STATUS_CMD);
                udelay(10);
            }

            ts_write_cmd(i2c_client, ZINITIX_SLEEP_CMD);
            hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
            return ret;

        case TOUCH_IOCTL_GET_X_RESOLUTION:
    printk("tpd_sys TOUCH_IOCTL_GET_X_RESOLUTION:%x \n",TOUCH_IOCTL_GET_X_RESOLUTION);
            ret = misc_touch_dev->cap_info.x_resolution;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

        case TOUCH_IOCTL_GET_Y_RESOLUTION:
    printk("tpd_sys TOUCH_IOCTL_GET_Y_RESOLUTION:%x \n",TOUCH_IOCTL_GET_Y_RESOLUTION);
            ret = misc_touch_dev->cap_info.y_resolution;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

        case TOUCH_IOCTL_GET_X_NODE_NUM:
    printk("tpd_sys TOUCH_IOCTL_GET_X_NODE_NUM:%x \n",TOUCH_IOCTL_GET_X_NODE_NUM);
            ret = misc_touch_dev->cap_info.x_node_num;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

        case TOUCH_IOCTL_GET_Y_NODE_NUM:
    printk("tpd_sys TOUCH_IOCTL_GET_Y_NODE_NUM:%x \n",TOUCH_IOCTL_GET_Y_NODE_NUM);
            ret = misc_touch_dev->cap_info.y_node_num;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

        case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
    printk("tpd_sys TOUCH_IOCTL_GET_Y_NODE_NUM:%x \n",TOUCH_IOCTL_GET_Y_NODE_NUM);
            ret = misc_touch_dev->cap_info.total_node_num;
            if (copy_to_user(argp, &ret, sizeof(ret)))
                return -1;
            break;

        case TOUCH_IOCTL_HW_CALIBRAION:
                printk("tpd_sys TOUCH_IOCTL_HW_CALIBRAION=%x\n",TOUCH_IOCTL_HW_CALIBRAION);
#if 0
            ret = -1;
            disable_irq(misc_touch_dev->irq);
            down(&misc_touch_dev->work_proceedure_lock);
            if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
                printk(KERN_INFO"other process occupied.. (%d)\r\n",
                        misc_touch_dev->work_proceedure);
                up(&misc_touch_dev->work_proceedure_lock);
                return -1;
            }
            misc_touch_dev->work_proceedure = TS_HW_CALIBRAION;
            mdelay(100);

            /* h/w calibration */
            if(ts_hw_calibration(misc_touch_dev) == true)
                ret = 0;

            mode = misc_touch_dev->touch_mode;
            if (ts_write_reg(misc_touch_dev->client,
                        ZINITIX_TOUCH_MODE, mode) != I2C_SUCCESS) {
                printk(KERN_INFO "fail to set touch mode %d.\n",
                        mode);
                goto fail_hw_cal;
            }

            if (ts_write_cmd(misc_touch_dev->client,
                        ZINITIX_SWRESET_CMD) != I2C_SUCCESS)
                goto fail_hw_cal;
            ts_enable_irq(misc_touch_dev);
            misc_touch_dev->work_proceedure = TS_NO_WORK;
            up(&misc_touch_dev->work_proceedure_lock);
            return ret;
fail_hw_cal:
            ts_enable_irq(misc_touch_dev);
            misc_touch_dev->work_proceedure = TS_NO_WORK;
            up(&misc_touch_dev->work_proceedure_lock);
            return -1;
#endif 
            if(ts_hw_calibration() == true)
                return 0;
            else
                return -1;

        case TOUCH_IOCTL_SET_RAW_DATA_MODE:
                printk("tpd_sysTOUCH_IOCTL_SET_RAW_DATA_MODE =%x\n",TOUCH_IOCTL_SET_RAW_DATA_MODE);
            if (copy_from_user(&nval, argp, 4)) {
            printk("[zinitix_touch] error : copy_from_user\r\n");
            return -1;
            }
            ts_set_touchmode_apk((u16)nval);    

            return 0;

        case TOUCH_IOCTL_GET_REG:
            if (copy_from_user(&reg_ioctl,argp, sizeof(struct _reg_ioctl))) 
                {
                    printk("[zinitix_touch] error : copy_from_user\n");
                    return -1;
                }
                printk("tpd_sys TOUCH_IOCTL_GET_REG=%x\n",TOUCH_IOCTL_GET_REG);
            if (ts_read_data(misc_touch_dev->client,
                        reg_ioctl.addr, (u8 *)&val, 2) < 0)
            {
                printk("tpd_sys reg_ioctl.addr=%x,val=%x\n",reg_ioctl.addr,val);
                ret = -1;
            }
                printk("tpd_sys 2 reg_ioctl.addr=%x,val=%x\n",reg_ioctl.addr,val);

                nval = (int)val; 
            if (copy_to_user(reg_ioctl.val, (u8 *)&nval, 4)) {
                printk("[zinitix_touch] error : copy_to_user\n");
                return -1;
            }

                printk("tpd_sys TOUCH_IOCTL_GET_REG 3 =%x\n",TOUCH_IOCTL_GET_REG);

            return ret;

        case TOUCH_IOCTL_SET_REG:
                printk("tpd_sys TOUCH_IOCTL_SET_REG=%x\n",TOUCH_IOCTL_SET_REG);

            if (copy_from_user(&reg_ioctl,
                        argp, sizeof(struct _reg_ioctl))) {
                printk("[zinitix_touch] error : copy_from_user\n");
                return -1;
            }

            if (copy_from_user(&val, reg_ioctl.val, 4)) {
                printk("[zinitix_touch] error : copy_from_user\n");
                return -1;
            }
            if (ts_write_reg(misc_touch_dev->client,
                        reg_ioctl.addr, val) != I2C_SUCCESS)
                ret = -1;

            return ret;

        case TOUCH_IOCTL_DONOT_TOUCH_EVENT:

                printk("tpd_sys TOUCH_IOCTL_DONOT_TOUCH_EVENT=%x\n",TOUCH_IOCTL_DONOT_TOUCH_EVENT);
            if (ts_write_reg(misc_touch_dev->client,
                        ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
                ret = -1;
            return ret;

        case TOUCH_IOCTL_SEND_SAVE_STATUS:
                printk("tpd_sys TOUCH_IOCTL_SEND_SAVE_STATUS=%x\n",TOUCH_IOCTL_SEND_SAVE_STATUS);
            ret = 0;
            ts_write_reg(misc_touch_dev->client, 0xc003, 0x0001);
            ts_write_reg(misc_touch_dev->client, 0xc104, 0x0001);
            if (ts_write_cmd(misc_touch_dev->client,
                        ZINITIX_SAVE_STATUS_CMD) != I2C_SUCCESS)
                ret =  -1;

            mdelay(1000);   /* for fusing eeprom */
            ts_write_reg(misc_touch_dev->client, 0xc003, 0x0000);
            ts_write_reg(misc_touch_dev->client, 0xc104, 0x0000);

            return ret;

        case TOUCH_IOCTL_GET_RAW_DATA:
         printk("TOUCH_IOCTL_GET_RAW_DATA\r\n");
        #if 0
          if( ts_get_raw_data()==0)
	{
            printk("ts_get_raw_data error\r\n");
	    return -2;
	}
          #endif
       
        if (misc_touch_dev->update == 0) {
           printk("ts_get_raw_data update state error\r\n");
            return -2;
        }

        if (copy_from_user(&raw_ioctl,
            argp, sizeof(raw_ioctl))) {
           
            printk(KERN_INFO "[zinitix_touch] error : copy_from_user\r\n");
            return -1;
        }

        misc_touch_dev->update = 0;

        u8Data = (u8 *)&misc_touch_dev->cur_data[0];
        
        if (copy_to_user(raw_ioctl.buf, (u8 *)u8Data,
            raw_ioctl.sz)) {
            up(&misc_touch_dev->raw_data_lock);
            return -1;
        }

       
        return 0;

        default:
            break;
    }

    printk("tpd_sys out ts_misc_fops_ioctl ==================\n");
    return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	 .tpd_device_name = "bt4xx",
	 .tpd_local_init = tpd_local_init,
	 .suspend = tpd_suspend,
	 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	 .tpd_have_button = 1,
#else
	 .tpd_have_button = 0,
#endif		
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
 printk("MediaTek bt4xx touch panel driver init\n");
 
 i2c_register_board_info(0, &zinitix_i2c_tpd, 1);
	 if(tpd_driver_add(&tpd_device_driver) < 0)
		 TPD_DMESG("add bt4xx driver failed\n");
 return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
 TPD_DMESG("MediaTek bt4xx touch panel driver exit\n");
 //input_unregister_device(tpd->dev);
 tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


