/* drivers/hwmon/mt6516/amit/tmd2772.c - TMD2772 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
//#include <mach/eint.h>


#define POWER_NONE_MACRO MT65XX_POWER_NONE
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <tmd2772_cust.h>
#include "tmd2772.h"
/******************************************************************************
 * configuration
 *******************************************************************************/
/*----------------------------------------------------------------------------*/

#define TMD2772_DEV_NAME     "TMD2772"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define JOHN_DEBUG  1  //use for debug log system..
#define MAX_THRESHOLD 1024

#define JOHN_ADD_ATTR 1 //use for add the sys debug system.

#define JOHN_CAL 1 // use for add the calibration to file system.

#define DO_CALIBARTION 1 //use for power on calibration system.


#if JOHN_DEBUG

#define JOHN_TAG "john_debug"
#define JOHN_LOG(fmt, args...)    printk(KERN_ERR  JOHN_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#endif
/******************************************************************************
 * extern functions
 *******************************************************************************/
/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/
//extern void mt_eint_unmask(unsigned int line);
//extern void mt_eint_mask(unsigned int line);
//extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
//extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
//extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
//extern void mt_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt_eint_set_sens(kal_uint8 eintno, kal_bool sens);
//extern void mt_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
//                kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
//                kal_bool auto_umask);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow,
		void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#if defined(MTK_AUTO_DETECT_ALSPS)
static int tmd2772_local_init(void);
static int tmd2772_remove(struct platform_device *pdev);

static struct sensor_init_info tmd2772_init_info = {
		.name = "tmd2772",
		.init = tmd2772_local_init,
		.uninit = tmd2772_remove,
	
};
static int tmd2772_init_flag = 0;
extern int hwmsen_alsps_add(struct sensor_init_info* obj);

#endif//#if defined(MTK_AUTO_DETECT_ALSPS)

/*----------------------------------------------------------------------------*/
static struct i2c_client *tmd2772_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id tmd2772_i2c_id[] = {{TMD2772_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_TMD2772={ I2C_BOARD_INFO("TMD2772", (0X72>>1))};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short tmd2772_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const tmd2772_forces[] = { tmd2772_force, NULL };
//static struct i2c_client_address_data tmd2772_addr_data = { .forces = tmd2772_forces,};
/*----------------------------------------------------------------------------*/

#if JOHN_CAL
static int32_t tmd_get_ps_cali_thd(u16 ps_thd[]);
static void tmd_set_ps_thd_to_file(u16 ps_thd_data[]);
static int32_t tmd_set_ps_cali_file(u16 * w_buf, int8_t w_buf_size);
static int32_t tmd_get_ps_cali_file(u16 * r_buf, int8_t buf_size);
static int32_t tmd_limit_thd_range(int16_t ps_thd_data[]);
static void  tmd_set_ps_thd_to_driver(int16_t ps_thd_data[]);
static int32_t tmd_set_ps_cali(uint8_t force_cali) ;
#endif


static void john_debug_log_function(void);
static int32_t tmd_get_several_ps_data(u16 ps_stat_data[]);
static int32_t tmd_get_data(uint8_t data);

static int tmd2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tmd2772_i2c_remove(struct i2c_client *client);
static int tmd2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int tmd2772_i2c_resume(struct i2c_client *client);
static void tmd2772_ps_calibrate(struct i2c_client *client, int count);


static struct tmd2772_priv *g_tmd2772_ptr = NULL;

struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
} ;



//static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct tmd2772_i2c_addr {    /*define a series of i2c slave address*/
	u8  write_addr;
	u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct tmd2772_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct  eint_work;

	/*i2c address group*/
	struct tmd2772_i2c_addr  addr;

	/*misc*/
	u16		    als_modulus;
	atomic_t    i2c_retry;
	atomic_t    als_suspend;
	atomic_t    als_debounce;   /*debounce time after enabling als*/
	atomic_t    als_deb_on;     /*indicates if the debounce is on*/
	atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
	atomic_t    ps_mask;        /*mask ps: always return far away*/
	atomic_t    ps_debounce;    /*debounce time after enabling ps*/
	atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
	atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
	atomic_t    ps_suspend;


	/*data*/
	u16         als;
	u16          ps;
	u8          _align;
	u16         als_level_num;
	u16         als_value_num;
	u32         als_level[C_CUST_ALS_LEVEL-1];
	u32         als_value[C_CUST_ALS_LEVEL];

	atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
	ulong       enable;         /*enable mask*/
	ulong       pending_intr;   /*pending interrupt*/
#if defined(JOHN_CAL)

	atomic_t ps_cali_done; /*Whether calibration*/
	u8 first_boot;                  /*Whether true power on*/
	u8 cali_file_exist;	    /* Whether calibration file exist */
	u16 ps_CTK_val;
#endif
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver tmd2772_i2c_driver = {
	.probe      = tmd2772_i2c_probe,
	.remove     = tmd2772_i2c_remove,
	//	.detect     = tmd2772_i2c_c,
	.suspend    = tmd2772_i2c_suspend,
	.resume     = tmd2772_i2c_resume,
	.id_table   = tmd2772_i2c_id,
	//	.address_data = &tmd2772_addr_data,
	.driver = {
		//		.owner          = THIS_MODULE,
		.name           = TMD2772_DEV_NAME,
	},
};

static struct tmd2772_priv *tmd2772_obj = NULL;
#if !defined(MTK_AUTO_DETECT_ALSPS)
static struct platform_driver tmd2772_alsps_driver;
#endif
static u16 cal_params[4] = {0};//max, mean, high, low;
static u16  cal_data[20]={0}; //cal data 20 times;
/*----------------------------------------------------------------------------*/
#if JOHN_CAL

#define TMD_CALI_FILE_SIZE 10
#define TMD_CALI_FILE "/mobile_info/TMDpscali.conf"
#define TMD_HIGH_THD				0
#define TMD_LOW_THD				1
#define TMD_THR_AVE                                2
#define TMD_THR_MAX                               3

#define TMD_CT_OFFSET			70
#define TMD_DEFAULT_CTK			100
#define TMD_THD_H_ABOVE_CT 30
#define TMD_THD_L_ABOVE_CT 15
#define TMD_THD_H_MAX			250 // can modify
#define TMD_THD_H_MIN			35
#define TMD_THD_L_MAX			235 // can modify
#define TMD_THD_L_MIN			20

#define TMD_DATA_MAX 0
#define TMD_DATA_MIN 1
#define TMD_DATA_AVE 3
#define TMD_CALI_VER0     20  //Calibration bits1
#define TMD_CALI_VER1	 12  //Calibration bits2
#define TMD_CALI_VER2	 12  //Calibration bits3
#define TMD_CALI_VER3	 10  //Calibration bits4

#define TMD_CALI_FILE_SIZE 10
#define TMD_PS_CAL_TIMES   10
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#endif
int tmd2772_get_addr(struct alsps_hw *hw, struct tmd2772_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2772_power(struct alsps_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "TMD2772"))
			{
				APS_ERR("power on fails!!\n");
			}
			john_debug_log_function();
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "TMD2772"))
			{
				APS_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static long tmd2772_enable_als(struct i2c_client *client, int enable)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	long res = 0;
	//u8 buffer[1];
	//u8 reg_value[1];
	uint32_t testbit_PS;


	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

#if 1
	/*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
	testbit_PS = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
	if(enable)
	{
		if(testbit_PS){
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		else{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x2B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}

		}
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		APS_DBG("tmd2772 power on\n");
	}
	else
	{
		if(testbit_PS){
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		else{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x00;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
		atomic_set(&obj->als_deb_on, 0);
		APS_DBG("tmd2772 power off\n");
	}
#endif

	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_enable_als fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static long tmd2772_enable_ps(struct i2c_client *client, int enable)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	long res = 0;

	uint32_t testbit_ALS;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

#if 1
	/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
	testbit_ALS = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
	printk("john_testbit_ALS=%d\n",testbit_ALS);
	if(enable)
	{
		if(testbit_ALS){
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x0F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*debug code for reading register value*/
#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			printk("john:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
#endif
		}else{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x0D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		/*debug code for reading register value*/
#if 0
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		printk("john:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
#endif
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2772 power on\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			JOHN_LOG();
			databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}

			databuf[0] = TMD2772_CMM_Persistence;
			databuf[1] = 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			if(testbit_ALS){
				databuf[0] = TMD2772_CMM_ENABLE;
				databuf[1] = 0x2F;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
				}
				/*debug code for reading register value*/
#if 0
				res = i2c_master_recv(client, reg_value, 0x1);
				if(res <= 0)
				{
					goto EXIT_ERR;
				}
				printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
#endif
			}else{
				databuf[0] = TMD2772_CMM_ENABLE;
				databuf[1] = 0x2D;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			/*debug code for reading register value*/
#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
#endif

			mt_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}
	else
	{
		/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
		if(testbit_ALS){
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x2B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}else{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x00;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2772 power off\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
#endif
	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int tmd2772_check_and_clear_intr(struct i2c_client *client)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
	//    return 0;

	buffer[0] = TMD2772_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;
	}

	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int tmd2772_check_intr(struct i2c_client *client)
{
	//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
	//    return 0;

	buffer[0] = TMD2772_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_intr fail\n");
	return 1;
}

static int tmd2772_clear_intr(struct i2c_client *client)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];

#if 0
	if((1 == intp) && (0 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
	}
	else if((0 == intp) && (1 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
	}
	else
#endif
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	}
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}


/*-----------------------------------------------------------------------------*/
void tmd2772_eint_func(void)
{
	//printk("john_tmd2772_eint_func start\n");
	struct tmd2772_priv *obj = g_tmd2772_ptr;
	if(!obj)
	{
		return;
	}

	schedule_work(&obj->eint_work);
	//printk("john_tmd2772_eint_func_end\n");
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int tmd2772_setup_eint(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);

	g_tmd2772_ptr = obj;

	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	//	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	//	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	//	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	//	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	//	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	//	mt_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	//	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	//	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, tmd2772_eint_func, 0);
	//
	//	mt_eint_unmask(CUST_EINT_ALS_NUM);

#if (defined MT6592)
	//mt_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, tmd2772_eint_func, 0);
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/

#ifdef DO_CALIBARTION
static int tmd2772_init_client_for_cali(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_ATIME;
	databuf[1] = 0xFF;//0xEE
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_PTIME;
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_WTIME;
	databuf[1] = 0xFF;//0xFF
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_CONFIG;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_PPCOUNT;
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;//0x02
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_CONTROL;
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;//0x22
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x0F;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	return TMD2772_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
#endif

static int tmd2772_init_client(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_ATIME;
	databuf[1] = 0xF6;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_PTIME;
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_WTIME;
	databuf[1] = 0xFC;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{

		databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
		databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
		databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}

		databuf[0] = TMD2772_CMM_Persistence;
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}

	}

	databuf[0] = TMD2772_CMM_CONFIG;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	/*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = TMD2772_CMM_PPCOUNT;
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = TMD2772_CMM_CONTROL;
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#if 0
	databuf[0] = TMD2772_CMM_OFFSET;
	databuf[1] = 0x01;

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#endif
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if((res = tmd2772_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if((res = tmd2772_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}

	return TMD2772_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/******************************************************************************
 * Function Configuration
 ******************************************************************************/
int tmd2772_read_als(struct i2c_client *client, u16 *data)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u16 c0_value, c1_value;
	u32 c0_nf, c1_nf;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 atio;
	//	u16 als_value;
	int res = 0;
	//	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	/*debug tag for yucong*/
#if 0
	buffer[0]=TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
#endif
	//get adc channel 0 value
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_L = 0x%x\n", als_value_low[0]);

	buffer[0]=TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_H = 0x%x\n", als_value_high[0]);
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	c0_nf = obj->als_modulus*c0_value;
	//APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

	//get adc channel 1 value
	buffer[0]=TMD2772_CMM_C1DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_L = 0x%x\n", als_value_low[0]);

	buffer[0]=TMD2772_CMM_C1DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_H = 0x%x\n", als_value_high[0]);

	c1_value = als_value_low[0] | (als_value_high[0]<<8);
	c1_nf = obj->als_modulus*c1_value;
	//APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);

	if((c0_value > c1_value) &&(c0_value < 50000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;

		//APS_DBG("atio = %d\n", atio);
		if(atio<30)
		{
			*data = (13*c0_nf - 24*c1_nf)/10000;
		}
		else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
		{
			*data = (16*c0_nf - 35*c1_nf)/10000;
		}
		else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
		{
			*data = (9*c0_nf - 17*c1_nf)/10000;
		}
		else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
		{
			*data = (6*c0_nf - 10*c1_nf)/10000;
		}
		else
			*data = 0;
		/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
	}
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else if(c0_value == 0)
	{
		*data = 0;
	}
	else
	{
		APS_DBG("als_value is invalid!!\n");
		return -1;
	}
	APS_DBG("als_value_lux = %d\n", *data);
	//printk("yucong: als_value_lux = %d\n", *data);
	return 0;



EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
int tmd2772_read_als_ch0(struct i2c_client *client, u16 *data)
{
	//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u16 c0_value;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	//get adc channel 0 value
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]=TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	*data = c0_value;
	return 0;



EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int tmd2772_get_als_value(struct tmd2772_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}

	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}

		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	}
	else
	{
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
long tmd2772_read_ps(struct i2c_client *client, u16 *data)
{
	//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	//u16 ps_value;
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=TMD2772_CMM_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]=TMD2772_CMM_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
	APS_DBG("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static void john_debug_log_function(void)
{

	u8 i;

	printk("john_prox_threshold_hi =     %d\n",cal_params[0]);
	printk("john_prox_threshold_lo =     %d\n",cal_params[1]);
	printk("john_prox_mean= %d\n",cal_params[2]);
	printk("john_prox_max = %d\n",cal_params[3]);


	for(i=0;i<20;i++)
	{
		printk("john_cal_data[%d]=%d\n",i,cal_data[i]);
	}



}
static int tmd2772_get_ps_value(struct tmd2772_priv *obj, u16 ps)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int val;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;
	//	 tmd_set_ps_thd_to_file(cal_params);
	john_debug_log_function();
	//mdelay(160);
	//tmd2772_read_ps(obj->client,temp_ps);
	if((ps  > atomic_read(&obj->ps_thd_val_high)))
	{
		val = 0;  /*close*/
		val_temp = 0;
		intr_flag_value = 1;
	}
	else if((ps  < atomic_read(&obj->ps_thd_val_low)))
	{
		val = 1;  /*far away*/
		val_temp = 1;
		intr_flag_value = 0;
	}
	else
		val = val_temp;


	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}

		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		return val;
	}
	else
	{
		return -1;
	}

}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void tmd2772_eint_work(struct work_struct *work)
{
	struct tmd2772_priv *obj = (struct tmd2772_priv *)container_of(work, struct tmd2772_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	//	u8 buffer[1];
	//	u8 reg_value[1];
	u8 databuf[2];
	int res = 0;
	//printk("john_eint_work start\n");
	if((err = tmd2772_check_intr(obj->client)))
	{
		APS_ERR("tmd2772_eint_work check intrs: %d\n", err);
	}
	else
	{
#if  0//john add use for get-data stable start
		int i;
		int ps_tmp[3];
		int ps_sum=0;
		int ps_ave=0;

		//get raw data
		for(i=0;i<3;i++)
		{
			tmd2772_read_ps(obj->client, &obj->ps);
			ps_tmp[i]=obj->ps;
#ifdef JOHN_DEBUG
			printk("john_ps_tmp[%d]=%d\n ",i,ps_tmp[i]);
#endif
			ps_sum+=ps_tmp[i];
			mdelay(60);
		}
		ps_ave=ps_sum/3;
#ifdef JOHN_DEBUG
		printk("john_tmd2772_ps_operate read ps valuses ps_sum=%d,ps_ave=%d\n",ps_sum,ps_ave);
#endif
		obj->ps=ps_ave;
#endif  //john add use for get-dat stable end.
		//get raw data
		tmd2772_read_ps(obj->client, &obj->ps);
		//mdelay(160);
		tmd2772_read_als_ch0(obj->client, &obj->als);
		APS_DBG("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		//printk("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = tmd2772_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		/*singal interrupt function add*/
#if 1
		if(intr_flag_value){
			//	printk("john_eint_work ps>ps_thd_val_high\n");
			//printk("yucong interrupt value ps will < 750");
			databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
			databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
			databuf[1] = (u8)(0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
			databuf[1] = (u8)((0xFF00) >> 8);;
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
		}
		else{
			//	printk("john_eint_work ps<ps_thd_val_high\n");
			//printk("yucong interrupt value ps will > 900");
			databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
			databuf[1] = (u8)(0 & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
			databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
			databuf[1] = (u8)((0 & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
			{
				return;
			}
		}
#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
			APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	tmd2772_clear_intr(obj->client);
	mt_eint_unmask(CUST_EINT_ALS_NUM);
}




/******************************************************************************
 * Function Configuration
 ******************************************************************************/
static int tmd2772_open(struct inode *inode, struct file *file)
{
	file->private_data = tmd2772_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tmd2772_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long tmd2772_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	//struct PS_CALI_DATA_STRUCT ps_cali_temp;
	//struct TEST_PS_CALI_DATA_STRUCT ps_cali_test;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = tmd2772_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err);
					goto err_out;
				}

				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = tmd2772_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err);
					goto err_out;
				}

				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			if((err = tmd2772_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			//	printk("john_tmd2772_unlocked_ioctl ALSPS_GET_PS_DATA satrt!\n ");
			dat = tmd2772_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			if((err = tmd2772_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}

			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = tmd2772_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err);
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = tmd2772_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err);
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA:
			if((err = tmd2772_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = tmd2772_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_RAW_DATA:
			if((err = tmd2772_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

			/*		case ALSPS_SET_PS_CALI:
					dat = (void __user*)arg;
					if(dat == NULL)
					{
					APS_LOG("dat == NULL\n");
					err = -EINVAL;
					break;
					}
					if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
					{
					APS_LOG("copy_from_user\n");
					err = -EFAULT;
					break;
					}
					tmd2772_WriteCalibration(&ps_cali_temp);
					APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
					break;
					case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
					tmd2772_init_client_for_cali(obj->client);
					err = tmd2772_read_data_for_cali(obj->client,&ps_cali_temp);
					if(err)
					{
					goto err_out;
					}
					tmd2772_init_client(obj->client);
			// tmd2772_enable_ps(obj->client, 1);
			tmd2772_enable(obj->client, 0);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
			err = -EFAULT;
			goto err_out;
			}
			break;
			 */

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

err_out:
	return err;
}
/*----------------------------------------------------------------------------*/

/*john add use for add attr for debug start*/
#ifdef JOHN_ADD_ATTR
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t tmd2772_show_als(struct device_driver *ddri, char *buf)
{
	int res;

	if(!g_tmd2772_ptr)
	{
		APS_ERR("g_tmd2772_ptr is null!!\n");
		return 0;
	}
	if((res = tmd2772_read_als_ch0(g_tmd2772_ptr->client, &g_tmd2772_ptr->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", g_tmd2772_ptr->als);
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t tmd2772_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!g_tmd2772_ptr)
	{
		APS_ERR("g_tmd2772_ptr is null!!\n");
		return 0;
	}

	if((res = tmd2772_read_ps(g_tmd2772_ptr->client, &g_tmd2772_ptr->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "ps_dec= %d\n", g_tmd2772_ptr->ps);
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t tmd2772_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if(!g_tmd2772_ptr)
	{
		APS_ERR("g_tmd2772_ptr is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d %x %x)\n",
			atomic_read(&g_tmd2772_ptr->i2c_retry), atomic_read(&g_tmd2772_ptr->als_debounce),
			atomic_read(&g_tmd2772_ptr->ps_mask), atomic_read(&g_tmd2772_ptr->ps_debounce),
			atomic_read(&g_tmd2772_ptr->ps_thd_val_high),atomic_read(&g_tmd2772_ptr->ps_thd_val_low),
			TMD2772_CMM_PPCOUNT_VALUE, TMD2772_CMM_CONTROL_VALUE);
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t tmd2772_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int ppcount,cmmctl;
	if(!g_tmd2772_ptr)
	{
		APS_ERR("g_tmd2772_ptr is null!!\n");
		return 0;
	}
	if(2 ==sscanf(buf,"%x %x",&ppcount,&cmmctl))
	{
		TMD2772_CMM_PPCOUNT_VALUE=ppcount;
		TMD2772_CMM_CONTROL_VALUE=cmmctl;
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;
}

static ssize_t tmd2772_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if(!g_tmd2772_ptr)
	{
		APS_ERR("g_tmd2772_ptr is null!!\n");
		return 0;
	}

	if(g_tmd2772_ptr->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST:i2c_num= %d\nppcount=%x\ncmm=%x\nhigh=%d\nlow=%d\n",
				g_tmd2772_ptr->hw->i2c_num, TMD2772_CMM_PPCOUNT_VALUE,  TMD2772_CMM_CONTROL_VALUE,
				g_tmd2772_ptr->hw->ps_threshold_high,g_tmd2772_ptr->hw->ps_threshold_low);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02lX %02lX\n",
			atomic_read(&g_tmd2772_ptr->als_cmd_val), atomic_read(&g_tmd2772_ptr->ps_cmd_val),
			g_tmd2772_ptr->enable, g_tmd2772_ptr->pending_intr);


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&g_tmd2772_ptr->als_suspend), atomic_read(&g_tmd2772_ptr->ps_suspend));

	return len;
}

static ssize_t tmd2772_show_pscali(struct device_driver *ddri, char *buf)
{
	u16 thd_cali[2]={0};

	int32_t ret;

	if(!g_tmd2772_ptr)
	{
		APS_ERR("g_tmd2772_ptr is null!!\n");
		return 0;
	}

	tmd_get_ps_cali_thd(thd_cali);

	APS_LOG("%s : read PS calibration data from file, thd_cali[TMD_LOW_THD]=%d, thd_cali[TMD_HIGH_THD]=%d\n",
			__func__, thd_cali[TMD_LOW_THD], thd_cali[TMD_HIGH_THD]);

	return snprintf(buf, PAGE_SIZE, "%d %d \n", thd_cali[TMD_HIGH_THD],thd_cali[TMD_LOW_THD]);

}

static ssize_t tmd2772_store_pscali(struct device_driver *ddri, const char *buf, size_t count)
{
	int value;
	int32_t ret;

	if(!g_tmd2772_ptr)
	{
		APS_ERR("tmd2772_priv   g_tmd2772_ptr is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}


	if(value ==1)
	{
		tmd_set_ps_cali(1);
	}
	else if(value ==2)
	{
		tmd_get_data(1);
	}


	return count;
}
static ssize_t tmd2772_factory_cali(struct device_driver *ddri,char *buf)
{
	int value;
	int32_t ret;
	if(!g_tmd2772_ptr)
	{
		APS_ERR("tmd2772_priv   g_tmd2772_ptr is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &value))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	if(value == 1)

		tmd_set_ps_cali(2);


}

static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, tmd2772_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, tmd2772_show_ps,    NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, tmd2772_show_config,tmd2772_store_config);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, tmd2772_show_status,  NULL);
#if JOHN_CAL
static DRIVER_ATTR(pscali,  S_IWUSR | S_IRUGO, tmd2772_show_pscali,tmd2772_store_pscali);
#endif
static DRIVER_ATTR(psfac,	S_IWUSR | S_IRUGO,tmd2772_factory_cali,NULL);
static struct driver_attribute *tmd2772_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_config,
	&driver_attr_status,
	&driver_attr_pscali,
	&driver_attr_psfac,
};

static int tmd2772_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(tmd2772_attr_list)/sizeof(tmd2772_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, tmd2772_attr_list[idx])))
		{
			APS_ERR("driver_create_file (%s) = %d\n", tmd2772_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int tmd2772_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(tmd2772_attr_list)/sizeof(tmd2772_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, tmd2772_attr_list[idx]);
	}

	return err;
}

#endif


/*john add use for add attr for debug end*/

static struct file_operations tmd2772_fops = {
	.owner = THIS_MODULE,
	.open = tmd2772_open,
	.release = tmd2772_release,
	.unlocked_ioctl = tmd2772_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tmd2772_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &tmd2772_fops,
};
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	//int err;
	APS_FUN();
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}

		atomic_set(&obj->als_suspend, 1);
		if(err = tmd2772_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = tmd2772_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}

		tmd2772_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_resume(struct i2c_client *client)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	//int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	tmd2772_power(obj->hw, 1);
	if(err = tmd2772_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2772_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = tmd2772_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);
		}
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2772_early_suspend(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct tmd2772_priv *obj = container_of(h, struct tmd2772_priv, early_drv);
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

#if 1
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = tmd2772_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err);
		}
	}
#endif
}
/*----------------------------------------------------------------------------*/
static void tmd2772_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct tmd2772_priv *obj = container_of(h, struct tmd2772_priv, early_drv);
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

#if 1
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = tmd2772_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);

		}
	}
#endif
}

int tmd2772_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2772_priv *obj = (struct tmd2772_priv *)self;

	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				//tmd2772_ps_calibrate(obj->client, 3);
				value = *(int *)buff_in;
				if(value)
				{
					tmd2772_ps_calibrate(obj->client, 3);
					if((err = tmd2772_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					atomic_set(&obj->ps_thd_val_high, MAX_THRESHOLD);
					atomic_set(&obj->ps_thd_val_low,  MAX_THRESHOLD);

					if((err = tmd2772_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;

				tmd2772_read_ps(obj->client, &obj->ps); //john note:if not use ave data ,please open this line.

				//mdelay(160);
				tmd2772_read_als_ch0(obj->client, &obj->als);
				sensor_data->values[0] = tmd2772_get_ps_value(obj, obj->ps);
#ifdef JOHN_DEBUG
				APS_ERR("john_tmd2772_ps_operate ps_data=%d!\n",obj->ps);
#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

static int temp_als = 0;
int tmd2772_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2772_priv *obj = (struct tmd2772_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if((err = tmd2772_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = tmd2772_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err);
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}

			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing know issue*/
#if 1
				tmd2772_read_als(obj->client, &obj->als);
				if(obj->als == 0)
				{
					sensor_data->values[0] = temp_als;
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
						tmd2772_read_als(obj->client, &obj->als);
						b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = tmd2772_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TMD2772_DEV_NAME);
	return 0;
}

#ifdef DO_CALIBARTION//Added by jrd.john for boot-up calibration
/*----------------------------------------------------------------------------*/
/* when do boot-up calibration,  tmd2772_init_client_for_cali() changed some regs values which
 * were set by tmd2772_init_client() before. So when calibration completes, call this func to restore
 * those regs values.*/
static int tmd2772_init_client_for_cali_restore(struct i2c_client *client)
{
	u8 databuf[2];
	int res = 0;

	databuf[0] = TMD2772_CMM_ATIME;
	databuf[1] = 0xC9;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_WTIME;
	databuf[1] = 0xEE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	return TMD2772_SUCCESS;

EXIT_ERR:
	APS_ERR("JRD.john |cali restore error: %d\n", res);
	return res;
}
static void tmd2772_ps_calibrate(struct i2c_client *client, int count)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int prox_sum = 0, prox_mean = 0, prox_max = 0;
	int prox_threshold_hi = 0, prox_threshold_lo = 0;
	int i, ret = 0;
	u16 prox_data[40];

	tmd2772_init_client_for_cali(obj->client);

	if(count>40)
	{
		APS_ERR("ps calibrate count is too large!");
	}

	for(i = 0; i < count; i++)
	{
		if(i==0)
		{
			msleep(60);
		}
		if(ret = tmd2772_read_ps(client, &prox_data[i]))
		{
			APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i);
			return ret;
		}

		prox_sum += prox_data[i];
		if (prox_data[i] > prox_max)
			prox_max = prox_data[i];

		msleep(60);//160
		cal_data[i]=prox_data[i];
#ifdef JOHN_DEBUG
		printk(KERN_ERR "john tmd2772_ps_calibrate  ps-val = %d XXXXX\n", prox_data[i]);
#endif
	}

	prox_mean = prox_sum / count;

#if 0
	if(prox_mean<40){
		prox_threshold_hi = prox_mean*3;
		prox_threshold_lo = prox_mean*2;
	}
	else if(prox_mean<45){
		prox_threshold_hi = prox_mean*25/10;
		prox_threshold_lo = prox_mean*19/10;
	}
	else if(prox_mean<50){
		prox_threshold_hi = prox_mean*25/10;
		prox_threshold_lo = prox_mean*19/10;
	}
	else if(prox_mean<60){
		prox_threshold_hi = prox_mean*23/10;
		prox_threshold_lo = prox_mean*18/10;
	}
	else if(prox_mean<70){
		prox_threshold_hi = prox_mean*22/10;
		prox_threshold_lo = prox_mean*17/10;
	}
	else if(prox_mean<80){
		prox_threshold_hi = prox_mean*22/10;
		prox_threshold_lo = prox_mean*17/10;
	}
	else if(prox_mean<100){
		prox_threshold_hi = prox_mean*2;
		prox_threshold_lo = prox_mean*18/10;
	}
	else if(prox_mean<150){
		prox_threshold_hi = prox_mean*17/10;
		prox_threshold_lo = prox_mean*15/10;
	}
	else if(prox_mean<200){
		prox_threshold_hi = prox_mean*15/10;
		prox_threshold_lo = prox_mean*13/10;
	}
	else if(prox_mean<300){
		prox_threshold_hi = prox_mean*16/10;
		prox_threshold_lo = prox_mean*14/10;
	}
	else {
		prox_threshold_hi = prox_mean*14/10;
		prox_threshold_lo = prox_mean*13/10;
	}
#endif

	if(prox_mean < 200)
	{
		prox_threshold_hi = prox_mean+200;
		prox_threshold_lo = prox_mean+5;
	}
	else if(prox_mean < 350)
	{
		prox_threshold_hi = prox_mean+350;//
		prox_threshold_lo = prox_mean+75;//
	}
	else
	{
		prox_threshold_hi = prox_mean+350;
		prox_threshold_lo = prox_mean+50;
	}

	//prox_threshold_hi = prox_mean*4;//((((prox_max - prox_mean) * 300) + 50) / 100) + prox_mean;
	//prox_threshold_lo = ((((prox_max - prox_mean) * 140) + 50) / 100) + prox_mean;

	atomic_set(&obj->ps_thd_val_high, prox_threshold_hi);
	atomic_set(&obj->ps_thd_val_low,  prox_threshold_lo);
	/* get smaller value */

	if(atomic_read(&obj->ps_thd_val_low) < ((1000 * 2) / 100))
	{
		atomic_set(&obj->ps_thd_val_low, ((1000 * 5) / 100));
		atomic_set(&obj->ps_thd_val_high, ((1000 * 8) / 100));
	}
	/* panel down */
	if(atomic_read(&obj->ps_thd_val_high) > ((1000 * 70) / 100)  || 0 == prox_mean )
	{
		atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high);
		atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	}
#if 0
	if(prox_mean < 20)
	{
		atomic_set(&obj->ps_thd_val_low, 50);
		atomic_set(&obj->ps_thd_val_high, 80);
		printk("john | ps value extreme low case happen\d");
	}

	/* panel down */
	if(prox_mean > 600 && prox_mean <= 700)
	{
		atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high);
		atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
		printk("john | ps value extreme high case1 happen");
	}
	if(prox_mean > 700 && prox_mean <= 800)
	{
		atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high+100);
		atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low+100);
		printk("john | ps value extreme high case2 happen");
	}
	if(prox_mean > 800 && prox_mean <= 900)
	{
		atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high+200);
		atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low+200);
		printk("john | ps value extreme high case3 happen");
	}

	if(prox_mean > 900)
	{
		atomic_set(&obj->ps_thd_val_high, 1023);
		atomic_set(&obj->ps_thd_val_low,  1023);
		printk("john | ps value extreme high case4 happen");
	}
#endif

#ifdef JOHN_DEBUG
	printk("john tmd2772_ps_calibrate  ps_thd_val_low = %d\n",  atomic_read(&obj->ps_thd_val_low));
	printk("john tmd2772_ps_calibrate  ps_thd_val_high = %d \n", atomic_read(&obj->ps_thd_val_high));
#endif


#ifdef JOHN_DEBUG
	printk(KERN_ERR "john | computed high = %d, low = %d; max: %d, mean: %d\n", prox_threshold_hi, prox_threshold_lo, prox_max, prox_mean);//john tmp debug
#endif


	cal_params[0] = (u16)(atomic_read(&obj->ps_thd_val_high));
	cal_params[1] = (u16)(atomic_read(&obj->ps_thd_val_low));
	cal_params[2] = prox_mean;
	cal_params[3] = prox_max;

	//    tmd2772_init_client_for_cali_restore(obj->client);
}
#endif
#if JOHN_CAL
static int32_t tmd_set_ps_cali(uint8_t force_cali)
{

	tmd_set_ps_thd_to_file(cal_params);

}
static int32_t tmd_get_data(uint8_t data)
{
	u16 calibrate_data[3]={0};
	tmd_get_several_ps_data(calibrate_data);
	JOHN_LOG("john_calibrate_data[0]=ave =%d,[1]=max=%d,[2]=min=%d\n",calibrate_data[0],calibrate_data[1],calibrate_data[2]);


}
static int32_t tmd_get_several_ps_data(u16 ps_stat_data[])
{
	u16 ps_data[TMD_PS_CAL_TIMES];
	u16  ps_sum = 0;
	u16 ps_ave = 0;
	u16 ps_max=0,ps_min=1025;
	int8_t i;
	ssize_t ret;
	u8 databuf[2];

#if 0
	databuf[0] = TMD2772_CMM_PPCOUNT;
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;
	ret = i2c_master_send(g_tmd2772_ptr->client, databuf, 0x02);
	if(ret <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = TMD2772_CMM_CONTROL;
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;
	ret = i2c_master_send(g_tmd2772_ptr->client, databuf, 0x02);
	if(ret <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#endif


#if DO_CALIBARTION//Added by jrd.john for boot-up calibration
	tmd2772_ps_calibrate(g_tmd2772_ptr->client, 40);
#endif

	if((ret = tmd2772_init_client(g_tmd2772_ptr->client)))
	{
		APS_ERR("tmd2772 reinitiate client after change the configure parameter failure\n");
		goto EXIT_ERR;
	}

	for(i = 0; i < TMD_PS_CAL_TIMES; i++)
	{
		if(ret = tmd2772_read_ps(g_tmd2772_ptr->client, &ps_data[i]))
		{
			APS_ERR("tmd2772_read_data fail: %d\n", i);
			return ret;
		}
		JOHN_LOG( "john tmd_get_several_ps_data  ps-val = %d XXXXX\n", ps_data[i]);

		ps_sum += ps_data[i];
		//ps_sum=ps_data[0]+ps_data[1]+ps_data[2]+ps_data[3]+ps_data[4]+ps_data[5]+ps_data[6]+ps_data[7]+ps_data[8]+ps_data[9];

		ps_ave=ps_sum/10;

		if (ps_data[i] > ps_max)
			ps_max = ps_data[i];
		if(ps_data[i] < ps_min)
			ps_min = ps_data[i];
		msleep(160);
	}

	JOHN_LOG("john_ps_sum=%d\n",ps_sum);
	JOHN_LOG("john_ps_ave=%d\n",ps_ave);
	JOHN_LOG("john_ps_ave value = %d,ps_max=%d,ps_min=%d\n",ps_ave,ps_max,ps_min);

	ps_stat_data[0]=ps_ave;
	ps_stat_data[1]=ps_max;
	ps_stat_data[2]=ps_min;

	JOHN_LOG("john_ps_stat_data[0]=%d,ps_stat_data[1]=%d,ps_stat_data[2]=%d\n",ps_stat_data[0],ps_stat_data[1],ps_stat_data[2]);
EXIT_ERR:
	APS_ERR("init dev: %d\n", ret);
	return ret;

}
static void tmd_set_ps_thd_to_file(u16 ps_thd_data[])
{

	u16 w_buf[TMD_CALI_FILE_SIZE] = {0};

	w_buf[0] = TMD_CALI_VER0;
	w_buf[1] = TMD_CALI_VER1;
	w_buf[2] = ps_thd_data[TMD_HIGH_THD];
	w_buf[3] = ps_thd_data[TMD_LOW_THD];
	w_buf[4] = ps_thd_data[TMD_THR_AVE];
	w_buf[5] = ps_thd_data[TMD_THR_MAX];
	w_buf[6] = TMD_CALI_VER2;
	w_buf[7] = TMD_CALI_VER3;

	tmd_set_ps_cali_file(w_buf, sizeof(u16)*TMD_CALI_FILE_SIZE);
}
static int32_t tmd_set_ps_cali_file(u16 * w_buf, int8_t w_buf_size)
{


	struct file  *cali_file;
	u16  r_buf[TMD_CALI_FILE_SIZE] = {0};
	mm_segment_t fs;
	ssize_t ret;
	int8_t i;

	//	struct i2c_client *client = (struct i2c_client*)cali_file->private_data;
	//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	JOHN_LOG();
	cali_file = filp_open(TMD_CALI_FILE, O_CREAT | O_RDWR,0666);
	if(IS_ERR(cali_file))
	{
		APS_ERR("%s: filp_open for write error!\n", __func__);
		return -ENOENT;
	}
	else
	{
		fs = get_fs();
		set_fs(get_ds());
		JOHN_LOG();
		ret = cali_file->f_op->write(cali_file,w_buf,w_buf_size,&cali_file->f_pos);
		if(ret != w_buf_size)
		{
			APS_ERR("%s: write error!\n", __func__);
			filp_close(cali_file,NULL);
			return -EIO;
		}
		cali_file->f_pos=0x00;
		ret = cali_file->f_op->read(cali_file,r_buf,w_buf_size ,&cali_file->f_pos);
		if(ret < 0)
		{
			APS_ERR("%s: read error!\n", __func__);
			filp_close(cali_file,NULL);
			return -EIO;
		}
		JOHN_LOG();
		set_fs(fs);
		for(i=0;i<w_buf_size;i++)
		{
			if(r_buf[i] != w_buf[i])
			{
				APS_ERR("%s: read back error!, r_buf[%d](0x%x) != w_buf[%d](0x%x)\n", __func__, i, r_buf[i], i, w_buf[i]);
				filp_close(cali_file,NULL);
				return -EIO;
			}
		}
		JOHN_LOG();
	}
	filp_close(cali_file,NULL);
	JOHN_LOG();
	tmd2772_obj->cali_file_exist = 1;
	APS_DBG("%s john_successfully\n", __func__);
	return 0;
}
/*get the ps calibration value from file.*/
static int32_t tmd_get_ps_cali_thd(u16 ps_thd[])
{

	JOHN_LOG();
	u16 r_buf[TMD_CALI_FILE_SIZE] = {0};
	int32_t ret;
	uint8_t i;

	tmd2772_obj->cali_file_exist=0;

	if ((ret = tmd_get_ps_cali_file(r_buf, 2*TMD_CALI_FILE_SIZE)) < 0) //get the file threshold value from file to r_buf
	{
		return ret;
	}
	else
	{
		for(i=0;i<TMD_CALI_FILE_SIZE;i++)

		{
			JOHN_LOG("r_buf[%d] = %d\n",i,r_buf[i]);
		}


		if(r_buf[0] == TMD_CALI_VER0 && r_buf[1] == TMD_CALI_VER1&&r_buf[6] == TMD_CALI_VER2 &&r_buf[7]==TMD_CALI_VER3)
		{
			ps_thd[TMD_HIGH_THD] = r_buf[2];
			ps_thd[TMD_LOW_THD] = r_buf[3];

			JOHN_LOG("ps_thd[0]=%d,ps_thd[1]=%d\n",ps_thd[0],ps_thd[1]);

			atomic_set(&tmd2772_obj->ps_cali_done,1);
			tmd2772_obj->cali_file_exist=1;
		}
		else
		{
			APS_ERR("%s: cali version number error! r_buf=0x%x,0x%x,0x%x,0x%x\n", __func__, r_buf[0], r_buf[1], r_buf[6], r_buf[7] );
			return -EINVAL;
		}
	}
	return 0;
}
/*get the ps threshold high and threshold low value etcfrom file.*/

static int32_t tmd_get_ps_cali_file(u16 * r_buf, int8_t buf_size)
{

	JOHN_LOG();
	struct file  *cali_file;
	mm_segment_t fs;
	ssize_t ret;

	cali_file = filp_open(TMD_CALI_FILE, O_RDWR,0);
	JOHN_LOG();
	if(IS_ERR(cali_file))
	{
		APS_ERR("%s: filp_open error!\n", __func__);
		return -ENOENT;
	}
	else
	{
		JOHN_LOG();
		fs = get_fs();
		set_fs(get_ds());
		ret = cali_file->f_op->read(cali_file,r_buf,buf_size,&cali_file->f_pos);
		if(ret < 0)
		{
			APS_ERR("%s: read error, ret=%d\n", __func__, ret);
			filp_close(cali_file,NULL);
			return -EIO;
		}
		set_fs(fs);
	}
	JOHN_LOG();
	filp_close(cali_file,NULL);
	return 0;
}
static int32_t tmd_limit_thd_range(int16_t ps_thd_data[])
{


	int32_t ret = 0;
	if(ps_thd_data[TMD_HIGH_THD] > TMD_THD_H_MAX)
	{
		ps_thd_data[TMD_HIGH_THD] = TMD_THD_H_MAX;
		ret |= 0x01;
	}
	else if(ps_thd_data[TMD_HIGH_THD] < TMD_THD_H_MIN)
	{
		ps_thd_data[TMD_HIGH_THD] = TMD_THD_H_MIN;
		ret |= 0x02;
	}
	if(ps_thd_data[TMD_LOW_THD] > TMD_THD_L_MAX)
	{
		ps_thd_data[TMD_LOW_THD] = TMD_THD_L_MAX;
		ret |= 0x04;
	}
	else if(ps_thd_data[TMD_LOW_THD] < TMD_THD_L_MIN)
	{
		ps_thd_data[TMD_LOW_THD] = TMD_THD_L_MIN;
		ret |= 0x08;
	}
	return ret;

}
static void  tmd_set_ps_thd_to_driver(int16_t ps_thd_data[])
{
	int ret;
	struct file  *cali_file;
	struct i2c_client *client = (struct i2c_client*)cali_file->private_data;
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	atomic_set(&obj->ps_thd_val_high, ps_thd_data[TMD_HIGH_THD] );
	atomic_set(&obj->ps_thd_val_low, ps_thd_data[TMD_LOW_THD] );

}


#endif

/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmd2772_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	tmd2772_obj = obj;

	obj->hw = get_cust_alsps_hw_tmd();
	tmd2772_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	INIT_WORK(&obj->eint_work, tmd2772_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
	//(400)/16*2.72 here is amplify *100 //16
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

#if DO_CALIBARTION//Added by jrd.john for boot-up calibration
	tmd2772_ps_calibrate(client, 20);
#endif

	tmd2772_i2c_client = client;


	if((err = tmd2772_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("tmd2772_init_client() OK!\n");



	if((err = misc_register(&tmd2772_device)))
	{
		APS_ERR("tmd2772_device register failed\n");
		goto exit_misc_device_register_failed;
	}
#ifdef JOHN_ADD_ATTR
#if defined(MTK_AUTO_DETECT_ALSPS)
	if((err = tmd2772_create_attr(&(tmd2772_init_info.platform_diver_addr->driver))))
#else
	if(err = tmd2772_create_attr(&tmd2772_alsps_driver.driver))
#endif
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#endif

#if JOHN_CAL
	/*we can add below code under the IC_i2C_probe.init the variables value. start */
	tmd2772_obj->first_boot =1;
	atomic_set(&obj->ps_cali_done, 0);
	tmd2772_obj->cali_file_exist= 0;
	tmd2772_obj->ps_CTK_val =0;
	/*we can add below code under the IC_i2C_probe. end*/


#endif

	obj_ps.self = tmd2772_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
		//if (1)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = tmd2772_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	obj_als.self = tmd2772_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = tmd2772_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
		obj->early_drv.suspend  = tmd2772_early_suspend,
		obj->early_drv.resume   = tmd2772_late_resume,
		register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);

#if defined(MTK_AUTO_DETECT_ALSPS)
    tmd2772_init_flag = 0;
#endif
	return 0;

exit_create_attr_failed:
	misc_deregister(&tmd2772_device);
exit_misc_device_register_failed:
exit_init_failed:
    i2c_release_client(client);
	kfree(obj);
exit:
	tmd2772_i2c_client = NULL;
	//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);


#if defined(MTK_AUTO_DETECT_ALSPS)
    tmd2772_init_flag = -1;
#endif
	return err;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_remove(struct i2c_client *client)
{
	int err;
#ifdef JOHN_ADD_ATTR
	if(err = tmd2772_delete_attr(&tmd2772_i2c_driver.driver))
	{
		APS_ERR("tmd2772_delete_attr fail: %d\n", err);
	}
#endif
	if((err = misc_deregister(&tmd2772_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);
	}

	tmd2772_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
#if defined(MTK_AUTO_DETECT_ALSPS)
static int tmd2772_local_init(void)
#else
static int tmd2772_probe(struct platform_device *pdev) 
#endif
{
	struct alsps_hw *hw = get_cust_alsps_hw_tmd();

	tmd2772_power(hw, 1);
	//tmd2772_force[0] = hw->i2c_num;
	//tmd2772_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",tmd2772_force[0],tmd2772_force[1]);
	if(i2c_add_driver(&tmd2772_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 

#if defined(MTK_AUTO_DETECT_ALSPS)
    if(tmd2772_init_flag)
    {
        return -1;
    }
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw_tmd();
	APS_FUN();
	tmd2772_power(hw, 0);
	i2c_del_driver(&tmd2772_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
#if !defined(MTK_AUTO_DETECT_ALSPS)
static struct platform_driver tmd2772_alsps_driver = {
	.probe      = tmd2772_probe,
	.remove     = tmd2772_remove,
	.driver     = {
		.name  = "als_ps",
		//		.owner = THIS_MODULE,
	}
};
#endif
/*----------------------------------------------------------------------------*/
static int __init tmd2772_init(void)
{
	APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw_tmd();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_TMD2772, 1);
#if defined(MTK_AUTO_DETECT_ALSPS)    
	hwmsen_alsps_sensor_add(&tmd2772_init_info);
#else
	if(platform_driver_register(&tmd2772_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit tmd2772_exit(void)
{
	APS_FUN();
#if !defined(MTK_AUTO_DETECT_ALSPS)
	platform_driver_unregister(&tmd2772_alsps_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(tmd2772_init);
module_exit(tmd2772_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("tmd2772 driver");
MODULE_LICENSE("GPL");
