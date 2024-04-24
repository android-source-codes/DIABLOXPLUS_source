#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
//#define SOC_BY_HW_FG
#define SOC_BY_SW_FG

//#define CONFIG_DIS_CHECK_BATTERY
//#define FIXED_TBAT_25

/* ADC Channel Number */
#define CUST_TABT_NUMBER 17
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5

/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	0

/* Qmax for battery  */
#define Q_MAX_POS_50	2490
#define Q_MAX_POS_25	2468
#define Q_MAX_POS_0		2310
#define Q_MAX_NEG_10	1858

#define Q_MAX_POS_50_H_CURRENT	2472
#define Q_MAX_POS_25_H_CURRENT	2430
#define Q_MAX_POS_0_H_CURRENT	1837
#define Q_MAX_NEG_10_H_CURRENT	1030
//JRD BSP START
//--- HX BATTERY---
#define HX_MAX_POS_50	2583.72
#define HX_MAX_POS_25	2565.32
#define HX_MAX_POS_0		1521.77
#define HX_MAX_NEG_10	1856.53

#define HX_MAX_POS_50_H_CURRENT	2565.06
#define HX_MAX_POS_25_H_CURRENT	2539.3
#define HX_MAX_POS_0_H_CURRENT	1463.41
#define HX_MAX_NEG_10_H_CURRENT	1805.99
//--- JN BATTERY---
#define JN_MAX_POS_50	2602
#define JN_MAX_POS_25	2595
#define JN_MAX_POS_0		2476
#define JN_MAX_NEG_10	2270

#define JN_MAX_POS_50_H_CURRENT	2580
#define JN_MAX_POS_25_H_CURRENT	2547
#define JN_MAX_POS_0_H_CURRENT	2246
#define JN_MAX_NEG_10_H_CURRENT	1935
//---SCUD BATTERY
#define SCUD_MAX_POS_50	2561
#define SCUD_MAX_POS_25	2546
#define SCUD_MAX_POS_0		2320
#define SCUD_MAX_NEG_10	1787

#define SCUD_MAX_POS_50_H_CURRENT	2545
#define SCUD_MAX_POS_25_H_CURRENT	2506
#define SCUD_MAX_POS_0_H_CURRENT	 1928
#define SCUD_MAX_NEG_10_H_CURRENT	854
// --BYD BATTERY
#define BYD_MAX_POS_50	2508
#define BYD_MAX_POS_25	2491
#define BYD_MAX_POS_0		2375
#define BYD_MAX_NEG_10	2136

#define BYD_MAX_POS_50_H_CURRENT	2494
#define BYD_MAX_POS_25_H_CURRENT	2461
#define BYD_MAX_POS_0_H_CURRENT	 2194
#define BYD_MAX_NEG_10_H_CURRENT	1666
//JRD BSP END

/* Discharge Percentage */
#define OAM_D5		 0		//  1 : D5,   0: D2


/* battery meter parameter */
#define CUST_TRACKING_POINT  14
#define CUST_R_SENSE         68
#define CUST_HW_CC           0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#define CAR_TUNE_VALUE		94 //1.00


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			0 // mOhm, base is 20

#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	40
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30

/* Disable Battery check for HQA */
#ifdef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP		3600		//3.6V
#define VBAT_LOW_POWER_WAKEUP		3500		//3.5v
#define NORMAL_WAKEUP_PERIOD		5400 		//90 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD		300		//5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s

#endif	//#ifndef _CUST_BATTERY_METER_H
