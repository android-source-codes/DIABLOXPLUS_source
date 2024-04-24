
ifeq ($(TARGET_BUILD_MMITEST),true)
EXTRA_CFLAGS += -DCHARGING_META
EXTRA_CFLAGS += -DCHARGER_VOLTAGE
endif

ifeq ($(PROJECT),diabloxplus_kk)
EXTRA_CFLAGS += -DDIABLOXPLUS_CHARGING_TEMPERATURE_PLOICY
#JRD BSP START
EXTRA_CFLAGS += -DDIABLOXPLUS_BATT_ID_SUPPORT
#JRD BSP END
EXTRA_CFLAGS += -DJRD_FUEL_GAUGE_OPTIME
endif


ifeq ($(PROJECT),eos_3g)
EXTRA_CFLAGS += -DBATT_ID_CHECK_SUPPORT
#JRD BSP START
EXTRA_CFLAGS += -DEOS_3G_CHARGING_TEMPERATURE_PLOICY
#JRD BSP END
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter.o battery_common.o

ifeq ($(MTK_FAN5402_SUPPORT),yes)
    obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(MTK_FAN5405_SUPPORT),yes)
        obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o 
else
    ifeq ($(MTK_SWCHR_SUPPORT),yes)
        obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
    else
        ifeq ($(MTK_BQ24158_SUPPORT),yes)
            obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o 
        else
            ifeq ($(MTK_BQ24196_SUPPORT),yes)
                obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
            else
                ifeq ($(MTK_BQ24296_SUPPORT),yes)
                    obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                else
                    ifeq ($(MTK_NCP1851_SUPPORT),yes)
                        obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                    else
                        ifeq ($(MTK_NCP1854_SUPPORT),yes)
                            obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                        else
                            ifeq ($(MTK_BQ24160_SUPPORT),yes)
                                obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                            else
                                obj-$(CONFIG_MTK_SMART_BATTERY) += linear_charging.o
                            endif        
                        endif
                   endif
                endif
            endif
        endif  
    endif    
endif
