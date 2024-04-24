
#include <utils/Log.h>
#include <utils/Errors.h>
#include <fcntl.h>
#include <math.h>

#include "MediaTypes.h"

//#include "lens_custom_cfg.h"
//#include "msdk_lens_exp.h"
#include "camera_custom_lens.h"
//#include "lens.h"
//nclude "image_sensor.h"
#include "kd_imgsensor.h"

extern PFUNC_GETLENSDEFAULT pDummy_getDefaultData;

#if defined(SENSORDRIVE)
extern PFUNC_GETLENSDEFAULT pSensorDrive_getDefaultData;
#endif

#if defined(FM50AF)
extern PFUNC_GETLENSDEFAULT pFM50AF_getDefaultData;
#endif

#if defined(OV8825AF)
extern PFUNC_GETLENSDEFAULT pOV8825AF_getDefaultData;
#endif
//added by xiewei for diabloxplus kk  start
#if defined(AK7345AF)
extern PFUNC_GETLENSDEFAULT pAK7345AF_getDefaultData;
#endif
#if defined(DW9718AF)
extern PFUNC_GETLENSDEFAULT pDW9718AF_getDefaultData;
#endif
#if defined(DW9714A)
extern PFUNC_GETLENSDEFAULT pDW9714A_getDefaultData;
#endif
#if defined(DW9714ASUNWIN)
extern PFUNC_GETLENSDEFAULT pDW9714ASUNWIN_getDefaultData;
#endif
//added by xiewei for diabloxplus kk  end

MSDK_LENS_INIT_FUNCTION_STRUCT LensList[MAX_NUM_OF_SUPPORT_LENS] =
{
	{DUMMY_SENSOR_ID, DUMMY_LENS_ID, "Dummy", pDummy_getDefaultData},

#if defined(SENSORDRIVE)
    //	{DUMMY_SENSOR_ID, SENSOR_DRIVE_LENS_ID, "kd_camera_hw", pSensorDrive_getDefaultData},	

    //  for backup lens, need assign correct SensorID
    {OV3640_SENSOR_ID, SENSOR_DRIVE_LENS_ID, "kd_camera_hw", pSensorDrive_getDefaultData},
#endif

//#if defined(OV8825AF)
//		{OV8825_SENSOR_ID, OV8825AF_LENS_ID, "OV8825AF", pOV8825AF_getDefaultData},
//#endif

#if defined(FM50AF)
	{DUMMY_SENSOR_ID, FM50AF_LENS_ID, "FM50AF", pFM50AF_getDefaultData},
#endif

//add by xiewei for diablox+ kk start
#if defined(AK7345AF)
	{IMX135TRULY_SENSOR_ID, AK7345AF_LENS_ID, "AK7345AF", pAK7345AF_getDefaultData},
#endif
#if defined(DW9718AF)
	{OV13850_SENSOR_ID, DW9718AF_LENS_ID, "DW9718AF", pDW9718AF_getDefaultData},
#endif
#if defined(DW9714A)
	{IMX135BLX_SENSOR_ID, DW9714A_LENS_ID, "DW9714A", pDW9714A_getDefaultData},
#endif
#if defined(DW9714ASUNWIN)
	{IMX135SUNWIN_SENSOR_ID, DW9714ASUNWIN_LENS_ID, "DW9714ASUNWIN", pDW9714ASUNWIN_getDefaultData},
#endif
//add by xiewei for diablox+ kk end

    //  for new added lens, need assign correct SensorID

};

UINT32 GetLensInitFuncList(PMSDK_LENS_INIT_FUNCTION_STRUCT pLensList)
{
    memcpy(pLensList, &LensList[0], sizeof(MSDK_LENS_INIT_FUNCTION_STRUCT)* MAX_NUM_OF_SUPPORT_LENS);
    return MHAL_NO_ERROR;
} // GetLensInitFuncList()

