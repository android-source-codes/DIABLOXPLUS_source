




#ifndef _CFG_AUDIO_JRD_CUST_H
#define _CFG_AUDIO_JRD_CUST_H

#include "CFG_Audio_Default_Cust.h"
#include "../cfgfileinc/CFG_AUDIO_File.h"
#include "../inc/sph_coeff_record_mode_default.h"
#include "../inc/sph_coeff_dmnr_default.h"
#include "../inc/audio_hd_record_custom.h"
#include "../inc/audio_acf_default.h"
#include "../inc/audio_hcf_default.h"
#include "../inc/audio_effect_default.h"
#include "../inc/audio_gain_table_default.h"
#include "../inc/audio_ver1_volume_custom_default.h"
#include "../inc/audio_hd_record_48k_custom.h"
#include "../inc/voice_recognition_custom.h"
#include "../inc/audio_audenh_control_option.h"



#define DEFAULT_SPEECH_NORMAL_MODE_PARA2 \
   96,   253,  16388,     SPEECH_MODE_PARA03,   57351,     799,   400,     64, \
   SPEECH_MODE_PARA08,  4325,      611,       0,   20488,      0|SPEECH_MODE_PARA13,     0|SPEECH_MODE_PARA14,  8192

AUDIO_CUSTOM_PARAM_STRUCT speech_custom_default2 =
{
    /* INT8 volume[MAX_VOL_CATE][MAX_VOL_TYPE] */
    /* Normal volume: TON, SPK, MIC, FMR, SPH, SID, MED */
    GAIN_NOR_TON_VOL, GAIN_NOR_KEY_VOL, GAIN_NOR_MIC_VOL, GAIN_NOR_FMR_VOL, GAIN_NOR_SPH_VOL, GAIN_NOR_SID_VOL, GAIN_NOR_MED_VOL
    ,
    /* Headset volume: TON, SPK, MIC, FMR, SPH, SID, MED */
    GAIN_HED_TON_VOL, GAIN_HED_KEY_VOL, GAIN_HED_MIC_VOL, GAIN_HED_FMR_VOL, GAIN_HED_SPH_VOL, GAIN_HED_SID_VOL, GAIN_HED_MED_VOL
    ,
    /* Handfree volume: TON, SPK, MIC, FMR, SPH, SID, MED */
    GAIN_HND_TON_VOL, GAIN_HND_KEY_VOL, GAIN_HND_MIC_VOL, GAIN_HND_FMR_VOL, GAIN_HND_SPH_VOL, GAIN_HND_SID_VOL, GAIN_HND_MED_VOL
    ,
    /* UINT16 speech_common_para[12] */
    DEFAULT_SPEECH_COMMON_PARA
    ,
    /* UINT16 speech_mode_para[8][16] */
    DEFAULT_SPEECH_NORMAL_MODE_PARA2,
    DEFAULT_SPEECH_EARPHONE_MODE_PARA,
    DEFAULT_SPEECH_LOUDSPK_MODE_PARA,
    DEFAULT_SPEECH_BT_EARPHONE_MODE_PARA,
    DEFAULT_SPEECH_BT_CORDLESS_MODE_PARA,
    DEFAULT_SPEECH_CARKIT_MODE_PARA,
    DEFAULT_SPEECH_AUX1_MODE_PARA,
    DEFAULT_SPEECH_AUX2_MODE_PARA
    ,
    /* UINT16 speech_volume_para[4] */
    DEFAULT_SPEECH_VOL_PARA
    ,
    /* UINT16 debug_info[16] */
    DEFAULT_AUDIO_DEBUG_INFO
    ,
    /* INT16 sph_in_fir[6][45], sph_out_fir */
    SPEECH_INPUT_FIR_COEFF,
    SPEECH_OUTPUT_FIR_COEFF
    ,
    /* UINT16 DG_DL_Speech */
    DG_DL_Speech
    ,
    /* UINT16 DG_Microphone */
    DG_Microphone
    ,
    /* UINT16 FM record volume*/
    FM_Record_Vol
    ,
    /* UINT16 BT sync type and length*/
    DEFAULT_BLUETOOTH_SYNC_TYPE,
    DEFAULT_BLUETOOTH_SYNC_LENGTH
    ,
    /* UINT16 BT PCM in/out digital gain*/
    DEFAULT_BT_PCM_IN_VOL,
    DEFAULT_BT_PCM_OUT_VOL
    ,
    /* user mode : normal mode, earphone mode, loud speaker mode */
    /* UCHAR  user_mode             */
    VOL_NORMAL
    ,
    /* auto VM record setting */
    DEFAULT_VM_SUPPORT,
    DEFAULT_AUTO_VM,
    /* Micbais voltage 1900 --> 2200 */
    MICBAIS,
};


#define DEFAULT_WB_SPEECH_NORMAL_MODE_PARA2 \
    96,   253, 16388,    SPEECH_MODE_PARA03, 57607,    799,   400,     64, \
    SPEECH_MODE_PARA08,  4325,   611,     0,  16392,    0|SPEECH_MODE_PARA13,     0|SPEECH_MODE_PARA14,  8192

AUDIO_CUSTOM_WB_PARAM_STRUCT wb_speech_custom_default2 =
{
    /* unsigned short speech_mode_wb_para[8][16] */
    DEFAULT_WB_SPEECH_NORMAL_MODE_PARA2,
    DEFAULT_WB_SPEECH_EARPHONE_MODE_PARA,
    DEFAULT_WB_SPEECH_LOUDSPK_MODE_PARA,
    DEFAULT_WB_SPEECH_BT_EARPHONE_MODE_PARA,
    DEFAULT_WB_SPEECH_BT_CORDLESS_MODE_PARA,
    DEFAULT_WB_SPEECH_CARKIT_MODE_PARA,
    DEFAULT_WB_SPEECH_AUX1_MODE_PARA,
    DEFAULT_WB_SPEECH_AUX2_MODE_PARA,
    /* short sph_wb_in_fir[6][90] */
    WB_Speech_Input_FIR_Coeff,
    /* short sph_wb_out_fir[6][90] */
    WB_Speech_Output_FIR_Coeff,
};


#define VER1_AUD_VOLUME_SPH2 \
   88,96,104,116,124,136,148,0,0,0,0,0,0,0,0,\
    72,80,88,96,104,112,120,0,0,0,0,0,0,0,0,\
    92,100,108,116,124,132,140,0,0,0,0,0,0,0,0,\
    40,52,64,76,88,100,112,0,0,0,0,0,0,0,0

AUDIO_VER1_CUSTOM_VOLUME_STRUCT audio_ver1_custom_default2 = {
    VER1_AUD_VOLUME_RING,
    VER1_AUD_VOLUME_SIP,
    VER1_AUD_VOLUME_MIC,
    VER1_AUD_VOLUME_FM,
    VER1_AUD_VOLUME_SPH2,
    VER1_AUD_VOLUME_SPH2, // sph2 now use the same
    VER1_AUD_VOLUME_SID,
    VER1_AUD_VOLUME_MEDIA,
    VER1_AUD_VOLUME_MATV,
    VER1_AUD_NORMAL_VOLUME_DEFAULT,
    VER1_AUD_HEADSER_VOLUME_DEFAULT,
    VER1_AUD_SPEAKER_VOLUME_DEFAULT,
    VER1_AUD_HEADSETSPEAKER_VOLUME_DEFAULT,
    VER1_AUD_EXTAMP_VOLUME_DEFAULT,
    VER1_AUD_VOLUME_LEVEL_DEFAULT
};

#endif


