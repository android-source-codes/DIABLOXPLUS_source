
#ifndef AUDIO_ACF_DEFAULT_H
#define AUDIO_ACF_DEFAULT_H
#if defined(MTK_AUDIO_BLOUD_CUSTOMPARAMETER_V4)
    /* Compensation Filter HSF coeffs: default all pass filter       */
    /* BesLoudness also uses this coeffs    */ 
    #define BES_LOUDNESS_HSF_COEFF \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
\
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, \
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 
   

    /* Compensation Filter BPF coeffs: default all pass filter      */ 
    #define BES_LOUDNESS_BPF_COEFF \
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
\
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
\
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
\    
 	0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \         
\    
 	0x00000000,0x00000000,0x00000000, \ 
    0000000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \     
\
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000, \ 
    0x00000000,0x00000000,0x00000000     
    
    #define BES_LOUDNESS_LPF_COEFF \
    0x00000000, 0x00000000, 0x00000000,\ 
	0x00000000, 0x00000000, 0x00000000,\ 
	0x00000000, 0x00000000, 0x00000000,\ 
	0x00000000, 0x00000000, 0x00000000,\ 
	0x00000000, 0x00000000, 0x00000000,\ 
	0x00000000, 0x00000000, 0x00000000 

    #define BES_LOUDNESS_WS_GAIN_MAX  0
           
    #define BES_LOUDNESS_WS_GAIN_MIN  0
           
    #define BES_LOUDNESS_FILTER_FIRST  0
           
    #define BES_LOUDNESS_GAIN_MAP_IN \
    0, 0, 0, 0,  0
   
    #define BES_LOUDNESS_GAIN_MAP_OUT \            
    0, 0, 0, 0, 0

	#define BES_LOUDNESS_ATT_TIME	164
	#define BES_LOUDNESS_REL_TIME	16400               
#else

    /* Compensation Filter HSF coeffs: default all pass filter       */
    /* BesLoudness also uses this coeffs    */ 
    #define BES_LOUDNESS_HSF_COEFF \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, 0x00000000   


    /* Compensation Filter BPF coeffs: default all pass filter      */ 
    #define BES_LOUDNESS_BPF_COEFF \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
\
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
\
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
\
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000, \
    0x00000000, 0x00000000, 0x00000000   
    
    #define BES_LOUDNESS_DRC_FORGET_TABLE \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000, \
    0x00000000, 0x00000000   

    #define BES_LOUDNESS_WS_GAIN_MAX  0
           
    #define BES_LOUDNESS_WS_GAIN_MIN  0
           
    #define BES_LOUDNESS_FILTER_FIRST  0
           
    #define BES_LOUDNESS_GAIN_MAP_IN \
    0, 0, 0, 0, 0     
              
    #define BES_LOUDNESS_GAIN_MAP_OUT \            
    0, 0, 0, 0, 0               
	
#endif
#endif
