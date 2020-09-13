#include <p16f18854.inc>
    
#ifdef BOOTLOADER
#define BLOFFSET 0x200
#else
__config _CONFIG1, _FEXTOSC_ECH & _RSTOSC_HFINT1 & _CSWEN_ON & _FCMEN_ON
__config _CONFIG2, _MCLRE_ON & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LO & _ZCD_OFF & _PPS1WAY_OFF & _STVREN_ON
__config _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_SWDTEN & _WDTCWS_WDTCWS_7 & _WDTCCS_SC
__config _CONFIG4, _WRT_WRT_upper & _SCANE_not_available & _LVP_ON
__config _CONFIG5, _CP_OFF & _CPD_OFF
#define BLOFFSET 0
#endif
    
    
    org BLOFFSET
end