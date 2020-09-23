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
; pre define
; address in Common RAM, no need for `BANKSEL`
udata_shr
N0      res     1h
N1      res     1h
N2      res     1h

RST     code    BLOFFSET
PAGESEL         MAIN
GOTO            MAIN

code
; =============================================
MAIN
; SETUP
; init ports, select port C, set as Digital Output
; select port C
BANKSEL     PORTC
CLRF        PORTC

; select digital signal
BANKSEL     ANSELC
CLRF        ANSELC
MOVLW       B'00000000'
MOVWF       ANSELC

; select output
BANKSEL     TRISC
MOVLW       B'00000000'
MOVWF       TRISC

; init output
MOVLW       B'00000000'
MOVWF       PORTC

; =============================================
; LOOP
; main loop
LOOP
    ; W is used as XOR
    MOVLW       B'11111111'
    ; XOR port c & reg f
    XORWF       PORTC,  1
    ; delay
    CALL        DELAY
GOTO    LOOP

; sub program for delay
DELAY
    ; Instruction Num = 3 * N0 * N1 * N2 + 4 * N0 * N1 + 4 * N0 + 5
    MOVLW       0x4F
    MOVWF       N0
    DELAY_LOOP_0

        MOVLW       0x19
        MOVWF       N1
        DELAY_LOOP_1

            MOVLW   0x53
            MOVWF   N2
            DELAY_LOOP_2
                DECFSZ  N2
            GOTO    DELAY_LOOP_2

            DECFSZ  N1
        GOTO    DELAY_LOOP_1

        DECFSZ  N0
    GOTO    DELAY_LOOP_0
RETURN


END
