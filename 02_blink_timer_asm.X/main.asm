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

; ============================================
pagesel     MAIN
GOTO        MAIN
; ============================================

code

MAIN
; ============================================
; init PORTC
BANKSEL     PORTC
CLRF        PORTC
BANKSEL     LATC
CLRF        LATC
BANKSEL     ANSELC
CLRF        ANSELC

BANKSEL     TRISC
MOVLW       B'00000000'
MOVWF       TRISC
; ============================================
; init TMR0

BANKSEL     TMR0H
MOVLW       0xF3
MOVWF       TMR0H

BANKSEL     TMR0L
MOVLW       0x00
MOVWF       TMR0L

BANKSEL     T0CON0
MOVLW       B'11001111';
; bit7      enable timer0
; bit4      8-bit timer
; bit 3-0   postscaler  1:16
MOVWF       T0CON0

BANKSEL     T0CON1
MOVLW       B'01000111';
; bit 7-5   clk source  F_OSC / 4
; bit 4     sync
; bit 3-0   prescaler   1:128
MOVWF       T0CON1
 
; clear the output signal
BANKSEL     PIR0
BCF         PIR0,   5
; ============================================

LOOP
    BANKSEL     PIR0
    BTFSS       PIR0,   5
    ; no output from TMR0
    GOTO        LOOP
    ; have output from TMR0
    ; BANKSEL	PIR0
    BCF         PIR0,   5           ; clear TMR0IF
       ; set TMR0L as 0x00
    ; make sure TMR0IF overflow at same time during test

    
    
    BANKSEL     PORTC
    MOVLW       B'11111111'
    XORWF       PORTC,    f         ; flip PORTC
    

    
    GOTO        LOOP
    END

END