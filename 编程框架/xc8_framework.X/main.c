/*
 * File:   main.c
 * Author: QuickFox
 *
 * Created on 2020?1?19?, ??5:16
 */


#include <xc.h>
#ifndef BOOTLOADER

// PIC16F18854 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = OFF    // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = SWDTEN    // WDT operating mode (WDT enabled/disabled by SWDTEN bit in WDTCON0)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = WRT_upper  // UserNVM self-write protection bits (0x0000 to 0x01FF write protected)
#pragma config SCANE = not_available// Scanner Enable bit (Scanner module is not available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = ON          // UserNVM Program memory code protection bit (Program Memory code protection enabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#endif
const char decode[10] = {0x60,0xda,0xf2,0x66,0xb6,0xbe,0xe0,0xfe,0xf6,0xfc};
const char stay[4]={0xFE,0xFD,0xFB,0xF7};

void PORTinit(void) {
    TRISA = 0x00;
    TRISC = 0x00;     
    ANSELA = 0;
    ANSELC = 0;
    PORTA = 0xFF;
    PORTC = 0x00;
}

void delay(void) {
    int i=100;
    while(i--);
}

void main(void) {
    PORTinit();
    int en = 0;
    while (1) {
        PORTA=0b11111111;
        switch (PORTA) {
            case 0b11111110: PORTC=decode[6]; //break;//S7
            case 0b11111101: PORTC=decode[7];// break;//S8
            case 0b11111011: PORTC=decode[8];// break;//S9
            case 0b11110111: PORTC=decode[9];// break;//S10
            default: en = 0;
        }

        if (en == 0) {
            PORTA=0b11110111;
            delay();
            switch (PORTA) {
                case 0b11110110: PORTC=decode[1]; //break;//S2
                case 0b11110101: PORTC=decode[3]; //break;//S4
                case 0b11110011: PORTC=decode[5]; //break;//S6
                default: en = 0;
            }
        }

        if (en == 0) {
            PORTA=0b11111011;
            delay();
            switch (PORTA) {
                case 0b11111010: PORTC=decode[0];// break;//S1
                case 0b11111001: PORTC=decode[2];// break;//S3
                default: en = 0;
            }
        }

        if (en == 0) {
            PORTA=0b11111101;
            delay();
            if (PORTA == 0b11111100) {
                PORTC=decode[4];//S5
            }
        }
    }
    return;
}
