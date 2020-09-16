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
const char decode[10] = {0x60, 0xda, 0xf2, 0x66, 0xb6, 0xbe, 0xe0, 0xfe, 0xf6, 0xfc};
const char stay[4] = {0xFE, 0xFD, 0xFB, 0xF7};

void delay(void) {
    int i;
    i = 10000;
    while (i--);
}

int line;

void __interrupt() isr(void) {
    PORTA = 0b11111111;
    if (PORTA == 0b11111110)PORTC = decode[6]; //S7
    else if (PORTA == 0b11111101)PORTC = decode[7]; //8
    else if (PORTA == 0b11111011)PORTC = decode[8]; //9
    else if (PORTA == 0b11110111)PORTC = decode[9]; //10
    else if (line == 2) {
        PORTA = 0b11110111;
        if (PORTA == 0b11110110)PORTC = decode[1]; //2
        if (PORTA == 0b11110101)PORTC = decode[3]; //4
        if (PORTA == 0b11110011)PORTC = decode[5]; //6
        line++;
    } else if (line == 3) {
        PORTA = 0b11111011;
        if (PORTA == 0b11111010)PORTC = decode[0]; //1
        if (PORTA == 0b11111001)PORTC = decode[2]; //3
        line++;
    } else if (line == 4) {
        PORTA = 0b11111101;
        if (PORTA == 0b11111100)PORTC = decode[4]; //5
        line = 2;
    }
    PIR0bits.TMR0IF = 0; //clear,if not,keep interrupting
}

void TMR0init(void) {
    TMR0H = 0x1; //1
    TMR0L = 0x00;
    T0CON0 = 0xC9; //B'11001001' 1:10 
    T0CON1 = 0x45; //B'01000101'  32
    INTCONbits.GIE = 1; //global interrupt,enables all active interrupts
    PIR0bits.TMR0IF = 0; //clear TMR0 Interrupt flag
    PIE0bits.TMR0IE = 1; //enable TMR0 Interrupt flag
}

void PORTinit(void) {
    TRISA = 0x00;
    TRISC = 0x00;
    ANSELA = 0;
    ANSELC = 0;
    PORTA = 0xFF;
    PORTC = 0x00;
}

void main(void) {
    line = 2;
    TMR0init();
    PORTinit();
    while (1);
    return;
}
