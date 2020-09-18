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
#pragma config FEXTOSC = OFF   // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1 // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF  // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON      // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF     // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON    // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF   // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON    // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO     // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF     // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = OFF // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config STVREN = ON   // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31 // WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = SWDTEN      // WDT operating mode (WDT enabled/disabled by SWDTEN bit in WDTCON0)
#pragma config WDTCWS = WDTCWS_7  // WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC        // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = WRT_upper       // UserNVM self-write protection bits (0x0000 to 0x01FF write protected)
#pragma config SCANE = not_available // Scanner Enable bit (Scanner module is not available for use)
#pragma config LVP = ON              // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = ON   // UserNVM Program memory code protection bit (Program Memory code protection enabled)
#pragma config CPD = OFF // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#endif






const char decode[10] = {0xfc, 0x60, 0xda, 0xf2, 0x66, 0xb6, 0xbe, 0xe0, 0xfe, 0xf6};
const char decoder[4] = {0xe0, 0xd0, 0xb0, 0x70};
const char COUNT[256] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned int count = 0;
unsigned int t = 0;
unsigned char i = 0;
unsigned char SUM = 0;



unsigned int push = 0;
unsigned int push1 = 0;
unsigned int out = 0;
unsigned int out1 = 0;
unsigned char index;

void key_scan(void) {
    t++;
    PORTB = 0b00001111;

    if (PORTBbits.RB0 == 0) {
        index = 7;
        if(t)
            SUM ++;
    } else if (PORTBbits.RB1 == 0)
        index = (8);
    else if (PORTBbits.RB2 == 0)
        index = (9);
    else if (PORTBbits.RB3 == 0)
        index = (0);

    else // if (round == 2)
    {
        PORTB = 0b00000111;
        if (PORTB != 7) {
            if (PORTBbits.RB0 == 0)
                index = (2);
            if (PORTBbits.RB1 == 0)
                index = (4);
            if (PORTBbits.RB2 == 0)
                index = (6);

        } else // if (round == 3)
        {
            PORTB = 0b00001011;
            if (PORTB != 0b00001011) {
                if (PORTBbits.RB0 == 0)
                    index = 1;
                if (PORTBbits.RB1 == 0)
                    index = 3;

            } else {
                PORTB = 0b00001101;
                if (PORTBbits.RB0 == 0)
                    index = (5);


            }
        }
    }
}

void __interrupt() isr(void) {
     // count=count+1;
    PIR0bits.TMR0IF = 0;
    TMR0H = 0xf7;
    TMR0L = 0xf3;
    if (i == 0) {
        PORTC = 0;
        PORTA = decoder[3];
        PORTC = decode[(SUM) / 10];
        i++;
    } else if (i == 1) {
        PORTC = 0;
        PORTA = decoder[2];
        PORTC = decode[(SUM) % 10];
        i++;
    } else if (i == 2) {
        PORTC = 0;
        PORTA = decoder[1];
        PORTC = 0;
        i++;
    } else if (i > 2) {
        PORTC = 0x00;
        PORTA = decoder[0];
        key_scan();
        PORTC = decode[index];
        i = 0;
    }
}

void init(void) {
    PORTA = 0;
    TRISA = 0;
    ANSELA = 0;
    LATB = 0x00;

    TRISB = 0;
    ANSELB = 0;
    PORTC = 0;
    TRISC = 0;
    ANSELC = 0;
    T0CON1 = 0b01000000;
    T0CON0 = 0b10010000;
    TMR0H = 0xf7;
    TMR0L = 0xf3;
    INTCONbits.GIE = 1;
    PIE0bits.TMR0IE = 1;
    PORTC = decode[0];

}

void main(void) {
    init();

    while (1) {
    };
    return;
}
