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
#define TMR0_rst TMR0H = 0xEC, TMR0L = 0x82, PIR0bits.TMR0IF = 0x00

const unsigned char digital_decode[10] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};
const unsigned int LED_select_signal[4] = {0xEE, 0xDD, 0xBB, 0x77};
// Display LEDs Select decode

unsigned char interrupt_count = 0;

unsigned char display_signal[4] = {0x00, 0x00, 0x00, 0x00};
unsigned char count[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //1 2 3 4 5 6 7 8 9 10
unsigned char long_or_short = 0x00;
unsigned char one_or_two = 0x00;
unsigned char key = 0x00;
unsigned int interval = 0;
unsigned int onecount = 0;

void scan(void) {
    PORTB = 0x0f;
    if (PORTB != 0x0f) {
        if (PORTBbits.RB0 == 0)
            key = 7;
        else if (PORTBbits.RB1 == 0)
            key = 8;
        else if (PORTBbits.RB2 == 0)
            key = 9;
        else if (PORTBbits.RB3 == 0)
            key = 10;
    } else {
        PORTB = 0x07; //0000 0111
        if (PORTB != 0x07) {
            if (PORTBbits.RB0 == 0)
                key = 2;
            else if (PORTBbits.RB1 == 0)
                key = 4;
            else if (PORTBbits.RB2 == 0)
                key = 6;
        } else {
            PORTB = 0x03; //0000 0011
            if (PORTB != 0x03) {
                if (PORTBbits.RB0 == 0)
                    key = 1;
                else
                    key = 3;
            } else {
                PORTB = 0x01; //0000 0001
                if (PORTB != 0x01)//0000 0001
                    key = 5;
                else
                    key = 0;
            }
        }
    }

    if (interval != 0)
        interval = interval - 1;

    if (key != 0) {
        if (onecount != 0)//key is continued
            onecount = onecount + 1;
        else if (onecount == 0)//key is started
        {
            if (interval >= 350)//key is too closed
            {
                one_or_two = 1;
            }

            if (one_or_two == 0)//key is not too closed
            {
                count[key - 1] = count[key - 1] + 1;
                interval = 400;
                onecount = onecount + 1;
            }
        }

        if (onecount > 200)
            long_or_short = 1;
        else
            long_or_short = 0;
    } else {
        if (onecount != 0)//key is ended
        {
            onecount = 0;
        }
        one_or_two = 0;
    }
}

void __interrupt() isr(void) {
    // reset TMR0
    TMR0_rst;
    interrupt_count++;
    // clear PORTC
    PORTC = 0x00;
    PORTA = LED_select_signal[interrupt_count & 0x03];

    scan();

}

void port_init(void) {
    // init PORTC
    ANSELA = 0x00;
    LATA = 0x00;
    TRISA = 0x00;
    ANSELB = 0x00;
    LATB = 0x00;
    TRISB = 0x00;
    ANSELC = 0x00;
    LATC = 0x00;
    TRISC = 0x00;
}

void int_tmr_init(void) {
    // init interrupt
    INTCONbits.GIE = 1;
    // global interrupt     enable
    INTCONbits.PEIE = 0;
    // peripheral interrupt disable
    INTCONbits.INTEDG = 1;
    // interrupt            rising edge
    PIE0bits.TMR0IE = 1;
    // Timer0 interrupt     enable

    // init TMR0
    T0CON0 = 0xD0;
    T0CON1 = 0x40;
    TMR0_rst;
}

void setup(void) {
    port_init();
    int_tmr_init();
}

void loop(void) {

    if (key == 0) {
        PORTC = display_signal[interrupt_count & 0x03];
    } else if (key == 10) {
        display_signal[0] = digital_decode[0];
        display_signal[1] = digital_decode[count[key - 1] % 10];
        display_signal[2] = digital_decode[one_or_two];
        display_signal[3] = digital_decode[long_or_short];
        PORTC = display_signal[interrupt_count & 0x03];
    } else {
        display_signal[0] = digital_decode[key];
        display_signal[1] = digital_decode[count[key - 1] % 10];
        display_signal[2] = digital_decode[one_or_two];
        display_signal[3] = digital_decode[long_or_short];
        PORTC = display_signal[interrupt_count & 0x03];
    }
    PORTC = display_signal[interrupt_count & 0x03];
}

void main(void) {
    setup();
    while (1) {
        loop();
    }
    return;
}
