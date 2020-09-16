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
//#define TMR0_rst TMR0H = 0xDF, TMR0L = 0x70, PIR0bits.TMR0IF = 0x00

const unsigned char digital_decode[10] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};
const unsigned char LED_select_signal[4] = {0xEE, 0xDD, 0xBB, 0x77};
// Display LEDs Select decode
unsigned char interrupt_count = 0;
unsigned char display_signal[4] = {0xFC, 0x60, 0x03, 0x04};

unsigned int last_state = 0;
unsigned int current_state = 0;
unsigned int triggered_flaged = 0;
unsigned char key_buf;
unsigned char key_num = 0x00;

void keyboard_scan(void)
{
    key_buf = 0;
    PORTB = 0x0f;
    current_state = 0xFF;

    if (PORTB != 0x0f)
    {
        if (PORTBbits.RB0 == 0)
        {
            current_state = ~128;
        }
        else if (PORTBbits.RB1 == 0)
        {
            current_state = `256;
        }
        else if (PORTBbits.RB2 == 0)
        {
            current_state = ~512;
        }
        else if (PORTBbits.RB3 == 0)
        {
            current_state = ~1024;
        }
    }
    else
    {
        PORTB = 0x07; //0000 0111
        if (PORTB != 0x07)
        {
            if (PORTBbits.RB0 == 0)
            {
                current_state = ~4;
            }
            else if (PORTBbits.RB1 == 0)
            {
                current_state = ~16;
            }
            else if (PORTBbits.RB2 == 0)
            {
                current_state = ~64;
            }
        }
        else
        {
            PORTB = 0x03; //0000 0011
            if (PORTB != 0x03)
            {
                if (PORTBbits.RB0 == 0)
                {
                    current_state = ~2;
                }
                else
                {
                    current_state = ~8;
                }
            }
            else
            {
                PORTB = 0x01;      //0000 0001
                if (PORTB != 0x01) //0000 0001
                {
                    current_state = ~32;
                }
            }
        }
    }

    triggered_flaged |= (last_state & (~current_state));
    last_state = current_state;
}

void __interrupt() isr(void)
{
    // reset TMR0
    TMR0_rst;
    interrupt_count++;
    // clear PORTC
    PORTC = 0x00;
    PORTA = LED_select_signal[interrupt_count & 0x03];
    PORTC = display_signal[interrupt_count & 0x03];

    if (interrupt_count & 0x03)
    {
        keyboard_scan();
        display_signal[0] = triggered_flaged & 0xFF;
        display_signal[1] = triggered_flaged >> 8;
    }
}

void port_init(void)
{
    // init PORTC
    ANSELA = 0x00;
    LATA = 0x00;
    TRISA = 0x00;
    ANSELC = 0x00;
    LATC = 0x00;
    TRISC = 0x00;
    ANSELB = 0x00;
}

void int_tmr_init(void)
{
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

void setup(void)
{
    port_init();
    int_tmr_init();
}

void loop(void)
{
    if (triggered_flaged)
    {
        for (int i = 1; i < 11; i++)
        {
            display_signal[0] = digital_decode[i - 1];
        }
    }
}

void main(void)
{
    setup();
    while (1)
    {
        loop();
    }
    return;
}
