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
#define TMR0H_rst 0xEC
#define TMR0L_rst 0x82

#define TMR0_rst TMR0H = TMR0H_rst, TMR0L = TMR0L_rst, PIR0bits.TMR0IF = 0x00
/*
 * ISR definition 
 */
const unsigned int digital_decode[10] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};
// const unsigned int Char_Decode[128];
// decode list for '0' to '1'
const unsigned int LedSel[4] = {0xEE, 0xDD, 0xBB, 0x77};
// Display LEDs Select decode
unsigned int repeat_num;
const unsigned int frame_repeat_num = 50;
unsigned int interrupt_count = 0;

unsigned int display_signal[4];

unsigned int display_cache[40] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};
unsigned int sum_frame_num = 10; // 保存在chche中帧的数量
unsigned int dis_shift = 1;     // 每次刷新位移的数目 1 or 4
unsigned int dis_byte_num = 10;  // 保存在cache中的待显示的字节数
unsigned int frame_num;   // 当前正在显示的帧编号
unsigned int frame_start; // 起始帧地址

void __interrupt() isr(void)
{
    // reset TMR0
    TMR0_rst;
    interrupt_count++;
    // clear PORTC
    PORTC = 0x00;

    if ((interrupt_count & 0x03) == 0x00)
    {
        repeat_num++;
    }
    PORTA = LedSel[interrupt_count & 0x03];
    PORTC = display_signal[interrupt_count & 0x03];

    if (repeat_num == frame_repeat_num)
    {
        repeat_num = 0;
        if (frame_num == sum_frame_num)
        {
            frame_num = 0;
            frame_start = 0;
        }
        if (dis_shift == 4)
        {
            display_signal[0] = display_cache[frame_start];
            display_signal[1] = display_cache[frame_start + 1];
            display_signal[2] = display_cache[frame_start + 2];
            display_signal[3] = display_cache[frame_start + 3];
            frame_start += 4;
        }
        else if (dis_shift == 1)
        {
            frame_start = frame_num;
            for (int j = 0; j < 4; j++)
            {
                if (frame_num + j > dis_byte_num - 1)
                {
                    display_signal[j] = display_cache[frame_start + j - dis_byte_num];
                }

                else
                {
                    display_signal[j] = display_cache[frame_start + j];
                }
            }
        }
        frame_num++;
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
    /***********************************************************/
    // init TMR0
    T0CON0 = 0xD0;
    T0CON1 = 0x40;
    TMR0_rst;
}

void display_init(void)
{
    repeat_num = 0;
    frame_num = 0;
    frame_start = 0;
}

void setup(void)
{
    port_init();
    int_tmr_init();
    display_init();
}

void loop(void)
{
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
