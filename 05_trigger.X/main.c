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
unsigned char repeat_num;       //
unsigned char frame_repeat_num; // max number of a frame repeat
unsigned char interrupt_count = 0;

unsigned char display_signal[4] = {0xFF, 0xFF, 0xFF, 0xFF};

unsigned char display_cache[128] = {
    28, 0xFC, 238, 0xFC, // LOAD
    28, 0xFC, 238, 0xFC, // LOAD

    0x00, 0x00, 0x00, 0x00,
    16, 0x00, 0x00, 0x00,
    16, 16, 0x00, 0x00,
    16, 16, 16, 0x00,
    16, 16, 16, 16,
    16, 16, 16, 48,
    16, 16, 16, 112,
    16, 16, 16, 240,
    16, 16, 144, 240,
    16, 144, 144, 240,
    144, 144, 144, 240,
    148, 144, 144, 240,
    156, 144, 144, 240,

    0x00, 0x00, 0x00, 0x00,
    182, 158, 182, 0, // SES
    0xDA, 0xFC, 0xDA, 0xFC,
    0xFC, 0xF7, 0x60, 0xE0,
    0x02, 0x02, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x00};

unsigned char sum_frame_num = 22; // number of frame, in the display signal cache
//unsigned char display_ctrl = 0b00000001;
unsigned char display_ctrl;
/*
    bit 0
        1   shift 4
        0   shift 1
    bit 1
        1   loop
        0   clear
    bit 2
        0   frame
        1   real time
 */

#define display_clear_4 display_ctrl = 3, repeat_num = 0
#define display_clear_1 display_ctrl = 2, repeat_num = 0
#define display_loop_4 display_ctrl = 1, repeat_num = 0, frame_num = 0, frame_start = 0, frame_cache_num = (dis_cache_size >> 2)
#define display_loop_1 display_ctrl = 0, repeat_num = 0, frame_num = 0, frame_start = 0, frame_cache_num = dis_cache_size
#define display_real_time display_ctrl = 4
unsigned char dis_cache_size = 128; // byte saved in cache
unsigned char frame_cache_num;
unsigned char frame_num;   // index of displaying frame
unsigned char frame_start; // start index in display signal cache
// used when loading data from display signal cache to register

unsigned char frame_refresh_enable = 0;
unsigned char display_write_in = 0x00;

inline void frame_switch(void)
{
    if (display_ctrl != 4)
    {
        if (repeat_num == frame_repeat_num)
        {
            repeat_num = 0;

            if (display_ctrl & 0x02)
            {

                if (display_ctrl & 0x01) // shift 4 byte once
                {
                    display_signal[0] = display_cache[0];
                    display_signal[1] = display_cache[1];
                    display_signal[2] = display_cache[2];
                    display_signal[3] = display_cache[3];
                    for (unsigned char j = 0; j < dis_cache_size; j++)
                    {
                        display_cache[j] = display_cache[j + 4];
                    }
                    display_cache[dis_cache_size - 1] = 0x00;
                    display_cache[dis_cache_size - 2] = 0x00;
                    display_cache[dis_cache_size - 3] = 0x00;
                    display_cache[dis_cache_size - 4] = 0x00;
                    dis_cache_size -= 4;
                }
                else // shift 1 byte once
                {
                    display_signal[0] = display_cache[0];
                    display_signal[1] = display_cache[1];
                    display_signal[2] = display_cache[2];
                    display_signal[3] = display_cache[3];

                    for (unsigned char j = 0; j < dis_cache_size; j++)
                    {
                        display_cache[j] = display_cache[j + 1];
                    }
                    display_cache[dis_cache_size - 1] = 0x00;

                    dis_cache_size--;
                }
            }
            else
            {
                if (frame_num == sum_frame_num)
                {
                    frame_num = 0;
                    frame_start = 0;
                }
                if (display_ctrl & 0x01)
                {
                    display_signal[0] = display_cache[frame_start];
                    display_signal[1] = display_cache[frame_start + 1];
                    display_signal[2] = display_cache[frame_start + 2];
                    display_signal[3] = display_cache[frame_start + 3];
                    frame_start += 4;
                }
                else
                {
                    frame_start = frame_num;
                    for (unsigned char j = 0; j < 4; j++)
                    {

                        if (frame_num + j > frame_cache_num - 1)
                        {
                            display_signal[j] = display_cache[frame_start + j - frame_cache_num];
                        }
                        else
                        {
                            display_signal[j] = display_cache[frame_start + j];
                        }
                    }
                }
            }
            frame_num++;
        }
    }
}
unsigned char key = 0x00;
unsigned int last_state, current_state, key_press, key_loose;
unsigned char scan_period;
unsigned char press_count[10] = {0x00};
unsigned char is_pressed;
unsigned char is_double;
unsigned char key_buf;
void key_scan(void)
{
    current_state = 0x00;
    PORTB = 0x0f;
    if (PORTB != 0x0f)
    {
        if (PORTBbits.RB0 == 0)
            key = 7;
        else if (PORTBbits.RB1 == 0)
            key = 8;
        else if (PORTBbits.RB2 == 0)
            key = 9;
        else if (PORTBbits.RB3 == 0)
            key = 0;
    }
    else
    {
        PORTB = 0x07; //0000 0111
        if (PORTB != 0x07)
        {
            if (PORTBbits.RB0 == 0)
                key = 2;
            else if (PORTBbits.RB1 == 0)
                key = 4;
            else if (PORTBbits.RB2 == 0)
                key = 6;
        }
        else
        {
            PORTB = 0x03; //0000 0011
            if (PORTB != 0x03)
            {
                if (PORTBbits.RB0 == 0)
                    key = 1;
                else
                    key = 3;
            }
            else
            {
                PORTB = 0x01;      //0000 0001
                if (PORTB != 0x01) //0000 0001
                    key = 5;
                else
                    key = 15;
            }
        }
    }
    if (is_pressed)
    {
        scan_period++;
    }
    else if (scan_period < 30)
    {
        scan_period++;
    }
    else
    {
        scan_period = 0xFF;
    }

    current_state = (1 << key);
    key_press |= (~last_state) & current_state;
    key_loose |= last_state & (~current_state);
    last_state = current_state;
}

void __interrupt() isr(void)
{
    // reset TMR0
    TMR0_rst;
    interrupt_count++;
    // clear PORTC
    PORTC = 0x00;
    /************display part************/
    PORTA = LED_select_signal[interrupt_count & 0x03];
    PORTC = display_signal[interrupt_count & 0x03];

    if (!(interrupt_count & 0x03))
    {
        repeat_num++;
        frame_switch();
    }

    if ((interrupt_count & 0x03) == 2)
    {
        key_scan();
    }
}

void display_cache_push_back()
{
    //    if (display_ctrl & 0x02)
    // can only push display data in clear mode

    display_cache[dis_cache_size] = display_write_in;
    dis_cache_size++;
    display_write_in = 0x00;
}

void display_cache_clear(void)
{
    for (unsigned short j = 0; j < 128; j++)
    {
        display_cache[j] = 0x00;
    }
    dis_cache_size = 0;
}

void port_init(void)
{
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

void start_disp(void)
{
    frame_repeat_num = 5;
    display_clear_4;
    while (dis_cache_size)
    {
    };
    display_cache_clear();
}
unsigned char key_num;

void key_init(void)
{
    key = 0;
    key_num = 0;
    last_state = 0;
    current_state = 0;
    key_press = 0;
    key_loose = 0;
    is_pressed = 0;
    is_double = 0;
}

void setup(void)
{
    port_init();
    int_tmr_init();
    start_disp();
    key_init();
    display_real_time;
}

void loop(void)
{

    if (key_press << 1)
    {
        key_press = 0;
        is_pressed = 1;

        if (scan_period == 0xFF)
        {
            // single click
            display_signal[0] = digital_decode[key];
            key_buf = key;
            press_count[key]++;
            scan_period = 0;
        }
        else
        {
            // double click
            is_double = 1;
        }
    }
    if (key_loose << 1)
    {

        key_loose = 0;
        is_pressed = 0;
        if (is_double)
        {
            display_signal[1] = digital_decode[2];
        }
        else if (scan_period > 30)
        {
            display_signal[1] = digital_decode[1];
        }
        else
        {
            display_signal[1] = digital_decode[0];
        }
        if (press_count[key_buf] == 100)
        {
            press_count[key_buf] = 0x00;
        }
        display_signal[2] = digital_decode[press_count[key_buf] / 10];
        display_signal[3] = digital_decode[press_count[key_buf] % 10];
        is_double = 0;
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
