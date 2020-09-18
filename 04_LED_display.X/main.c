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
unsigned char repeat_num; //
unsigned char frame_repeat_num; // max number of a frame repeat
unsigned char interrupt_count = 0;

unsigned char display_signal[4] = {0xFF, 0xFF, 0xFF, 0xFF};

unsigned char display_cache[128] = {
    28, 0xFC, 238, 0xFC, // LOAD
    29, 0xFC, 238, 0xFC, // LOAD
    29, 0xFD, 238, 0xFC, // LOAD
    29, 0xFD, 239, 0xFC, // LOAD
    29, 0xFD, 239, 0xFD, // LOAD
    182, 158, 182, 0, // SES

    0xDA, 0xFC, 0xDA, 0xFC,
    0xDA, 0xFC, 0xDA, 0xFC,
    0xFC, 0xF7, 0x60, 0xE0,
    0xFC, 0xF7, 0x60, 0xE0,
    0x02, 0x02, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x00
};
unsigned char dis_cache_size = 52; // byte saved in cache
unsigned char sum_frame_num = 13; // number of frame, in the display signal cache
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
unsigned char frame_cache_num;
unsigned char frame_num; // index of displaying frame
unsigned char frame_start; // start index in display signal cache
// used when loading data from display signal cache to register

unsigned char frame_refresh_enable = 0;
unsigned char display_write_in = 0x00;

void frame_switch(void) {
    if (display_ctrl != 4) {
        if (repeat_num == frame_repeat_num) {
            repeat_num = 0;

            if (display_ctrl & 0x02) {

                if (display_ctrl & 0x01) // shift 4 byte once
                {
                    display_signal[0] = display_cache[0];
                    display_signal[1] = display_cache[1];
                    display_signal[2] = display_cache[2];
                    display_signal[3] = display_cache[3];
                    for (unsigned char j = 0; j < dis_cache_size; j++) {
                        display_cache[j] = display_cache[j + 4];
                    }
                    display_cache[dis_cache_size - 1] = 0x00;
                    display_cache[dis_cache_size - 2] = 0x00;
                    display_cache[dis_cache_size - 3] = 0x00;
                    display_cache[dis_cache_size - 4] = 0x00;
                    dis_cache_size -= 4;
                } else // shift 1 byte once
                {
                    display_signal[0] = display_cache[0];
                    display_signal[1] = display_cache[1];
                    display_signal[2] = display_cache[2];
                    display_signal[3] = display_cache[3];

                    for (unsigned char j = 0; j < dis_cache_size; j++) {
                        display_cache[j] = display_cache[j + 1];
                    }
                    display_cache[dis_cache_size - 1] = 0x00;

                    dis_cache_size--;
                }
            } else {
                if (frame_num == sum_frame_num) {
                    frame_num = 0;
                    frame_start = 0;
                }
                if (display_ctrl & 0x01) {
                    display_signal[0] = display_cache[frame_start];
                    display_signal[1] = display_cache[frame_start + 1];
                    display_signal[2] = display_cache[frame_start + 2];
                    display_signal[3] = display_cache[frame_start + 3];
                    frame_start += 4;
                } else {
                    frame_start = frame_num;
                    for (unsigned char j = 0; j < 4; j++) {

                        if (frame_num + j > frame_cache_num - 1) {
                            display_signal[j] = display_cache[frame_start + j - frame_cache_num];
                        } else {
                            display_signal[j] = display_cache[frame_start + j];
                        }
                    }
                }
            }
            frame_num++;
        }
    }
}

void __interrupt() isr(void) {
    // reset TMR0
    TMR0_rst;
    interrupt_count++;
    // clear PORTC
    PORTC = 0x00;
    /************display part************/
    PORTA = LED_select_signal[interrupt_count & 0x03];
    PORTC = display_signal[interrupt_count & 0x03];

    if (!(interrupt_count & 0x03)) {
        repeat_num++;
        frame_switch();
    }
}

void display_cache_push_back() {
    //    if (display_ctrl & 0x02)
    // can only push display data in clear mode

    display_cache[dis_cache_size] = display_write_in;
    dis_cache_size++;
    display_write_in = 0x00;
}

void display_cache_clear(void) {
    for (unsigned short j = 0; j < 128; j++) {
        display_cache[j] = 0x00;
    }
    dis_cache_size = 0;
}

void port_init(void) {
    // init PORTC
    ANSELA = 0x00;
    LATA = 0x00;
    TRISA = 0x00;

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

void start_disp(void) {
    frame_repeat_num = 1;
    display_clear_4;
    while (dis_cache_size) {
    };
    display_cache_clear();
}

void digital_num_loop(void) {
    frame_repeat_num = 1;
    for (unsigned char i = 0; i < 10; i++) {
        display_write_in = digital_decode[i];
        display_cache_push_back();
    }
    display_loop_1;
}

void setup(void) {
    port_init();
    int_tmr_init();
    start_disp();
    digital_num_loop();
}

void loop(void) {
}

void main(void) {
    setup();
    while (1) {
        loop();
    }
    return;
}
