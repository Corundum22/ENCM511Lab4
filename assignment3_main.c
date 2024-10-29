/*
 * File:   main.c
 * Author: UPDATE THIS WITH YOUR GROUP MEMBER NAMES OR POTENTIALLY LOSE POINTS
 *
 * Created on: USE THE INFORMATION FROM THE HEADER MPLAB X IDE GENERATES FOR YOU
 */

// FBS
#pragma config BWRP = OFF               // Table Write Protect Boot (Boot segment may be written)
#pragma config BSS = OFF                // Boot segment Protect (No boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Code Flash Write Protection bit (General segment may be written)
#pragma config GCP = OFF                // General Segment Code Flash Code Protection bit (No protection)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Select (Fast RC oscillator (FRC))
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-Speed Start-up disabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = ON            // CLKO Enable Configuration bit (CLKO output disabled; pin functions as port I/O)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)

// FICD
#pragma config ICS = PGx2               // ICD Pin Placement Select bits (PGC2/PGD2 are used for programming and debugging the device)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

// #pragma config statements should precede project file includes.

#include <xc.h>
#include <p24F16KA101.h>

#include "clkChange.h"
#include "uart.h"
#include "stdio.h"

#define BUTTON1 (PORTAbits.RA2 == 0)
#define BUTTON2 (PORTBbits.RB4 == 0)
#define BUTTON3 (PORTAbits.RA4 == 0)

enum presentation_type {
    HEX,
    DEC,
};

enum bar_type {
    EQUALS = '=',
    DASH = '-',
    TILDE = '~',
    EX = 'X',
    PERCENTAGE = '%',
    OHS = 'O',
    IDK = 'y',
};

uint16_t slow = 0;
uint8_t received;
uint8_t received_char = 0;
uint8_t RXFlag = 0;

uint16_t ADC_val = 0;

uint8_t PB_last = 0b000;
uint8_t PB_current = 0b000;
uint8_t PB_complete = 0;

uint8_t T1_triggered = 0;

enum presentation_type current_type = HEX;
enum bar_type current_bar = DASH;

void CNinit() {
    IPC4bits.CNIP = 6;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;
}

void IOinit() {
    TRISBbits.TRISB8 = 0;
    LATBbits.LATB8 = 0;
    
    TRISAbits.TRISA4 = 1;
    CNPU1bits.CN0PUE = 1;
    CNEN1bits.CN0IE = 1;
    
    TRISBbits.TRISB4 = 1;
    CNPU1bits.CN1PUE = 1;
    CNEN1bits.CN1IE = 1;
    
    TRISAbits.TRISA2 = 1;
    CNPU2bits.CN30PUE = 1;
    CNEN2bits.CN30IE = 1;
}

void Timerinit() {
    IEC0bits.T1IE = 1;
    T1CONbits.TCKPS = 0b11;
    PR1 = 15625;
    
    IEC0bits.T2IE = 1; //enable timer interrupt
    PR2 = 2000; // count for 1 ms
    
    IEC0bits.T3IE = 1; //enable timer interrupt
    T3CONbits.TCKPS = 0b11;
    PR3 = 235; // count for 150 ms
}

void primary_loop() {
    char* output_str[50];
    output_str[49] = '\0';
    get_sample();
    
    //sprintf(output_str, "                ");
    //sprintf(output_str, "[================]");
    
    make_bar(&output_str[0], 16, &current_bar);
    
    if (current_type == HEX) sprintf(&output_str[8], " %x    ", ADC_val);
    else sprintf(&output_str[8], " %d    ", ADC_val);
    
    Disp2String(output_str);
    
    
    wait_until_T1();
}

void make_bar(char *target, uint8_t target_len, char *bar_char) {
    uint16_t segment_num = ADC_val / 64;
    
    int i;
    for (i = 0; i < segment_num; i++) target[i] = *bar_char;
    for (; i < target_len; i++) target[i] = ' ';
}

void wait_until_T1() {
    while (T1_triggered == 0) Idle();
    T1_triggered = 0;
    TMR1 = 0;
}

uint16_t do_ADC() {
    uint16_t ADC_value;
    AD1CON2bits.VCFG = 0b000;
    AD1CON3bits.ADRC = 0;
    AD1CHSbits.CH0SA = 0b0101;
    
    AD1CON1bits.ADON = 1;
    AD1CON1bits.SAMP = 1;
    
    while (AD1CON1bits.DONE == 0);
    ADC_value = ADC1BUF0;
    AD1CON1bits.SAMP = 0;
    AD1CON1bits.ADON = 0;
    
    return ADC_value;
}

void ADCinit() {
    AD1CON1bits.FORM = 0b00;
    AD1CON1bits.SSRC = 0b111;
    AD1CON1bits.ASAM = 0b0;
    AD1CON2bits.VCFG = 0b000;
    AD1CON3bits.ADRC = 0;
    AD1CHSbits.CH0SA = 0b0101;
    AD1CHSbits.CH0NA = 0;
    AD1CON2bits.ALTS = 0;
    AD1CON3bits.SAMC = 0b11111;
}

void get_sample() {
    AD1CON1bits.ADON = 1;
    AD1CON1bits.SAMP = 1;
    
    while (AD1CON1bits.DONE == 0);
    ADC_val = ADC1BUF0;
    AD1CON1bits.SAMP = 0;
    AD1CON1bits.ADON = 0;
}

int main(void) {
    newClk(8);
    
    AD1PCFG = 0xFFFF; /* keep this line as it sets I/O pins that can also be analog to be digital */
    
    InitUART2();
    CNinit();
    IOinit();
    Timerinit();
    ADCinit();
    
    Disp2String("something");
    
    T1CONbits.TON = 1;
    while(1) primary_loop();
    
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void){    
    IFS0bits.T1IF = 0; // clear interrupt flag
    
    T1_triggered = 1;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void){    
    IFS0bits.T2IF = 0; // clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){
    IFS0bits.T3IF = 0; // clear interrupt flag
    
    PB_complete = 1;
    PB_last = 0b000;
    
    T3CONbits.TON = 0;
}


void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    IFS1bits.CNIF = 0; // clear interrupt flag
    
    //PB_last = PB_current;
    PB_last |= PB_current; // accumulates the buttons that have been pressed
    PB_current = ((BUTTON1) | (BUTTON2 << 1) | (BUTTON3 << 2));
    
    if (PB_current == 0b000) {
        if (PB_last == 0b001) current_bar = EQUALS;
        else if (PB_last == 0b010) current_bar = DASH;
        else if (PB_last == 0b100) current_bar = TILDE;
        else if (PB_last == 0b011) current_bar = EX;
        else if (PB_last == 0b110) current_bar = PERCENTAGE;
        else if (PB_last == 0b101) current_bar = OHS;
        else if (PB_last == 0b111) current_bar = IDK;
    }
    
    if (T3CONbits.TON == 0) {
        TMR3 = 0;
        T3CONbits.TON = 1;
    }
}