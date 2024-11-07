/*
 * File:   main.c
 * Author: Jarred Eisbrenner, Angad Minhas, Sam Schroeder
 *
 * Created on: 29 Oct, 2024
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

uint8_t received;
uint8_t received_char = 0;
uint8_t RXFlag = 0;

uint16_t ADC_val = 0;

uint8_t PB_last = 0b000;
uint8_t PB_current = 0b000;

uint8_t T1_triggered = 0;

enum presentation_type current_type = HEX;
enum bar_type current_bar = DASH;

int main(void) {
    newClk(8);
    
    AD1PCFG = 0b1111111111011111;
    
    InitUART2();
    CNinit();
    IOinit();
    Timerinit();
    ADCinit();
    
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
    
    // Sets the state of the current bar type depending on the buttons
    if (PB_current == 0b000) {
        if (PB_last == 0b001) current_bar = EQUALS;
        else if (PB_last == 0b010) current_bar = DASH;
        else if (PB_last == 0b100) current_bar = TILDE;
        else if (PB_last == 0b011) current_bar = EX;
        else if (PB_last == 0b110) current_bar = PERCENTAGE;
        else if (PB_last == 0b101) current_bar = OHS;
        else if (PB_last == 0b111) current_bar = IDK;
    }
    
    PB_last = 0b000; // Resets the last button state
    
    T3CONbits.TON = 0;
}


void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    IFS1bits.CNIF = 0; // clear interrupt flag
    
    // PB_last = PB_current;
    PB_last |= PB_current; // accumulates the buttons that have been pressed
    PB_current = ((BUTTON1) | (BUTTON2 << 1) | (BUTTON3 << 2));
   
    // Starts timer 3 if it is not yet started
    if (T3CONbits.TON == 0) {
        TMR3 = 0;
        T3CONbits.TON = 1;
    }
}
