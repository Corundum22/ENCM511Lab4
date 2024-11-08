#include "functions.h"
#include "ADC.h"

extern enum presentation_type {
    HEX,
    DEC,
};

extern enum bar_type {
    EQUALS = '=',
    DASH = '-',
    TILDE = '~',
    EX = 'X',
    PERCENTAGE = '%',
    OHS = 'O',
    IDK = 'y',
};

extern uint8_t received;
extern uint8_t received_char;
extern uint8_t RXFlag;

extern uint16_t ADC_val;

extern uint8_t PB_last;
extern uint8_t PB_current;

extern uint8_t T1_triggered;

extern enum presentation_type current_type;
extern enum bar_type current_bar;

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
    IEC0bits.T1IE = 1; // eabled timer 1 interrupt
    T1CONbits.TCKPS = 0b11;
    PR1 = 15625; // count for 1 s
    
    IEC0bits.T2IE = 1; // enable timer interrupt
    PR2 = 2000; // count for 1 ms
    
    IEC0bits.T3IE = 1; // enable timer interrupt
    T3CONbits.TCKPS = 0b11;
    PR3 = 235; // count for 150 ms
}

void primary_loop() {
    // Create output string
    char output_str[50];

    // Add null character to the end of the output string in case other
    // functions that would add the null character fail
    output_str[49] = '\0';

    // Get sample from ADC
    get_sample();
    
    // Add the bar to the output string, using the state of current_bar as the character type
    make_bar(&output_str[0], 16, &current_bar);
    
    // Add formatted (hex or decimal) ADC_val to the output string depending on current_type
    if (current_type == HEX) sprintf(&output_str[16], " %x    ", ADC_val);
    else sprintf(&output_str[16], " %d    ", ADC_val);
    
    // Display the output string
    Disp2String(output_str);
    
    // Idle until 1 second has passed (counted by timer 1)
    wait_until_T1();
}

void make_bar(char *target, uint8_t target_len, char *bar_char) {
    uint16_t segment_num = ADC_val / 64;
    
    int i;
    for (i = 0; i < segment_num; i++) target[i] = *bar_char;
    for (; i < target_len; i++) target[i] = ' ';
}

void wait_until_T1() {
    // So long as timer 1 has not been triggered, go into Idle
    while (T1_triggered == 0) Idle();

    T1_triggered = 0;
    TMR1 = 0;
}