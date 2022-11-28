
/*
 * Antonio Buentello
 * 1001337439
 */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "uart0.h"
#include "wait.h"
#include "gpio.h"
#include "clock.h"
#include "tm4c123gh6pm.h"

uint32_t frequency = 0;
uint32_t time = 0;

// PC6
#define FREQ_IN_MASK 64
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3

void enableCounterMode(){
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;             // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;       // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;      // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;                   // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;             // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;              // turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);         // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;           // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                    // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; //configure for edge count mode, count up
    WTIMER1_CTL_R = 0;                               //
    WTIMER1_IMR_R = 0;                           // turn-off interrupts
    WTIMER1_TAV_R = 0;                          // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;            // turn-on counter
}
// Frequency counter service publishing latest frequency measurements every second
void timer1Isr(){
    frequency = WTIMER1_TAV_R;                   // read counter input
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}

// Period timer service publishing latest time measurements every positive edge
void wideTimer1Isr(){
    time = WTIMER1_TAV_R;                        // read counter input
    WTIMER1_TAV_R = 0;                           // zero counter for next edge
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void initHw(){
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    enablePort(PORTF);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(GREEN_LED);

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input
}
int main(void){
    // Initialize hardware
    initHw();
    initUart0();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    enableCounterMode();

    char str[10];

    while(1){
        putsUart0("Frequency: ");
        sprintf(str, "%7lu", frequency);
        putsUart0(str);
        putsUart0(" (Hz)\n");

        if (frequency >= 105000){
        
            setPinValue(GREEN_LED,1);
        }
        else{
            setPinValue(GREEN_LED,0);

        }
        waitMicrosecond(100000);
    }

	return 0;
}
