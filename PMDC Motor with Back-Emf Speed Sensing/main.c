/*
* Antonio Buentello
*/

//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

// Frequency counter and timer input:
//   SIGNAL_IN on PC6 (WT1CCP0)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "adc0.h"
#include "nvic.h"

//Analog AIN3/PE0
#define AIN3_MASK 1

// PortC masks PC6 SIGNAL_IN on PC6 (WT1CCP0)
#define FREQ_IN_MASK 64

// 50hz timer (T3CCP0) PB2
#define FIFTY_TIME 4

//switches
#define SW1 PORTF,4
#define SW2 PORTF,0

// PortA masks PA7 for PWM  (M1PWM3)
// 1b
#define PWM_MASK 128
#define PWM_MOTOR PWM1_1_CMPB_R

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint32_t pwmVal = 512;
uint32_t frequency = 0;
uint32_t time = 0;
uint16_t rpm = 1;
uint16_t rawAnalog = 0;
uint16_t backEmfRpm = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initPWM(){
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure PWM
    GPIO_PORTA_DEN_R |= PWM_MASK;
    GPIO_PORTA_AFSEL_R |= PWM_MASK;
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA7_M);
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA7_M1PWM3;

    // Configure PWM module 1 to drive
    // MotorPWM    M1PWM3 (PA7), M1PWM1b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state

    PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 1
    PWM1_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    PWM1_1_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_1_CMPB_R = 512;                               // P0 50%
    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 1
    PWM1_ENABLE_R = PWM_ENABLE_PWM3EN;
}

void enablefiftyTimer(){
    // Configure Timer 1 as the time base
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;             // turn-off timer before reconfiguring
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;       // configure as 32-bit timer (A+B)
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;      // configure for periodic mode (count down)
    TIMER3_TAILR_R = 800000;                   // set load value to 40e6 for 1 Hz interrupt rate
    TIMER3_IMR_R = TIMER_IMR_TATOIM;             // turn-on interrupts
    TIMER3_CTL_R |= TIMER_CTL_TAEN;              // turn-on timer
    //INT_TIMER3A
    enableNvicInterrupt(INT_TIMER3A);
    enablePinInterrupt(PORTB,2);
}

void enableCounterMode(){
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = 0;                               //
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
}

void fiftyTimerIsr(){
    PWM_MOTOR = 0;
    waitMicrosecond(200);
    rawAnalog = readAdc0Ss3();
    PWM_MOTOR = pwmVal;
    TIMER3_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
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

// Initialize Hardware
void initHw(){
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2;
    initAdc0Ss3();
    initUart0();

    enablePort(PORTB);
    enablePort(PORTA);
    enablePort(PORTC);
    enablePort(PORTE);
    enablePort(PORTF);
    enablePort(PORTD);
    _delay_cycles(3);

    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN3_MASK;                 // select alternative functions for AN3 (PE0)
    GPIO_PORTE_DEN_R &= ~AIN3_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN3_MASK;                 // turn on analog operation on pin PE0

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Use AIN3 input with N=4 hardware sampling
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(64);

    // Configure pushbutton pins
    setPinCommitControl(SW2);
    enablePinPullup(SW2);
    enablePinPullup(SW1);

    selectPinDigitalInput(SW2);
    selectPinDigitalInput(SW1);
    selectPinPushPullOutput(PORTC,6);

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTB_AFSEL_R |= FIFTY_TIME;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB2_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB2_T3CCP0;
    GPIO_PORTB_DEN_R |= FIFTY_TIME;                // enable bit 6 for digital input
}

int main(void){
    initHw();
    initPWM();
    enableCounterMode();
    enablefiftyTimer();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    pwmVal = PWM_MOTOR;

    // Endless loop performing multiple tasks
    char str[10];
    while (true){
        // R(Vin) = floor(Vin/3.3V * 4096) -> Vin(R) ~= 3.3V * ((R+0.5) / 4096)
        // display freq value/ backemf derived rpm/ pwm %
        if (pwmVal >= 1024){
            pwmVal = 0;
        }

        if(!getPinValue(SW2) && pwmVal <= 1024){
            pwmVal += 32;
            PWM_MOTOR = pwmVal;
        }

        if (!getPinValue(SW1) && pwmVal <= 1024){
            pwmVal -= 32;
            PWM_MOTOR = pwmVal;
        }

        putsUart0("Frequency: ");
        sprintf(str, "%7lu", frequency);
        putsUart0(str);
        putsUart0(" (Hz)\n");

        putsUart0("RPM: ");
        rpm = ((frequency * 60) / 32);
        sprintf(str, "%7lu", rpm);
        putsUart0(str);
        putsUart0("\n");

        // y = -0.9359x + 1821.8
        putsUart0("Back-emf RPM: ");
        backEmfRpm = -0.9359 * rawAnalog + 1821;
        sprintf(str, "%7lu", backEmfRpm);
        putsUart0(str);
        putsUart0("\n");

        putsUart0("Analog: ");
        sprintf(str, "%7lu", rawAnalog);
        putsUart0(str);
        putsUart0("\n");

        putsUart0("PWM: ");
        sprintf(str, "%7lu", pwmVal);
        putsUart0(str);
        putsUart0("\n\n");
        waitMicrosecond(50000);
    }
}

