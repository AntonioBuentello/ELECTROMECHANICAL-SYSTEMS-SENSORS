/*
 * Antonio Buentello
 * Lab 4 BLDC Motor
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "wait.h"
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "nvic.h"
#include "uart0.h"

// Motor driver pins, not actual pwm
#define pwm1 PORTD,6
#define en1 PORTD,7
#define pwm2 PORTE,0
#define en2 PORTE,1
#define pwm3 PORTE,2
#define en3 PORTE,3

// PortC masks PC6 SIGNAL_IN on PC6 (WT1CCP0)
#define FREQ_IN_MASK 64

// Hall effect sensors
// PB4, PB5, PB6
#define HE1 PORTB,4
#define HE2 PORTB,5
#define HE3 PORTB,6

//switches
#define SW1 PORTF,4
#define SW2 PORTF,0


bool hall1;
bool hall2;
bool hall3;
// PC6 SIGNAL_IN on PC6 (WT1CCP0)

uint8_t phase = 0;          // Current electrical phase the motor is in
uint8_t inputPhase = 0;     // Phase to be applied on the motor
uint32_t timing = 10000;
void setElectricalPhase(uint8_t input);

uint32_t frequency = 0;
uint32_t time = 0;
uint16_t rpm = 1;

volatile uint32_t waitTiming = 1;

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
    //enablePinInterrupt(PORTC,6);
}

// Frequency counter service publishing latest frequency measurements every second
void timer1Isr(){
    frequency = WTIMER1_TAV_R;                   // read counter input
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}

void wideTimer1Isr(){
    time = WTIMER1_TAV_R;                        // read counter input
    WTIMER1_TAV_R = 0;                           // zero counter for next edge
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void hallIsr(void){

    hall1 = getPinValue(HE1);
    hall2 = getPinValue(HE2);
    hall3 = getPinValue(HE3);

    if ((hall1 && hall3) && !hall2){
        setElectricalPhase(1);
        //step_CW();
    }
    else if (!hall1 && !hall2 && hall3){
        setElectricalPhase(2);
    }
    else if (!hall1 && hall2 && hall3){
        setElectricalPhase(3);
    }
    else if (!hall1 && hall2 && !hall3){
        setElectricalPhase(4);
    }
    else if (hall1 && hall2 && !hall3){
        setElectricalPhase(5);
    }
    else if (hall1 && !hall2 && !hall3){
        setElectricalPhase(0);
    }
    clearPinInterrupt(HE1);
    clearPinInterrupt(HE2);
    clearPinInterrupt(HE3);
}

void initHw(void){
    initSystemClockTo40Mhz();

    // Enable clocks

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;


    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTF);
    enablePort(PORTE);
    _delay_cycles(3);

    // Configure pushbutton pins
    setPinCommitControl(SW2);
    enablePinPullup(SW2);
    enablePinPullup(SW1);

    selectPinDigitalInput(SW2);
    selectPinDigitalInput(SW1);

    setPinCommitControl(en1);

    selectPinPushPullOutput(en1);
    selectPinPushPullOutput(pwm1);
    selectPinPushPullOutput(pwm2);
    selectPinPushPullOutput(en2);
    selectPinPushPullOutput(pwm3);
    selectPinPushPullOutput(en3);

    selectPinDigitalInput(HE1);
    selectPinDigitalInput(HE2);
    selectPinDigitalInput(HE3);

    disablePinInterrupt(HE1);
    disablePinInterrupt(HE2);
    disablePinInterrupt(HE3);

    selectPinInterruptBothEdges(HE1);
    selectPinInterruptBothEdges(HE2);
    selectPinInterruptBothEdges(HE3);

    enableNvicInterrupt(INT_GPIOB);

    enablePinInterrupt(HE1);
    enablePinInterrupt(HE2);
    enablePinInterrupt(HE3);

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input
}

void setElectricalPhase(uint8_t input){
    waitMicrosecond(waitTiming);
    switch (input){
            case 0:
                setPinValue(pwm1, 1);
                setPinValue(en1, 1);
                setPinValue(pwm2, 0);
                setPinValue(en2, 1);
                setPinValue(pwm3, 0);
                setPinValue(en3, 0);
                break;
            case 1:
                setPinValue(pwm1, 0);
                setPinValue(en1, 0);
                setPinValue(pwm2, 0);
                setPinValue(en2, 1);
                setPinValue(pwm3, 1);
                setPinValue(en3, 1);
                break;
            case 2:
                setPinValue(pwm1, 0);
                setPinValue(en1, 1);
                setPinValue(pwm2, 0);
                setPinValue(en2, 0);
                setPinValue(pwm3, 1);
                setPinValue(en3, 1);
                break;
            case 3:
                setPinValue(pwm1, 0);
                setPinValue(en1, 1);
                setPinValue(pwm2, 1);
                setPinValue(en2, 1);
                setPinValue(pwm3, 0);
                setPinValue(en3, 0);
                break;
            case 4:
                setPinValue(pwm1, 0);
                setPinValue(en1, 0);
                setPinValue(pwm2, 1);
                setPinValue(en2, 1);
                setPinValue(pwm3, 0);
                setPinValue(en3, 1);
                break;
            case 5:
                setPinValue(pwm1, 1);
                setPinValue(en1, 1);
                setPinValue(pwm2, 0);
                setPinValue(en2, 0);
                setPinValue(pwm3, 0);
                setPinValue(en3, 1);
                break;
        }
        phase = input;
}
void step_CW(){

        if (phase == 6){
            phase = 0;
        }
        else{
            phase = (phase + 1) % 6;
        }
        setElectricalPhase(phase);
}
int main(void){
    char str[10];
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    enableCounterMode();

    //bool flag = true;
    phase = 0;
    step_CW();

    while(1){
	
        if (waitTiming < 1 || waitTiming > 1000000){
            waitTiming = 1;
        }


        if(!getPinValue(SW2) && waitTiming <= 1000000){
            waitTiming += 100;
        }

        if (!getPinValue(SW1) && waitTiming <= 1000000){
            waitTiming -= 100;
        }
        putsUart0("Frequency: ");
        sprintf(str, "%7lu", frequency / 2);
        putsUart0(str);
        putsUart0(" (Hz)\n");

        waitMicrosecond(10000);
        //(Hz x 60 x 2) / number of poles = no-load RPM

        putsUart0("RPM: ");
        rpm = ((frequency * 60) / 4 );
        sprintf(str, "%7lu", rpm);
        putsUart0(str);
        putsUart0("\n");

        putsUart0("WaitTime: ");
        sprintf(str, "%7lu", waitTiming);
        putsUart0(str);
        putsUart0("\n");



    }
}
