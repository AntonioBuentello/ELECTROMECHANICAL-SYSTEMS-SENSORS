/**
 * Antonio Buentello
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "clock.h"
#include "nvic.h"

#define CLK PORTF,1
#define DATA PORTF,2

int32_t dataIn = 0;

uint32_t array[16];

void initHw()
{
    initSystemClockTo40Mhz();
    initUart0();
    setUart0BaudRate(115200, 40e6);

    enablePort(PORTF);

    selectPinPushPullOutput(CLK);

    disablePinInterrupt(DATA);
    selectPinDigitalInput(DATA);
    enablePinPulldown(DATA);
    selectPinInterruptFallingEdge(DATA);
    enableNvicInterrupt(INT_GPIOF);
    enablePinInterrupt(DATA);
}

void dataIsr(void)
{
    uint8_t i;

    dataIn = 0;
    char str[10];

    for(i = 0; i < 24; i++)
    {
        waitMicrosecond(1);
        setPinValue(CLK, 1);
        waitMicrosecond(1);
        dataIn <<= 1;
        dataIn |= getPinValue(DATA);
        setPinValue(CLK, 0);
    }
    waitMicrosecond(1);
    setPinValue(CLK, 1);
    waitMicrosecond(1);
    setPinValue(CLK, 0);
    clearPinInterrupt(DATA);
}


int main(void)
{

    initHw();
    char str[35];
    uint32_t sum = 0;
    uint8_t index = 0;
    uint32_t average= 0;
    uint8_t i;

    float force=0;
    float weight = 0;

    for (i = 0; i < 16; i++)
        array[i] = 0;

    while(1)
    {
        waitMicrosecond(100000);

        sum -= array[index];
        sum += dataIn;
        array[index] = dataIn;
        index = (index + 1) & 15;

        average = (sum >> 4);

        putsUart0("RAW: ");
        sprintf(str, "%d", dataIn);
        putsUart0(str);
        putsUart0("\n");

        putsUart0("Filtered: ");
        sprintf(str, "%d", average);
        putsUart0(str);
        putsUart0("\n");


        weight = (-0.0167 * average) + 203404;

        putsUart0("Calculated Weight (Grams): ");
        sprintf(str, "%.4f", weight);
        putsUart0(str);
        putsUart0("\n");

        force = (9.81 * (weight / 1000));

        putsUart0("Calculated FORCE (N): ");
        sprintf(str, "%.4f", force);
        putsUart0(str);
        putsUart0("\n\n");


    }

	return 0;
}
