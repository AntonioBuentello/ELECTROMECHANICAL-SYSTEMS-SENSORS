/**
 * Antonio Buentello
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "nvic.h"

// PortA masks PA7 for PWM  (M1PWM3)
// 1b
#define PWM_MASK 128
#define PWM_MOTOR PWM1_1_CMPB_R

// Pins
#define UART_TX_1 PORTB,1
#define UART_RX_1 PORTB,0

uint32_t pwmVal = 1023;

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
    PWM1_1_CMPB_R = 900;                               // P0 50%
    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 1
    PWM1_ENABLE_R = PWM_ENABLE_PWM3EN;
}

// Initialize UART1
void initUart1(void){
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    _delay_cycles(3);

    // Configure UART0 pins
    selectPinPushPullOutput(UART_TX_1);
    selectPinDigitalInput(UART_RX_1);
    setPinAuxFunction(UART_TX_1, GPIO_PCTL_PB1_U1TX);
    setPinAuxFunction(UART_RX_1, GPIO_PCTL_PB0_U1RX);

    // Configure UART1 with default baud rate
    UART1_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (usually 40 MHz)
}

// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc){
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART1_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART1_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART1_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART1
}

// Initialize Hardware
void initHw(){
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    enablePort(PORTA);
    enablePort(PORTB);

    initUart0();
    setUart0BaudRate(115200, 40e6);

    initUart1();
    setUart1BaudRate(115200, 40e6);

}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart1(uint8_t c){
    while (UART1_FR_R & UART_FR_TXFF);               // wait if uart1 tx fifo full
    UART1_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char *str){
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
uint8_t getcUart1(void){
    while (UART1_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART1_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart1(void){
    return !(UART1_FR_R & UART_FR_RXFE);
}


void itoa_h(uint32_t num){
    char table[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    char buff[9];
    uint32_t indx = 0;
    uint8_t offset = 28;
    uint8_t i;
    for( i = 0; i < 8; i++){
        indx = (num & (0xF << offset));
        indx = indx >> offset;
        buff[i] = table[indx];
        offset -= 4;
    }
    buff[8] = '\0';

    putsUart0("0x");
    putsUart0(buff);
    putsUart0("\n");

}
void getInfo(){
    putcUart1(0xA5);
    putcUart1(0x50);
}
void getScanned(){
    putcUart1(0xA5);
    putcUart1(0x20);
}
void stopCommand(){
    putcUart1(0xA5);
    putcUart1(0x25);
    waitMicrosecond(2000);
}
typedef struct _dataRecieved{
    uint8_t quality;
    uint8_t angle_lower;
    uint8_t angle_upper;
    uint8_t distance_lower;
    uint8_t distance_upper;

}dataRecieved;

dataRecieved received[400];

int main(void){
    initHw();
    initPWM();
    char str[30];
    uint32_t i, j;
    j = 0;
    uint16_t angleInt = 0;
    uint16_t distanceInt = 0;
    float angle = 0.0;
    float distance = 0.0;
    uint8_t descriptor[7];
    uint8_t data[2000];


    for (i = 0; i < 2000; i++){
        data[i] = 0;
    }

    stopCommand();
    getScanned();

    while(1){
        j = 0;

        for (i = 0; i < 7; i++){
            descriptor[i] = getcUart1();
        }

        for (i = 0; i < 2000; i++){
            data[i] = getcUart1();
        }

        putcUart0('\n');

        PWM1_1_CMPB_R = 0;
        i = 0;

        while (i < 2000){
            received[j].quality = data[i++] ;
            received[j].angle_lower = data[i++];
            received[j].angle_upper = data[i++];
            received[j].distance_lower = data[i++];
            received[j].distance_upper = data[i++];

            angleInt = (((received[j].angle_upper << 8) | (received[j].angle_lower)) >> 1);
            angle = angleInt / 64.0;

            distanceInt = ((received[j].distance_upper << 8) | (received[j].distance_lower));
            distance = distanceInt / 4.0;

            sprintf(str, "Angle: %.4f\n", angle);
            putsUart1(str);
            sprintf(str, "Distance: %.4f\n", distance);
            putsUart1(str);
            j++;
        }
        putcUart0('\n');

    }
}
