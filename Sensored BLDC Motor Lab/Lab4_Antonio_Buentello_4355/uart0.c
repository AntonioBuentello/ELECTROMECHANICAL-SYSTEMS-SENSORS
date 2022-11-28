// UART0 Library

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "gpio.h"

// Pins
#define UART_TX PORTA,1
#define UART_RX PORTA,0

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

/*
 * reentrant atoi function
 * (*p) - '0' is subtracting value of char '0' from char pointed to be p
 * this turns it into a number
 */
int r_atoi(char *ptr) {

    int value = 0;
    bool negFlag = false;

    if (*ptr && *ptr == '-')        // checks for negative numbers, set the flag if so
    {
        negFlag = true;
        ptr++;
    }

    while (*ptr) {

        value = (value * 10) + (*ptr) - '0';
        ptr++;
     }

    if (negFlag)                    // makes the value negative
    {
        value *= -1;
    }
    return value;
}

/*
   Function to receive chars from the UI, processing special chars such as backspace
   and writing the resultant string into the buffer
   Backspace = 8, DEL = 127
*/
void getsUart0 (USER_DATA *data)
{
    char c;
    uint8_t count = 0;

    while (true)
    {
        c = getcUart0();                                 // get a char and put in buffer
        if ((c == 8 || c == 127) && (count > 0))         // remove backspace char and check if backspace is the first char
        {
            count--;
        }
        else if (c == 13 || c == 10)                     // check if <enter key> was pressed
        {
            data->buffer[count] = '\0';
            break;
        }
        else if (c >= 32)                                // check if <space> or any printable char is pressed
        {
            data->buffer[count++] = c;

            if (count == MAX_CHARS)                      // program will exit if max char are input
            {
                data->buffer[count] = '\0';
                break;
            }
        }
    }
}

/*
 * LETTER A : 65 , Z : 90 , a : 97 , z : 122
 * NUMBER 0 : 48 , 9 : 57, includes (-)
 * Everything else is a delimiter
 */
void parseFields (USER_DATA *data)
{
    char previous = 'd';

    data->fieldCount = 0;

    uint8_t count = 0;
    uint8_t index = 0;

    while (data->buffer[count] != '\0')
    {
        char c = data->buffer[count];

        // exit the loop if we already have our max fields
        if ( data->fieldCount == MAX_FIELDS )
        {
            break;
        }

        // check if it's an alpha
        //&& ( (previous == 'd') || (previous == 'n') )
        else if ( ( (c >= 65 && c <= 90) || (c >= 97 && c <= 122) ) )
        {
            if (previous == 'a')
            {
                count++;
                continue;
            }
            data->fieldCount++;
            data->fieldType[index] = 'a';
            data->fieldPosition[index++] = count;
            previous = 'a';
        }

        //check if its numeric
        // || (previous == 'a') && ( (previous == 'd') )
        else if ( (c >= 48 && c <= 57) || (c == '-') )
        {
            if (previous == 'n')
            {
                count++;
                continue;
            }
            data->fieldCount++;
            data->fieldType[index] = 'n';
            data->fieldPosition[index++] = count;
            previous = 'n';
        }

        // otherwise, it's a delimiter
        else
        {
            previous = 'd';
            data->buffer[count] = '\0';
        }

        count++;
    }
}


/*
 *  Returns the value of a field requested if the field
 *  is in range or NULL otherwise.
 *  returns the address of
 */
char *getFieldString (USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber <= data->fieldCount)
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return '\0';
    }
}

/*
 * Function to return the integer value of the field if the
 * field number is in range and the field type is numeric or 0 otherwise.
 */
int32_t getFieldInteger (USER_DATA *data, uint8_t fieldNumber)
{
    if ( (fieldNumber <= data->fieldCount) && (data->fieldType[fieldNumber] == 'n') )
    {
        return r_atoi( &data->buffer[ data->fieldPosition[ fieldNumber ] ] );
    }
    else
    {
        return 0;
    }
}

int strCmp (const char *str1, const char *str2)
{
    while ( *str1 && ( (*str1 == *str2)  || (*str1+32 == *str2) || (*str1-32 == *str2) ))
    {
        str1++;
        str2++;
    }
    return *(const unsigned char*)str1 - *(const unsigned char*)str2;
}

 /*
  * Returns true if the command matches the first field
  * and the number of arguments (excluding the command field) is greater
  * than or equal to the requested number of minimum arguments.
  */
bool isCommand (USER_DATA *data, const char strCommand[], uint8_t minArguements)
{
    uint8_t fieldNums = data->fieldCount;

    if (fieldNums-1 >= minArguements && ( strCmp(data->buffer, strCommand) == 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Initialize UART0
void initUart0(void)
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    _delay_cycles(3);
    enablePort(PORTA);

    // Configure UART0 pins
    selectPinPushPullOutput(UART_TX);
    selectPinDigitalInput(UART_RX);
    setPinAuxFunction(UART_TX, GPIO_PCTL_PA1_U0TX);
    setPinAuxFunction(UART_RX, GPIO_PCTL_PA0_U0RX);

    // Configure UART0 with default baud rate
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (usually 40 MHz)
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0(void)
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0(void)
{
    return !(UART0_FR_R & UART_FR_RXFE);
}
