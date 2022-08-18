// UART0 Library
// Khaled Ahmed

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

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128 + 1) >> 1) & 63;   // set fractional value to round(fract(r)*64)
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

// Blocking function that writes a string
void getsUart0(USER_DATA* data)
{
    char c;
    uint8_t count = 0;

    while (true)
    {
        c = getcUart0();

        if ((c == 8 || c == 127) && count > 0 )
        {
            count--;
        }
        else if (c == 13 || c == 8)
        {
            data->buffer[count] = '\0';
            return;
        }
        else if (c >= 32)
        {
            if (count == MAX_CHARS)
            {
                data->buffer[count] = '\0';
                return;
            }
            else
            {
                data->buffer[count] = c;
                count++;
            }
        }
    }
}

// Blocking function that parse  a string
void parseFields(USER_DATA *data)
{
    char c;
    uint8_t position = 0;
    uint8_t alphaNum = 0;
    uint8_t index = 0;

    c = data->buffer[index];
    data->fieldCount = 0;
    if (((c > 47 && c < 58) || (c > 64 && c < 91) || (c > 96 && c < 123)))
    {
        alphaNum = 1;
    }

    while (data->buffer[index] != '\0')
    {

        if (c > 47 && c < 58)
        {
            if ((alphaNum == 1) && (data->fieldCount < MAX_FIELDS))
            {
                data->fieldType[position] = 'n';
                data->fieldPosition[position] = index;
                data->fieldCount++;
                alphaNum = 0;
                position++;
            }
        }
        else if (((c > 64 && c < 91) || (c > 96 && c < 123)) && (data->fieldCount < MAX_FIELDS))
        {
            if (alphaNum == 1)
            {
                data->fieldType[position] = 'a';
                data->fieldPosition[position] = index;
                data->fieldCount++;
                alphaNum = 0;
                position++;
            }
        }
        else
        {
            alphaNum = 1;
            data->buffer[index] = '\0';
        }

        index++;
        c = data->buffer[index];
    }
}

//Blocking function to return the value of a field requested
char* getFieldString(USER_DATA* data,uint8_t fieldNumber)
{

    if ((fieldNumber < data->fieldCount) && (data->fieldType[fieldNumber] == 'a'))
    {
        return &(data->buffer[data->fieldPosition[fieldNumber]]);
    }
    else
    {
        return 0;
    }
}

//Blocking function to return  pointer of a field requested
uint32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if ((fieldNumber < data->fieldCount) && (data->fieldType[fieldNumber] == 'n'))
    {
        uint8_t i = 0;
        uint32_t sum = 0;

        i = 0;
        while(data->buffer[data->fieldPosition[fieldNumber]+i] != '\0')
        {
            if(data->buffer[data->fieldPosition[fieldNumber]+i] < 48 ||
                    data->buffer[data->fieldPosition[fieldNumber]+i] > 57)
            {
                return 0;
            }
            else
            {
                sum = sum*10 + (data->buffer[data->fieldPosition[fieldNumber]+i] - 48);
                i++;
            }

        }

        return sum;
    }
    else
    {
        return 0;
    }
}

// Blocking function that compare two string
bool strcompare (char* str1, const char str2[])
{
    uint8_t i = 0;
    uint8_t count1 = 0;
    uint8_t count2 = 0;
    bool equal = true;

    while (str1[i] != '\0')
    {
        count1++;
        i++;
    }

    i=0;

    while (str2[i] != '\0')
    {
        count2++;
        i++;
    }

    if (count1 == count2)
    {
        for (i = 0; i < count2; i++)
        {
            if (str1[i] != str2[i])
            {
                equal = false;
                break;
            }
        }
    }
    else
    {
        equal = false;
    }

    return equal;
}

// Blocking function that check if a valid command
bool isCommand(USER_DATA* data, const char strCommand[],uint8_t minArguments)
{
    bool Command;

    if (minArguments <= data->fieldCount - 1) // -1 for the command in arg 0
    {
        if (strcompare(data->buffer,strCommand))
        {
            Command = 1;
        }
        else
        {
            Command = 0;
        }
    }
    else
    {
        Command = 0;
    }


    return Command;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}
