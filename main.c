// Khaled Ahmed

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "mov_func.h"
#include "tm4c123gh6pm.h"

//Bitband aliases
#define DATA_IN_PIN         (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000)*32 + 4*4))) // pin 4
#define RED_LED             (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000)*32 + 1*4))) // pin 1
#define BLUE_LED            (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000)*32 + 2*4))) // pin 2
#define GREEN_LED           (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000)*32 + 3*4))) // pin 3


// PortF masks
#define DATA_IN_PIN_MASK 16
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8

//Global variables
uint32_t time[50];
uint8_t  code;
uint8_t  count =  0;
bool     valid = false;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHwIr(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks for Port F, Timer 1 and, wide timer 1
    SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGCGPIO_R5;    // port F
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2; // wide timer 2
    _delay_cycles(3);

    // Configure GPIO pins
    GPIO_PORTF_DIR_R  |= BLUE_LED_MASK | GREEN_LED_MASK | RED_LED_MASK;                      // bit 2 output
	GPIO_PORTF_DIR_R  &= ~DATA_IN_PIN_MASK;                                                  // bit 1 an input
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK | GREEN_LED_MASK | RED_LED_MASK;                      // set drive strength to 2mA
    GPIO_PORTF_DEN_R  |= BLUE_LED_MASK | DATA_IN_PIN_MASK  | GREEN_LED_MASK | RED_LED_MASK;  // enable Digital

    // Configure falling edge
    GPIO_PORTF_PUR_R |= DATA_IN_PIN_MASK;
    GPIO_PORTF_IS_R  &= ~DATA_IN_PIN_MASK;
    GPIO_PORTF_IBE_R &= ~DATA_IN_PIN_MASK;
    GPIO_PORTF_IEV_R &= ~DATA_IN_PIN_MASK;
    GPIO_PORTF_ICR_R |= DATA_IN_PIN_MASK;
    GPIO_PORTF_IM_R  |= DATA_IN_PIN_MASK;
    NVIC_EN0_R       |= 1 << (INT_GPIOF-16);

    // Configure wide timer 1 as free running timer
     WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
     WTIMER2_CFG_R = 4;                              // configure as 32-bit counter (A only)
     WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
     WTIMER2_CTL_R = TIMER_CTL_TAEVENT_NEG;           // measure time from negative edge to negative edge
     WTIMER2_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
     WTIMER2_TAV_R = 0;                               // zero counter for first period
     WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
     NVIC_EN3_R |= 1 << (INT_WTIMER2A-16-96);         // turn-on interrupt 112 (WTIMER1A)

}

void gpioFIsr()
{

    if (count <= 35)
    {
        time[count] = WTIMER2_TAV_R;

		if (count == 0)
		{
			count++;
		}
	    else if (count == 1 && (time[1]-time[0] > 520000) && (time[1]-time[0] < 560000))
		{
			count++;
		}
		else
		{
		   if (((time[count] - time[count-1]) > (1.5*(22500))) && ((time[count] - time[count-1]) < (2.5*(22500))))
			{
	 			count++;
			}
		    else if (((time[count] - time[count-1]) > (3.5*(22500))) && ((time[count] - time[count-1]) < (4.5*(22500))))
			{
	 			count++;
			}
			else
			{
				count = 0;
			}
		}
	}

	 if (count == 34)
	 {
	 	count = 0;
	 	WTIMER2_TAV_R = 0;

	 	uint8_t i;
	 	uint32_t value = 0;
	 	uint8_t firstbits = 0;
	 	uint8_t notfirstbits = 0;

	 	for(i = 0; i < 34; i++)
	 	{
    		if(((time[i+1] - time[i]) > (1.5*(22500))) && ((time[i+1] - time[i]) < (2.5*(22500)))) //0
	 		{
                value = value << 1;
	 		}
	 	    else if(((time[i+1] - time[i]) > (3.5*(22500))) && ((time[i+1] - time[i]) < (4.5*(22500))))//1
	 		{
	 		   value  = value << 1;
	 		   value |= 1;
	 		}
	 	}

	 		firstbits    = ((value >> 24) & 0xFF);  // Address Bits (A)
	 		notfirstbits = ~((value >> 16) & 0xFF); // NOT Address bits (~A)

	 		if (firstbits == notfirstbits)
	 		{
	 		    firstbits    = ((value >> 8) & 0xFF);  // Data bits (D)
	 		    notfirstbits = ~((value ) & 0xFF);     // NOT Data bits (~D)

	 		    if (firstbits == notfirstbits)
	 		    {
	 		        code = firstbits;
	 		        valid = true;
	 		    }
	 		}
	 	}

	 	GPIO_PORTF_ICR_R |= DATA_IN_PIN_MASK;               // clear interrupt flag for port F
}

int main(void)
{
    // Initialize hardware
    initHwIr();
    initHw();
    initUart0();

    USER_DATA data;
    char* command;
    uint8_t arg;
    uint8_t key;
    uint8_t i;

    char str[100];
    char c;
    RED_LED   = 0;
    GREEN_LED = 0;
    BLUE_LED  = 0;

    uint8_t remote_Data [48] = {  0b00010000, 0b11010000, 0b10011100, 0b01011110, 0b00011110, 0b11110000, 0b10001000, 0b01001000,
                                  0b11001000, 0b00101000, 0b10101000, 0b01101000, 0b11101000, 0b00011000, 0b10011000, 0b00110010,
                                  0b00001000, 0b01011000, 0b01000000, 0b11000000, 0b01111000, 0b01010101, 0b10010000, 0b00000000,
                                  0b10000000, 0b01101010, 0b00111110, 0b00111010, 0b11000010, 0b00000010, 0b11110101, 0b11100000,
                                  0b00100010, 0b01100000, 0b00010100, 0b10000010, 0b11011010, 0b10001001, 0b01110000, 0b10001101,
                                  0b11110001, 0b00001101, 0b01011101, 0b01110001, 0b01001110, 0b10001110, 0b11000110, 0b10000110  };


    char * button_Name[50] = {  "Start/Stop", "input", "CAPTION", "?", "Search", "TV", "1", "2", "3", "4", "5", "6", "7","8", "9",
                                "LIST", "0", "FLASHBK", "VOL UP", "VOL DOWN", "FAV", "INFO", "MUTE", "CH UP", "CH DOWN", "NETFLIX",
                                "HOME", "AMAZON", "SETTING", "UP", "LIVE ZOOM", "LIFT", "OK", "RIGHT", "BACK", "DOWN", "EXIT", "SAP/*",
                                "SLEEP", "STOP (square)", "<<", "PAUSE (Triangle)", "||", ">>", "RED", "GREEN", "YELLOW", "BLUE" };
    putsUart0("Enter: ");

    while(true)
    {

        if (kbhitUart0()) // UART0 command
        {
            putsUart0("Enter string: ");
            getsUart0(&data);
            parseFields(&data);



            if (isCommand(&data, "forward", 1) )
            {
                int32_t dist = getFieldInteger(&data, 1);
                forward(dist);

            }

            if (isCommand(&data, "reverse", 1) )
            {
                int32_t dist = getFieldInteger(&data, 1);
                reverse(dist);

            }

            if (isCommand(&data, "ccw", 1) )
            {
                int32_t degree = getFieldInteger(&data, 1);
                ccw(degree);

            }

            if (isCommand(&data, "cw", 1) )
            {
                int32_t degree = getFieldInteger(&data, 1);
                cw(degree);

            }

            if (strcompare(data.buffer,"stop"))
            {
                Stop();
            }
        }
        else if (valid) // IR receiver command
        {

            for ( i = 0; i < 48; i++)
            {
                if (remote_Data[i] == code)
                {
                     c = button_Name[i][0];
                    if (code == 0b10001101) // stop
                    {
                        Stop();
                    }
                    else if (code == 0b00000010) // forward ^ (arrow up button)
                    {
                        command = "forward";
                        putsUart0("forward \n");
                        arg = 0;
                    }
                    else if (code == 0b10000010) // reverse !^ (arrow down  button)
                    {
                        command = "reverse";
                        putsUart0("reverse \n");
                        arg = 0;
                    }
                    else if (code == 0b01100000) // cw > (right arrow button)
                    {
                        command = "cw";
                        putsUart0("cw \n");
                        arg = 0;
                    }
                    else if (code == 0b11100000) // ccw < (left arrow button)
                    {
                        command = "ccw";
                        putsUart0("ccw \n");
                        arg = 0;
                    }
                    else if (c > 47 && c < 58) // is the arg between 0-9
                    {
                        arg *= 10;
                        key  = c - 48;
                        arg  = arg + key;     //arg = arg + my key;
                        sprintf(str,"%3u",arg);
                        putsUart0(str);
                    }
                    else  if (code == 0b01001110) // Red LED
                    {
                        RED_LED ^= 1;
                        putsUart0("RED_LED \n");
                    }
                    else  if (code == 0b10000110) // Blue LED
                    {
                        BLUE_LED ^= 1;
                        putsUart0("BLUE_LED \n");
                    }
                    else  if (code == 0b10001110)// Green LED
                    {
                        GREEN_LED ^= 1;
                        putsUart0("GREEN_LED \n");
                    }
                    else  if (code == 0b11000110)//  Yellow LED
                    {
                        if ((GPIO_PORTF_DATA_R == GREEN_LED_MASK) || (GPIO_PORTF_DATA_R == RED_LED_MASK))
                        {
                            GREEN_LED = 0;
                            RED_LED   = 0;
                        }

                        GREEN_LED ^= 1;
                        RED_LED ^= 1;
                        putsUart0("YELLOW_LED \n");
                    }
                    else if (code == 0b00100010) // ok OK (ok button)
                    {
                        putsUart0("\nOK \n");
                        if (strcompare(command,"forward"))
                        {
                            forward(arg);
                        }
                        else if (strcompare(command,"reverse"))
                        {
                            reverse(arg);
                        }
                        else if (strcompare(command,"cw"))
                        {
                            cw(arg);
                        }
                        else if (strcompare(command,"ccw"))
                        {
                            ccw(arg);
                        }
                    }
                    valid = false;
                    break;
                }
            }
        }
    }
}
