// MOVING_FUNCTION Library
// Khaled Ahmed

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "uart0.h"
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "mov_func.h"

// Bitband aliases
#define SLEEP_PIN     (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4))) // pin 0 PB0

// Port B masks FOR DRV8833 (PWM)
#define PWM0_IN1_PIN_MASK 64  // MOPWM0 at PB6
#define PWM1_IN2_PIN_MASK 128 // MOPWM1 at PB7
#define PWM2_IN3_PIN_MASK 16  // MOPWM2 at PB4
#define PWM3_IN4_PIN_MASK 32  // MOPWM3 at PB5
#define SLEEP_PIN_MASK 1  //  PB0

// Port C masks FOR Hall sensor (DRV 5023)
#define LEFT_SENSOR_OUT_PIN_MASK 16   // WTOCCP0 at PC4
#define RIGHT_SENSOR_OUT_PIN_MASK 64  // WT1CCP0 at PC6


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint32_t Dist = 0; // forward Add Distance reverse Subtract Distance
uint32_t Total_Dist = 0; // total distance travel (forward or reverse Add Distance)
uint32_t Wheel_Radius = 3.25;
uint32_t Left_wheel_count = 0;
uint32_t Right_wheel_count = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;    // PWM Module 0
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1; // Wide Timer 1 & 0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2; // GPIO Port B & C
    _delay_cycles(3);


    // Configure GPIO pins
    // Port B
    GPIO_PORTB_DIR_R |= PWM0_IN1_PIN_MASK | PWM1_IN2_PIN_MASK | PWM2_IN3_PIN_MASK | PWM3_IN4_PIN_MASK;  // bits are outputs
    GPIO_PORTB_DIR_R &= SLEEP_PIN_MASK; // bit is input
    GPIO_PORTB_PUR_R |= SLEEP_PIN_MASK; // Pull up
    GPIO_PORTB_DR2R_R |= PWM0_IN1_PIN_MASK | PWM1_IN2_PIN_MASK | PWM2_IN3_PIN_MASK | PWM3_IN4_PIN_MASK | SLEEP_PIN_MASK; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= PWM0_IN1_PIN_MASK | PWM1_IN2_PIN_MASK | PWM2_IN3_PIN_MASK | PWM3_IN4_PIN_MASK | SLEEP_PIN_MASK;  // enable LEDs
    GPIO_PORTB_AFSEL_R |= PWM0_IN1_PIN_MASK | PWM1_IN2_PIN_MASK | PWM2_IN3_PIN_MASK | PWM3_IN4_PIN_MASK;;
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB4_M  | GPIO_PCTL_PB5_M | GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M);
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_M0PWM2 | GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB6_M0PWM0 | GPIO_PCTL_PB7_M0PWM1;
    // Port C
    GPIO_PORTC_DIR_R &= (LEFT_SENSOR_OUT_PIN_MASK | RIGHT_SENSOR_OUT_PIN_MASK);    // bits are input
    GPIO_PORTC_AFSEL_R |= LEFT_SENSOR_OUT_PIN_MASK | RIGHT_SENSOR_OUT_PIN_MASK;    // select alternative functions
    GPIO_PORTC_PUR_R |= LEFT_SENSOR_OUT_PIN_MASK | RIGHT_SENSOR_OUT_PIN_MASK;      // PUll up resistor for pin 4 & 6
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC6_M | GPIO_PCTL_PC4_M);                     // map alt fns
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0 | GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_DEN_R |= LEFT_SENSOR_OUT_PIN_MASK | RIGHT_SENSOR_OUT_PIN_MASK;      // enable bit 4 & 6 for digital input

    // Configure PWM module 1
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_0_CTL_R = 0;                                // turn-off PWM1 generator 0 (drives outs 0 and 1)
    PWM0_1_CTL_R = 0;                                // turn-off PWM1 generator 1 (drives outs 2 and 3)

    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO; //PB6  M0PWM0    // output 0 on PWM0, gen 0A, cmpa
    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO; //PB7  M0PWM1    // output 1 on PWM0, gen 0B, cmpb
    PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO; //PB4  M0PWM2    // output 2 on PWM0, gen 1A, cmpa
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO; //PB5  M0PWM3    // output 3 on PWM0, gen 1B, cmpb

    PWM0_0_LOAD_R = 1024;
    PWM0_1_LOAD_R = 1024;

    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = 0;
    PWM0_1_CMPA_R = 0;
    PWM0_1_CMPB_R = 0;

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 0
    PWM0_1_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN;
    // enable outputs

    // Configure Wide Timer 0 for left motor (Wheel) PC4
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER0_CTL_R = 0;                               //
    WTIMER0_IMR_R = 0;                               // turn-off interrupts
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R &= ~(1 << (INT_WTIMER0A-16-96));      // turn-off interrupt 112 (WTIMER1A)


    // Configure Wide Timer 1 for right motor (Wheel) PC6
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = 0;                               //
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R &= ~(1 << (INT_WTIMER1A-16-96));      // turn-off interrupt 112 (WTIMER1A)
}

void Stop()
{
    SLEEP_PIN = 0;
    PWM0_0_CMPA_R = 0;        // in1 off
    PWM0_0_CMPB_R = 0;        // in2 off
    PWM0_1_CMPA_R = 0;        // in3 off
    PWM0_1_CMPB_R = 0;        // in4 off


}

void setPWMsignal(uint16_t in1, uint16_t in2, uint16_t in3, uint16_t in4)
{
    PWM0_0_CMPA_R = in1;     // in1
    PWM0_0_CMPB_R = in2;     // in2
    PWM0_1_CMPA_R = in3;     // in3
    PWM0_1_CMPB_R = in4;     // in4
}

void forward(uint16_t dist_cm)
{

    float R = 0;        // # of Rotation for small wheel should rotate
    uint32_t left = 0;
    uint32_t right = 0;

    SLEEP_PIN = 1;

    if (dist_cm == 0)
    {
        setPWMsignal(0, 950, 1010, 0);
    }
    else
    {
        setPWMsignal(0, 950, 1010, 0);

        char str[100];
        R = ((dist_cm * 46) / (2 * 3.14159 * Wheel_Radius));

        while((left < R) || (right < R))
        {
            right =  WTIMER0_TAV_R;                   // read counter 0 input
            left  =  WTIMER1_TAV_R;                   // read counter 1 input

            if (left > R)
            {
                PWM0_0_CMPA_R = 0;     // in1 off
                PWM0_0_CMPB_R = 0;     // in2 off
            }
            if (right > R)
            {
                PWM0_1_CMPA_R = 0;     // in3 off
                PWM0_1_CMPB_R = 0;     // in4 off
            }
        }
        Left_wheel_count  +=left;
        Right_wheel_count +=right;
        Stop();
    }

    WTIMER0_TAV_R = 0;
    WTIMER1_TAV_R = 0;


}

void reverse(uint16_t dist_cm)
{

    float R = 0;        // # of Rotation for small wheel should rotate
    uint32_t left = 0;
    uint32_t right = 0;

    SLEEP_PIN = 1;
    if (dist_cm == 0)
    {
        setPWMsignal(850, 0, 0, 1020);
    }
    else
    {
        setPWMsignal(850, 0, 0, 1020);

        R = ((dist_cm * 46) / (2 * 3.14159 * Wheel_Radius));

        char str[100];
        while((left < R) || (right < R))
        {
            left  =  WTIMER0_TAV_R;                   // read counter 0 input
            right =  WTIMER1_TAV_R;                  // read counter 1 input

            if (left > R)
            {
                PWM0_0_CMPA_R = 0;     // in1 off
                PWM0_0_CMPB_R = 0;     // in2 off
            }
            if (right > R)
            {
                PWM0_1_CMPA_R = 0;     // in3 off
                PWM0_1_CMPB_R = 0;     // in4 off
            }

        }

        Left_wheel_count  -=left;
        Right_wheel_count -=right;
        Stop();
    }

    WTIMER0_TAV_R = 0;
    WTIMER1_TAV_R = 0;

}

void cw(uint16_t degrees)
{

    float R ;        // # of Rotation for small wheel should rotate
    uint32_t left = 0;
    uint32_t right = 0;

    SLEEP_PIN = 1;
    if (degrees == 0)
    {
        setPWMsignal(0, 850, 0, 1000);
    }
    else
    {
        setPWMsignal(0, 850, 0, 1000);

        // R = (the degree I need * 46 motor rotation/360) * (track width / 2 * Wheel_Radius)
        R = (degrees * 46 /360) * (18.5 / (2 * Wheel_Radius));

        while((left < R) && (right < R))
        {
            left  =  WTIMER0_TAV_R;                   // read counter 0 input
            right =  WTIMER1_TAV_R;                  // read counter 1 input

            if (left > R)
            {
                PWM0_0_CMPA_R = 0;     // in1 off
                PWM0_0_CMPB_R = 0;     // in2 off
            }
            if (right > R)
            {
                PWM0_1_CMPA_R = 0;     // in3 off
                PWM0_1_CMPB_R = 0;     // in4 off
            }
        }

        Left_wheel_count  +=left;
        Right_wheel_count -=right;
        Stop();
    }

    WTIMER0_TAV_R = 0;
    WTIMER1_TAV_R = 0;

}

void ccw(uint16_t degrees)
{

    uint32_t R = 0;        // # of Rotation for small wheel should rotate
    uint32_t left = 0;
    uint32_t right = 0;

    SLEEP_PIN = 1;
    if (degrees == 0)
    {
        setPWMsignal(800, 0, 1020, 0);
    }
    else
    {
        setPWMsignal(800, 0, 1020, 0);

        R = (degrees * 46 /360) * (18.5 / (2 * Wheel_Radius));

        while((left < R) && (right < R))
        {
            left  =  WTIMER0_TAV_R;                   // read counter 0 input
            right =  WTIMER1_TAV_R;                  // read counter 1 input

            if (left > R)
            {
                PWM0_0_CMPA_R = 0;     // in1 off
                PWM0_0_CMPB_R = 0;     // in2 off
            }
            if (right > R)
            {
                PWM0_1_CMPA_R = 0;     // in3 off
                PWM0_1_CMPB_R = 0;     // in4 off
            }
        }

        Left_wheel_count  -=left;
        Right_wheel_count +=right;
        Stop();
    }

    WTIMER0_TAV_R = 0;
    WTIMER1_TAV_R = 0;

}
