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

#ifndef MOVFUN_H_
#define MOVFUN_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void initHw();
void Stop();
void forward(uint16_t dist_cm);
void reverse(uint16_t dist_cm);
void cw(uint16_t degrees);
void ccw(uint16_t degrees);
void setPWMsignal(uint16_t in1, uint16_t in2, uint16_t in3, uint16_t in4);

#endif
