/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file encoder.c
 *
 * Code for encoder setup and position acquisition. Contains gray code converter code from Peter Dannegger from Mikrocontroller.net, for glitchless encoder position evaluation
 *  \version 1.0
 *  \author Julio Sagardoy
 */

/// Standard includes
#include <stdlib.h>

/// AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/// Own includes
#include "encoder.h"
#include "utils.h"

#define PHASE_A BITREAD(PIND,PD7)
#define PHASE_B BITREAD(PINE,PE6)

volatile int8_t enc_delta;	// Contains the difference between the last sample and the current sample in binary, if any
volatile int16_t enc_accum;	// Contains the absolute count of steps
static int16_t last;		// Contains last sampling binary values

/** Resets encoder counter to 0. Mandatory to run this after calibration, so it counts from 0.
*/
void encoder_reset()
{
	enc_accum = 0;
	enc_delta = 0;
	last = 0;
}

/** Returns actual value in the counter.
*/
int16_t encoder_read()
{
	return ~enc_accum;
}

ISR( TIMER3_COMPA_vect )
{
	int8_t new, diff;
	
	// convert from gray to binary, code by Peter Dannegger
	new = 0;
	if( PHASE_A )
		new = 3;
	if( PHASE_B )
		new ^= 1;

	diff = last - new;
	
	// See if there is difference with last
	if(diff & 1)	//	Bit 0 = value
	{
		last = new;			//Store new as next last
		enc_delta += (diff & 2) - 1;	//BIt 1 = direction. enc_delta: 1/-1
		enc_accum += enc_delta;
		enc_delta = 0;
	}
}

/** Encoder init routine.
*/
void encoder_init()
{
	int8_t new;
	
	new=0;
	//Encoder output is gray-coded. At each sampling of the encoder input, we convert gray to binary, so value can be easily added or substracted. Gray code is:
	//____^^^^____^^^^____ PHASE_A
	//  ____^^^^____^^^^____ 	PHASE_B
	//  0 1 1 0 0 1 1 0 0 
	//  0 0 1 1 0 0 1 1 0

	if( PHASE_A )
		new = 3;
	if( PHASE_B )
		new ^= 1;
	last = new;
	
	enc_delta = 0;
	
	TCCR3A = 0x03;	//PWM, TOP=OCR3A, BOTTOM=TOV
	TCCR3B = 0x11;	//no preescaling
	OCR3A = 3500;// Sampling rate for loseless sampling calculation: stepper steps at 3ms/step max. 1 step of stepper are 5 encoder steps. This is 0.6ms/e_step. Nyquist with factor 2.5 = 0.24ms/e_sambple --> around OCR3A = 3500
	BITSET(TIMSK3,OCIE3A);	//enable interrupt at OC3A

	// Index signal is not sampled, instead, it triggers an external interrupt.
	BITSET(PCICR,PCIE0);
	BITSET(PCMSK0,PCINT0);
}
