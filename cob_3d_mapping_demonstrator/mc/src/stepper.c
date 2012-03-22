/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file stepper.c
 *
 * Code for stepper setup.
 *  \version 1.0
 *  \author Julio Sagardoy
 */

/// AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/// Own includes
#include "encoder.h"
#include "stepper.h"
#include "utils.h"
#include "usart.h"

static int16_t curr_step_num = 0;		/// contains current counter step number in the encoder
volatile char indexSet = 0;				/// flag for detecting index overpassed
static int16_t rem_steps;
static uint8_t latency;


/** Change stepper ic controller direction. 1 is CW, 0 is CCW seen from top
*/
void stepper_set_direction( int8_t dir )
{
	if(dir >= 1)
		BITSET(PORTL,PL3);	//Rotation direcion CW
	else
		BITCLR(PORTL,PL3);	//Rotation direcion to CCW
}

/** Get current direction status
*/
int8_t stepper_get_direction()
{
	//Seen from top:
	// 1: CW
	// 0: CCW
	return BITREAD(PINL,PL3);
}

/** Retrieve current step counter number
*/
int16_t stepper_read()
{
	return curr_step_num;
}

/** Command stepper to go to desired step position
*/
void stepper_set( const int16_t target_pos )
{
	for(int16_t i=curr_step_num; i<target_pos; i++)
		stepper_advance();
}

void stepper_set_rem_steps( const int16_t steps)
{
	rem_steps = steps;
}
int16_t stepper_get_rem_steps()
{
	return rem_steps;
}
void stepper_set_latency( const uint8_t lat)
{
	latency = lat;
}
uint8_t stepper_get_latency()
{
	return latency;
}
	
/** Generate step
*/
void stepper_advance()
{
	BITCLR(PORTL,PL0);	
	_delay_us(900);	// Hold line down to advance one step
	BITSET(PORTL,PL0);
	
	// check direction to update step counter
	if( stepper_get_direction() )
		curr_step_num++ ;
	else
		curr_step_num-- ;
}

/** Calibration routine. Must be called for a correct operation of counter registers
*/
void stepper_cal()
{
	//Rotate pan CCW till index is found
	indexSet = 0;
	stepper_set_direction(0);

	while( !indexSet )
	{
		_delay_ms(15);		// 20 ms interstep is really slow, but securs no extra step after index is passed
		stepper_advance();	// comand single step advance
	}
	
	_delay_ms(100);		// delay to let motor settle
	indexSet = 0;		// reset index flag
	curr_step_num = 0;	// reset step count
	encoder_reset();	// reset encoder counter

	uart_puts("L");		// from now on, the platform is calibrated. Notify host:
}

/** External pin change interrupt for Index signal 
*/
ISR(PCINT0_vect)
{
	indexSet = 1;
}

/** Inits stepper.
*/
void stepper_init()
{
	DDRL = 0x6F;		// set pin direction

	// Set stepper "configuration"
	BITSET(PORTL,PL0);		// keep clk high. 
	BITSET(PORTL,PL5);		// reset counter, drive down for a clk cycle
	BITSET(PORTL,PL1);		// step type.
							// | Full step | Low  |
							// | Half step | High |
							
	BITSET(PORTL,PL2);		// chopper controls __ lines:
							// | ABCD | High |
							// | INH  | Low  |

	BITSET(PORTL,PL4);		// enable pull-up for HOME pin of L297
	BITSET(PORTL,PL5);		// reset counter, drive down for a clk cycle
	BITSET(PORTL,PL6);		// enable ic
}
