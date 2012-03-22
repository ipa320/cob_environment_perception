/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file scheduler.c
 *
 * Code for scheduler. In this case, regulates the output throughput
 *  \version 1.0
 *  \author Julio Sagardoy
 */

/// AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>

/// Own includes
#include <publisher.h>
#include <scheduler.h>
#include <servo.h>
#include <stepper.h>
#include <utils.h>

#define SCHED1 25	//(milliseconds)

static char sl1_isEnabled;
static uint8_t p_count;
static uint8_t t_count;

void scheduler_layer0()
{	
	int16_t p_rem_steps;
	uint8_t t_rem_steps;
	uint8_t p_lat;
	uint8_t t_lat;
	
	p_rem_steps = stepper_get_rem_steps();	// Remaining steps of the stepper
	p_lat = stepper_get_latency();			// Latency btw steps
	t_rem_steps = servo_get_rem_steps();	// Remaining steps of the servo
	t_lat = servo_get_latency();			// Latency btw steps
	
	/// Check whether its time to pan
	if (p_rem_steps > 0)
	{
		if(p_count == p_lat )	// Yes, its time to move once
		{
			stepper_advance();
			stepper_set_rem_steps(p_rem_steps - 1);
			p_count = 0;
		}
		else
			p_count++;	// Not yet but just keep waiting
	}
	
	/// Check whether its time to tilt
	if (t_rem_steps > 0)
	{
		if(t_count == t_lat )	// Yes, its time to move once
		{
			servo_advance();
			servo_set_rem_steps(t_rem_steps - 1);
			t_count = 0;
		}
		else
			t_count++;	// Not yet, but just keep waiting
	}
}

/** Get-set scheduler value
*/
void scheduler_Set_sl1( int8_t stat ) {
	sl1_isEnabled = stat;
}
int8_t scheduler_Get_sl1( ) {
	return sl1_isEnabled;
}

/** Layer 1 gets updated position and sends them
*/
void scheduler_layer1()
{
	if( sl1_isEnabled )
	{
		publishData();
	}
}

ISR(TIMER4_COMPA_vect)
{
	/// avoid interrupts
	cli();
	
	static uint8_t counter1;	
	
	scheduler_layer0();
	
	counter1++;
	if( counter1 == SCHED1 )
	{
		scheduler_layer1();
		counter1 = 0;
	}
	
	/// reactivate interrupts
	sei();
}

/** Scheduler init routine
*/
void scheduler_init( )
{
	sl1_isEnabled = false;
	
	// Timer 4 is the scheduler itself
	TCCR4A = 0;		// 
	TCCR4B = 0x09; 	// 0b00001001. CTC mode Prescaler 1

	OCR4A = 14746;	// CTC every 1ms
	
	TIMSK4 = 0x02;	// 0b00000010; Enable irq OCIE4A
}
