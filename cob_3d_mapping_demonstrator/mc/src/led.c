/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file led.c
 *
 * Code for led setup.
 *  \version 1.0
 *  \author Julio Sagardoy
 */

/// AVR includes
#include <avr/io.h>

/// Own includes
#include "utils.h"

inline void led_grn_off() {
	BITCLR(PORTA,PA5);
}
inline void led_grn_on() {
	BITSET(PORTA,PA5);	
}
inline void led_red_off() {
	BITCLR(PORTA,PA4);
}
inline void led_red_on() {
	BITSET(PORTA,PA4);
}

void led_init()
{
	BITSET(DDRA,4);		//Set PINA4 as output
	BITSET(DDRA,5);		//Set PINA5 as output
}
