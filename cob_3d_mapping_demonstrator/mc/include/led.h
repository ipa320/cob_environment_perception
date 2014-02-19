/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file led.h
 *
 * Header for led setup.
 *  \version 1.0
 *  \author Julio Sagardoy
 */

#ifndef INC_LED_H
#define INC_LED_H

/**
This method initialises the LED ports and pins accordingly
*/
void led_init();

void led_grn_off();

void led_grn_on();

void led_red_off();

void led_red_on();


#endif

