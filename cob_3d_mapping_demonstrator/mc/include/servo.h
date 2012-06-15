/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file servo.h
 *
 * Code for servo pwm setup.
 *  \version 1.0
 *  \author Julio Sagardoy
 */
 
#ifndef INC_SERVO_H
#define INC_SERVO_H

void servo_init( void );

void servo_set_val( const int8_t , const int8_t );

void servo_set_neutral();

void servo_set_max();

void servo_set_min();

char servo_read();

void servo_set_direction( const uint8_t );

void servo_advance();

void servo_set_rem_steps( const uint8_t );

int16_t servo_get_rem_steps();

void servo_set_latency( const uint8_t );

uint8_t servo_get_latency();

#endif
