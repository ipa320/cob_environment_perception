/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file stepper.h
 *
 * Header for stepper setup.
 *  \version 1.0
 *  \author Julio Sagardoy
 */
#ifndef INC_STEPPER_H
#define INC_STEPPER_H

void stepper_init( void );

void stepper_advance( void );

void stepper_set_direction( int8_t );

int8_t stepper_get_direction( void );

int16_t stepper_read( void );

void stepper_set( const int16_t );

void stepper_cal( void );

void stepper_set_rem_steps( const int16_t );

int16_t stepper_get_rem_steps();

void stepper_set_latency( const uint8_t );

uint8_t stepper_get_latency();

#endif
