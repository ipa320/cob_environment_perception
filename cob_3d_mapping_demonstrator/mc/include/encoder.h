/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file encoder.h
 *
 * Header for encoder setup and position acquisition. Contains gray code converter code from Peter Dannegger from Mikrocontroller.net, for glitchless encoder position evaluation
 *  \version 1.0
 *  \author Julio Sagardoy
 */

#ifndef INC_ENCODER_H
#define INC_ENCODER_H

#define ENC_COUNTA TCNT0	// encoder Channel A counter
#define ENC_COUNTB TCNT3	// encoder Channel B counter
#define INDEX BITREAD(PORTB,0)	// Encoder Index pin

int16_t encoder_read();
void encoder_reset();
void encoder_init( void );

#endif
