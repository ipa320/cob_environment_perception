/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file scheduler.h
 *
 * Header for scheduler. In this case, regulates the output throughput
 *  \version 1.0
 *  \author Julio Sagardoy
 */

#ifndef INC_SCHEDULER_H
#define INC_SCHEDULER_H

void scheduler_init( void );

void scheduler_layer0( void );

void scheduler_layer1( void );

void scheduler_Set_sl1( int8_t );

int8_t scheduler_Get_sl1( void );

#endif
