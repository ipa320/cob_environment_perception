/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file publisher.c
 *
 * Code for data publisher
 *  \version 1.0
 *  \author Julio Sagardoy
 */
 
/// AVR includes
#include <avr/io.h>

/// Standard includes
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/// Own includes
#include <encoder.h>
#include <stepper.h>
#include <servo.h>
#include <usart.h>
#include <publisher.h>
#include <utils.h>

int8_t publishData()
{
	char* p_pos_str;
	char* e_pos_str;
	char* t_pos_str;
	char* message;

	// allocate memory for the strings that will be used
	p_pos_str = malloc(sizeof(char) * 5 + 1);
	e_pos_str = malloc(sizeof(char) * 5 + 1);
	t_pos_str = malloc(sizeof(char) * 5 + 1);
	message = malloc(sizeof(char) * 32 + 1);

	// check for correct memory allocation
	if( (p_pos_str || e_pos_str || t_pos_str || message ) == '\0' )
		return false;
	
	// initialise first position of pointers to null, so strcat can work well
	*p_pos_str = '\0';
	*e_pos_str = '\0';
	*t_pos_str = '\0';
	*message = '\0';

	// read current positions	
	int16_t p_pos = stepper_read();
	int16_t e_pos = encoder_read();
	int16_t t_pos = servo_read();

	// convert integers to strings
	itoa( p_pos, p_pos_str, 10);
	itoa( e_pos, e_pos_str, 10);
	itoa( t_pos, t_pos_str, 10);

	// create message to output
	strcat(message, "R:");
	strcat(message, p_pos_str);
	strcat(message, ",");
	strcat(message, e_pos_str);
	strcat(message, ",");
	strcat(message, t_pos_str);

	// send message
	uart_puts(message);

	// deallocate memory
	free(p_pos_str);
	free(e_pos_str);		
	free(t_pos_str);		
	free(message);
	
	return true;
}
