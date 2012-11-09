/***************************************************************************
 ***  Fraunhofer IPA
 ***  Robotersysteme
 ***  Projekt: 3D cartography demonstrator
 ****************************************************************************
 ****************************************************************************
 ***  Autor: Julio Sagardoy (goa-js)
 ***************************************************************************/

/** \file code.c
 *
 * Main code of the microcontroller.
 *  \version 1.0
 *  \author Julio Sagardoy
 */

/// AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/// Standard includes
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/// Own includes
#include <usart.h>
#include <led.h>
#include <stepper.h>
#include <encoder.h>
#include <servo.h>
#include <scheduler.h>
#include <publisher.h>

/** Processes pan movement command, which is composed by target position and latency between steps.
 */
void pan_command()
{
  int16_t p_pos;		// target position
  uint16_t p_lat;		// inter-step latency
  int16_t steps;		// signed integer containing the number of steps to jump
  char* message;		// will contain the entire message from uart
  char* comma_ptr;
  char* token[2];

  message = malloc(32*sizeof(char));	// allocate space for uart message

  uart_gets( message );	// ref receive string (null terminated already)

  comma_ptr = strchr(message, ',');	// locate comma position
  *comma_ptr = '\0';		// replace comma by null, so substring is created

  token[0] = message;		// token[0] is ptr to first substring -position
  token[1] = comma_ptr+1;	// comma_ptr+1 is ptr to second substring -latency

  p_pos = atoi(token[0]);
  p_lat = atoi(token[1]);

  free(message);

  /// compute steps to jump
  steps = p_pos - stepper_read();

  /// compute rotation direction, recall 0 is CCW and 1 CW in this setup, seen from top
  if (steps > 0)
    stepper_set_direction(1);
  else
    stepper_set_direction(0);

  stepper_set_rem_steps( abs(steps) );
  stepper_set_latency(p_lat);

  /*for(int16_t i=0; i<steps; i++)
	{
		stepper_advance();
		_delay_ms(p_lat);
	}*/
}

/** Processes tilt movement command, which is composed by target position and latency between steps.
 */
void tilt_command()
{
  int8_t t_pos;			// target position
  int16_t t_lat=1;		// inter-step latency
  char* message;		// will contain the entire message from uart
  char* comma_ptr;
  char* token[2];

  message = malloc(32*sizeof(int8_t));	// allocate space for uart message

  uart_gets( message );	// ref receive string (null terminated already)

  comma_ptr = strchr(message, ',');	// locate comma position
  *comma_ptr = '\0';		// replace comma by null, so substring is created

  token[0] = message;		// token[0] is ptr to first substring -position
  token[1] = comma_ptr+1;	// comma_ptr+1 is ptr to second substring -latency

  t_pos = (int8_t)atoi(token[0]);	// token[0] is ptr to first substring -position
  t_lat = atoi(token[1]);			// comma_ptr+1 is ptr to second substring -latency

  free(message);

  int8_t old_pos = servo_read();	// Get current servo value

  if (t_pos > old_pos)			// Compare with target position, to see direction of rotation
    servo_set_direction(1);
  else
    servo_set_direction(0);

  uint8_t steps = abs(t_pos - old_pos);

  servo_set_rem_steps( steps );
  servo_set_latency( t_lat );
  //servo_set_val( t_pos, t_lat );
}

void StopMovement()
{
  stepper_set_rem_steps(0);
  servo_set_rem_steps(0);
}

/** main routine
 */
int main()
{
  char keychar;

  // Initialisation routines
  cli();			//Disable global interrupt flag
  led_init();
  uart_init();
  servo_init();
  encoder_init();
  stepper_init();
  scheduler_init();
  sei();			//Enable global interrupt flag

  led_red_on();			// red led is always lit
  servo_set_neutral();	// set servo to neutral position

  while(1)
  {
    if ( uart_DataInReceiveBuffer() )	// poll for available data in rx buffer
    {
      keychar = uart_getc();	// read a single character, check if its a key character

      switch( keychar )
      {
        case 'P':
          led_grn_on();
          pan_command();
          led_grn_off();
          break;

        case 'T':
          led_grn_on();
          tilt_command();
          led_grn_off();
          break;

        case 'R':
          StopMovement();
          if( uart_getc() == '1' )
            scheduler_Set_sl1( 1 );
          else
            scheduler_Set_sl1( 0 );
          break;

        case 'E':
          StopMovement();
          uart_bufferFlush(2);
          uart_puts("E");
          break;

        case 'B':
          uart_bufferFlush(1);
          break;

        case 'r':
          led_grn_on();
          publishData();
          led_grn_off();
          break;

        case 'L':
          led_grn_on();
          StopMovement();
          uart_bufferFlush(2);
          scheduler_Set_sl1( 0 );
          servo_set_neutral();	// set servo to neutral, reducing torque moment when calibrating
          _delay_ms(500);			// wait for servo to settle
          stepper_cal();			// begins calibration
          led_grn_off();
          break;

        case 'C':
          StopMovement();
          scheduler_Set_sl1( 0 );
          uart_loopback();		// loopback test until 'q' is received
          break;

        case 'N':
          StopMovement();
          scheduler_Set_sl1( 0 );
          uart_puts( "COB3DMD" );	// retrieve robot name
          break;
      }
    }
  }
}
