/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file usart.h
 *
 * Header usart module setup and operation
 *  \version 1.0
 *  \author Julio Sagardoy
 */
#ifndef INC_UARTIRQ_INC
#define INC_UARTIRQ_INC

/**
Please enter here the required number for the desired baudrate.
Possible values are (for a 14.7456MHz system clock and U2X0=0):
|=======|=======|
|  bps  | value |
|=======|=======|
| 2400  | 383   |
| 4800  | 191   |
| 9600  | 95    |
| 14.4K | 63    |
| 19.2K | 47    |
| 28.8K | 31    |
| 38.4K | 23    |
| 57.6K | 15    |
| 76.8K | 11    |
| 115.2K| 7     |
| 230.4K| 3     |
*/
#define BAUD 15

void uart_init( void );

void uart_putc( unsigned char );

unsigned char uart_getc( void );

void uart_puts( const char * );

char uart_gets( char * );

void uart_bufferFlush( const int8_t );

unsigned char uart_DataInReceiveBuffer();

void uart_loopback();

#endif
