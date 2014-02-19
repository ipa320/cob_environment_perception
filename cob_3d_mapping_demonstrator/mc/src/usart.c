/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: 3D cartography demonstrator
****************************************************************************
****************************************************************************
***  Autor: Julio Sagardoy (goa-js)
***************************************************************************/

/** \file usart.c
 *
 * Code usart module setup and operation
 *  \version 1.0
 *  \author Julio Sagardoy
 */
 
/// AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/// Own includes
#include "utils.h"
#include "usart.h"

#define UART_RX_BUFFER_SIZE 64     /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART_TX_BUFFER_SIZE 64

#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif

static unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile char UART_RxHead;
static volatile char UART_RxTail;
static unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile char UART_TxHead;
static volatile char UART_TxTail;

/** Extract a single character from rx cycle buffer
*/
unsigned char uart_getc( void )
{
	unsigned char tmptail;
	
	while ( UART_RxHead == UART_RxTail )  /* Wait for incomming data  */
		;
	tmptail = ( UART_RxTail + 1 ) & UART_RX_BUFFER_MASK;/* Calculate buffer index */

	UART_RxTail = tmptail;				/* Store new index */

	return UART_RxBuf[tmptail];			/* Return data */
}

/** Put a single character to the tx cycle buffer
*/
void uart_putc( unsigned char data )
{
	unsigned char tmphead;

	tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;  /* Calculate buffer index */
	while ( tmphead == UART_TxTail )	 /* Wait for free space in buffer */
		;

	UART_TxBuf[tmphead] = data;           /* Store data in buffer */
	UART_TxHead = tmphead;                /* Store new index */

	//UCR0 |= (1<<UDRIE);                    /* Enable UDRE interrupt */
	UCSR0B |= (1<<UDRIE0);		/* Enable UDRE interrupt */

}

/** Points to an entire string from rx cycle buffer. The end is signaled by a Unix <LF>. Returns the number of characters of the string
*/
char uart_gets( char* str )
{
	char count = 0;
	char c;
	
	while(1)
	{
		c = uart_getc();
		if(c == '\n')
		{
			*str = '\0';	// null terminate string
			return count;	// return read byte count
		}
		*str = c;
		str++;
	}
}

/** Put an entire string into the tx cycle buffer. Appends a Unix <LF>
*/
void uart_puts( const char* str )
{
	while( *str != '\0' )
		uart_putc(*str++);	
		
	uart_putc('\n');	//EOL
}

/** Tells if there is data in the rxcycle buffer
*/
unsigned char uart_DataInReceiveBuffer()
{
	return ( UART_RxHead != UART_RxTail ); /* Return 0 (FALSE) if the receive buffer is empty */
}

/** Echoes back received characters, except for 'q', which ends mode
*/
void uart_loopback( )
{
	char c;
	do	
	{
		c = uart_getc();
		uart_putc( c );
	}
	while( c != 'q' );
}

void uart_bufferFlush( const int8_t b )
{
	int8_t x=0;
	if( b == 0 )	// fush output buff
	{
		UART_TxTail = x;
		UART_TxHead = x;
	}
	else if (b == 1)	// flush input buff
	{
		UART_RxTail = x;
		UART_RxHead = x;
	}
	else		// flush both buffers
	{
		UART_RxTail = x;
		UART_RxHead = x;
		UART_TxTail = x;
		UART_TxHead = x;
	}
}

ISR(USART0_RX_vect)
{
	unsigned char data;
	unsigned char tmphead;

	if( BITREAD(UCSR0A,UPE0) )	//Check for parity error
	{
		UDR0;		//Discard character
		#if( UART_ERROR_REPORT_ENABLED )
		uart_putc(0x15);		//Send a NACK to the host	
		#endif
	}
	else
	{
		data = UDR0;                 /* Read the received data */
		/* Calculate buffer index */
		tmphead = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK;
		UART_RxHead = tmphead;      /* Store new index */

		if ( tmphead == UART_RxTail )
		{
			/* ERROR! Receive buffer overflow */
			//uart_flush();
		}
		UART_RxBuf[tmphead] = data; /* Store received data in buffer */
	}
}

ISR(USART0_UDRE_vect)
{
	unsigned char tmptail;

	/* Check if all data is transmitted */
	if ( UART_TxHead != UART_TxTail )
	{
		/* Calculate buffer index */
		tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
		UART_TxTail = tmptail;      /* Store new index */

		UDR0 = UART_TxBuf[tmptail];  /* Start transmition */
	}
	else
	{
		UCSR0B &= ~(1<<UDRIE0);     /* Disable UDRE interrupt */
	}
}

/** Initializes UART ports and cycle buffers
*/
void uart_init( )
{
	int8_t x;
	
	UBRR0 = BAUD;
	/* Enable UART receiver and transmitter, receive interrupt */
	UCSR0B = ( (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0) );
	UCSR0C = 0x26;;	//Set parity even
	
    /* Flush receive buffer */
	x=0;
    	
	UART_RxTail = x;
	UART_RxHead = x;
	UART_TxTail = x;
	UART_TxHead = x;
}

