// AVR306: Using the AVR UART in C
// Routines for polled USART
// Last modified: 02-06-21
// Modified by: AR

/* Includes */
#include "usart.h"

/* Main - a simple test program*/
//void main( void )
//{
//	USART1_Init( 104 ); /* Set the baudrate to 19,200 bps using a 3.6864MHz crystal */
//
//	for(;;) 	    /* Forever */
//	{
//		USART1_Transmit( USART1_Receive() ); /* Echo the received character */
//	}
//}

/* Initialize UART */
void USART_Init( unsigned int baudrate )
{
	/* Set the baud rate */
	UBRRH = (unsigned char) (baudrate>>8);                  
	UBRRL = (unsigned char) baudrate;
	
	/* Enable UART receiver and transmitter */
	UCSRB = ( ( 1 << RXEN ) | ( 1 << TXEN ) ); 
	
	/* Set frame format: 8 data 2stop */
	//UCSRC = (1<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);              //For devices with Extended IO
	UCSRC = (1<<URSEL)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);   //For devices without Extended IO
}


/* Read and write functions */
unsigned char USART_Receive( void )
{
	/* Wait for incomming data */
	while ( !(UCSRA & (1<<RXC)) ) 	
		;			                
        //lcd_goto(0x40);
        //lcd_putstr("-");
	//lcd_putch(UDR1);
	//lcd_putstr("-");
	/* Return the data */

	return UDR;
}


void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !(UCSRA & (1<<UDRE)) )
		; 			                
	/* Start transmittion */
	UDR = data; 			        
}

void USART_printhex(uint8_t i)
{
    uint8_t hi,lo;

    hi=i&0xF0;               // High nibble
    hi=hi>>4;
    hi=hi+'0';
    if (hi>'9')
        hi=hi+7;

    lo=(i&0x0F)+'0';         // Low nibble
    if (lo>'9')
        lo=lo+7;

USART_Transmit( hi );
USART_Transmit( lo );
}


void USART_printstring(unsigned char *data)
{
    while(*data) {
        USART_Transmit(*data);
        data++;
    }

}

