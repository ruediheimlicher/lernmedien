/*
    hw_atmega.c
    Atmel ATMega128/ATMega103 hardware low-level routines
    Part of MicroVGA CONIO library / demo project
    Copyright (c) 2008-9 SECONS s.r.o., http://www.MicroVGA.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    MCU:   ATMega128/ATMega103
    Internal osc:  1 MHz
    Operating frequency:  1 MHz
    UART Speed:     57600 kbit
    Connection:
      MicroVGA.TXD = ATMega128.RCD0 (RXD)
      MicroVGA.RXD = ATMega128.TXD0 (TXD)
      MicroVGA.CTS = ATMega128.PB7 (GPIO configuration)
	
	Defines are used to provide code compatibility between one UART and 
	two UART atmel devices.

   _putch, _getch, _kbhit must be defined for lib (conio.c and ui.c)
*/

#include <avr/io.h>
#include <avr/wdt.h>

#define FOSC 1000000    /* oscillator-frequency in Hz */
#define BAUD 57600  //valid values:9600, 19200, 57600 kbits


void _putch (char ch)
{

	while(PINB & (1<<PORT7)); //handshaking

#ifdef USR
	while(!(USR & (1<<UDRE))); //transmit buffer is ready to receive data
	
	UDR = ch;    // send character
	while(!(USR & (1<<TXC))); //wait for char to be send

	USR &= ~(1<<TXC || 1<<UDRE);
#else //2 UART MCU
	while(!(UCSR0A & (1<<UDRE0))); //transmit buffer is ready to receivce data
	
	UDR0 = ch;    // send character
	while(!(UCSR0A & (1<<TXC0))); //wait for char to be send

	UCSR0A &= ~(1<<TXC0 || 1<<UDRE0);

#endif
	
}

int _getch (void)
{
	int ch;

#ifdef USR
	while (!_kbhit()); /* Wait for incomming data */
	
	ch=UDR; //read uart

	if (ch==0)
	{
		while (!_kbhit()); /* Wait for incomming data */
		ch=UDR;
		ch = 0x100 | ch;
	}
#else
	while (!_kbhit()); /* Wait for incomming data */
	
	ch=UDR0; //read uart

	if (ch==0)
	{
		while (!_kbhit()); /* Wait for incomming data */
		ch=UDR0;
		ch = 0x100 | ch;
	}
#endif

	return ch;
}

int _kbhit (void)
{
int state;

#ifdef USR
	state = (USR & (1<<7));
#else
	state = (UCSR0A & (1<<7));
#endif
	return state;
}

void MCU_Init(void)
{

   DDRB &= ~(1<<DDB7); //Handshaking: set port PB7 direction (input)
   PORTB &= ~(1<<PB7);

#ifdef USR
	//set uart speed
   UBRR = ((FOSC/16)/BAUD-1);
   // Enable the receiver and the transmitter
   UCR = (1<<RXEN) | (1<<TXEN);
#else
   UBRR0H = (((FOSC/16)/BAUD-1)>>8);  // The high byte, UBRR0H
   UBRR0L = ((FOSC/16)/BAUD-1);       // The low byte, UBRR0L
	
   // Frame format (8 data bits, 1 stop bit, no parity)
   UCSR0C = UCSR0C | (1<<UCSZ02) | (1<<UCSZ01) | (1<<UCSZ00);
	
	// Enable the receiver and the transmitter
   UCSR0B = UCSR0B | (1<<RXEN0) | (1<<TXEN0); 	
#endif

}
