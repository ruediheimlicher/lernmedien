/**************************************************************
Es soll alle halbe Sekunde im Wechsel 0 bzw. 1 gesendet werden.
Am korrespondierenden Slave soll zur Indikation jeweils die 
LEDs an bzw. aus gehen
Verdrahtung:	MISO(Master) --> MISO(Slave)
				MOSI(Master) --> MOSI(Slave)
				SCK(Master)  --> SCK(Slave)
				PB0(Master)	 --> SS(Slave)
**************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#include "spi_slave.h"
volatile unsigned char data;
unsigned char spi_status;

volatile uint8_t isrcontrol=0;
//extern  volatile uint8_t buffer[32];

volatile uint16_t                SPI_Data_counter; // Zaehler fuer Update des screens
uint8_t zahl=0;

uint8_t SPI_SlaveReceive(void)
{
   /* Wait for reception complete */
   while (!(SPSR & (1<<SPIF)))   ;
   /* Return data register */
   return    SPDR;
}

static inline uint8_t
inc (volatile uint8_t *a)
{
   return (*a)++;
}


ISR (SPI_STC_vect) // Neue Zahl angekommen
{
   OSZI_B_LO;
   if (inindex==0)
   {
      //OSZI_B_LO;
      //OSZI_B_HI;
   //isrcontrol = spi_txbuffer[inindex] ;
   }
   isrcontrol++;
   spi_rxbuffer[inindex] = SPDR;
   //isrcontrol = inindex;
   //isrcontrol +=inindex;
   SPDR = spi_txbuffer[inindex];
    //uint8_t input = SPDR;

   spi_rxdata=1;
   //inindex = inc(&inindex);
   inindex++;
   //inindex &= 0x0F;
   //SPI_Data_counter++;
    OSZI_B_HI;
}


// TODO: Init von SCK, MISO LO oder HI

void spi_slave_init (void)
{
   SPI_DDR = ~((1<<SPI_SS) | (1<<SPI_MOSI) | (1<<SPI_SCK));		// setze SCK,MOSI,PB0 (SS) als Eingang
   
   SPI_DDR |= (1<<SPI_MISO);							// setze MISO als Ausgang
//	SPI_PORT &= ~(1<<SPI_MISO);
//  SPI_PORT |= (1<<SPI_SCK) | (1<<SPI_SS);
   
   
	SPCR = (1<<SPE) | (1<<SPIE);			//Aktivierung des SPI + Interrupt
	SPDR=0;
   status = SPSR;								//Status loeschen
   
   /*
   // Atmega328
   EICRA |= (1<<ISC01); // falling edge von INT0
   EIMSK |= (1<<INT0);
   */
   
   /*
   // Atmega8
   MCUCR |= (1<<ISC01); // falling edge von INT0
   GICR |= (1<<INT0);
    */
   
   // atmega32u4
   
}


