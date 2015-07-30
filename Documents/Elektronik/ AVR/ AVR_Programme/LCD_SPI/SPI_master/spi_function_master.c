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


unsigned char status = 0;
volatile unsigned char count;


volatile uint8_t			spistatus=0;

#define TEENSY_SEND  7
#define TEENSY_RECV  6
#define WHILEMAX 0xFFFF // Wartezeit in while-Schleife : 5 ms

#define SPI_DELAY 5

//http://www.ermicro.com/blog/?p=1050
// MCP23S17 Registers Definition for BANK=0 (default)
#define IODIRA 0x00
#define IODIRB 0x01
#define IOCONA 0x0A
#define GPPUA  0x0C
#define GPPUB  0x0D
#define GPIOA  0x12
#define GPIOB  0x13


uint16_t spiwaitcounter = WHILEMAX; // 5 ms

extern volatile uint8_t arraypos;


void timer1 (void);
void master_init (void);
void master_transmit (unsigned char data);

ISR (SPI_STC_vect)
{
	return;
}

/*
ISR (TIMER1_OVF_vect)
{						//Senderoutine
	if (count == 1) {
		master_transmit ('1');
		count--;
		return;
	}
	if (count == 0) {
		master_transmit ('0');
		count++;
	}
}
*/

void spi_master_init (void)
{
   /*
    // in defines.h:
    #define SPI_PORT     PORTB
    #define SPI_PIN      PINB
    #define SPI_DDR      DDRB
    
    
    #define SPI_SS       4
    
    #define SPI_MOSI     5
    #define SPI_MISO     6
    #define SPI_SCK      7
    

    */
   
	SPI_DDR = (1<<SPI_SS) | (1<<SPI_MOSI) | (1<<SPI_SCK);		// setze SCK,MOSI,PB0 (SS) als Ausgang
	SPI_DDR &= ~(1<<SPI_MISO);							// setze MISO als Eingang
	//SPI_PORT = (1<<SPI_SCK) | (1<<SPI_SS);				// SCK und PB0 high (ist mit SS am Slave verbunden)
	//SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);	//Aktivierung des SPI, Master, Taktrate fck/16
	
   
   SPI_PORT |= (1<<SPI_SS); // HI,
   
   //SPI_PORT |= (1<<SPI_CS); // HI, ohne Optokoppler
   
   SPCR |= (1<<MSTR);// Set as Master
   
 //  SPCR0 |= (1<<CPOL0)|(1<<CPHA0);
   
   SPI_PORT &= ~(1<<SPI_MISO); // LO
   /*
    SPI2X 	SPR1 	SPR0     SCK Frequency
    0       0        0     fosc/4
    0       0        1     fosc/16
    0       1        0     fosc/64
    0       1        1     fosc/128
    1       0        0     fosc/2
    1       0        1     fosc/8
    1       1        0     fosc/32
    1       1        1     fosc/64
    */
   
   //SPCR |= (1<<SPR0);               // div 16 SPI2X: div 8
   SPCR |= (1<<SPR1);               // div 64 SPI2X: div 32
   //SPCR |= (1<<SPR1) | (1<<SPR0);   // div 128 SPI2X: div 64
  // SPCR0 |= (1<<SPI2X0);
   
   SPCR |= (1<<SPE); // Enable SPI
   status = SPSR;								//Status loeschen
   
   
   
}

void spi_master_restore(void)
{
   //SPCR0=0;
   SPCR &= ~(1<<CPOL);
   SPCR &= ~(1<<CPHA);

}

/*
void setSPI_Teensy(void)
{
   uint8_t spidelay = 5;
   uint8_t spiwaitdelay = 4;

   uint8_t outindex=0;
   SPI_PORT &=  ~(1<<SPI_CS); // CS LO, Start, Slave soll erstes Byte laden
   _delay_us(spidelay);
   
   //SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
   //_delay_us(1);
   
   //PORTB &= ~(1<<0);
   
   
   
   for (outindex=0;outindex < SPI_BUFFERSIZE;outindex++)
      //for (outindex=0;outindex < 4;outindex++)
   {
      //OSZI_A_LO;
      // _delay_us(spidelay);
      SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
      _delay_us(spidelay);
      
      SPDR = spi_txbuffer[outindex];
      
      while(!(SPSR & (1<<SPIF0)) && spiwaitcounter < WHILEMAX)
      {
          spiwaitcounter++;
      }
      spiwaitcounter=0;
      //_delay_us(spidelay);
      //uint8_t incoming = SPDR0;
      
      if (outindex == 0) // slave warten lassen, um code zu laden
      {
         uint8_t incoming = SPDR;
         
         _delay_us(spiwaitdelay);
      }
      else if (outindex ==1) // code lesen, spistatus steuern
      {
         spi_rxbuffer[0] = SPDR;
         
         if (spi_rxbuffer[0] & (1<<TEENSY_SEND))
         {
            spistatus |= (1<< TEENSY_SEND);
            _delay_us(spiwaitdelay);
         }
         else
         {
            spistatus &= ~(1<< TEENSY_SEND);
            spistatus &= ~(1<< TEENSY_RECV);
         }
      }
      else if (spistatus & (1<< TEENSY_SEND))
      {
         if (spi_rxbuffer[0] & 0x7F)
         {
         spi_rxbuffer[outindex-1] = SPDR0; // erster durchgang liest dummy
         _delay_us(spiwaitdelay);
         }
         //spi_rxbuffer[outindex] = incoming;
      }

      
      _delay_us(spidelay);
      SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
      //OSZI_A_HI;
      
      
   }
   //PORTB |=(1<<0);
   arraypos++;
   arraypos &= 0x07;
   //spi_rxbuffer[outindex] = '\0';
   //outbuffer[outindex] = '\0';
   //char rest = SPDR;
   
   // wichtig
   _delay_us(10);
   
   //SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
   //_delay_us(1);
   
   SPI_PORT |= (1<<SPI_CS); // CS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
   //OSZI_A_HI;
   //_delay_us(10);

}
*/
unsigned char SPI_get_put_char(uint8_t cData)
{
   /* Start transmission */
   SPDR = cData;
   /* Wait for transmission complete */
  // while(!(SPSR & (1<<SPIF)))
      while(!(SPSR & (1<<SPIF)) && spiwaitcounter < WHILEMAX)
      {
         spiwaitcounter++;
      }
      ;
   /* Return data register */
   return SPDR;
}

uint8_t  get_SR(uint8_t outData)
{
 //  SWITCH_CS_LO;
   uint8_t in = 0;
   LCD_SS_HI;
   _delay_us(1);
 //  SWITCH_LOAD_LO; // Data lesen start
   _delay_us(1);
 //  SWITCH_LOAD_HI;
   _delay_us(1);
   LCD_SS_LO;
   //return 0;
   //SPCR0 |= (1<<CPOL0);
   SPDR = outData;
   while(!(SPSR & (1<<SPIF)) && spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   
   //SPCR0 &= ~(1<<CPOL0);
   //in = SPDR0;
   
 //  SWITCH_CS_HI;
   return SPDR0;
   
   
   
   
}

uint8_t set_LCD(uint8_t outData)
{
   //OSZITOG;
   LCD_SS_LO;
   _delay_us(SPI_DELAY);
  // _delay_ms(2);
   SPDR = outData;
   spiwaitcounter=0;
   while(!(SPSR & (1<<SPIF)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   
   //SPCR0 &= ~(1<<CPOL0);
   //uint8_t in = SPDR0;
   _delay_us(SPI_DELAY);
   LCD_SS_HI;
   return SPDR0;

   
}


/*
uint8_t set_SR_595(uint8_t outData)
{
   SRB_CS_LO;
   _delay_us(1);
   SPDR0 = outData;
   spiwaitcounter=0;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   
   //SPCR0 &= ~(1<<CPOL0);
   //uint8_t in = SPDR0;
   
   SRB_CS_HI;
   SRB_CS_LO;
   SRB_CS_HI;
   return SPDR0;
   
   
}
*/

/*
void SPI_Write(unsigned char addr,unsigned char data)
{
   // http://www.ermicro.com/blog/?p=1050
   // Activate the CS pin
   SRA_CS_LO;
   // Start MCP23S17 OpCode transmission
   SPDR0 = SPI_SLAVE_ID | ((SPI_SLAVE_ADDR << 1) & 0x0E)| SPI_SLAVE_WRITE;
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   // Start MCP23S17 Register Address transmission
   SPDR0 = addr;
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   
   // Start Data transmission
   SPDR0 = data;
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   // CS pin is not active
   SRA_CS_HI;
}
*/

/*
void init_SR_23S17(void)
{
   
   uint8_t init_opcode = 0x40; // 0x40 & write
   uint8_t init_adresseA = 0x00; // SPI_IODRA PIN-OUT/IN
   uint8_t init_adresseB = 0x01; // SPI_IODRB PIN-OUT/IN
   uint8_t init_dataA = 0xE0; // bit 0-4 output bit 5-7 input (Drehschalter)
   uint8_t init_dataB = 0x03; // bit 0-1 input (Imax, Umax) bit 5-7 output (Relais)
   SRA_CS_LO;
   _delay_us(1);
   
   SPI_Write(IODIRA,init_dataA);   // GPIOA As Input/Output
    _delay_us(1);
   SPI_Write(IODIRB,init_dataB);
   
    _delay_us(1);
   
   SPI_Write(IOCONA,0x28);   // I/O Control Register: BANK=0, SEQOP=1, HAEN=1 (Enable Addressing)
   //SPI_Write(IODIRA,0x00);   // GPIOA As Output
   //SPI_Write(IODIRB,0x00);   // GPIOB As Input
   //SPI_Write(GPPUB,0x00);    // Enable Pull-up Resistor on GPIOB
   SPI_Write(GPIOB,0x03);    // Reset Output on GPIOB
   

  // SPI_Write(GPPUA,0xF0);    // Enable Pull-up Resistor on GPIOA
   
   SPI_Write(GPPUB,0x03);    // Enable Pull-up Resistor on GPIOB
   
   SRA_CS_HI;
   
}
*/
/*
uint8_t set_SR_23S17_A(uint8_t outData)
{
   uint8_t write_opcode = 0x40; // 0x40 & write
   uint8_t write_adresse = 0x12; // GPIOA PIN-OUT/IN
   //uint8_t write_adresse = 0x14; // OLATA PIN-OUT/IN
   uint8_t write_dataA = outData; // alle output
   SRA_CS_LO;
   _delay_us(1);
   spiwaitcounter=0;
   SPDR0 = write_opcode;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SPDR0 = write_adresse;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SPDR0 = write_dataA;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SRA_CS_HI;
   
   return SPDR0;
   
   
}
*/
/*
uint8_t set_SR_23S17_B(uint8_t outData)
{
   uint8_t write_opcode = 0x40; // 0x40 & write
   uint8_t write_adresse = 0x13; // GPIOB PIN-OUT/IN
   //uint8_t write_adresse = 0x15; // OLATB PIN-OUT/IN
   uint8_t write_dataA = outData; // alle output
   SRA_CS_LO;
   _delay_us(1);
   spiwaitcounter=0;
   SPDR0 = write_opcode;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SPDR0 = write_adresse;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SPDR0 = write_dataA;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SRA_CS_HI;
   
   return SPDR0;
   
   
}
*/
/*
uint8_t set_SR_23S17(uint8_t addr,uint8_t outData)
{
   uint8_t write_opcode = 0x40; // 0x40 & write
   uint8_t write_adresse = 0x13; // GPIOB PIN-OUT/IN
   //uint8_t write_adresse = 0x15; // OLATB PIN-OUT/IN
   uint8_t write_dataA = outData; // alle output
   SRA_CS_LO;
   _delay_us(1);
   spiwaitcounter=0;
   SPDR0 = write_opcode;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SPDR0 = addr;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SPDR0 = write_dataA;
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   SRA_CS_HI;
   
   return SPDR0;
   
   
}
*/
/*
uint8_t get_SR_23S17(uint8_t addr)
{
   uint8_t read_opcode = 0x41; // 0x40 & read
   uint8_t read_adresse = 0x13; // GPIOB PIN-OUT/IN

   // Activate the CS pin
   SRA_CS_LO;
   // Start MCP23S17 OpCode transmission
   //SPDR = SPI_SLAVE_ID | ((SPI_SLAVE_ADDR << 1) & 0x0E)| SPI_SLAVE_READ;
   _delay_us(1);
   spiwaitcounter=0;
   SPDR0 = read_opcode;

   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   _delay_us(1);
   // Start MCP23S17 Address transmission
   SPDR0 = addr;
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   
   _delay_us(1);
   // Send Dummy transmission for reading the data
   SPDR0 = 0x00;
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   
   // CS pin is not active
   SRA_CS_HI;
  
   return(SPDR0);
}
*/
/*
uint8_t io_SR_23S17(uint8_t addr, uint8_t data)
{
   uint8_t read_opcode = 0x41; // 0x40 & read
   uint8_t read_adresse = 0x13; // GPIOB PIN-OUT/IN
   
   // Activate the CS pin
   SRA_CS_LO;
   // Start MCP23S17 OpCode transmission
   //SPDR = SPI_SLAVE_ID | ((SPI_SLAVE_ADDR << 1) & 0x0E)| SPI_SLAVE_READ;
   _delay_us(1);
   spiwaitcounter=0;
   SPDR0 = read_opcode;
   
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   _delay_us(1);
   // Start MCP23S17 Address transmission
   SPDR0 = addr;
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   
   _delay_us(1);
   // Send Dummy transmission for reading the data
   SPDR0 = data;
   // Wait for transmission complete
   while(!(SPSR0 & (1<<SPIF0)));
   
   // CS pin is not active
   SRA_CS_HI;
   return(SPDR0);
}
*/

/*


void setMCP4821_U(uint16_t data)
{
   //OSZI_A_LO;
   
   MCP_U_CS_LO;
   //MCP_PORT &= ~(1<<MCP_DAC_U_CS);
   _delay_us(1);
   //OSZI_A_HI;
   uint8_t hbyte = (((data & 0xFF00)>>8) & 0x0F); // bit 8-11 von data als bit 0-3
   
   //hbyte |= 0x30; // Gain 1
   hbyte |= 0x10; // Gain 2
   //hbyte = 0b00010000;
   SPDR0 = (hbyte);
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }

   SPDR0 = (data & 0x00FF);
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   _delay_us(1);
   
   //
   MCP_U_CS_HI;
   //MCP_PORT &= (1<<MCP_DAC_U_CS);
   _delay_us(1);
   
   // Daten laden
   //MCP_LOAD_LO;
   _delay_us(1);
   //MCP_LOAD_HI;
   
   
}
*/
/*
void setMCP4821_I(uint16_t data)
{
   MCP_I_CS_LO;
   //MCP_PORT &= ~(1<<MCP_DAC_U_CS);
   _delay_us(1);
   uint8_t hbyte = (((data & 0xFF00)>>8) & 0x0F); // bit 8-11 von data als bit 0-3
   
   //hbyte |= 0x30; // Gain 1
   hbyte |= 0x10; // Gain 2
   //hbyte = 0b00010000;
   SPDR0 = (hbyte);
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   
   SPDR0 = (data & 0x00FF);
   while(!(SPSR0 & (1<<SPIF0)) )//&& spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }
   _delay_us(1);
   
   //
   MCP_I_CS_HI;
   //MCP_PORT &= (1<<MCP_DAC_U_CS);
   _delay_us(1);
   
   // Daten laden
   //MCP_LOAD_LO;
  // _delay_us(1);
   //MCP_LOAD_HI;
   
   
}
*/
