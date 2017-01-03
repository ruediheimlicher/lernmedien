//http://read.pudn.com/downloads163/sourcecode/embed/741340/Mega16_MCP3208/mcp3208.c__.htm


#include <avr/io.h>
#include "spi_adc.h"


void MCP3208_spiDelay(unsigned int NOPcount)
{
   unsigned int n;
   for(n=0;n<=NOPcount;n++)
   {
      asm volatile ("nop" ::);
   }
}

void spiadc_init()
{
   SPCR=0;
   SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(0<<SPR0);
   SPCR |=(1<<CPOL)|(1<<CPHA);
   
}

/* in Master
void MCP3208_init(void)
{
   // Set MOSI, SCK and SS output, all other input
   SPI_DDR = (1<<SPI_MOSI_PIN)|(1<<SPI_SCK_PIN)|(1<<SPI_SS_PIN);  // set DD_SS to output
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   // Enable SPI, Master, set clock rate fclk/64
   // Setup (Falling) Sample (Rising) SPI set mode 3
   // CPOL=1 : CPHA=1
   SPCR=0;
   SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(0<<SPR0)|(1<<CPOL)|(1<<CPHA);
   SPI_PORT |= (1<<(SPI_SS_PIN));    //setbitHigh CS   Pin
   SPI_PORT |= (1<<(SPI_MOSI_PIN));  //setbitHigh MOSI Pin
   SPI_PORT |= (1<<(SPI_MISO_PIN));  //setbitHigh MOSI Pin
   SPI_PORT |= (1<<(SPI_SCK_PIN));   //setbitHigh CLK  Pin
}
 

unsigned char MCP3208_spiWrite(char cData)
{
   // Start transmission
   SPDR = cData;
   // Wait for transmission complete
   while (!(SPSR & (1<<SPIF)))
      ;
   return SPDR;
}
 

unsigned int MCP3208_spiRead(unsigned char AD_type,unsigned char ADchanel)
{
   unsigned char  tempHigh,tempLow,tempADtype,dummyData;
   //OSZI_D_LO ;
   SPI_PORT &= ~(1<<(SPI_SS_PIN));     //setbitLow CS  Pin
   MCP3208_spiDelay(delayCount );
   
   tempADtype = (AD_type & 0x01) << 1 ;
   tempLow = (ADchanel & 0x03) << 6;
   tempHigh = (ADchanel & 0x04) >> 2;
   tempHigh |= (0x04)|(tempADtype);     // 0x04 --> startBit
   
   dummyData = MCP3208_spiWrite(tempHigh);        // Write control HighByte return not care
   gReciveHighByte = MCP3208_spiWrite(tempLow);  // Write control LowByte return A/D-MSB data
   gReciveLowByte = MCP3208_spiWrite(0x00);      // Write Null byte 0x00 return A/D-LSB data
   
   MCP3208_spiDelay(delayCount );
   SPI_PORT |= (1<<(SPI_SS_PIN));        //setbitHigh CS  Pin
   //OSZI_D_HI ;
   return (((gReciveHighByte & 0x0F)<<8)|gReciveLowByte);  // return 16bit variable (12bit A/D data)
}

*/


