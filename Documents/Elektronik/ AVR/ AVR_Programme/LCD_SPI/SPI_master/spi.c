//http://read.pudn.com/downloads163/sourcecode/embed/741340/Mega16_MCP3208/mcp3208.c__.htm


#include <avr/io.h>
#include "spi_adc.h"
#include <avr/delay.h>

// http://read.pudn.com/downloads163/sourcecode/embed/741340/Mega16_MCP3208/mcp3208.c__.htm
// http://read.pudn.com/downloads163/sourcecode/embed/741340/Mega16_MCP3208/mcp3208.h__.htm

void MCP3208_spiDelay(unsigned int NOPcount)
{
   unsigned int n;
   for(n=0;n<=NOPcount;n++)
   {
      asm volatile ("nop" ::);
   }
}

void spi_adc_restore()
{
   //SPCR0=0;
 //  SPI_PORT |= (1<<(SPI_CS_ADC));    //setbitHigh CS   Pin
//   SPI_PORT |= (1<<(SPI_MOSI_PIN));  //setbitHigh MOSI Pin
//   SPI_PORT |= (1<<(SPI_MISO_PIN));  //setbitHigh MOSI Pin
   SPI_PORT &= ~(1<<(SPI_SCK_PIN));   //setbitHigh CLK  Pin
   SPCR0 &= ~(1<<CPOL0)|(1<<CPHA0);    //|(0<<SPR0);
//   SPI_PORT &= ~(1<<(SPI_CS_ADC));    //setbitHigh CS   Pin
   
}

void MCP3208_spi_Init(void)
{
   // Set MOSI, SCK and SS output, all other input
   SPI_DDR = (1<<SPI_MOSI_PIN)|(1<<SPI_SCK_PIN)|(1<<SPI_CS_ADC);  // set DD_SS to output
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   // Enable SPI, Master, set clock rate fclk/64
   // Setup (Falling) Sample (Rising) SPI set mode 3
   // CPOL=1 : CPHA=1
   SPCR0=0;
   
   SPCR0 = (1<<SPE0)|(1<<MSTR0);
   
//   SPCR0 |= (1<<CPOL0)|(1<<CPHA0);
   
   SPCR0 |= (1<<SPR00); // f/16
   //SPCR |= (1<<SPR1); // f/64
   
   SPI_PORT |= (1<<(SPI_CS_ADC));    //setbitHigh CS   Pin
   SPI_PORT |= (1<<(SPI_MOSI_PIN));  //setbitHigh MOSI Pin
   SPI_PORT &= ~(1<<(SPI_MISO_PIN));  //setbitHigh MISO Pin
   SPI_PORT |= (1<<(SPI_SCK_PIN));   //setbitHigh CLK  Pin
}

unsigned char MCP3208_spiWrite(char cData)
{
   // Start transmission
   SPDR0 = cData;
   // Wait for transmission complete
   while (!(SPSR0 & (1<<SPIF0)))
      ;
   return SPDR0;
}

unsigned int MCP3208_spiRead(unsigned char AD_type,unsigned char ADchanel)
{
   //OSZI_A_LO ;
   unsigned char  tempHigh,tempLow,tempADtype,dummyData;
   
   SPI_PORT &= ~(1<<(SPI_CS_ADC));     //setbitLow CS  Pin
   _delay_us(ADC_DELAY);
   
   tempADtype = (AD_type & 0x01) << 1 ;
   tempLow = (ADchanel & 0x03) << 6;
   tempHigh = (ADchanel & 0x04) >> 2;
   tempHigh |= (0x04)|(tempADtype);     // 0x04 --> startBit
   
   dummyData = MCP3208_spiWrite(tempHigh);        // Write control HighByte return not care
   _delay_us(ADC_DELAY);
   gReciveHighByte = MCP3208_spiWrite(tempLow);  // Write control LowByte return A/D-MSB data
   _delay_us(ADC_DELAY);
   gReciveLowByte = MCP3208_spiWrite(0x00);      // Write Null byte 0x00 return A/D-LSB data
   
   _delay_us(ADC_DELAY);
   SPI_PORT |= (1<<(SPI_CS_ADC));        //setbitHigh CS  Pin
   
   return (((gReciveHighByte & 0x0F)<<8)|gReciveLowByte);  // return 16bit variable (12bit A/D data)
}




