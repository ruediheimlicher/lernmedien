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

void spiADC_init()
{
   SPCR=0;
   SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);//|(0<<SPR1);
   //SPCR |=(1<<CPOL)|(1<<CPHA);
   
}


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
 

uint16_t MCP3208_spiRead(unsigned char AD_type,unsigned char ADchanel)
{
   //OSZI_A_LO ;
   unsigned char  tempHigh,tempLow,tempADtype,dummyData;
   uint8_t byte,data_high,data_low;
   
   SPI_ADC_CE_PORT &= ~(1<<(SPI_ADC_CE));     //setbitLow CS  Pin
   _delay_us(ADC_DELAY);
   
   
   tempADtype = (AD_type & 0x01) << 1 ;
   tempLow = (ADchanel & 0x03) << 6;
   tempHigh = (ADchanel & 0x04) >> 2;
   tempHigh |= (0x04)|(tempADtype);     // 0x04 --> startBit
   
   dummyData = MCP3208_spiWrite(tempHigh);        // Write control HighByte return not care
   _delay_us(ADC_DELAY);
   data_high = MCP3208_spiWrite(tempLow);  // Write control LowByte return A/D-MSB data
   _delay_us(ADC_DELAY);
   data_low = MCP3208_spiWrite(0xFF);      // Write Null byte 0x00 return A/D-LSB data
   
   _delay_us(ADC_DELAY);
   SPI_ADC_CE_PORT |= (1<<(SPI_ADC_CE));        //setbitHigh CS  Pin
   
   return (((data_high )<<8)|data_low);  // return 16bit variable (12bit A/D data)
}

uint16_t MCP3204_spiRead(uint8_t ch)
{
   // http://extremeelectronics.co.in/avr-tutorials/interfacing-12-bit-spi-adc-mcp3204-with-avr-micro/
   uint8_t byte,data_high,data_low;
   
   unsigned char  tempHigh,tempLow,tempADtype,dummyData,ADchanel,AD_type;
   ADchanel = 1;
   AD_type= 1;
   tempADtype = (AD_type & 0x01) << 1 ;
   tempLow = (ADchanel & 0x03) << 6;
   tempHigh = (ADchanel & 0x04) >> 2;
   tempHigh |= (0x04)|(tempADtype);     // 0x04 --> startBit
   
   dummyData = MCP3208_spiWrite(tempHigh);
   _delay_us(ADC_DELAY);
   byte=0x06;// 0b00000110;
   
   if(ch>3)
   {
      byte|=0x01;// 0b00000001;
   }
   
   SPI_ADC_CE_PORT &= ~(1<<(SPI_ADC_CE));
   _delay_us(ADC_DELAY);
   MCP3208_spiWrite(byte);
   _delay_us(ADC_DELAY);
   
   byte=ch<<6;
   
   data_high=MCP3208_spiWrite(byte);
   _delay_us(ADC_DELAY);
   data_high&=0b00001111;
   
   data_low=MCP3208_spiWrite(0xFF);
   _delay_us(ADC_DELAY);
   SPI_ADC_CE_PORT |= (1<<(SPI_ADC_CE));        //setbitHigh CS  Pin
   
   return ((data_high<<8)|data_low);
}



