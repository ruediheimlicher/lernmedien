//
//  dac.c
//  Power_Master
//
//  Created by Ruedi Heimlicher on 10.11.2014.
//
//

#include "soft_spi.h"
#include "defines.h"
#include <avr/io.h>



extern volatile uint8_t spi_txbuffer[SPI_BUFFERSIZE];
extern volatile uint8_t adc_H;
extern volatile uint8_t adc_L;

extern volatile uint8_t out_H;
extern volatile uint8_t out_L;
extern volatile uint8_t in_H;
extern volatile uint8_t in_L;

//extern volatile uint8_t switch_in[8];

#define nop()  asm volatile("nop\n\t"::)

//#define DAC_CS  2

void dac_init(void)
{
   SOFT_SPI_DDR |= (1<<SOFT_DAC_CS); // Ausgang fuer CS DAC
   SOFT_SPI_PORT |= (1<<SOFT_DAC_CS); // HI
   
   SOFT_SPI_DDR |= (1<<SOFT_DAC_LOAD); // Ausgang fuer CS ADC
   SOFT_SPI_PORT |= (1<<SOFT_DAC_LOAD); // HI

 //  SOFT_SPI_DDR |= (1<<SOFT_SWITCH_LOAD); // Ausgang fuer LOAD SWITCH
 //  SOFT_SPI_PORT |= (1<<SOFT_SWITCH_LOAD); // HI
   
   //SOFT_SPI_DDR |= (1<<SOFT_SWITCH_SS);
   
 //  SOFT_SPI_DDR|=(1<<SOFT_SWITCH_CS); // Ausgang fuer CS SWITCH
 //  SOFT_SPI_PORT |= (1<<SOFT_SWITCH_CS); // HI
   
   DDRA |= (1<<0);
   PORTA |= (1<<0);
   
   SOFT_SPI_DDR |= (1<<SOFT_SCK); // Ausgang fuer SCK
   SOFT_SPI_PORT |= (1<<SOFT_SCK); // HI
   SOFT_SPI_DDR |= (1<<SOFT_MOSI); // Ausgang fuer MOSI DAC
   SOFT_SPI_PORT |= (1<<SOFT_MOSI); // HI
   SOFT_SPI_DDR |= (1<<SOFT_DAC_LOAD); // Ausgang fuer MOSI DAC
   SOFT_SPI_PORT &= ~(1<<SOFT_DAC_LOAD); // LO
   
   
   //SOFT_SPI_DDR &= ~(1<<SOFT_MISO); // Eingang fuer MISO ADC
   //SOFT_SPI_PORT |= (1<<SOFT_MISO); // HI

}




uint8_t spi_out(uint8_t dataout) // von LCD_DOG_Graph
{
   //cli();
   
   // OSZI_B_LO;
   DAC_CS_LO; // Chip enable
   uint8_t datain=0xFF;
   _delay_us(1);
   uint8_t pos=0;
   SCL_LO; // SCL LO
   _delay_us(1);
   uint8_t tempdata=dataout;
   
   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         //DATA_HI;
         SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         //DATA_LO;
         SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
       _delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      _delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   //  OSZI_B_HI;
   
   DAC_CS_HI;// Chip disable
   
   sei();
   return datain;
}


uint8_t spi_out16(uint8_t dataHI, uint8_t dataLO) // von LCD_DOG_Graph
{
   cli();
   
   // OSZI_B_LO;
   DAC_CS_LO; // Chip enable
   uint8_t datain=0xFF;
   //_delay_us(1);
   uint8_t pos=0;
   SCL_LO; // SCL LO
   
   SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
   //_delay_us(1);
   SOFT_SPI_PORT |= (1<<SOFT_SCK);
   
   
   
   //_delay_us(1);
   uint8_t tempdata=dataHI;
   
   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         DATA_HI;
         //SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         DATA_LO;
         //SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
      // _delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      //_delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   
   
   tempdata=dataLO;
   
   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         DATA_HI;
         //SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         DATA_LO;
         //SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
      //_delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      //_delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   
   
   
   //  OSZI_B_HI;
   
   DAC_CS_HI;// Chip disable
   
   sei();
   return datain;
}


uint8_t spi_out_long(uint16_t data) // von LCD_DOG_Graph
{
   cli();
   
   // OSZI_B_LO;
   DAC_CS_LO; // Chip enable
   uint8_t datain=0xFF;
   //_delay_us(1);
   uint8_t pos=0;
   SCL_LO; // SCL LO
   
   SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
   //_delay_us(1);
   SOFT_SPI_PORT |= (1<<SOFT_SCK);
   
   
   
   //_delay_us(1);
   // data HI
   uint8_t tempdata=((data & 0xFF00)>>8);
   
   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         DATA_HI;
         //SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         DATA_LO;
         //SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
      // _delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      //_delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   
   
   tempdata=(data & 0x00FF);
   
   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         DATA_HI;
         //SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         DATA_LO;
         //SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
      //_delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      //_delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   
   
   
   //  OSZI_B_HI;
   
   DAC_CS_HI;// Chip disable
   
   sei();
   return datain;
}


//##############################################################################################



uint8_t spi_out_7612(uint16_t data) // von LCD_DOG_Graph
{
   cli();
   
   data &= 0x0FFF; // 12 bit
   
   data |= 0x2000; //Output A
   // OSZI_B_LO;
   uint8_t datain=0xFF;
   //_delay_us(1);
   uint8_t pos=0;
   SCL_LO; // SCL LO
   uint16_t tempdata=data;
   
   for (pos=14;pos>0;pos--)
   {
      
      if (tempdata & 0x2000)
      {
         DATA_HI;
         //SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         DATA_LO;
         //SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
     // _delay_us(1);
      SCL_HI;
      //SOFT_SPI_PORT |= (1<<SOFT_SCK);
      //_delay_us(1);
      SCL_LO;
      //SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   
   
   
  
   
   //  OSZI_B_HI;
   
   
   sei();
   return datain;
}


//##############################################################################################
//Writes one byte to data or cmd register
//
//##############################################################################################
void display_write_byte(unsigned cmd_data, unsigned char data)
{
   spi_out(data);
   
}

//##############################################################################################


void setDAC_test(void)
{
   uint8_t i=0;
   SOFT_SPI_PORT &= ~(1<<SOFT_DAC_CS); // CS DAC Lo
   _delay_us(1);

   for (i=0;i<0x0f;i++)
   {
      SOFT_SPI_PORT ^= (1<<3);
   }

   _delay_us(1);
   SOFT_SPI_PORT |= (1<<SOFT_DAC_CS); // HI

}
//##############################################################################################


void setDAC(void)
{
   
   SOFT_SPI_PORT ^= (1<<SOFT_SCK);
   // data ausgeben an DAC
   //cli();
//   DAC_PORT &= ~(1<<DAC_CS); // CS DAC Lo
//   _delay_us(10);
   spi_out16(spi_txbuffer[3],spi_txbuffer[2]);
   
   //spi_out16(0x44,0x88);
   _delay_us(1);
//   DAC_PORT |= (1<<DAC_CS); // HI
   sei();
}// end setDAC


void setDAC_long(uint16_t data)
{
   
   SOFT_SPI_PORT ^= (1<<SOFT_SCK);
   // data ausgeben an DAC
   //cli();
   //   DAC_PORT &= ~(1<<DAC_CS); // CS DAC Lo
   //   _delay_us(10);
   spi_out_long(data);
   
   _delay_us(1);
   //   DAC_PORT |= (1<<DAC_CS); // HI
   sei();
}// end setDAC

//##############################################################################################

void setDAC7612(uint16_t data)
{
   DAC_CS_LO;
   //_delay_us(1);
   nop();
   SCL_LO; // SCL LO
   spi_out_7612(data);
   nop();
   //_delay_us(1);
   
   DAC_LOAD_LO;
   //_delay_us(1);
   nop();
   DAC_LOAD_HI;
   nop();
   //_delay_us(1);
   DAC_CS_HI;
   
}

//##############################################################################################

void setMCP4821(uint16_t data)
{
   DAC_CS_LO;
   //_delay_us(1);
   nop();
   SCL_LO; // SCL LO
   spi_out_7612(data);
   nop();
   //_delay_us(1);
   
   DAC_LOAD_LO;
   //_delay_us(1);
   nop();
   DAC_LOAD_HI;
   nop();
   //_delay_us(1);
   DAC_CS_HI;
   
}

//##############################################################################################

void getADC(void)
{
   //cli();
   // wert lesen an ADC
   
   DAC_LOAD_LO;
   _delay_us(1); // init
   DAC_LOAD_HI;
   _delay_us(3); // conv
   DAC_LOAD_LO; // Data lesen start
   _delay_us(1);
   SCL_LO;
   //_delay_us(1);
   
   for (uint8_t i=0;i<SPI_BUFFERSIZE;i++)
   {
      SCL_HI;
      //_delay_us(1);
      
      //adc_H |= (((SOFT_SPI_PIN & (1<<SOFT_MISO))==1)<< (7-i));
      
      
      if (SOFT_SPI_PIN & (1<<SOFT_MISO)) // Pin ist HI
      {
         adc_H |= (1<< (7-i));
      }
      else    // Pin ist LO
      {
         adc_H  &= ~(1<< (7-i));
      }
      
      SCL_LO;
     // _delay_us(1);
   
   }
   
   for (uint8_t i=0;i<SPI_BUFFERSIZE;i++)
   {
      SCL_HI;
      //_delay_us(1);

      if (SOFT_SPI_PIN & (1<<SOFT_MISO)) // Pin ist HI
      {
         adc_L  |= (1<< (7-i));
      }
      else    // Pin ist LO
      {
         adc_L  &= ~(1<< (7-i));
      }
      SCL_LO;
      //_delay_us(1);

   }
    
   
   _delay_us(1);
   DAC_LOAD_HI;
   _delay_us(1);
   SCL_HI;
   sei();
}

//##############################################################################################





uint8_t getSwitch(void)
{
//   cli();
   
 //  SWITCH_CS_LO;
   
   // wert lesen an Switch
   uint8_t temp=0;
   SCL_HI;
   //_delay_us(1);
//   SWITCH_LOAD_LO; // Data lesen start
   //_delay_us(1);
   
 //  SWITCH_LOAD_HI;
   //_delay_us(1);
  // PORTA &= ~(1<<SOFT_SWITCH_LOAD);

   SCL_LO;
   //_delay_us(2);
   
  
   for (uint8_t i=0;i<8;i++)
   {
      SCL_LO;
      //_delay_us(1);
      
      //adc_H |= (((SOFT_SPI_PIN & (1<<SOFT_MISO))==1)<< (7-i));
      
      
      if (SOFT_SPI_PIN & (1<<SOFT_MISO)) // Pin ist HI
      {
         temp |= (1<< (7-i));
         //switch_in[i] |= (1<< (7-i));
      }
      else    // Pin ist LO
      {
         //switch_in[i]  &= ~(1<< (7-i));
         temp  &= ~(1<< (7-i));
      }
      
      SCL_HI;
       //_delay_us(1);
      
   }
   
   //_delay_us(1);
  // SWITCH_CS_HI;
   //_delay_us(1);
   //SCL_HI;
   sei();
   
  // SWITCH_CS_HI;
   return temp;
}

//##############################################################################################


uint8_t exch_data(uint8_t out_byte)
{
   
      uint8_t in_byte = 0;
      cli();
      // wert lesen an Slave, data senden an slave
      
      DAC_LOAD_LO; // Data lesen start
      _delay_us(1);
      SCL_LO;
      //_delay_us(1);
      
      for (uint8_t i=SPI_BUFFERSIZE;i>0;i--)
      {
         SCL_HI;
         //_delay_us(1);
         
         //adc_H |= (((SOFT_SPI_PIN & (1<<SOFT_MISO))==1)<< (7-i));
         
         
         if (SOFT_SPI_PIN & (1<<SOFT_MISO)) // Pin ist HI
         {
            in_byte |= (1<< (i));
         }
         else    // Pin ist LO
         {
            in_byte  &= ~(1<< (i));
         }
         
         //SOFT_SPI_PORT |=  (((out_byte & 0x80) > 0)<< SOFT_MOSI);
         if ((out_byte & 0x80))
         {
            SOFT_SPI_PORT |= (1<< SOFT_MOSI); // HI
         }
         else
         {
            SOFT_SPI_PORT &= ~(1<< SOFT_MOSI); // LO
         }
         out_byte >>= 1;
         
         SCL_LO;
         //
         
      }
      _delay_us(1);
       //
      DAC_LOAD_HI;
      _delay_us(1);
      SCL_HI;
      sei();
   
   return in_byte;
}


// HC495 Stuff von Tux


static uint8_t e74595val=0;

// SER pin 14:
#define S74595_0 PORTC&=~(1<<PORTC5)
#define S74595_1 PORTC|=(1<<PORTC5)
// RCLK pin 12:
#define S74595_RCLKDOWN PORTC&=~(1<<PORTC4)
#define S74595_RCLKUP PORTC|=(1<<PORTC4)
// SRCLK pin 11:
#define S74595_CLOCKDOWN PORTC&=~(1<<PORTC3)
#define S74595_CLOCKUP PORTC|=(1<<PORTC3)
// Inline assembly, nop = do nothing for a clock cycle.
//#define nop()  asm volatile("nop\n\t" "nop\n\t"::)
#define nop()  asm volatile("nop\n\t"::)


void set74595(uint8_t val)
{
   uint8_t i=8;
   while(i)
   {
      i--;
      S74595_CLOCKDOWN;
      if (val & (1<<i))
      {
         S74595_1;
      }
      else
      {
         S74595_0;
      }
      S74595_CLOCKUP;
      nop();
   }
   S74595_CLOCKDOWN;
   S74595_RCLKUP;
   nop();
   S74595_RCLKDOWN;
   e74595val=val;
}
void init74595(void)
{
   DDRC|= (1<<PORTC4); // enable as output line
   S74595_RCLKDOWN;
   DDRC|= (1<<PORTC5); // enable as output line
   S74595_0;
   DDRC|= (1<<PORTC3); // enable as output line
   S74595_CLOCKDOWN;
   e74595val=0; // variable where we keep the current state
   set74595(0);
}



