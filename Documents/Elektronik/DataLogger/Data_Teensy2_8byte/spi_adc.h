//http://read.pudn.com/downloads163/sourcecode/embed/741340/Mega16_MCP3208/mcp3208.h__.htm
//  spi_adc.h
//  RC_PPM
//
//  Created by Ruedi Heimlicher on 26.07.2013.
//
//

#ifndef RC_PPM_spi_adc_h
#define RC_PPM_spi_adc_h



#endif
#ifndef __MCP3208_H_
#define __MCP3208_H_

#define SingleEnd    0x01        // set to Single-End A/D input
#define Differential 0x00        // set to Differential A/D input

#define SPI_DDR                  DDRB    //DDRB
#define SPI_PORT                 PORTB   //PORTB
#define SPI_SS_PIN                 0
#define SPI_SCK_PIN                1
#define SPI_MOSI_PIN               2
#define SPI_MISO_PIN               3

// SPI clock modes aus spi.h
#define SPI_MODE_0 0x00 /* Sample (Rising) Setup (Falling) CPOL=0, CPHA=0 */
#define SPI_MODE_1 0x01 /* Setup (Rising) Sample (Falling) CPOL=0, CPHA=1 */
#define SPI_MODE_2 0x02 /* Sample (Falling) Setup (Rising) CPOL=1, CPHA=0 */
#define SPI_MODE_3 0x03 /* Setup (Falling) Sample (Rising) CPOL=1, CPHA=1 */
// data direction
#define SPI_LSB 1 /* send least significant bit (bit 0) first */
#define SPI_MSB 0 /* send most significant bit (bit 7) first */
// slave or master with clock diviser
#define SPI_SLAVE 0xF0
#define SPI_MSTR_CLK4 0x00 /* chip clock/4 */
#define SPI_MSTR_CLK16 0x01 /* chip clock/16 */
#define SPI_MSTR_CLK64 0x02 /* chip clock/64 */
#define SPI_MSTR_CLK128 0x03 /* chip clock/128 */
#define SPI_MSTR_CLK2 0x04 /* chip clock/2 */
#define SPI_MSTR_CLK8 0x05 /* chip clock/8 */
#define SPI_MSTR_CLK32 0x06 /* chip clock/32 */
#define READ_ADC_COMMAND 0x01

// whether to raise interrupt when data received (SPIF bit received)
#define SPI_NO_INTERRUPT 0
#define SPI_INTERRUPT 1


#define delayCount   ((F_CPU)/1000000UL)

volatile unsigned char gReciveHighByte, gReciveLowByte; // global Variables

void spiadc_init(void);

void MCP3208_spiDelay(unsigned int NOPcount);
void MCP3208_spiInit(void);
unsigned char MCP3208_spiWrite(char cData);
unsigned int MCP3208_spiRead(unsigned char AD_type,unsigned char ADchanel);

#endif
