//
//  spi_master.h
//  AVR_Starter
//
//  Created by Ruedi Heimlicher on 19.10.2014.
//
//


#define SPI_BUFFERSIZE 4

#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB



// Teensy2:
#define SPI_SS       0
#define SPI_SCK      1
#define SPI_MOSI     2
#define SPI_MISO     3




static volatile uint8_t spi_rxbuffer[SPI_BUFFERSIZE];
static volatile uint8_t spi_txbuffer[SPI_BUFFERSIZE];
static volatile uint8_t spi_rxdata=0;

static volatile uint8_t inindex=0;

