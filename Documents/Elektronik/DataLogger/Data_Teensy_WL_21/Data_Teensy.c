//
//  Data_Teensy.c
//
//
//  Created by Sysadmin on 20.07.17
//  Copyright Ruedi Heimlicher 2017. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdlib.h>


#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
#include "defines.h"

#include "tastatur.c"

//#include "spi.c"
//#include "spi_adc.c"

//#include "spi_slave.c"
//#include "soft_SPI.c"

//#include "ds18x20.c"

#include "defines.h"

#include "Data_Teensy.h"

/*
 const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};
 */






uint16_t key_state;				// debounced key state:
// bit = 1: key pressed
uint16_t key_press;				// key press detect
volatile uint16_t tscounter =0;

extern volatile uint8_t isrcontrol;

volatile uint8_t do_output=0;
//static volatile uint8_t testbuffer[USB_DATENBREITE]={};


//static volatile uint8_t buffer[USB_DATENBREITE]={};


static volatile uint8_t recvbuffer[USB_DATENBREITE]={};

static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

// fuer daten von teensy
static volatile uint8_t teensybuffer[USB_DATENBREITE]={};


#define TIMER0_STARTWERT	0x40

#define EEPROM_STARTADRESSE   0x7FF

volatile uint8_t timer0startwert=TIMER0_STARTWERT;


void delay_ms(unsigned int ms);

//volatile uint8_t payload[wl_module_PAYLOAD];

// WL def
//volatile uint8_t wl_spi_status;



volatile uint8_t                    in_taskcounter=0;
volatile uint8_t                    out_taskcounter=0;


volatile  uint16_t                  mmcwritecounter=0; // Zaehler fuer Messungen auf SD

volatile uint16_t                   linecounter=0;

volatile uint16_t                   startblock = 0;

volatile uint8_t                    packetcount = 0;

volatile uint8_t                    blockanzahl = 0;

volatile uint8_t                    downloadblocknummer = 0;
volatile uint16_t                    downloaddatacounter = 0;

// counter fuer Mess-Intervall
volatile uint16_t                    intervallcounter=0;

// mess-Intervall
volatile uint16_t                    intervall=1; // defaultwert, 1s
// counter fuer Mess-Intervall
volatile uint16_t                    messungcounter=0; // Anzahl messungen fortlaufend

volatile uint16_t                    writedatacounter=0; // Anzahl mmc-writes fortlaufend
volatile uint16_t                    writemmcstartcounter=0; // Anzahl mmc-writes fortlaufend


//static volatile uint8_t            substatus=0x00; // Tasks fuer Sub
volatile uint8_t                    hoststatus=0x00;

static volatile uint8_t             usbstatus=0x00;
static volatile uint8_t             sd_status=0x00; // recvbuffer[1]

volatile uint8_t                    spistatus=0x00; // was ist zu tun infolge spi

static volatile uint8_t             potstatus=0x00; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t             impulscounter=0x00;

static volatile uint8_t             masterstatus = 0; // was ist zu tun in der loop? 

static volatile uint8_t             tastaturstatus = 0;

static volatile uint8_t             wl_callback_status = 0;
static volatile uint8_t             wl_callback_status_check = 0;
static volatile uint8_t             wl_callback_status_def = 0;
volatile uint8_t                    loggerstatus = 0;
volatile uint16_t                   loggertestwert = 0;

volatile uint8_t  kanalstatusarray[8] = {0};

volatile uint8_t status=0;

volatile int devicenummer =0;


// Logger
volatile uint16_t saveSDposition = 0;
volatile uint16_t blockcounter = 0; // Block, in den gesichert werden soll, mit einem Offset von 1 (Block 0 ist header der SD).
volatile uint16_t blockoffset = 1;
volatile uint16_t blockdatacounter = 0; //  Anzahl gespeicherter Messungen auf aktuellem Block. Fuer rescue verwendet

volatile uint16_t teensydatacounter = 0; //  Anzahl gespeicherter Messungen auf aktuellem Block. Fuer rescue verwendet


uint16_t eeprom_intervall EEMEM = 1; // default intervall
uint16_t eeprom_startblock EEMEM = 1; // default startblock
uint16_t eeprom_blockcount EEMEM = 0; // Anzahl geschriebene Blocks
uint16_t eeprom_messungcount EEMEM = 0; // Anzahl geschriebene Messungen
uint8_t eeprom_kanalbyte0 = 0;
uint8_t eeprom_kanalbyte1 = 0;
uint8_t eeprom_kanalbyte2 = 0;
uint8_t eeprom_kanalbyte3 = 0;

//declare an eeprom array
uint8_t EEMEM  eeprom_kanalstatus_array[4];

volatile uint16_t startminute = 0; // 

// WL
volatile uint8_t                   wl_isr_counter = 0;

// PWM

volatile uint8_t                    PWM=0;
static volatile uint8_t             pwmposition=0;
static volatile uint8_t             pwmdivider=0;

static volatile uint8_t spi_rxdata=0;

static volatile uint8_t inindex=0;

volatile char SPI_data='0';
volatile uint16_t teststartadresse=0xA0;


volatile uint16_t Batteriespannung =0;

volatile uint16_t adc_counter =0; // zaehlt Impulspakete bis wieder die Batteriespannung gelesen werden soll

volatile short int received=0;

//volatile uint16_t abschnittnummer=0;

volatile uint16_t usbcount=0;

volatile uint16_t devicecount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;

volatile uint16_t eepromstartadresse=0;

volatile uint16_t inchecksumme=0;

volatile uint16_t bytechecksumme=0;
volatile uint16_t outchecksumme=0;

volatile uint8_t eeprom_databyte=0;
volatile uint8_t anzahlpakete=0;
//volatile uint8_t usb_readcount = 0;

uint8_t  eeprom_indata=0;

#pragma mark WL def
//Variablen WL
// MARK: WL Defs

#define WL_MAX_DEVICE   6
volatile uint8_t wl_status=0;

volatile uint8_t wl_recv_status=0;

volatile uint8_t wl_send_status=0;

//volatile uint8_t PTX=0;
volatile uint8_t int0counter=0;
volatile uint8_t int1counter=0;


volatile uint8_t wl_spi_status=0;
char itoabuffer[20] = {};
volatile uint8_t wl_data[wl_module_PAYLOAD] = {};
//volatile uint8_t wl_data_array[4][wl_module_PAYLOAD] = {};

volatile uint8_t pipenummer = 1; // nur pipes < 7

volatile uint8_t loop_pipenummer = 1;
volatile uint8_t loop_channelnummer = 0;

volatile uint8_t akt_pipenummer = 1;

//volatile uint8_t module_channel[4] = {wl_module_Temp_channel,wl_module_ADC_channel,wl_module_Temp_channel,wl_module_ADC_channel};

volatile uint8_t module_channel[4] = {wl_module_Temp_channel,wl_module_ADC_channel,wl_module_ADC12BIT_channel,wl_module_AUX_channel};



volatile uint8_t wl_blockedcounter=0;
volatile uint8_t wl_sendcounter=0;

volatile uint16_t temperatur0=0;
volatile uint16_t temperatur1=0;

volatile uint8_t devicetemperatur[WL_MAX_DEVICE] = {0};


volatile uint16_t spannung0=0;

volatile uint8_t batteriespannung=0;

volatile uint8_t devicebatteriespannung[WL_MAX_DEVICE] = {0};


#pragma mark mmc def
FATFS Fat_Fs;		/* FatFs work area needed for each volume */
FIL Fil;			/* File object needed for each open file */
//FATFS FatFs[2];		/* File system object for each logical drive */
FIL File[2];		/* File object */
DIR Dir;			/* Directory object */
FILINFO Finfo;
DWORD AccSize;				/* Work register for fs command */
WORD AccFiles, AccDirs;

#define WRITENEXT 1
#define WRITETAKT 0x0032

#define ADCTAKT   0x0019 // 0.5s Abfrage teensy


BYTE RtcOk;				/* RTC is available */
volatile UINT Timer;	/* Performance timer (100Hz increment) */
volatile uint8_t mmcbuffer[SD_DATA_SIZE] = {};
//const uint8_t rambuffer[SD_DATA_SIZE] PROGMEM = {};
const uint8_t databuffer[SD_DATA_SIZE] PROGMEM = {};
//volatile uint8_t writebuffer[512] = {};
volatile uint8_t mmcstatus = 0;
volatile uint16_t                   writecounter1=0; // Takt fuer write to SD
volatile uint16_t                   writecounter2=0; // Takt fuer write to SD

volatile uint16_t                   datapendcounter=0; // Takt fuer write to SD


//#define CLOCK_DIV 15 // timer0 1 Hz bei Teilung /4 in ISR 16 MHz
#define CLOCK_DIV 8 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz
#define BLINK_DIV 4 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz


volatile uint16_t                TastaturCount=0; // counter fuer prellen
volatile uint16_t                tastaturcounter=0; // counter fuer entfernen der Anzeige
volatile uint8_t                 Menu_Ebene = 0;

volatile uint16_t                manuellcounter=0; // Counter fuer Timeout
volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint16_t                mscounter=0; // Counter fuer ms in timer-ISR
volatile uint8_t                 blinkcounter=0;


uint8_t                          ServoimpulsSchrittweite=10;
uint16_t                         Servoposition[]={1000,1250,1500,1750,2000,1750,1500,1250};

volatile uint16_t                Tastenwert=0;
volatile uint16_t                Trimmtastenwert=0;
volatile uint8_t                 adcswitch=0;



/*
 #define TASTE1		15
 #define TASTE2		23
 #define TASTE3		34
 #define TASTE4		51
 #define TASTE5		72
 #define TASTE6		94
 #define TASTE7		120
 #define TASTE8		141
 #define TASTE9		155
 #define TASTE_L	168
 #define TASTE0		178
 #define TASTE_R	194
 
 */
//const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};

/*
 
 static inline
 uint16_t key_no( uint8_t adcval )
 {
 uint16_t num = 0x1000;
 PGM_P pointer = wertearray;
 
 
 while( adcval < pgm_read_byte(pointer))
 {
 pointer++;
 num >>= 1;
 }
 return num & ~0x1000;
 }
 
 uint16_t get_key_press( uint16_t key_mask )
 {
 cli();
 key_mask &= key_press;		// read key(s)
 key_press ^= key_mask;		// clear key(s)
 sei();
 return key_mask;
 }
 
 */


#pragma mark 1-wire

//#define MAXSENSORS 2
//static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
//static int16_t gTempdata[MAXSENSORS]; // temperature times 10
//static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
//static int8_t gNsensors=0;

uint8_t search_sensors(void)
{
   return 0;
   /*
   uint8_t i;
   uint8_t id[OW_ROMCODE_SIZE];
   uint8_t diff, nSensors;
   
   ow_reset();
   
   nSensors = 0;
   
   diff = OW_SEARCH_FIRST;
   while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS )
   {
      DS18X20_find_sensor( &diff, &id[0] );
      
      if( diff == OW_PRESENCE_ERR )
      {
         lcd_gotoxy(0,1);
         lcd_puts("No Sensor found\0" );
         
         delay_ms(800);
         lcd_clr_line(1);
         break;
      }
      
      if( diff == OW_DATA_ERR )
      {
         lcd_gotoxy(10,0);
         lcd_puts("BusErr\0" );
         lcd_puthex(diff);
         return OW_DATA_ERR;
         break;
      }
      //lcd_gotoxy(4,1);
      
      for ( i=0; i < OW_ROMCODE_SIZE; i++ )
      {
         //lcd_gotoxy(15,1);
         //lcd_puthex(id[i]);
         
         gSensorIDs[nSensors][i] = id[i];
         //delay_ms(100);
      }
      
      nSensors++;
   }
   
   return nSensors;
    */
}

// start a measurement for all sensors on the bus:
/*
void start_temp_meas(void)
{
   
   gTemp_measurementstatus=0;
   if ( DS18X20_start_meas(NULL) != DS18X20_OK)
   {
      gTemp_measurementstatus=1;
   }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
   uint8_t i;
   uint8_t subzero, cel, cel_frac_bits;
   for ( i=0; i<gNsensors; i++ )
   {
      
      if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
                             &cel, &cel_frac_bits) == DS18X20_OK )
      {
         gTempdata[i]=cel*10;
         gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
         if (subzero)
         {
            gTempdata[i]=-gTempdata[i];
         }
      }
      else
      {
         gTempdata[i]=0;
      }
   }
}

*/
// Code 1_wire end


/*
void startTimer2(void)
{
   //timer2
   TCNT2   = 0;
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}
*/
void Master_Init(void)
{
   LOOPLEDDDR |=(1<<LOOPLED);
   LOOPLEDPORT |= (1<<LOOPLED);	// HI
   
   SPI_DDR |=(1<<SPI_MOSI);
   SPI_PORT |= (1<<SPI_MOSI);	// HI
   
   //Pin 0 von   als Ausgang fuer OSZI
   OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
   
   OSZIPORTDDR |= (1<<PULSB);		//Pin 1 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSB);		//Pin   von   als Ausgang fuer OSZI
   
   
   //OSZIPORTDDR &= ~(1<<TEST_PIN);		//Pin 1 von  als Eingang fuer Test-Pin. active LO
   //OSZIPORT |= (1<<TEST_PIN);		//Pin   von   als Ausgang fuer OSZI

   /*
    TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
    TASTENPORT |= (1<<TASTE0);	//Pull-up
    */
   TASTATUR_DDR &= ~(1<<TASTATUR_PIN); // input
   
   DDRB &= ~(1<<4);
   
   DDRF &= ~(1<<1); //ADC1
   // ---------------------------------------------------
   // Pin Change Interrupt enable on PCINT0 (PD7)
   // ---------------------------------------------------
   
   // PCIFR |= (1<<PCIF0);
   // PCICR |= (1<<PCIE0);
   //PCMSK0 |= (1<<PCINT7);
   
   // ---------------------------------------------------
   // USB_Attach
   // ---------------------------------------------------
   
   EICRA |= (1<<ISC01); // falling edge
   EIMSK=0;
   EIMSK |= (1<<INT0); // Interrupt en
   
   
   // ---------------------------------------------------
   //LCD
   // ---------------------------------------------------
   LCD_DDR |= (1<<LCD_RSDS_PIN);		// PIN als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin   als Ausgang fuer LCD
   
   
}


void SPI_PORT_Init(void) // SPI-Pins aktivieren
{
   
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   
   
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO);
   //SPI_PORT &= ~(1<<SPI_MISO; // HI
   SPI_DDR |= (1<<SPI_MOSI);
   SPI_DDR |= (1<<SPI_CLK);
   //SPI_PORT &= ~(1<<SPI_CLK); // LO
   SPI_DDR |= (1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS); // HI
  
   /*
    // SPI via Isolator
#define SPI_EXT_PORT    PORTD
#define SPI_EXT_DDR     DDRD
#define SPI_EXT_CS      PD3 // CS fuer SPI via Isolator
  */
   SPI_EXT_DDR |= (1<<SPI_EXT_CS);
   SPI_EXT_PORT |= (1<<SPI_EXT_CS); // HI

   /*
   // Ausgang via Isolator
#define SPI_AUX_PORT    PORTC
#define SPI_AUX_DDR     DDRC
#define SPI_AUX_A       PC6 // CS fuer SPI-Isolator-Aux-Kanal A
*/
   SPI_AUX_DDR |= (1<<SPI_AUX_A);
   SPI_AUX_PORT |= (1<<SPI_AUX_A); // HI

   
   /*
    // Slave init
    SPI_DDR |= (1<<SPI_MISO); // Output
    //SPI_PORT &= ~(1<<SPI_MISO; // HI
    SPI_DDR &= ~(1<<SPI_MOSI); // Input
    SPI_DDR &= ~(1<<SPI_CLK); // Input
    //SPI_PORT &= ~(1<<SPI_SCK; // LO
    SPI_DDR &= ~(1<<SPI_SS); // Input
    SPI_PORT |= (1<<SPI_SS); // HI
    */
   
   
   
}


void SPI_Master_init (void)
{
   
   SPCR |= (1<<MSTR);// Set as Master
   
   //  SPCR0 |= (1<<CPOL0)|(1<<CPHA0);
   
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
   //SPCR |= (1<<SPI2X0);
   
   SPCR |= (1<<SPE); // Enable SPI
   status = SPSR;								//Status loeschen
   
}


void spi_start(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO);
   SPI_PORT &= ~(1<<SPI_MISO); // LO
   
   SPI_DDR |= (1<<SPI_MOSI);
   SPI_PORT &= ~(1<<SPI_MOSI); // LO
   
   SPI_DDR |= (1<<SPI_CLK);
   SPI_PORT &= ~(1<<SPI_CLK); // LO
   
   SPI_DDR |= (1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS); // HI
   
   
   
}

void spi_end(void) // SPI-Pins deaktivieren
{
   SPCR=0;
   
   SPI_DDR &= ~(1<<SPI_MOSI); // MOSI off
   SPI_DDR &= ~(1<<SPI_CLK); // SCK off
   SPI_DDR &= ~(1<<SPI_SS); // SS off
   
   //SPI_RAM_DDR &= ~(1<<SPI_RAM_CS; // RAM-CS-PIN off
   //SPI_EE_DDR &= ~(1<<SPI_EE_CS; // EE-CS-PIN off
}

/*
 void spi_slave_init()
 {
 SPCR=0;
 SPCR = (1<<SPE)|(1<<SPR1)|(0<<SPR0)|(1<<CPOL)|(1<<CPHA);
 
 }
 */

uint16_t read_LM35(uint8_t lm35kanal)
{
   if (! (VREF_Quelle == ADC_REF_INTERNAL))
   {
      VREF_Quelle = ADC_REF_INTERNAL;
      uint8_t i=0;
      for (i=0;i<16;i++) // 3.5ms  // Warten auf neueinstellung von Vref
      {
         readKanal(lm35kanal);
      }
   }
   uint16_t adc2wert = readKanal(lm35kanal);
   uint32_t temperatur2 = adc2wert;
   temperatur2 *=VREF;
   temperatur2 = temperatur2/0x20;
   temperatur2 = 10*temperatur2/0x20;
   //   lcd_gotoxy(8,3);
   //   lcd_putint12(temperatur2&0xFFFF);
   return temperatur2 & 0xFFFF;
}

uint16_t read_bat(uint8_t batkanal) // im teensy kommt die halbe vBatt an
{
   if (! (VREF_Quelle == ADC_REF_INTERNAL))
   {
      VREF_Quelle = ADC_REF_INTERNAL;
      uint8_t i=0;
      for (i=0;i<16;i++) // 3.5ms  // Warten auf neueinstellung von Vref
      {
         readKanal(batkanal);
      }
   }
   uint8_t i=0;
   uint16_t adc2wert = readKanal(batkanal);
//  lcd_gotoxy(5,3);
//  lcd_putint12(adc2wert);

   uint32_t temperatur2 = adc2wert;
   temperatur2 *=VREF;
   temperatur2 = temperatur2/0x20;
   temperatur2 = temperatur2/0x20;
   
    //  lcd_gotoxy(8,3);
   //   lcd_putint12(temperatur2&0xFFFF);
   return temperatur2 & 0xFFFF;
}




void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
   // we use a calibrated macro. This is more
   // accurate and not so much compiler dependent
   // as self made code.
   while(ms){
      _delay_ms(0.96);
      ms--;
   }
}

// http://www.co-pylit.org/courses/COSC2425/lectures/AVRNetworks/index.html

/* nicht verwendet
 void timer1_init(void)
 {
 // Quelle http://www.mikrocontroller.net/topic/103629
 
 _HI ; // Test: data fuer SR
 _delay_us(5);
 //#define FRAME_TIME 20 // msec
 KANAL_DDR |= (1<<KANAL; // Kanal Ausgang
 
 DDRD |= (1<<PORTD5); //  Ausgang
 PORTD |= (1<<PORTD5); //  Ausgang
 
 //TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
 //TCCR1B = (1<<WGM13) | (1<<WGM12) ;		// TOP = ICR1, clk = sysclk/8 (->1us)
 TCCR1B |= (1<<CS11);
 TCNT1  = 0;														// reset Timer
 
 // Impulsdauer
 OCR1B  = 0x80;				// Impulsdauer des Kanalimpulses
 
 TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
 TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
 } // end timer1
 */
/*
 void timer1_stop(void)
 {
 // TCCR1A = 0;
 
 }
 */
/*
 ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
 {
 
 KANAL_HI;
 impulscounter++;
 
 if (impulscounter < ANZ_POT)
 {
 // Start Impuls
 
 TCNT1  = 0;
 //KANAL_HI;
 
 // Laenge des naechsten Impuls setzen
 
 //OCR1A  = POT_FAKTOR*Pot_Array[1]; // 18 us
 //OCR1A  = POT_FAKTOR*Pot_Array[impulscounter]; // 18 us
 
 }
 }
 */

/*
 ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 ms
 {
 //PORTB &= ~(1<<PORTB5); // OC1A Ausgang
 KANAL_LO;
 
 if (impulscounter < ANZ_POT)
 {
 }
 else
 {
 timer1_stop();
 
 }
 }
 */


void timer0 (void) // Grundtakt fuer Stoppuhren usw.
{
   // Timer fuer Exp
   //TCCR0 |= (1<<CS01);						// clock	/8
   //TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
   //TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
   //TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
   
   //TCCR0B |= (1 << CS02);//
   //TCCR0B |= (1 << CS00);
   
   // TCCR0B |= (1 << CS10); // Set up timer
   
   //OCR0A = 0x02;
   
   //TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
   //TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
   //TCNT0 = 0;					//RŸcksetzen des Timers
   
   
   // chan_n
   /* Start 100Hz system timer with TC0 */
   OCR0A = F_CPU / 1024 / 100 - 1;
   TCCR0A |= (1<<WGM01);
   TCCR0B |= (1 << CS02);//
   TCCR0B |= (1 << CS00);
   
   //TCCR0B = 0b101;
   TIMSK0 |= (1<<OCIE0A);
   // lcd_putc('9');
   hoststatus |= (1<<MESSUNG_OK); // Messung ausloesen

}

/*
 void timer2 (uint8_t wert)
 {
 //timer2
    TCCR2B |= ( (1<<CS22) | (1<<CS20));
    TIMSK2 |= ( (1<<TOIE2));
 }

*/
volatile uint16_t timer2Counter=0;
volatile uint16_t timer2BatterieCounter=0;

/*---------------------------------------------------------*/
/* 100Hz timer interrupt generated by OC0A                 */
/*---------------------------------------------------------*/

#pragma mark TIMER0_COMPA
ISR(TIMER0_COMPA_vect)
{
   //OSZIA_LO;
   //lcd_putc('4');
   //lcd_putc('+');
   mmc_disk_timerproc();	/* Drive timer procedure of low level disk I/O module */
   
   if ((writecounter1 % ADCTAKT) == 0)
   {
      if (hoststatus & (1<<DOWNLOAD_OK))// Download von SD, Messungen unterbrechen
      {
         
      }
      else
      {
         hoststatus |= (1<<TEENSY_ADC_OK);
      }
   }
  
   writecounter1++;
   //if (writecounter1 >= WRITETAKT) // 1s
   if ((writecounter1 % WRITETAKT) == 0)// 1s
   {
      
      intervallcounter++;
      if (intervallcounter >= intervall)
      {
         intervallcounter = 0;
         if (hoststatus & (1<<DOWNLOAD_OK))// Download von SD, Messungen unterbrechen
         {
            
         }
         else
         {
            hoststatus |= (1<<MESSUNG_OK); // Messung ausloesen
            wl_callback_status_def = wl_callback_status;
         }
      }
      
//      writecounter1=0;
      writecounter2++;
      //      lcd_gotoxy(0,2);
      //      lcd_putint12(writecounter2);
      if (writecounter2 >= 0x0002)
      {
         mmcstatus |= (1<<WRITENEXT);
         writecounter2 = 0;
      }
   }
   
   if ((wl_spi_status & (1<<WL_DATA_PENDENT)))
   {
      datapendcounter++;
   }
   //OSZIA_HI;
}


#pragma mark TIMER0_OVF

ISR (TIMER0_OVF_vect)
{
   lcd_putc('+');
   mscounter++;
   
   if (mscounter%BLINK_DIV ==0)
   {
      blinkcounter++;
   }
   
}

#pragma mark timer1
void timer1(void)
{
   
   //SERVODDR |= (1<<SERVOPIN0);
   /*
    TCCR1A = (1<<WGM10)|(1<<COM1A1)   // Set up the two Control registers of Timer1.
    |(1<<COM1B1);             // Wave Form Generation is Fast PWM 8 Bit,
    TCCR1B = (1<<WGM12)|(1<<CS12)     // OC1A and OC1B are cleared on compare match
    |(1<<CS10);               // and set at BOTTOM. Clock Prescaler is 1024.
    
    OCR1A = 63;                       // Dutycycle of OC1A = 25%
    //OCR1B = 127;                      // Dutycycle of OC1B = 50%
    
    return;
    */
   // https://www.mikrocontroller.net/topic/83609
   
   
 //  OCR1A = 0x3E8;           // Pulsdauer 1ms
   OCR1A = 0x300; // 768 Bereich 512 -1024
   //OCR1A = Servoposition[2];
   //OCR1B = 0x0FFF;
   ICR1 = 0x6400;          // 0x6400: Pulsabstand 50 ms
   // http://www.ledstyles.de/index.php/Thread/18214-ATmega32U4-Schaltungen-PWM/
   DDRB |= (1<<DDB6)|(1<<DDB5);
   
   TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)|(1<<WGM11)|(1<<WGM12);
   
   TCCR1B |= (1<<WGM13)|(1<<CS11);
   //   TCCR1A=0xAA;
   //   TCCR1B=0x19;
   // TCCR1B |= (1<<CS10);
   
   
   
   //  TIMSK |= (1<<OCIE1A) | (1<<TICIE1); // OC1A Int enablad
}


#pragma mark INT0
ISR(INT0_vect) // Interrupt bei CS, falling edge
{
   //OSZIB_LO;
   inindex=0;
   //wl_status = wl_module_get_status();
   
   wl_spi_status |= (1<<WL_ISR_RECV);
   wl_isr_counter++;
   //OSZIB_HI;
}





#pragma mark PIN_CHANGE
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328

/*
 ISR (PCINT0_vect)
 {
 
 if(INTERRUPT_PIN & (1<< MASTER_EN)// LOW to HIGH pin change, Sub ON
 {
 //OSZI_C_LO;
 
  
 }
 else // HIGH to LOW pin change, Sub ON
 {
 displaystatus |= (1<<UHR_UPDATE);
 //masterstatus &= ~(1<<SUB_TASK_BIT);
 }
 
 }
 */






/*
 //#define OSZIA_LO() OSZIPORT &= ~(1<<4)
 #define OSZI_A_HI() OSZIPORT |= (1<<4)
 #define OSZI_A_TOGG() OSZIPORT ^= (1<<4)
 */

//#define OSZIA_LO( bit) PORTD &= ~(1 << (PULSA))

#define setbit(port, bit) (port) |= (1 << (bit))
#define clearbit(port, bit) (port) &= ~(1 << (bit))



uint16_t writerand(uint16_t wert)
{
   uint16_t s1 = 50*(sin(M_PI * wert / 180.0))+100;
   s1 += 20*(cos((M_PI  * wert / 130.0)*7));
   s1 += 10*(sin((M_PI  * wert / 50.0)*3));
   
   return s1;
}

uint8_t writelin(uint16_t wert)
{
   uint8_t s1 = wert%48;
   
   return s1;
}
// pi 3.14159 26535

void clear_sendbuffer(void)
{
   for (int i=0;i<USB_PACKETSIZE;i++)
   {
      sendbuffer[i] = 0;
   }

}
void manuellmessung_start(void)
{
//   hoststatus |= (1<<USB_READ_OK);
   lcd_gotoxy(0,3);
   lcd_puts("SD ");
   messungcounter = 0;
   blockdatacounter = 0;            // Zaehler fuer Data auf dem aktuellen Block
   mmcwritecounter = 0; // Zaehler fuer Messungen auf MMC
   
   //hoststatus |= (1<<USB_READ_OK);
   saveSDposition = 0; // Start der Messung immer am Anfang des Blocks
   intervall = eeprom_read_word(&eeprom_intervall);
   lcd_putint2(intervall);
   blockcounter = eeprom_read_word(&eeprom_startblock);
   lcd_putint2(blockcounter);
   
   // Kanalbytes fuer jedes device schreiben. 0 heist device inaktiv
   uint8_t  delta = 0;
   
   uint8_t kan = 0;
   for(kan = 0;kan < 4;kan++)
   {
      kanalstatusarray[kan] = eeprom_read_byte(&eeprom_kanalstatus_array[kan]);
   }
   
   lcd_gotoxy(8,2);
   for(kan = 0;kan < 4;kan++)
   {
      lcd_putint2(kanalstatusarray[kan]);
   }
   
   delay_ms(100);          

    sd_status |= (1<<SAVE_SD_RUN_BIT);   // fortlaufendes Schreiben starten
}

void manuellmessung_stop(void)
{
   lcd_gotoxy(0,3);
   lcd_puts("--");

   hoststatus &= ~(1<<USB_READ_OK);
   
   hoststatus  |= (1<<TEENSY_MMC_OK); // resc fuer Daten
   // lcd_clr_line(1);
   //lcd_gotoxy(12,0);
   //lcd_putc('h');
   //lcd_putc(':');
   //lcd_puthex(code); // code
   //lcd_putc('*');
   
   eeprom_update_word(&eeprom_blockcount,blockcounter);
   eeprom_update_word(&eeprom_messungcount,messungcounter);
   sd_status &= ~(1<<SAVE_SD_RUN_BIT);   // fortlaufendes Schreiben stoppen
   sd_status |= (1<<SAVE_SD_STOP_BIT);
}

// MARK:  - main
int main (void)
{
   uint8_t payload[wl_module_PAYLOAD];
   payload[0] = 3;
   payload[1] = 0;
   payload[2] = 1;
   payload[3] = 4;
   payload[4] = 1;
   payload[5] = 5;
   payload[6] = 9;
   payload[7] = 2;
   payload[8] = 6;

   uint16_t tempwert = 444;
   int8_t r;
   
   uint16_t spi_count=0;
   
   // set for 16 MHz clock
   CPU_PRESCALE(CPU_8MHz); // Strom sparen
   
   timer0();
   sei();
   Master_Init();
   SPI_PORT_Init();
   SPI_Master_init();
   
   
   uint8_t usb_readcount =0x00;
   
   // ---------------------------------------------------
   // initialize the LCD
   // ---------------------------------------------------
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
   lcd_puts("Guten Tag\0");
   delay_ms(100);
   lcd_cls();
   //lcd_puts("READY\0");
   lcd_puts("V: \0");
   lcd_puts(VERSION);
   lcd_clr_line(1);
   
   lcd_gotoxy(0,0);
   lcd_puts("Data_Logger\0");
   delay_ms(1000);
   //lcd_cls();
   lcd_clr_line(0);
   
   
   initADC(0);
   
   /*
   uint8_t outData = 25;
   char buffer[5]={};
   itoa(outData, buffer,16);
   lcd_gotoxy(10,3);
   lcd_putc(buffer[0]);
   lcd_putc(' ');
   lcd_putc(buffer[1]);
    */
   // ---------------------------------------------------
   // in attach verschoben, nur wenn USB eingesteckt
   
   // Initialize the USB, and then wait for the host to set configuration.
   // If the Teensy is powered without a PC connected to the USB port,
   // this will wait forever.
   
   usb_init();
   
   uint16_t usbwaitcounter = 0;
   //lcd_putc('a');
   // Ueberspringen wenn kein Host eingesteckt
   //hoststatus &= ~(1<< TEENSYPRESENT);
   
   // checken, ob USB funktioniert
   while ((usbwaitcounter < 0xFFFA))// && (!usb_configured()))
   {
      //lcd_gotoxy(0,3);
      //lcd_putint12(usbwaitcounter);
      _delay_ms(1);
      if (usb_configured())
      {
         // status setzen und weiterfahren
         hoststatus |= (1<< TEENSYPRESENT);
         //hoststatus = 1;
         //lcd_gotoxy(19,3);
         //lcd_putc('$');
         hoststatus &= ~(1<<MANUELL_OK); // MANUELL OFF
         
         break;
         
      }
      // noch etwas warten
      usbwaitcounter++;
      if (usbwaitcounter > 0x100)
      {
         //lcd_gotoxy(19,3);
         //lcd_putc('!');
         break;
      }
   }
  
   // erfolg anzeigen
   lcd_gotoxy(19,0);
   if (hoststatus & (1<<TEENSYPRESENT))
   {
      lcd_putc('$');
      hoststatus &= ~(1<<MANUELL_OK);
   }
   else
   {
      lcd_putc('Y');
      //hoststatus |= (1<<MANUELL_OK);
      
   }
   /*
    lcd_putint12(usbwaitcounter);
    lcd_putc(' ');
    lcd_puthex(hoststatus);
    lcd_putc('*');
    lcd_puthex(usb_configured());
    lcd_putc('*');
    */
   // Wait an extra second for the PC's operating system to load drivers
   // and do whatever it does to actually be ready for input
   _delay_ms(100);
   
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;
   
   /*
    Bit 0: 1 wenn wdt ausgelšst wurde
    */
   
   PWM = 0;
   
   char* versionstring[4] = {};
   strncpy((char*)versionstring, (VERSION+9), 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi((char*)versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   uint8_t anzeigecounter=0;
   
   
   timer1();
   sei();
   
   
   
#pragma mark MMC init
   //lcd_gotoxy(0,0);
   //lcd_putc('a');
   
   DSTATUS initerr = mmc_disk_initialize();
   //lcd_putc('b');
   
   //lcd_gotoxy(0,0);
   //lcd_putc('*');
   //lcd_puthex(initerr);
   //lcd_putc('*');
   
   if (initerr)
   {
      lcd_gotoxy(0,0);
      lcd_puts("SD-");
      lcd_puthex(initerr);
      lcd_putc('*');
      
   }
   else
   {
      lcd_gotoxy(0,0);
      lcd_puts("SD+");
      //lcd_puthex(initerr);
      //lcd_putc('*');
      
   }
   
   /*
    FRESULT mounterr = f_mount((void*)FatFs,"",1);
    lcd_gotoxy(16,0);
    lcd_puthex(mounterr);
    */
   
   DRESULT readerr=0;
   DRESULT writeerr=0;
   
   //mmcbuffer[0] = 'A';
   // mmcbuffer[1] = 'B';
   // mmcbuffer[2] = 'C';
   //mmcbuffer[0] = 0;
   
   readerr = mmc_disk_read ((void*)mmcbuffer,1,	1);
   //readerr = mmc_disk_write ((void*)mmcbuffer,1,	1);
   
   //   lcd_gotoxy(0,1);
   
   //   lcd_puthex(readerr);
   
   
   //   lcd_putc('*');
   if (readerr==0)
   {
      //      lcd_putc('+');
      uint16_t i=0;
      for (i=0;i<8;i++)
      {
         if (mmcbuffer[i])
         {
            //            lcd_puthex(mmcbuffer[i]);
         }
      }
      //      lcd_putc('+');
   }
   //  lcd_putc('*');
   
   
   
   
#pragma mark DS1820 init
   
   // DS1820 init-stuff begin
   // OW_OUT |= (1<<OW;
   uint8_t i=0;
   /*
   uint8_t nSensors=0;
   uint8_t err = ow_reset();
   //   lcd_gotoxy(18,0);
   //   lcd_puthex(err);
   
   
   gNsensors = search_sensors();
   
   delay_ms(100);
   if (gNsensors>0)
   {
      lcd_gotoxy(0,0);
      lcd_puts("Sn:\0");
      lcd_puthex(gNsensors);
      
      lcd_clr_line(1);
      start_temp_meas();
      
      
   }
   i=0;
   while(i<MAXSENSORS)
   {
      gTempdata[i]=0;
      i++;
   }
   */
   // DS1820 init-stuff end
   
   
   
   // ---------------------------------------------------
   // Vorgaben fuer Homescreen
   // ---------------------------------------------------
   //substatus |= (1<<SETTINGS_READ);;
   // ---------------------------------------------------
   // Settings beim Start lesen
   // ---------------------------------------------------
   
   // timer1(); PORTB5,6
   
   
   
   volatile   uint8_t old_H=0;
   volatile   uint8_t old_L=0;
   uint8_t teensy_err =0;
   uint8_t testfix=0;
   
  
   
   OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
   
   
   //  OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer OSZI
   //  OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
   
   //DDRD |= (1<<4);
   //PORTD |= (1<<4);
   //OSZIA_HI;
   //OSZIPORT |= (1<<OSZI_PULS_A);
   
   
   
   
  if (TEST || (!(OSZIPORT & (1<<TEST_PIN)))) // Testdaten schreiben
  {
     uint16_t ii=0;
     uint16_t wert = 0;
     for (uint8_t block = 1;block < 5;block++)
     {
        
        while (ii<0xF0)
        {
           wert += 3 ;
           mmcbuffer[2*ii] = wert & 0x00FF;
           mmcbuffer[2*ii+1] = (wert & 0xFF00)>>8;
           
           ii += 1;
           if (wert > 133)
           {
              wert = 0;
           }
           
           
        }
        //lcd_gotoxy(0,3);
        //lcd_puthex(block);
        
        writeerr = mmc_disk_write ((void*)mmcbuffer,block ,1); // Block 0 ist system
        //lcd_putc('e');
        //lcd_puthex(writeerr);
        
        ii = 0;
     }
     
     // OSZIA_HI;
     
     lcd_gotoxy(6,3);
     lcd_puts("save ");
     lcd_puthex(writeerr);
     lcd_putc(' ');
     lcd_puthex(blockcounter);
  }
   
   // MARK: WL init
   
   
   uint8_t maincounter =0;
   //Array for Payload
   
   
   wl_module_init();
   
   
   _delay_ms(10);
   
 //  wl_module_tx_config(wl_module_TX_NR_0);
   
   
   uint8_t readstatus = wl_module_get_data((void*)&wl_data);
   
   uint16_t temperatur0=0;

   // MARK: +++++++++    WHILE
   sei();
    uint8_t clicktaste = 0;
   while (1)
   {
      
      //Blinkanzeige
      loopcount0+=1;
      if ((loopcount0 & 0x2FF) == 0)
      {
         if (!(hoststatus & (1<<TEENSYPRESENT))) // tastatur nur ohne host
         {
            DDRF &= ~(1<<4); //ADC4
            //Tastenwert=(adc_read(4)>>2);
            //lcd_putint(Tastenwert);
            
            uint8_t code = tastencode(4); // kanal 4
            
            //lcd_gotoxy(10,3);
            if (code)
            {
               lcd_gotoxy(10,3);
               lcd_putc('T');
               lcd_putint2(code);
               tastaturcounter = 0;
               
               
            }
            else
            {
               tastaturcounter++;
               if (tastaturcounter > 0xAF)
               {
                  lcd_gotoxy(10,3);
                  lcd_puts("   ");
                  tastaturcounter=0;
               }
            }
         }// end tastatur
      }
      // ********
      
      
      if (wl_spi_status & (1<<WL_ISR_RECV)) // in ISR gesetzt, ETWAS LOS AUF WL
      {
         
             
         OSZIB_LO;
         pipenummer=0;
         
         
         /* reset
          https://devzone.nordicsemi.com/question/5930/is-there-any-way-to-reset-nrf24l01/
          1)use power down mode (PWR_UP = 0)
          2)clear data ready flag and data sent flag in status register
          3)flush tx/rx buffer
          4)write status register as 0x0e;
          
          */
         
         wl_status = wl_module_get_status();
         
         /*
          lcd_putc(' ');
          lcd_gotoxy(10,2);
          lcd_puthex(wl_status);
          */
         //delay_ms(1);
         //OSZIB_LO;
         pipenummer = wl_module_get_rx_pipe_from_status(wl_status); // Pipenummer abfragen aus Daten von device
         
         delay_ms(1);
         
         //        wl_spi_status &= ~(1<<WL_ISR_RECV);
         
         if (pipenummer == 7) // ungueltige pipenummer
         {
            wl_module_get_one_byte(FLUSH_TX); // putzen
            delay_ms(1);
         }
         else
         {
            if (wl_status & (1<<TX_FULL))
            {
               lcd_gotoxy(16,2);
               lcd_putc('F');
               wl_module_config_register(STATUS, (1<<TX_FULL)); //Clear Interrupt Bit
            }
            
            // ******* RX_DR
            
            if (wl_status & (1<<RX_DR)) // IRQ: Package has been received
            {
               
               // Kontrolle, ob payloadlength ok
               uint8_t rec = wl_module_get_rx_pw(pipenummer); //gets the RX payload width
               delay_ms(1);
               //lcd_gotoxy(0,3);
               if (!(rec==0x10))
               {
                  lcd_gotoxy(14,1);
                  lcd_putc('!');
                  lcd_puthex(rec);
               }
               
               //	*********************************************************************                
               // MARK: WL get Data
               //	*********************************************************************                
               // payload lesen
               uint8_t readstatus = wl_module_get_data((void*)&wl_data); // returns status
               delay_ms(1);
               
               batteriespannung = (wl_data[BATT]);
               
               sendbuffer[BATT + DATA_START_BYTE] = 0;
               
               //      sendbuffer[DEVICE + DATA_START_BYTE] = wl_data[DEVICE]& 0x0F; // Wer sendet Daten? Sollte Devicenummer sein
               // task je nach channelnummer
               sendbuffer[DEVICE + DATA_START_BYTE] = 0;
               sendbuffer[DATA_START_BYTE -1] = 117;
               devicenummer = wl_data[DEVICE]& 0x0F;
               int codenummer = wl_data[DEVICE]& 0xF0;
               
               devicebatteriespannung[devicenummer] = batteriespannung;
               // lcd_gotoxy(14,3);
               // lcd_puts("  "); // delta loeschen
               //lcd_gotoxy(6,2);
               //lcd_puts("     ");
               
               wl_callback_status |= (1<<0);
               //sendbuffer[ANALOG3 + DATA_START_BYTE + 1]= linecounter++;
               switch(devicenummer)
               {
                     
                  case 0:
                  {
                     
                     /* 
                      sendbuffer[USB_BATT_BYTE] = 17;
                      sendbuffer[DEVICE + DATA_START_BYTE] = 0; // teensy
                      sendbuffer[ANALOG0  + DATA_START_BYTE]= 0; // LM335
                      sendbuffer[ANALOG0+1  + DATA_START_BYTE]= 0;
                      
                      sendbuffer[ANALOG1  + DATA_START_BYTE]= 1; // KTY
                      sendbuffer[ANALOG1+1  + DATA_START_BYTE]= 1;
                      
                      sendbuffer[ANALOG2  + DATA_START_BYTE]= 2; // PT1000
                      sendbuffer[ANALOG2+1  + DATA_START_BYTE]= 2;
                      
                      sendbuffer[ANALOG3  + DATA_START_BYTE]= 3;//wl_data[ANALOG3]; // 
                      sendbuffer[ANALOG3+1  + DATA_START_BYTE]= 3;//wl_data[ANALOG3+1];
                      
                      */ 
                     // sendbuffer[USB_PACKETSIZE-1] = 51;
                     
                  }break;
                     
                  case 1: // TEMPERATUR
                  {
                     devicecount++;
                     wl_callback_status |= (1<<devicenummer);
                     //lcd_gotoxy(8,2);
                     //lcd_putc('c');
                     //lcd_putint1(wl_data[DEVICE]); // devicenummer
                     sendbuffer[USB_BATT_BYTE] = wl_data[BATT];
                     //sendbuffer[BATT  + DATA_START_BYTE]= wl_data[BATT]; // Batteriespannung des device BATT ist 2
                     
                     sendbuffer[DEVICE + DATA_START_BYTE] = wl_data[DEVICE]& 0x0F; // Wer sendet Daten? Sollte Devicenummer sein
                     //sendbuffer[CHANNEL + DATA_START_BYTE] = wl_data[CHANNEL]& 0x0F; // Kanal
                     
                     sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Byte 3, 4
                     sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                     
                     
                     //      temperatur0 = (wl_data[ANALOG2+1]<<8); // LM335
                     //      temperatur0 |= wl_data[ANALOG2];
                     
                     //temperatur0 = temperatur1;
                     //lcd_gotoxy(16,2);
                     //lcd_putc('A');
                     /*
                      lcd_gotoxy(0,2);
                      lcd_putc('t');
                      lcd_putc('0');
                      lcd_putc(' ');
                      lcd_putint(temperatur0);
                      lcd_putc(' ');
                      lcd_puthex(wl_data[12]);
                      lcd_puthex(wl_data[13]);
                      */
                     
                     sendbuffer[ANALOG0  + DATA_START_BYTE]= wl_data[ANALOG0]; // LM335
                     sendbuffer[ANALOG0+1  + DATA_START_BYTE]= wl_data[ANALOG0+1];
                     
                     sendbuffer[ANALOG1  + DATA_START_BYTE]= wl_data[ANALOG1]; // KTY
                     sendbuffer[ANALOG1+1  + DATA_START_BYTE]= wl_data[ANALOG1+1];
                     
                     sendbuffer[ANALOG2  + DATA_START_BYTE]= wl_data[ANALOG2]; // PT1000
                     sendbuffer[ANALOG2+1  + DATA_START_BYTE]= wl_data[ANALOG2+1];
                     
                     
                     uint16_t a2 = wl_data[ANALOG2] | (wl_data[ANALOG2+1]<<8); // Temp PT 1000
                     
                     /*
                     lcd_gotoxy(14,1);
                     lcd_putc('A');
                     lcd_putint((a2 & 0xFF));
                     */
                     sendbuffer[ANALOG3  + DATA_START_BYTE]= 0;//wl_data[ANALOG3]; // 
                     sendbuffer[ANALOG3+1  + DATA_START_BYTE]= 0;//wl_data[ANALOG3+1];
                     
                     
                     sendbuffer[USB_PACKETSIZE-1] = 81;
                     
                     
                     
                     //	********************************************************************* 
                     //#pragma mark MMC Save device 1
                     //	*********************************************************************                     
                     
                     loggerstatus |= (1<<logpend); // ein mal senden
                     
                  }break;
                     
                  case 2: // ADC12BIT
                  {
                     devicecount++;
                     wl_callback_status |= (1<<devicenummer);
                     
                     sendbuffer[USB_BATT_BYTE] = wl_data[BATT];
                     //sendbuffer[BATT  + DATA_START_BYTE]= wl_data[BATT]; // Batteriespannung des device
                     
                     sendbuffer[DEVICE + DATA_START_BYTE] = wl_data[DEVICE]& 0x0F; // Wer sendet Daten? Sollte Devicenummer sein
                     //sendbuffer[CHANNEL + DATA_START_BYTE] = wl_data[CHANNEL]& 0x0F; // Wer sendet Daten? Sollte Devicenummer sein
                     
                     sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Byte 3, 4
                     sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                     
                     //       temperatur1 = (wl_data[ANALOG2+1]<<8);
                     //      temperatur1 |= wl_data[ANALOG2];
                     
                     /*
                      lcd_gotoxy(0,3);
                      lcd_putc('t');
                      lcd_putc('2');
                      lcd_putc(' ');
                      lcd_putint(temperatur0);
                      */
                     sendbuffer[USB_BATT_BYTE] = wl_data[BATT];
                     
                     sendbuffer[ANALOG0 + DATA_START_BYTE]= wl_data[ANALOG0];
                     sendbuffer[ANALOG0+1 + DATA_START_BYTE]= wl_data[ANALOG0+1];
                     
                     uint16_t temp = (wl_data[ANALOG0+1] <<8 |  wl_data[ANALOG0]);
                     
                     ADC_Array[0] =  temp; // (wl_data[ANALOG0+1] <<8 |  wl_data[ANALOG0]);
                     
                     sendbuffer[ANALOG1 + DATA_START_BYTE]= wl_data[ANALOG1];
                     sendbuffer[ANALOG1+1 + DATA_START_BYTE]= wl_data[ANALOG1+1];
                     
                     temp = (wl_data[ANALOG1+1] <<8 | wl_data[ANALOG1]);
                     
                     ADC_Array[1] =  temp; // (wl_data[ANALOG1+1] <<8 |  wl_data[ANALOG1]);
                     
                     sendbuffer[ANALOG2 + DATA_START_BYTE]= wl_data[ANALOG2];
                     sendbuffer[ANALOG2+1 + DATA_START_BYTE]= wl_data[ANALOG2+1];
                     
                     temp = (wl_data[ANALOG2+1] <<8 |  wl_data[ANALOG2]);
                     ADC_Array[2] =  temp; //(wl_data[ANALOG2+1] <<8);
                     
                     sendbuffer[ANALOG3 + DATA_START_BYTE]= wl_data[ANALOG3];
                     sendbuffer[ANALOG3+1 + DATA_START_BYTE]= wl_data[ANALOG3+1];
                     temp = (wl_data[ANALOG3+1] <<8 |  wl_data[ANALOG3]);
                     ADC_Array[3] =  temp; //(wl_data[ANALOG3+1] <<8 |  wl_data[ANALOG3]);
                     
                     // Batterierspannung  device 2
                     //spannung0 = (wl_data[BATT]);
                     
                     sendbuffer[USB_PACKETSIZE-1] = 82;
                     
                     loggerstatus |= (1<<logpend); // ein mal senden
                     
                     
                  }break; // ADC12BIT
                     
                  case 3:
                  {
                     devicecount++;
                     sendbuffer[USB_PACKETSIZE-1] = 83;
                     loggerstatus |= (1<<logpend); // ein mal senden
                  }break;
                  case 4:
                  {
                     sendbuffer[USB_PACKETSIZE-1] = 84;
                     loggerstatus |= (1<<logpend); // ein mal senden
                     devicecount++;
                  }break;
                     
                     
               }// switch devicenummer
               lcd_gotoxy(16,0);
               lcd_puthex(wl_callback_status);
               delay_ms(1);
               // start mmc
#pragma mark MMC Save             
               //	********************************************************************* 
               // SD laden
               //	*********************************************************************                     
               //lcd_gotoxy(6,2);
               //lcd_puts("           ");
               
               if ((sd_status & (1<<SAVE_SD_RUN_BIT)) || (hoststatus & (1<<MANUELL_OK)))// Daten in mmcbuffer speichern,                      
               {
                  // ****
                  writedatacounter++; // TEST: weitere Messung auf MMC

                  if (hoststatus & (1<<TEENSY_MMC_OK)) // teensy-daten bereit
                  {
                     uint8_t delta=0;
                     lcd_gotoxy(6,3);
                     lcd_putc('D');
                     lcd_putint1(devicenummer);
                     lcd_putc(' ');
                     //lcd_gotoxy(8,3);
                     lcd_putc('B');
                     lcd_putint2(kanalstatusarray[devicenummer]);
                     lcd_putc(' ');
                     lcd_putint(homeADCArray[0] & 0x00FF);
                     
                     mmcbuffer[saveSDposition+delta++] = 0;  // teensy
                     mmcbuffer[saveSDposition+delta++] = 0; //
                     mmcbuffer[saveSDposition+delta++] = (messungcounter & 0x00FF);
                     mmcbuffer[saveSDposition+delta++] = ((messungcounter & 0xFF00)>>8);
                     
                     //Byte 4
                     mmcbuffer[saveSDposition+delta++] = kanalstatusarray[0]; // ist kanal aktiv
                     // positionen fuellen bis byte 7
                     mmcbuffer[saveSDposition+delta++] = 98; //
                     mmcbuffer[saveSDposition+delta++] = 99;
                     mmcbuffer[saveSDposition+delta++] = 117; // end code 
                     
                     delta = 8; // Beginn Data-Block, 24 bytes
                    
                     mmcbuffer[saveSDposition+delta++] = (homeADCArray[0] & (0x00FF))  ; // LO
                     mmcbuffer[saveSDposition+delta++] = (homeADCArray[0] && 0xFF00) >> 8; // HI
                     mmcbuffer[saveSDposition+delta++] = homeADCArray[1] & (0x00FF); // LO
                     mmcbuffer[saveSDposition+delta++] = (homeADCArray[1] && 0xFF00) >> 8; // HI
                     mmcbuffer[saveSDposition+delta++] = homeADCArray[2] & (0x00FF); // LO
                     mmcbuffer[saveSDposition+delta++] = (homeADCArray[2] && 0xFF00) >> 8; // HI
                     mmcbuffer[saveSDposition+delta++] = homeADCArray[3] & (0x00FF); // LO
                     mmcbuffer[saveSDposition+delta++] = (homeADCArray[3] && 0xFF00) >> 8; // HI
                     
                     //writedatacounter
                     mmcbuffer[saveSDposition+delta++] = writedatacounter; // fortlaufener counter
                     mmcbuffer[saveSDposition+delta++] = 14; // Grenze

                     
                     hoststatus &= ~(1<<TEENSY_MMC_OK);
                     
                     
                     mmcwritecounter += 24; // Zaehlung write-Prozesse, 24 bytes pro device
                     
                     saveSDposition += 24; // 8 bit Admin, 16 bit Data
                     
                     
                     // neues Paket
                     // Daten ans Ende des Blocks schreiben
                     
                     delta = 0;
                     mmcbuffer[BLOCK_SIZE + delta++] = 0xFF; // Voller Block, Startblock: Gannzen block schreiben
                     mmcbuffer[BLOCK_SIZE + delta++] = 0xFF;
                     mmcbuffer[BLOCK_SIZE + delta++] = blockcounter & 0x00FF; // Nummer des geschriebenen Blocks lo
                     mmcbuffer[BLOCK_SIZE + delta++] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                     
                     mmcbuffer[BLOCK_SIZE + delta++] = (messungcounter & 0x00FF);   // Gesamte Anzahl Messungen
                     mmcbuffer[BLOCK_SIZE + delta++] = ((messungcounter & 0xFF00)>>8);
                     
                     mmcbuffer[BLOCK_SIZE + delta++] = (wl_callback_status_check);   // wl_callback_status
                     
                     mmcbuffer[BLOCK_SIZE + delta++] = 231;
                     
                     mmcbuffer[BLOCK_SIZE + delta++] = (intervall & 0x00FF);   // Intervall
                     mmcbuffer[BLOCK_SIZE + delta++] = ((intervall & 0xFF00)>>8);

                     if ((saveSDposition ) >= BLOCK_SIZE) // Block voll, 480 Bytes 
                     {
                        
                        
                        blockdatacounter = 0; //Zaheler resetten fuer neuen Block
                        
                        
                        writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
                        // OSZIA_HI;
                        lcd_gotoxy(8,2);
                        lcd_puts("save_t");
                        lcd_putint1(writeerr);
                        lcd_putc(' ');
                        lcd_puthex(blockcounter);
                        saveSDposition = 0;
                        sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Nummer des geschriebenen Blocks lo
                        sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                        
                        blockcounter++;
                        
                     }
                     
                     
                     // MMC Teensy end
                     
                     
                     
                     hoststatus |= (1<<TEENSY_MMC_OK); // Daten bei next gelegenheit in mmc schreiben
                     
                  } // if TEENSY_MMC_OK
                  
                  
                  
                  // ****
                  
                  
                  lcd_gotoxy(16,3);
                  lcd_puthex(writedatacounter);
                  if (hoststatus & (1<<MANUELL_OK))
                  {
                     lcd_putint1(devicenummer);
                  }
                  
                  blockdatacounter++; // weitere Messung auf aktuellem Block der MMC
                  
                  
                  
                  //lcd_gotoxy(7,3);
                  //lcd_putint12(saveSDposition); // 0 .. 255, pos im mmcbuffer, immer 2 byte pro messung
                  //lcd_putint(blockcounter);
                  
                   
                  if (devicenummer > 0)
                  {
                     uint8_t delta=0;
                     
                     mmcbuffer[saveSDposition+delta++] = devicenummer; 
                     mmcbuffer[saveSDposition+delta++] = wl_data[DEVICE];
                     mmcbuffer[saveSDposition+delta++] = (messungcounter & 0x00FF);
                     mmcbuffer[saveSDposition+delta++] = ((messungcounter & 0xFF00)>>8);
                     
                     //Byte 4
                     mmcbuffer[saveSDposition+delta++] = kanalstatusarray[devicenummer]; // ist kanal aktiv
                     // positionen fuellen bis byte 7
                     mmcbuffer[saveSDposition+delta++] = 112; //loggertestwert++;
                     
                     mmcbuffer[saveSDposition+delta++] = 112;
                     mmcbuffer[saveSDposition+delta++] = 113; // end code 
                     // Data ab Byte HEADER_SIZE (8)
                     // analoge Kanaele je 16bit
                     // begin data
                     delta = 8;
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG0]; 
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG0+1];
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG1]; // KTY
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG1+1];
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG2]; // PT1000
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG2+1];
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG3]; // 
                     mmcbuffer[saveSDposition+delta++] = wl_data[ANALOG3+1];
                     
                     //mmcbuffer[saveSDposition+delta++] = 78;
                     //mmcbuffer[saveSDposition+delta++] = 79;
                     
                     // Test fuer Linearitaet
                     
                     uint8_t delta0 = delta;
                     //writedatacounter
                     mmcbuffer[saveSDposition+delta++] = writedatacounter; // fortlaufener counter
                     mmcbuffer[saveSDposition+delta++] = 13;
                     mmcbuffer[saveSDposition+delta++] = delta0++;
                     mmcbuffer[saveSDposition+delta++] = 0;
                     mmcbuffer[saveSDposition+delta++] = delta0++;
                     mmcbuffer[saveSDposition+delta++] = 0;
                     mmcbuffer[saveSDposition+delta++] = delta0++;
                     mmcbuffer[saveSDposition+delta++] = 0;
                     mmcbuffer[saveSDposition+delta++] = delta0++;
                    // mmcbuffer[saveSDposition+delta++] = 0;
                    // mmcbuffer[saveSDposition+delta++] = delta0++;
                     mmcbuffer[saveSDposition+PACKET_SIZE-1] = delta0;
                     
                     /*
                      sendbuffer[ANALOG0  + DATA_START_BYTE]= wl_data[ANALOG0]; // LM335
                      sendbuffer[ANALOG0+1  + DATA_START_BYTE]= wl_data[ANALOG0+1];
                      
                      sendbuffer[ANALOG1  + DATA_START_BYTE]= wl_data[ANALOG1]; // KTY
                      sendbuffer[ANALOG1+1  + DATA_START_BYTE]= wl_data[ANALOG1+1];
                      
                      sendbuffer[ANALOG2  + DATA_START_BYTE]= wl_data[ANALOG2]; // PT1000
                      sendbuffer[ANALOG2+1  + DATA_START_BYTE]= wl_data[ANALOG2+1];
                      
                      sendbuffer[ANALOG3  + DATA_START_BYTE]= 0;//wl_data[ANALOG3]; // 
                      sendbuffer[ANALOG3+1  + DATA_START_BYTE]= 0;//wl_data[ANALOG3+1];
                      */                       
                     sendbuffer[USB_PACKETSIZE-1] = 81;
                     sendbuffer[USB_PACKETSIZE-2] = writedatacounter;
                  }
                  
                  
                  // Kontrolle
                  
                  //            lcd_gotoxy(10,3);
                  //            for (uint8_t pos = 0;pos<6;pos++) // buffer leeren ab 4
                  //            {
                  //               //lcd_puthex(mmcbuffer[saveSDposition + 4 + 2*pos] = 0;
                  //               lcd_puthex(mmcbuffer[saveSDposition + 4 + 2*pos+1]);
                  //            }
                  
                  // Testroutine
                  mmcwritecounter += 24; // Zaehlung write-Prozesse, 24 bytes pro device
                  
                  saveSDposition += 24; // 8 bit Admin, 16 bit Data
                  
                  
                  // neues Paket
                  // Daten ans Ende des Blocks schreiben
                  
                  uint8_t delta = 0;
                  mmcbuffer[BLOCK_SIZE + delta++] = 0xFF; // Voller Block, Startblock: Gannzen block schreiben
                  mmcbuffer[BLOCK_SIZE + delta++] = 0xFF;
                  mmcbuffer[BLOCK_SIZE + delta++] = blockcounter & 0x00FF; // Nummer des geschriebenen Blocks lo
                  mmcbuffer[BLOCK_SIZE + delta++] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                  
                  
                  if ((saveSDposition ) >= BLOCK_SIZE) // Block voll, 480 Bytes 
                  {
                     
                     //                          uint8_t delta = 0;
                     // Daten am Ende des Blocks mitgeben
                     //                          mmcbuffer[BLOCK_SIZE + delta++] = blockcounter & 0x00FF; // Nummer des geschriebenen Blocks lo
                     //                          mmcbuffer[BLOCK_SIZE + delta++] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                     
                     //                           mmcbuffer[BLOCK_SIZE + delta++] = 0xFF; // Voller Block
                     //                           mmcbuffer[BLOCK_SIZE + delta++] = 0xFF;
                     
                     blockdatacounter = 0; //Zaheler resetten fuer neuen Block
                     
                     
                     writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
                     // OSZIA_HI;
                     lcd_gotoxy(8,2);
                     lcd_puts("save_D");
                     lcd_putint1(writeerr);
                     lcd_putc(' ');
                     lcd_puthex(blockcounter);
                     saveSDposition = 0;
                     sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Nummer des geschriebenen Blocks lo
                     sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                     blockcounter++;
                     
                  }
               }
               else if (sd_status & (1<<SAVE_SD_STOP_BIT)) // Schreiben beenden, letzten Block noch schreiben
               {
                  sd_status &= ~(1<<SAVE_SD_STOP_BIT);
                  sd_status &= ~(1<<SAVE_SD_RUN_BIT);   // fortlaufendes Schreiben beenden
                  
                  // Daten sichern
                  
                  uint8_t delta = 0;
                  // Daten am Ende des Blocks mitgeben (32 Bytes)
                  mmcbuffer[BLOCK_SIZE + delta++] = blockdatacounter & 0x00FF;       // Anzahl Messungen auf letztem Block der SD lo
                  mmcbuffer[BLOCK_SIZE + delta++] = (blockdatacounter & 0xFF00)>>8;  // Anzahl Messungen auf letztem Block der SD hi
                  
                  mmcbuffer[BLOCK_SIZE + delta++] = blockcounter & 0x00FF;   // Nummer des zuletzt geschriebenen Blocks lo
                  mmcbuffer[BLOCK_SIZE + delta++] = (blockcounter & 0xFF00)>>8; // Nummer des zuletzt geschriebenen Blocks hi
                  
                  mmcbuffer[BLOCK_SIZE + delta++] = (messungcounter & 0x00FF);   // Gesamte Anzahl Messungen
                  mmcbuffer[BLOCK_SIZE + delta++] = ((messungcounter & 0xFF00)>>8);
                  
                  mmcbuffer[BLOCK_SIZE + delta++] = (wl_callback_status_check);   // wl_callback_status
                  
                  mmcbuffer[BLOCK_SIZE + delta++] = 231;
                  
                  mmcbuffer[BLOCK_SIZE + delta++] = (intervall & 0x00FF);   // Intervall
                  mmcbuffer[BLOCK_SIZE + delta++] = ((intervall & 0xFF00)>>8);
                  
                  
                  //mmcbuffer[DATA_START_BYTE] |= (1<<7);
                  // ** DIFF writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
                  
                  writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
                  // OSZIA_HI;
                  lcd_gotoxy(8,2);
                  lcd_puts("resc ");
                  lcd_putint1(writeerr);
                  lcd_putc(' ');
                  lcd_puthex(blockcounter);
                  lcd_putc(' ');
                  lcd_puthex(blockdatacounter);
                  saveSDposition = 0;
                  sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF;
                  sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
                  
               } //end if sd_status & (1<<SAVE_SD_STOP_BIT)
               else if (! usb_configured()) // kein USB, betrieb ohne host
               {
                  lcd_gotoxy(6,2);
                  lcd_puts("noHost");
                  
                  sd_status &= ~(1<<SAVE_SD_RUN_BIT);   //  Schreiben so oder so beenden
                  // hoststatus |= (1<<MANUELL_OK);
                  
               }
               
               // end SDladen
               
               // end mmc
               
               
               wl_callback_status_check = wl_callback_status; // behalten fuer check vom Interface
               
               delay_ms(20);
               wl_spi_status &= ~(1<<WL_DATA_PENDENT);    // Beim Senden gesetzt. Data angekommen, not busy
               
               PTX=0;
               
               wl_module_get_one_byte(FLUSH_RX);
               delay_ms(10);
               wl_module_get_one_byte(FLUSH_TX);
               
               //delay_ms(20);
               delay_ms(10);
               //if (loop_pipenummer == pipenummer)
               {
                  lcd_gotoxy(5+2*loop_channelnummer,1);
                  lcd_putc('r'); // recv anzeigen bei entsprechender pos
                  if (loop_channelnummer < 3)
                  {
                     loop_channelnummer++; // weiterschalten
                     wl_spi_status |= (1<<WL_SEND_REQUEST);
                     delay_ms(2);
 
                  }
                  else
                  {  
                     
                    }
                  
               }
               
               // MARK:  USB send          
               
               // senden an DataLogger
               if (hoststatus & (1<< TEENSYPRESENT) && hoststatus & (1<<USB_READ_OK)) // teensy da und Messreihe im Gang
               {
                  lcd_gotoxy(12,1);
                   lcd_puts("mess.");

                  //OSZIA_LO;
                  //sendbuffer[USB_PACKETSIZE-1] = 79;
                  //                 sendbuffer[0] = MESSUNG_DATA;
                  sendbuffer[2] = wl_callback_status_def;
                  uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 100);
                  //sendbuffer[0] = 0;//
                  
                  //OSZIA_HI;
               }
               
               
               
               wl_module_config_register(STATUS, (1<<RX_DR)); //Clear Interrupt Bit
               //delay_ms(50);
               //delay_ms(10);
               //              OSZIA_HI;
            }  // end if RX_DR
            
         } // if pipenummer gueltig (<7)
         
         
         if (wl_status & (1<<TX_DS)) // IRQ: Package has been sent
         {
            wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
            
            maincounter++;
            PTX=0;
            wl_module_get_one_byte(FLUSH_TX);
            //OSZIA_HI;
         }
         
         if (wl_status & (1<<MAX_RT)) // IRQ: Package has not been sent, send again
         {
            
            lcd_gotoxy(14,0);
            lcd_puts("RT");
            //wl_spi_status &= ~(1<<WL_DATA_PENDENT);    // reset, not busy
            
            wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
            //delay_ms(50);
            wl_module_config_register(STATUS, (1<<MAX_RT)); // Clear Interrupt Bit
            
            delay_ms(5);
            //wl_module_get_one_byte(FLUSH_TX);
            
            if (loop_channelnummer < 3)
            {
               loop_channelnummer++;
               wl_spi_status |= (1<<WL_SEND_REQUEST);
               
            }
            
         } // if RT
         
         //OSZIB_HI;
         //         } // if pipenummer <7
         OSZIB_HI;
         //    wl_spi_status = 0;
         
         wl_spi_status &= ~(1<<WL_ISR_RECV);

         
      } // end ISR abarbeiten (wl_spi_status & (1<<WL_ISR_RECV))
      
  
      
      
      /* **** spi_buffer abfragen **************** */
      // MARK:  spi_rxdata
      
      /* **** end spi_buffer abfragen **************** */
      
      uint16_t adcwert=0;
      float adcfloat=0;
      
      // MARK:  MMC writenext
      //   if ((mmcstatus & (1<<WRITENEXT)) )
      if (mmcstatus & (1<<WRITENEXT) ) // in ISR gesetzt, beschreiben der disc mit random
      {
         //if (usbstatus & (1<<WRITEAUTO))
         if ((usbstatus == WRITE_MMC_TEST)&& (mmcwritecounter < 512)) // Test: SD beschreiben
         {
            //uint16_t tempdata = writerand(mmcwritecounter);
            uint16_t tempdata = writerand(mmcwritecounter);
            //   OSZIA_LO;
            //     sendbuffer[16] = (tempdata & 0x00FF);
            //     sendbuffer[17] = ((tempdata & 0xFF00)>>8);
            mmcbuffer[mmcwritecounter] = (tempdata & 0x00FF);
            mmcbuffer[mmcwritecounter+1] = ((tempdata & 0xFF00)>>8);
            lcd_gotoxy(0,3);
            lcd_putint12(mmcwritecounter & 0x1FF);
            lcd_putc(' ');
            lcd_putint12(tempdata);
         }
         else
         {
            usbstatus = DEFAULT;
         }         
         mmcstatus &= ~(1<<WRITENEXT);
      }// test
      
      //      if ((hoststatus & (1<<TEENSY_ADC_OK)) && (!(hoststatus & (1<<MESSUNG_OK))) && (!(wl_spi_status & (1<<WL_SEND_REQUEST))))
   #pragma mark TEENSY_ADC_OK
      if (hoststatus & (1<<TEENSY_ADC_OK)) // in ISR gesetzt, wenn nichts anderes los ist
      {
         hoststatus &= ~(1<<TEENSY_ADC_OK);
         //setupADC(); // Bei Kanal > 7
          //uint16_t homeadc = readKanal11(1)>>4; 
         homeADCArray[0] = readKanal(1)>>2;
         lcd_gotoxy(0,3);
         lcd_putc('T');
         lcd_putint12(homeADCArray[0]);
         
         devicenummer = 0;
         
         teensybuffer[ANALOG0  + DATA_START_BYTE]= (homeADCArray[0] & 0x00FF); // Kanal 1
         teensybuffer[ANALOG0+1  + DATA_START_BYTE]= (homeADCArray[0] & 0xFF00)>>8;
         //uint16_t pt = 
          //  lcd_puthex(teensybuffer[ANALOG0  + DATA_START_BYTE]);
         // lcd_puthex(teensybuffer[ANALOG0+1  + DATA_START_BYTE]);
         
         homeADCArray[1] = readKanal(4);
         teensybuffer[ANALOG1  + DATA_START_BYTE]= homeADCArray[1] & 0x00FF; // Kanal 4
         teensybuffer[ANALOG1+1  + DATA_START_BYTE]= (homeADCArray[1] & 0xFF00)>>8;
         
         homeADCArray[2] = readKanal(10);
         //lcd_putint2(homeadc>>2);
         teensybuffer[ANALOG2  + DATA_START_BYTE]= homeADCArray[2] & 0x00FF; // Kanal 10
         teensybuffer[ANALOG2+1  + DATA_START_BYTE]= (homeADCArray[2] & 0xFF00)>>8;
         
         homeADCArray[3] = readKanal(11);
          
         teensybuffer[ANALOG3  + DATA_START_BYTE]= homeADCArray[3] & 0x00FF; // Kanal 11
         teensybuffer[ANALOG3+1  + DATA_START_BYTE]= (homeADCArray[3] & 0xFF00)>>8;
         
         teensybuffer[0] = TEENSY_DATA;
         teensybuffer[6] = wl_callback_status_def; // bisheriger status
         teensybuffer[DATACOUNT_LO_BYTE] = (messungcounter & 0x00FF);
         teensybuffer[DATACOUNT_HI_BYTE] = ((messungcounter & 0xFF00)>>8);
         
         adcwert = read_bat(0); // Batteriespannung / 2
         double adcfloat = adcwert;
         //lcd_gotoxy(10,2);
         //lcd_putc('B');
         //lcd_putint12(adcwert*2/10);
         
         adcfloat = adcfloat *2490/1024; // kalibrierung VREF, 1V zu 0.999, Faktor 10, 45 us
         // adcwert = (((uint16_t)adcfloat)&0xFFFF);
         //         lcd_putc(' ');
         //        lcd_putint(adcfloat);
         
         teensybuffer[USB_BATT_BYTE] = (uint8_t)(adcwert);
         
         hoststatus |= (1<<TEENSY_MMC_OK);
      }         
       
      // **********************************************************
#pragma mark Mess-Intervall
      // **********************************************************
      //      uint16_t adcwert=0;
      //      float adcfloat=0;
      
      if (hoststatus & (1<<MESSUNG_OK)) // Intervall abgelaufen. Flag wird in ISR gesetzt. Jetzt Messungen vornehmen. Auftrag an wl, Daten zu senden
      {
         
         uint8_t usberfolg = 0;
         
         // teensy-daten abschicken
         
         //     teensybuffer[0] = TEENSY_DATA;
         teensybuffer[0] = MESSUNG_DATA;
         
         teensybuffer[2] = wl_callback_status_def; // bisheriger status
         teensybuffer[DATACOUNT_LO_BYTE] = (messungcounter & 0x00FF) ;
         teensybuffer[DATACOUNT_HI_BYTE] = ((messungcounter & 0xFF00)>>8);
         teensybuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Byte 3, 4
         teensybuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
         
         teensybuffer[DEVICE + DATA_START_BYTE] = 0; // Wer sendet Daten? Sollte Devicenummer sein
         
         usberfolg = usb_rawhid_send((void*)teensybuffer, 50);
         if ((usberfolg != 0x20))
         {
           // lcd_gotoxy(12,2);
           // lcd_putc('T');
            //lcd_puthex(usberfolg);
         }
         
         
         sendbuffer[DEVICECOUNT_BYTE] = devicecount; // anz devicemitgeben
         devicecount=0;
         //lcd_gotoxy(6,2);
         //lcd_puts("        ");
         
         /*
          */
         
         if (usb_configured())
         {
            hoststatus |= (1<< TEENSYPRESENT);
            //hoststatus = 1;
            lcd_gotoxy(19,0);
            lcd_putc('$');
            
            hoststatus &= ~(1<<MANUELL_OK);
            //lcd_gotoxy(18,3);
            //lcd_putc('y');

         }
         else // Betrieb ohne host
         {
            if (hoststatus &(1<< TEENSYPRESENT)) // status war noch eingeschaltet: EEPROM lesen
            {
               hoststatus &= ~(1<< TEENSYPRESENT); // status reset, EEPROM nur einmal lesen
               lcd_gotoxy(19,0);
               lcd_putc('Z');
               
               // Intervall lesen
               intervall = eeprom_read_word(&eeprom_intervall);
               //lcd_gotoxy(12,3);
               //lcd_putc('e');
               //lcd_putint(intervall);
               
               
               blockcounter = eeprom_read_word(&eeprom_startblock);
               //lcd_putc('b');
               //lcd_putint(blockcounter);
            
               sd_status &= ~(1<<SAVE_SD_STOP_BIT);
               sd_status |= (1<<SAVE_SD_RUN_BIT);   // fortlaufendes Schreiben starten
               //hoststatus |= (1<<MANUELL_OK);
            }
         }
         hoststatus &= ~(1<<MESSUNG_OK);
         lcd_gotoxy(8,0);
         lcd_puthex(hoststatus);
         //lcd_putc(' ');
         //lcd_gotoxy(12,2);
         //lcd_putint1(wl_callback_status);
         //lcd_putint1(wl_callback_status_check);
         sendbuffer[6] = wl_callback_status_def; // bisheriger status
         
         
         // neuer check
         wl_callback_status = 0; // in der callbackfunktion  wird fuer jedes device, das antwortet, ein bit geesetzt
         
         
         
         lcd_gotoxy(4,0);
         lcd_putint(messungcounter);
         //lcd_putc(' ');
         
          sendbuffer[0]= MESSUNG_DATA;
         
         sendbuffer[DATA_START_BYTE] = 31; // Grenze zu DATA markieren
         
         
         //zaehler laden
         sendbuffer[DATACOUNT_LO_BYTE] = (messungcounter & 0x00FF);
         sendbuffer[DATACOUNT_HI_BYTE] = ((messungcounter & 0xFF00)>>8);
         
         // ADC lesen
         
         
         /*
          sd_status
          #define SAVE_SD_BIT             0
          #define SAVE_SD_RUN_BIT         1
          #define SAVE_SD_STOP_BIT        2
          */
         
         /* 
          Results of Disk Functions. def in diskio.h
          typedef enum {
          RES_OK = 0,         0: Successful 
          RES_ERROR,          1: R/W Error 
          RES_WRPRT,          2: Write Protected 
          RES_NOTRDY,         3: Not Ready 
          RES_PARERR          4: Invalid Parameter 
          } DRESULT;
          */
         
         
         messungcounter ++; // 8 Werte geschrieben, naechste zeile
         //lcd_clr_line(1);
         lcd_gotoxy(0,1);
         lcd_puts("            ");          
         //lcd_clr_line(2);
         //lcd_clr_line(3);
         
         
         
         wl_spi_status |= (1<<WL_SEND_REQUEST); // Auftrag an wl, Daten zu senden
         loop_pipenummer = 1; 
         loop_channelnummer=0;
         
         lcd_gotoxy(2,1);
         //lcd_puthex(wl_callback_status_def);
         
      } // end if (hoststatus & (1<<MESSUNG_OK))
      
      // Messung abgeschlossen, wl_devices aufrufen, Daten lesen
      
      
      
      
      // Messung abgeschlossen, wl_devices aufrufen, Daten lesen
      
      
      if (wl_spi_status & (1<<WL_SEND_REQUEST))
      {
         wl_send_status=0;
         //lcd_gotoxy(9,1);
         //lcd_puts(" "); // senden markieren, wird in WL_ISR_RECV-Routine mit r ueberschrieben
         
         //lcd_gotoxy(14,3);
         //lcd_puts("   ");
         
         // WL write start
         
         // ************** SEND **********************************
#pragma mark WL send
         // **********************************************************
         
         
         lcd_gotoxy(0,1);
         lcd_putc('l');
         lcd_putint1(loop_channelnummer);
         //         lcd_gotoxy(6,2);
         //        lcd_putc('l');
         //         lcd_putint1(akt_pipenummer);
         
         wl_blockedcounter = 0;
         
         wl_module_get_one_byte(FLUSH_TX);
         delay_ms(3);
         wl_module_get_one_byte(FLUSH_RX);
         
         delay_ms(3);
         
         OSZIA_LO;
         // ***** PIPE *********************************************
         
         // channel 1 setzen
         wl_module_tx_config_channel(loop_pipenummer,module_channel[loop_channelnummer]);
         
         
         //OSZIB_LO;
         delay_ms(2); // etwas warten Neu 2 ms anstatt 20
         // WL
         // Daten an devices, kontrolle
         payload[8] = loop_channelnummer;
         payload[9] = maincounter;
         payload[10] = adcwert & 0x00FF;
         payload[11] = (adcwert & 0xFF00)>>8;
         
         lcd_gotoxy(4+2*loop_channelnummer,1); // senden fuer jedes device zeigen
         lcd_putc('s');
         delay_ms(2);
         
         // ***** SENDEN *****************************************************
         
         wl_module_send(payload,wl_module_PAYLOAD);
         delay_ms(5);
         wl_sendcounter++;
         
         datapendcounter=0;
         
         wl_spi_status &= ~(1<<WL_SEND_REQUEST); // Auftrag an wl erfuellt
         
         OSZIA_HI; // 30 ms bis lesen
         
         // **********************************************************
         
         // neu: red auf 2ms bei neuem Print
         delay_ms(10); // etwas warten, wichtig, sonst wird rt nicht immer erkannt
         //delay_ms(20);
         //OSZIA_HI;
         
         wl_status = wl_module_get_status();
         
         //         lcd_gotoxy(11+ 2*loop_channelnummer,1); // alle werte in linie anzeigen
         //        lcd_puthex(wl_status);
         
         wl_send_status |= (1<<7);
         
         if (wl_status == 0x0E) // device nicht vorhanden, weiterschalten
         {
            if ((loop_channelnummer < 3) )
            {
               loop_channelnummer++;
               wl_spi_status |= (1<<WL_SEND_REQUEST);
               
            }
         }
         
         // //
         if (wl_status & (1<<TX_FULL))
         {
            lcd_gotoxy(16,2);
            lcd_putc('F');
            wl_module_config_register(STATUS, (1<<TX_FULL)); //Clear Interrupt Bit
         }
         
         if (wl_status & (1<<MAX_RT))
         {
            
            wl_module_config_register(STATUS, (1<<MAX_RT));	// Clear Interrupt Bits
            
            lcd_gotoxy(12,0);
            lcd_puts("rt");
            lcd_putint1(loop_channelnummer);
            //lcd_putc('b');
            wl_module_get_one_byte(FLUSH_TX);
            
            delay_ms(2);
            if ((loop_channelnummer < 3) )
            {
               loop_channelnummer++;
               wl_spi_status |= (1<<WL_SEND_REQUEST);
               
            }
            else
            {
               
               //loop_channelnummer=1;
               //wl_spi_status |= (1<<WL_SEND_REQUEST);
            }
            
            
         } // if MAX_RT
         
         /*
          // verhinderte data lesen
          if (wl_status & (1<<TX_DS)) // IRQ: Package has been sent
          {
          //OSZIA_LO;
          
          lcd_gotoxy(14,2);
          lcd_puts("tx");
          wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
          PTX=0;
          //OSZIA_HI;
          }
          */
         
         // lcd_putc('z');
         //        wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
         //wl_module_get_one_byte(FLUSH_TX);
         //wl_status = wl_module_get_status();
         //delay_ms(10);
         //lcd_putc('*');
         //// lcd_puthex(wl_status);
         
         //         wl_module_get_one_byte(FLUSH_TX);
         //         wl_module_get_one_byte(FLUSH_RX);
         wl_send_status |= (1<<6);
         
         //(wl_module_rx_config();
         wl_module_rx_config_channel(module_channel[loop_channelnummer]);
         
         delay_ms(5);
         if (loop_channelnummer == 3) // alle abgefragt
         {
            
         }
         
         
      } // if (wl_spi_status & (1<<WL_SEND_REQUEST))
      
      
      
      // if hoststatus & (1<<MESSUNG_OK)
      
      if (hoststatus & (1<<DOWNLOAD_OK))
      {
         
      }// if (hoststatus & (1<<DOWNLOAD_OK))
      
#pragma mark LED_LOOP
      if (loopcount0==0x2FFF)
      {
         
         loopcount0=0;
         loopcount1+=1;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //      continue;
         //lcd_gotoxy(16,0);
         //lcd_puthex(wl_sendcounter);
         
         //
  /*
         DDRB &= ~(1<<4); // ADC 11
         PORTB &= ~(1<<4);
         
         DDRF &= ~(1<<1); //ADC1
         PORTF &= ~(1<<1);

         lcd_gotoxy(7,2);
         lcd_putc('T');
        
         Tastenwert=(adc_read(11)>>2);
         lcd_putint12(Tastenwert);
         
         
         lcd_putc(' ');
         
         Tastenwert=(adc_read(11)>>2);
         lcd_putint(Tastenwert);
*/
         //
         lcd_gotoxy(0,2);
         lcd_putc('t');
         lcd_putc('0');
        // lcd_putc(' ');
         
         uint8_t i=0;
         lcd_putint(temperatur0);
         
         
         //lcd_putc(' ');
         //        lcd_puthex(wl_data[12]);
         //       lcd_puthex(wl_data[13]);
         
         /*        
          
          lcd_gotoxy(0,3);
          lcd_putc('t');
          lcd_putc('1');
          //lcd_putc(' ');
          //lcd_putint999(temperatur1);
          //lcd_putc(' ');
          lcd_gotoxy(18,3);
          lcd_putint2(devicecount);
          */       
         /*
          lcd_putint999(adc0);
          lcd_putc(' ');
          lcd_putint999(adc1);
          lcd_putc(' ');
          lcd_putint999(adc2);
          lcd_putc(' ');
          lcd_putint999(adc3);
          lcd_putc(' ');
          //lcd_putint999(ADC_Array[2]);
          */
         /*         
          lcd_gotoxy(3,3);
          for (i=0;i<4;i++)
          {
          lcd_putint999(ADC_Array[i]);
          
          
          lcd_putc(' ');
          }
          */         
         //lcd_gotoxy(8,3);
         //lcd_putc('v');
         // lcd_putc('1');
         //lcd_putc(' ');
         //lcd_putint12(spannung0);
         
         //lcd_gotoxy(8,2);
         //lcd_putc('b');
         // lcd_putc('1');
         //lcd_putc(' ');
         //lcd_putint(batteriespannung);
         /*
          uint32_t spannung= spannung0 * 25.9/1023;
          lcd_putc(' ');
          spannung0 = spannung & 0xFFFF;
          lcd_putint12(spannung0);
          */
         
         lcd_gotoxy(12,0);
         lcd_putc('i');
         //         lcd_puts("is");
         lcd_puthex(wl_isr_counter); // in ISR von INT0 gesetzt
         
         //         lcd_gotoxy(3,1);
         //         lcd_putc('p');
         //        lcd_putint1(pipenummer);
         
         
         
         
         if (usbstatus & (1<<WRITEAUTO))
         {
            uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 100);
         }
         //lcd_puthex(usberfolg);
         
         if ((loopcount1%2 == 0) && (usbstatus & (1<<WRITEAUTO)))
         {

         }
         
         if(loopcount1%32 == 0)
         {
            
#pragma mark Sensors
            // Temperatur messen mit DS18S20
            /*
             if (gNsensors) // Sensor eingesteckt
             {
             start_temp_meas();
             delay_ms(800);
             read_temp_meas();
             uint8_t line=0;
             //Sensor 1
             lcd_gotoxy(14,line);
             lcd_puts("T:     \0");
             if (gTempdata[0]/10>=100)
             {
             lcd_gotoxy(13,line);
             lcd_putint((gTempdata[0]/10));
             }
             else
             {
             lcd_gotoxy(12,line);
             lcd_putint2((gTempdata[0]/10));
             }
             
             lcd_putc('.');
             lcd_putint1(gTempdata[0]%10);
             
             sendbuffer[DSLO]=((gTempdata[0])& 0x00FF);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
             sendbuffer[DSHI]=((gTempdata[0]& 0xFF00)>>8);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
             //lcd_gotoxy(10,0);
             //lcd_putint(sendbuffer[DSLO]);
             //lcd_putc(' ');
             //lcd_putint(sendbuffer[DSHI]);
             // Halbgrad addieren
             if (gTempdata[0]%10 >=5) // Dezimalstelle ist >=05: Wert  aufrunden, 1 addieren
             {
             // txbuffer[INNEN] +=1;
             }
             }
             */
         } //
         
         
         
         
         
      } // if loopcount0
      
      /**	ADC	***********************/
      
      /**	END ADC	***********************/
      
      /**	Begin USB-routinen	***********************/
      // MARK USB read
      // Start USB
      //OSZI_D_LO;
      r=0;
      recvbuffer[0] = 0;
      //     if (usbstatus & (1<<READAUTO))
      {
         r = usb_rawhid_recv((void*)recvbuffer, 0); // 5us
      }
      //	*********************************************************************       
      // MARK: USB_READ
      //	*********************************************************************       
      if (r > 0)
      {
         cli();
         usb_readcount++;
         uint8_t code = recvbuffer[0];
         
         //lcd_gotoxy(18,2);
         //lcd_puthex(code);
         
         if (!(usbstatus == code))
         {
            usbstatus = code;
            //lcd_clr_line(1);
            lcd_gotoxy(18,1);
            lcd_puthex(code);
         }
         
         switch (code)
         {
               
            case DEFAULT: // cont read
            {
               //lcd_clr_line(2);
               sendbuffer[0] = DEFAULT;
               //sendbuffer[3] = 0;
               mmcwritecounter = 0;
               //            code = WRITE_MMC_TEST;
               //lcd_putc('c');
               //lcd_puthex(code); // code
               sd_status = recvbuffer[1]; // bit 0: sd mit testdaten beschreiben
               mmcwritecounter = 0;
               //lcd_putc('-');
               //lcd_puthex(sd_status); // code
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
               // PWM fuer Channel A
               
            }break;
               
               //	*********************************************************************            
               // MARK: READ_START  
               //	*********************************************************************                
            case READ_START:
            {
               lcd_gotoxy(16,2);
               lcd_putc('R');
               sendbuffer[0] = READ_START;
               messungcounter = 0;
               //    hoststatus |= (1<<USB_READ_OK);
               sendbuffer[30] = 73;
               lcd_putint1(wl_callback_status);
               sendbuffer[2] = wl_callback_status_def | 0x01; // teensy ist da
               sendbuffer[DEVICECOUNT_BYTE] = devicecount;
               uint8_t ind=0;
               for (ind = 0;ind  < WL_MAX_DEVICE;ind++)
               {
                  sendbuffer[DATA_START_BYTE + ind] = devicebatteriespannung[ind]; // kontrolle
               }
               
            }break;
               
               //	*********************************************************************            
               // MARK: CHECK_WL
               //	*********************************************************************            
               
            case CHECK_WL:
            {
               clear_sendbuffer();
               lcd_gotoxy(17,0);
               lcd_putc('W');
               hoststatus |= (1<< TEENSYPRESENT);
               lcd_gotoxy(19,0);
               lcd_putc('$');
               hoststatus &= ~(1<<MANUELL_OK);
               //lcd_gotoxy(18,3);
               //lcd_putc('x');
               
               //hoststatus |= (1<<MESSUNG_OK); // Messung ausloesen
               sendbuffer[0] = CHECK_WL;
               sendbuffer[31] = 77;
               //lcd_putint1(wl_callback_status);
               sendbuffer[2] = wl_callback_status_def;
               sendbuffer[DEVICECOUNT_BYTE] = devicecount; // 3
               
               // Batteriespannung senden
               adcwert = read_bat(0); // Batteriespannung
               devicebatteriespannung[0] = (uint8_t)(adcwert/2);
                uint8_t ind=0;
               for (ind = 0;ind  < WL_MAX_DEVICE;ind++)
               {
                  sendbuffer[DATA_START_BYTE + ind] = devicebatteriespannung[ind];
                  //             uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               }
               
               //sendbuffer[DATA_START_BYTE] = (uint8_t)(adcwert/2);
               messungcounter = 0;
               
            }break;
               //	********************************************************************* 
               // MARK: LOGGER_START
               //	*********************************************************************                
            case LOGGER_START:
            {
               clear_sendbuffer();
               loggerstatus = 0;
               hoststatus &= ~(1<<USB_READ_OK);
               hoststatus &= ~(1<<MESSUNG_OK);
               hoststatus &= ~(1<<TEENSY_ADC_OK);
               hoststatus |= (1<<DOWNLOAD_OK); // Download von SD, Messungen unterbrechen
               hoststatus &= ~(1<<MANUELL_OK);
               lcd_clr_line(1);
               lcd_gotoxy(10,1);
               lcd_putc('l');
               lcd_putc(':');
               lcd_puthex(code);
               
               sendbuffer[0] = LOGGER_START;
               
               writemmcstartcounter = 0;
               //code = LOGGER_START; // read block starten
               //lcd_putc('c');
               //lcd_puthex(code); // packetcount
               
               // Block lesen
               /*
                lcd_putc('l');
                lcd_puthex(recvbuffer[BLOCKOFFSETLO_BYTE]); // startblock lo
                lcd_putc('h');
                lcd_puthex(recvbuffer[BLOCKOFFSETHI_BYTE]); // startblock hi
                lcd_putc('*');
                lcd_puthex(recvbuffer[PACKETCOUNT_BYTE]); // packetcount
                */
               // old
               
               startblock = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8); // zu lesender Block auf mmc
               
               uint8_t paketindex = 0;
               
               // old
               //packetcount = recvbuffer[DEVICECOUNT_BYTE] ;// laufender Index Paket, beim Start 0
               
               packetcount = recvbuffer[PACKETCOUNT_BYTE] ;// laufender Index Paket, beim Start 0
               //               packetcount=0;
               // lcd_gotoxy(12,1);
               // lcd_puts(">mmc");
               lcd_gotoxy(18,1);
               lcd_puthex(startblock);
               
               blockanzahl = recvbuffer[BLOCK_ANZAHL_BYTE] ;// laufender Index Paket, beim Start 0
               
               downloaddatacounter = recvbuffer[DATACOUNT_LO_BYTE] | (recvbuffer[DATACOUNT_HI_BYTE]<<8); // 
               
               /*
               lcd_gotoxy(6,2);
               lcd_puts("load ");
               lcd_putint2(blockanzahl);
               lcd_putc(' ');
               lcd_putint12(downloaddatacounter);
               */
                
               uint16_t logger_blockcount = eeprom_read_word(&eeprom_blockcount);
               sendbuffer[23] = '*';
               sendbuffer[24] = logger_blockcount & 0x00FF;
               sendbuffer[25] = (logger_blockcount & 0xFF00)>>8;
               
               lcd_gotoxy(6,2);
               lcd_puts("     ");

               lcd_gotoxy(6,2);
               lcd_putc('*');
               lcd_putint(logger_blockcount & 0x00FF);
               lcd_putc('*');
               uint16_t logger_messungcount = eeprom_read_word(&eeprom_messungcount);
               sendbuffer[26] = logger_messungcount & 0x00FF;
               sendbuffer[27] = (logger_messungcount & 0xff00)>>8;
               sendbuffer[28] = '*';                  

               //lcd_gotoxy(7,3);
               //lcd_puthex(downloaddatacounter);
               
               
               // Beim Start Block aus SD lesen
               readerr = mmc_disk_read((void*)mmcbuffer,1+ startblock,1);
               
               
               if (readerr == 0)
               {
                  lcd_gotoxy(15,1);
                  lcd_puts(">OK ");
                  if (TEST == 0)
                  {
                     sendbuffer[DATA_START_BYTE -1] = 111;
                     sendbuffer[USB_PACKETSIZE-1] = 87;
                  }
                  
                  // Header uebertragen, HEADER_SIZE bytes nach BLOCK_SIZE im Block. Schreiben nach DATA_STARTBYTE + HEADER_OFFSET
                  
                  uint8_t headerindex=0;
                  for (headerindex = 0;headerindex < 10;headerindex++)
                  {
                     sendbuffer[DATA_START_BYTE + HEADER_OFFSET + headerindex] = mmcbuffer[BLOCK_SIZE  + headerindex];
                  }
                  //#define DATA_START_BYTE    8    // erstes byte fuer Data auf USB
                  //#define HEADER_OFFSET      4     // Erstes Byte im Block nach BLOCK_SIZE: Daten, die bei LOGGER_NEXT uebergeben werden

                  
                  
                  
               } // if readerr==0
               else
               {
                  lcd_gotoxy(14,1);
                  lcd_puts(">err");
               }
               
               sendbuffer[PACKETCOUNT_BYTE] = 0; //
               sendbuffer[1] = readerr;
               
               if (TEST == 1)
               {
                  sendbuffer[16] = 57;
                  sendbuffer[17] = startblock;
                  sendbuffer[18] = 58;
                  sendbuffer[19] = blockanzahl;
                  sendbuffer[20] = 59;                  
               }
            }break;
               
               //	*********************************************************************                
               // MARK: LOGGER_CONT 
               //	*********************************************************************                            
            case LOGGER_CONT: // weiteres Paket lesen (Datenzeile)
            {
               clear_sendbuffer();
               if (TEST)
               {
                  sendbuffer[0] = LOGGER_CONT;
                  sd_status = recvbuffer[1];
                  packetcount = recvbuffer[PACKETCOUNT_BYTE];
                  
                  
                  uint8_t paketindex = 0;
                  if (downloadblocknummer == 0 && packetcount == 0) // Start Logging
                  {
                     // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen    
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // PACKET_SIZE: 24 bytes fuer sendbuffer
                     {
                        
                        //lcd_gotoxy(18,3);
                        //lcd_putc(65); // Kontrolle: A
                        // Header ist schon geladen, Breite ist HEADER_SIZE (8)
                        //sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[HEADER_SIZE + (packetcount*PACKET_SIZE)+paketindex];
                        sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                  }
                  else
                  {
                     // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 24 bytes fuer sendbuffer
                     {
                        // Breite ist HEADER_SIZE (8)
                        sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                  }
               }
               else        
               {
                  
                  // lcd_clr_line(1);
                  //lcd_gotoxy(0,3);
                  //lcd_putc('c');
                  // lcd_putc(':');
                  sendbuffer[0] = LOGGER_CONT;
                   sd_status = recvbuffer[1];
                  packetcount = recvbuffer[PACKETCOUNT_BYTE];
                  uint8_t paketindex = 0;
                  
                  if (downloadblocknummer == 0 && packetcount == 0) // Start Logging
                  {
                     writedatacounter = 0;
                     // writemmcstartcounter++;
                     //lcd_gotoxy(13,3);
                     //lcd_puthex(writemmcstartcounter);
                      // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen    
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // PACKET_SIZE: 24 bytes fuer sendbuffer
                     {
                        
                        //                        lcd_gotoxy(18,3);
                        //                       lcd_putc(65); // Kontrolle: A
                         sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                     
                  }
                  else
                  {
                     writedatacounter++;
                     //                    lcd_gotoxy(15,3);
                     //                    lcd_puthex(writedatacounter);
                     
                     
                     
                     //                     lcd_gotoxy(19,3);
                     //                    lcd_putc(97); // Kontrolle: a
                     
                     // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 24 bytes fuer sendbuffer
                     {
                       
                        sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                  }
                  
                  // packetcount mitgeben und incrementieren
                  sendbuffer[PACKETCOUNT_BYTE] = ++packetcount; // Byte 2
                  
                  sendbuffer[USB_PACKETSIZE-1] = 74;
                  sendbuffer[USB_PACKETSIZE-2] = writedatacounter;
                  //              uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
                  
                  //lcd_gotoxy(18,2);
                  //lcd_puthex(usberfolg);
               }
               
            }break;
               //	*********************************************************************                
               // MARK: LOGGER_NEXT
               //	*********************************************************************                
            case LOGGER_NEXT: // Block an startblock + downloadblocknummer lesen
            {
               for (int i=0;i<USB_PACKETSIZE;i++)
               {
                  sendbuffer[i] = 0;
               }
               //clear_sendbuffer();
               //lcd_clr_line(1);
               //lcd_gotoxy(0,1);
               //lcd_putc('n');
               //lcd_putc(':');
               //lcd_puthex(code); // code
               sendbuffer[0] = LOGGER_NEXT;
               sendbuffer[USB_PACKETSIZE-1] = 71;
               //startblock = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8); // zu lesender Block auf mmc
               uint8_t paketindex = 0;
               //lcd_gotoxy(0,3);
               //lcd_puthex(startblock);
               
               downloadblocknummer  = recvbuffer[DOWNLOADBLOCKNUMMER_BYTE] ;// nummer des next blocks, Byte 10
               
               lcd_gotoxy(18,3);
               lcd_puthex(downloadblocknummer);
                              
               // Beim Start Block aus SD lesen
               readerr = mmc_disk_read((void*)mmcbuffer,1+ startblock + downloadblocknummer,1);
               if (readerr == 0)
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('+');
                  
                  // Header uebertragen, letzte HEADER_SIZE bytes im Block
                  
                  uint8_t headerindex=0;
                  for (headerindex = 0;headerindex < HEADER_SIZE;headerindex++)
                  {
                     sendbuffer[DATA_START_BYTE + HEADER_OFFSET + headerindex] = mmcbuffer[BLOCK_SIZE + headerindex];
                  }
               } // if readerr==0
               else
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('-');
               }
               uint8_t delta = 0;
               uint8_t offset = 4;
               
               //Daten am Ende des letzten Blocks lesen
               /*
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta];// blockcounter lo
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // blockcounter hi
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // blockdatacounter lo
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // blockdatacounter hi
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // messungcounter lo
                
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // messungcounter hi
              */  
               
               //            uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
            }break; // LOGGER_NEXT
               
               //	*********************************************************************                
               // MARK: LOGGER_STOP
               //	********************************************************************* 
            case LOGGER_STOP: // 0xAF
            {
               hoststatus &= ~(1<<DOWNLOAD_OK); // Download von SD beendet, 
               //lcd_clr_line(1);
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('s');
               lcd_putc(':');
               lcd_puthex(code); // code
               
               mmcwritecounter = 0;
               usbstatus = 0;
               sendbuffer[0] = LOGGER_STOP;
               sendbuffer[PACKETCOUNT_BYTE] = 0; // packetcount
               sendbuffer[BLOCKOFFSETLO_BYTE] = downloadblocknummer & 0x00FF;
               sendbuffer[BLOCKOFFSETHI_BYTE] = (downloadblocknummer & 0xFF00)>>8;
               

            }break;
               //	*********************************************************************                
               // MARK: WRITE_MMC_TEST
               //	*********************************************************************                
            case WRITE_MMC_TEST:
            {
               //lcd_clr_line(2);
               //lcd_gotoxy(0,2);
               //lcd_putc('t');
               //lcd_putc(':');
               sendbuffer[0] = WRITE_MMC_TEST;
               //            code = WRITE_MMC_TEST;
               //lcd_putc('c');
               //lcd_puthex(code); // code
               sd_status = recvbuffer[1]; // bit 0: sd mit testdaten beschreiben
               mmcwritecounter = 0;
               //lcd_putc('-');
               //lcd_puthex(sd_status); // code
               
            }break;
               
               //	*********************************************************************                
               // MARK: USB_STOP
               //	********************************************************************* 
            case USB_STOP: // Host ist ausgeschaltet
            {
               //hoststatus &= ~(1<<MESSUNG_OK);
               //hoststatus &= ~(1<<DOWNLOAD_OK);
               hoststatus &= ~(1<< TEENSYPRESENT);
               lcd_gotoxy(19,0);
               lcd_putc('X');
               //lcd_clr_line(1);
               //lcd_gotoxy(16,0);
               //lcd_putc('h');
               //lcd_putc(' ');
               //lcd_puthex(code); // code
               //sendbuffer[0] = USB_STOP;
               
               // EEPROM lesen
               
               //lcd_gotoxy(12,3);
               //lcd_puts("        ");
               intervall = eeprom_read_word(&eeprom_intervall);
               
               //lcd_gotoxy(12,3);
               //lcd_putc('e');
               //lcd_putint(intervall);
               
               blockcounter = eeprom_read_word(&eeprom_startblock);
               
               //lcd_putc('b');
               //lcd_putint(blockcounter);
               
               
            }break;
               
               //	*********************************************************************                
               // MARK: LOGGER_SETTING
               //	*********************************************************************                
            case LOGGER_SETTING:
            {
               //clear_sendbuffer();
               sendbuffer[0] = LOGGER_SETTING;
               sd_status = recvbuffer[1];
               
               intervall = recvbuffer[TAKT_LO_BYTE] | (recvbuffer[TAKT_HI_BYTE]<<8);
               eeprom_update_word(&eeprom_intervall, intervall);
               
               blockcounter = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8);
               eeprom_update_word(&eeprom_startblock, blockcounter);
               
               // Kanalbytes fuer jedes device schreiben. 0 heist device inaktiv
               /*
               uint8_t  delta = 0;
               uint8_t  kanalbyte = recvbuffer[KANAL_BYTE + delta++];
               eeprom_update_byte(&eeprom_kanalbyte0,kanalbyte);
               kanalbyte = recvbuffer[KANAL_BYTE + delta++];
               eeprom_update_byte(&eeprom_kanalbyte1,kanalbyte);
               kanalbyte = recvbuffer[KANAL_BYTE + delta++];
               eeprom_update_byte(&eeprom_kanalbyte2,kanalbyte);
               kanalbyte = recvbuffer[KANAL_BYTE + delta++];
               eeprom_update_byte(&eeprom_kanalbyte3,kanalbyte);
               */
               uint8_t kan = 0;
               lcd_gotoxy(6,2);
               for(kan = 0;kan < 4;kan++)
               {
                  eeprom_update_byte(&eeprom_kanalstatus_array[kan],recvbuffer[KANAL_BYTE + kan]);
                  lcd_putint2(recvbuffer[KANAL_BYTE + kan]);
                  sendbuffer[KANAL_BYTE + kan] = recvbuffer[KANAL_BYTE + kan];
                  
               }
               lcd_putc('*');
               
              
               /* Byte */
               
               //lcd_clr_line(1);
               /*
               lcd_gotoxy(12,3);
               lcd_putc('e');
               lcd_putint(intervall);
               
               lcd_putc('b');
               lcd_putint(blockcounter);
               lcd_gotoxy(6,3);
               lcd_puts("set  ");
                */
            }break;
               
               //	*********************************************************************    
               // MARK: MESSUNG_START
               //	********************************************************************* 
            case MESSUNG_START:
            {
 //              clear_sendbuffer();
               cli();
               
               //uint8_t ee = eeprom_read_word(&eeprom_intervall);
               //lcd_gotoxy(12,3);
               //lcd_putint(ee);
               hoststatus |= (1<<USB_READ_OK);
               messungcounter = 0;
               sendbuffer[0] = MESSUNG_START;
               blockdatacounter = 0;            // Zaehler fuer Data auf dem aktuellen Block
               writedatacounter = 0;
               linecounter=0;
               
               
               /*
                lcd_clr_line(1);
                lcd_gotoxy(0,1);
                lcd_putc('m');
                lcd_putc(':');
                lcd_puthex(code); // code
                */
               usbstatus = code;
               sd_status = recvbuffer[1]; 
               
               mmcwritecounter = 0; // Zaehler fuer Messungen auf MMC
               
               saveSDposition = 0; // Start der Messung immer am Anfang des Blocks
               
               // intervall
               intervall = recvbuffer[TAKT_LO_BYTE] | (recvbuffer[TAKT_HI_BYTE]<<8);
               //lcd_gotoxy(14,2);
               //lcd_putc('i');
               //lcd_putc(':');
               //lcd_putint(intervall);
               
               
               //               abschnittnummer = recvbuffer[ABSCHNITT_BYTE]; // Abschnitt,
               
               blockcounter = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8);
               // startminute  = recvbuffer[STARTMINUTELO_BYTE] | (recvbuffer[STARTMINUTEHI_BYTE]<<8); // in SD-Header einsetzen
               
               // Kanalstatus lesen. Wird beim Start der Messungen uebergeben
               
               // nicht verwendet
                uint8_t kan = 0;
               lcd_gotoxy(8,2);
               for(kan = 0;kan < 4;kan++)
               {
                  lcd_putint2(kanalstatusarray[kan]);
               }
                          
               delay_ms(100);          
                           
                           
               for(kan = 0;kan < 4;kan++)
               {
                  kanalstatusarray[kan] = recvbuffer[KANAL_BYTE + kan];
               }
               
               lcd_gotoxy(8,2);
               for(kan = 0;kan < 4;kan++)
               {
                  lcd_putint2(kanalstatusarray[kan]);
               }
          
               
               /*
                lcd_putc(' ');
                lcd_puthex(blockcounter);
                
                lcd_putc(' ');
                lcd_puthex(sd_status);
                lcd_putc(' ');
                lcd_puthex(saveSDposition);
                */
               lcd_gotoxy(12,1);
               lcd_puts("start ");
               sendbuffer[1] = sd_status; // rueckmeldung 
               //sendbuffer[2] = wl_callback_status;
               //               sendbuffer[5] = 18;//recvbuffer[STARTMINUTELO_BYTE];;
               //               sendbuffer[6] = 19;//recvbuffer[STARTMINUTEHI_BYTE];;
               //sendbuffer[7] = 21;
               
               sendbuffer[USB_PACKETSIZE-1] = 76;
               saveSDposition = 0; // erste Messung sind header
               sei();
               
               // _delay_ms(1000);
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
            }break;
               
               //	*********************************************************************                
               // MARK: MESSUNG_STOP
               //	*********************************************************************                
            case MESSUNG_STOP:
            {
 //              clear_sendbuffer();
               sendbuffer[0] = MESSUNG_STOP;
               hoststatus &= ~(1<<USB_READ_OK);
               
               hoststatus  |= (1<<TEENSY_MMC_OK);
               //lcd_clr_line(1);
               //lcd_gotoxy(12,0);
               //lcd_putc('h');
               //lcd_putc(':');
               //lcd_puthex(code); // code
               //lcd_putc('*');
               usbstatus = code;
               sd_status = recvbuffer[1]; // code fuer SD: schreiben, stoppen
               
               sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF;
               sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8;
               
               sendbuffer[DATACOUNT_LO_BYTE] = messungcounter & 0x00FF;
               sendbuffer[DATACOUNT_HI_BYTE] = (messungcounter & 0xFF00)>>8;
               
               
               
               sendbuffer[USB_PACKETSIZE-1] = 79;
               
               lcd_gotoxy(6,3);
               lcd_putint(blockcounter & 0x00FF);

               eeprom_update_word(&eeprom_blockcount,blockcounter);
               eeprom_update_word(&eeprom_messungcount,messungcounter);
               
               //          lcd_gotoxy(19,1);
               //         lcd_putc('+');
                        lcd_gotoxy(12,1);
                         lcd_puts(" stop");
               
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
            }break;
               
            case SERVO_OUT:
            {
               sendbuffer[0] = 0;
               OCR1A = recvbuffer[SERVOALO] | (recvbuffer[SERVOAHI]<<8);
               lcd_gotoxy(11,0);
               lcd_putint12(OCR1A);
               
            }break;
            default:
            {
               break;
            }
               
         }
         
         code=0;
         sei();
         //sendbuffer[DEVICECOUNT_BYTE] = devicecount;
         
         if (sendbuffer[0]) // nur senden, wenn code gesetzt ist. Bsp SERVO_OUT: kein code
         {
            writemmcstartcounter++;
            //  lcd_gotoxy(11,3);
            //  lcd_puthex(loggerstatus);
            if (TEST == 1)
            {
               lcd_gotoxy(13,3);
               lcd_puthex(writemmcstartcounter);
               sendbuffer[21] = sendbuffer[0];
               sendbuffer[22] = writemmcstartcounter;
               sendbuffer[23] = loggerstatus;
            }
            
            uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 100);
            sendbuffer[0] = 0;
         } // if sendbuffer[0] > 0
         
      } // r>0, neue Daten
      else     // Betrieb ohne host, Messungen vornehmen
      {
         
         
      }
      //	*********************************************************************       
      //	End USB-routinen	                            ***********************
      //	*********************************************************************       
      
      //   *********************************************************************       
      //   Tastatur-routinen                               ***********************
      //   *********************************************************************       
      
      
   }//while
   //free (sendbuffer);
   
   // return 0;
}
