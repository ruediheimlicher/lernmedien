//
//  RC_PPM.c
//
//
//  Created by Sysadmin on 20.07.13
//  Copyright Ruedi Heimlicher 2013. All rights reserved.
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

//#include "spi.c"
//#include "spi_adc.c"

//#include "spi_slave.c"
//#include "soft_SPI.c"

#include "ds18x20.c"

#include "transfer.c"


#include "chan_n/mmc_avr_spi.c"
#include "chan_n/ff.c"
#include "chan_n/diskio.c"



// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LOOPDELAY 5

#define SERVOMAX  4400
#define SERVOMIN  1400


#define USB_DATENBREITE USB_PACKETSIZE



#define CODE_OFFSET  4
#define ROTARY_OFFSET  10

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

//volatile uint8_t outbuffer[USB_DATENBREITE]={};
//volatile uint8_t inbuffer[USB_DATENBREITE]={};

//static volatile uint8_t kontrollbuffer[USB_DATENBREITE]={};

//static volatile uint8_t eeprombuffer[USB_DATENBREITE]={};

#define TIMER0_STARTWERT	0x40

#define EEPROM_STARTADRESSE   0x7FF

volatile uint8_t timer0startwert=TIMER0_STARTWERT;

//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];


void delay_ms(unsigned int ms);

static volatile uint8_t             displaystatus=0x00; // Tasks fuer Display
volatile uint8_t                   eepromsavestatus = 0;




volatile uint8_t                    in_taskcounter=0;
volatile uint8_t                    out_taskcounter=0;

volatile uint16_t                   startblock = 0;

volatile uint8_t                    packetcount = 0;

// counter fuer Mess-Intervall
volatile uint16_t                    intervallcounter=0;

// mess-Intervall
volatile uint16_t                    intervall=1; // defaultwert, 1s
// counter fuer Mess-Intervall
volatile uint16_t                    messungcounter=0; // Anzahl messungen fortlaufend


//static volatile uint8_t            substatus=0x00; // Tasks fuer Sub
static volatile uint8_t             hoststatus=0x00;

static volatile uint8_t             usbstatus=0x00;
static volatile uint8_t             usbstatus1=0x00; // recvbuffer[1]

volatile uint8_t                    spistatus=0x00; // was ist zu tun infolge spi

//static volatile uint8_t             eepromstatus=0x00;
static volatile uint8_t             potstatus=0x00; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t             impulscounter=0x00;

static volatile uint8_t             masterstatus = 0;

static volatile uint8_t             tastaturstatus = 0;

volatile uint8_t status=0;


// Logger
volatile uint16_t messintervall = 1;
volatile uint8_t saveSDposition = 0;
volatile uint8_t blockcounter = 0; // Block, in den gesichert werden soll, mit einem Offset von 1 (Block 0 ist header der SD).



// PWM

volatile uint8_t                    PWM=0;
static volatile uint8_t             pwmposition=0;
static volatile uint8_t             pwmdivider=0;


static volatile uint8_t spi_rxbuffer[SPI_BUFSIZE];
static volatile uint8_t spi_txbuffer[SPI_BUFSIZE];
static volatile uint8_t spi_rxdata=0;

static volatile uint8_t inindex=0;

volatile char SPI_data='0';
//volatile char SPI_dataArray[SPI_BUFSIZE];
//volatile uint16_t Pot_Array[SPI_BUFSIZE];

//volatile uint16_t Mitte_Array[8];

//volatile uint8_t Level_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal
//volatile uint8_t Expo_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal

//volatile uint16_t Mix_Array[8];// Mixings, 2 8-bit-bytes pro Mixing


//volatile uint16_t RAM_Array[SPI_BUFSIZE];

//volatile uint8_t testdataarray[8]={};
volatile uint16_t teststartadresse=0xA0;


volatile uint16_t Batteriespannung =0;

volatile uint16_t adc_counter =0; // zaehlt Impulspakete bis wieder die Batteriespannung gelesen werden soll

volatile short int received=0;

volatile uint16_t abschnittnummer=0;

volatile uint16_t usbcount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;

volatile uint16_t eepromstartadresse=0;

volatile uint16_t inchecksumme=0;

volatile uint16_t bytechecksumme=0;
volatile uint16_t outchecksumme=0;

volatile uint8_t eeprom_databyte=0;
volatile uint8_t anzahlpakete=0;
//volatile uint8_t usb_readcount = 0;

volatile uint8_t  eeprom_indata=0;


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
BYTE RtcOk;				/* RTC is available */
volatile UINT Timer;	/* Performance timer (100Hz increment) */
volatile uint8_t mmcbuffer[SD_DATA_SIZE] = {};
const uint8_t rambuffer[SD_DATA_SIZE] PROGMEM = {};
const uint8_t databuffer[SD_DATA_SIZE] PROGMEM = {};
//volatile uint8_t writebuffer[512] = {};
volatile uint8_t mmcstatus = 0;
volatile uint16_t                   writecounter1=0; // Takt fuer write to SD
volatile uint16_t                   writecounter2=0; // Takt fuer write to SD

//#define CLOCK_DIV 15 // timer0 1 Hz bei Teilung /4 in ISR 16 MHz
#define CLOCK_DIV 8 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz
#define BLINK_DIV 4 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz



volatile uint16_t                TastaturCount=0;
volatile uint16_t                manuellcounter=0; // Counter fuer Timeout
volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint16_t                mscounter=0; // Counter fuer ms in timer-ISR
volatile uint8_t                 blinkcounter=0;


uint8_t                          ServoimpulsSchrittweite=10;
uint16_t                         Servoposition[]={1000,1250,1500,1750,2000,1750,1500,1250};




//volatile uint16_t                SPI_Data_counter; // Zaehler fuer Update des screens



volatile uint16_t Tastenwert=0;
volatile uint16_t Trimmtastenwert=0;
volatile uint8_t adcswitch=0;


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
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;

uint8_t search_sensors(void)
{
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
}

// start a measurement for all sensors on the bus:
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


// Code 1_wire end



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

void Master_Init(void)
{
   LOOPLEDDDR |=(1<<LOOPLED);
   LOOPLEDPORT |= (1<<LOOPLED);	// HI
   
   SPI_DDR |=(1<<SPI_MOSI);
   SPI_PORT |= (1<<SPI_MOSI);	// HI
   
   //Pin 0 von   als Ausgang fuer OSZI
   OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
   /*
   OSZIPORTDDR |= (1<<PULSB);		//Pin 1 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSB);		//Pin   von   als Ausgang fuer OSZI
   */
   
   OSZIPORTDDR &= ~(1<<TEST_PIN);		//Pin 1 von  als Eingang fuer Test-Pin. active LO
   OSZIPORT |= (1<<TEST_PIN);		//Pin   von   als Ausgang fuer OSZI

   /*
    TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
    TASTENPORT |= (1<<TASTE0);	//Pull-up
    */
   
   
   
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
   LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
   
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
   //SPI_PORT &= ~(1<<SPI_SCK; // LO
   SPI_DDR |= (1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS); // HI
   
   
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

void SPI_ADC_init(void) // SS-Pin fuer EE aktivieren
{
   
   SPI_DDR |= (1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS); // HI
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
   
   SPCR |= (1<<SPR0);               // div 16 SPI2X: div 8
   //SPCR |= (1<<SPR1);               // div 64 SPI2X: div 32
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
 //OSZI_A_LO ;
 //PORTB &= ~(1<<PORTB5); // OC1A Ausgang
 //OSZI_A_HI ;
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
   
   
}

/*
 void timer2 (uint8_t wert)
 {
 //timer2
 TCNT2   = 0;
 //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
 TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
 //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
 TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
 //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
 TCCR2A = 0x00;
 
 
 OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
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
   //lcd_putc('+');
   //   Timer1--;			/* Performance counter for this module */
   mmc_disk_timerproc();	/* Drive timer procedure of low level disk I/O module */
   writecounter1++;
   if (writecounter1 >= WRITETAKT) // 1s
   {
      OSZIA_LO;
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
         }
         
         
         
      }
      
      
      
      writecounter1=0;
      writecounter2++;
      //      lcd_gotoxy(0,2);
      //      lcd_putint12(writecounter2);
      if (writecounter2 >= 0x0002)
      {
         mmcstatus |= (1<<WRITENEXT);
         writecounter2 = 0;
      }
   }
   OSZIA_HI;
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
   
   
   OCR1A = 0x3E8;           // Pulsdauer 1ms
   OCR1A = 0x200;
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
   //   OSZI_A_LO;
   inindex=0;
   //SPDR = 0;// Erstes Byte an Slave
   //   OSZI_A_HI;
   
   //  spi_txbuffer[0]++;
   //  spi_txbuffer[2]--;
   
}





#pragma mark PIN_CHANGE
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328

/*
 ISR (PCINT0_vect)
 {
 
 if(INTERRUPT_PIN & (1<< MASTER_EN)// LOW to HIGH pin change, Sub ON
 {
 //OSZI_C_LO;
 
 masterstatus |= (1<<SUB_TASK_BIT); // Zeitfenster fuer Task offen
 adc_counter ++; // loest adc aus
 
 }
 else // HIGH to LOW pin change, Sub ON
 {
 displaystatus |= (1<<UHR_UPDATE);
 //masterstatus &= ~(1<<SUB_TASK_BIT);
 }
 
 }
 */

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
   /*
    // Atmega168
    
    #define TASTE1		19
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		186
    #define TASTE9		212
    #define TASTE_L	234
    #define TASTE0		248
    #define TASTE_R	255
    
    
    // Atmega328
    #define TASTE1		17
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		166
    #define TASTE9		214
    #define TASTE_L	234
    #define TASTE0		252
    #define TASTE_R	255
    */
   
   //lcd_gotoxy(0,0);
   //lcd_putint(Tastaturwert);
   /*
    if (Tastaturwert < TASTE1)
    return 1;
    if (Tastaturwert < TASTE2)
    return 2;
    if (Tastaturwert < TASTE3)
    return 3;
    if (Tastaturwert < TASTE4)
    return 4;
    if (Tastaturwert < TASTE5)
    return 5;
    if (Tastaturwert < TASTE6)
    return 6;
    if (Tastaturwert < TASTE7)
    return 7;
    if (Tastaturwert < TASTE8)
    return 8;
    if (Tastaturwert < TASTE9)
    return 9;
    
    if (Tastaturwert < TASTE_L)
    return 10;
    if (Tastaturwert < TASTE0)
    return 0;
    if (Tastaturwert <= TASTE_R)
    return 12;
    */
   
   
   
   // Tastatur2 // Reihenfolge anders
   /*
    #define WERT1    11    // 1 oben  Taste 2
    #define WERT3    34    // 2 links  Taste 4
    #define WERT4    64    // 3 unten  Taste 8
    #define WERT6    103   // 4 rechts  Taste 6
    #define WERT9    174   // 5 Mitte  Taste 5
    #define WERT2 	26    //  A links oben Taste  1
    #define WERT5    72       //    B links unten Taste 7
    #define WERT7    116      //   C rechts oben Taste 3
    #define WERT8    161      // D rechts unten Taste 9
    
    */
   
   if (Tastaturwert < WERT1)
      return 2;
   if (Tastaturwert < WERT2)
      return 1;
   if (Tastaturwert < WERT3)
      return 4;
   if (Tastaturwert < WERT4)
      return 8;
   if (Tastaturwert < WERT5)
      return 7;
   if (Tastaturwert < WERT6)
      return 6;
   if (Tastaturwert < WERT7)
      return 3;
   if (Tastaturwert < WERT8)
      return 9;
   if (Tastaturwert < WERT9)
      return 5;
   
   return -1;
   
   
   
}




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

// MARK:  - main
int main (void)
{
   
   uint16_t tempwert = 444;
   int8_t r;
   
   uint16_t spi_count=0;
   
   // set for 16 MHz clock
   CPU_PRESCALE(CPU_8MHz); // Strom sparen
   
   
   //   sei();
   Master_Init();
   SPI_PORT_Init();
   SPI_Master_init();
   
   
   volatile    uint8_t outcounter=0;
   volatile    uint8_t testdata =0x00;
   //  volatile    uint8_t testaddress =0x00;
   volatile    uint8_t errcount =0x00;
   //   volatile    uint8_t ram_indata=0;
   
   //  volatile    uint8_t eeprom_indata=0;
   //  volatile    uint8_t eeprom_testdata =0x00;
   //  volatile    uint8_t eeprom_testaddress =0x00;
   volatile    uint8_t usb_readcount =0x00;
   
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
   
   uint8_t TastaturCount=0;
   
   initADC(0);
   
   // ---------------------------------------------------
   // in attach verschobe, nur wenn USB eingesteckt
   
   // Initialize the USB, and then wait for the host to set configuration.
   // If the Teensy is powered without a PC connected to the USB port,
   // this will wait forever.
   
   usb_init();
   uint16_t usbwaitcounter = 0;
   //lcd_putc('a');
   // Ueberspringen wenn kein Host eingesteckt
   hoststatus &= ~(1<< TEENSYPRESENT);
   
   
   while ((usbwaitcounter < 0xFFFA))// && (!usb_configured()))
   {
      //lcd_gotoxy(0,3);
      //lcd_putint12(usbwaitcounter);
      _delay_ms(1);
      if (usb_configured())
      {
         hoststatus |= (1<< TEENSYPRESENT);
         //hoststatus = 1;
         //lcd_gotoxy(19,3);
         //lcd_putc('$');
         break;
         
      }
      
      usbwaitcounter++;
      if (usbwaitcounter > 0x100)
      {
         //lcd_gotoxy(19,3);
         //lcd_putc('!');
         break;
      }
   }
   
   /*
    while ((!usb_configured()) )//|| (usbwaitcounter < 0xFF))
    {
    usbwaitcounter++;
    _delay_ms(1);
    if (usbwaitcounter > 0xFFFE)
    {
    continue;
    }
    
    }
    */
   /* wait */ ;
   lcd_gotoxy(19,0);
   if (hoststatus & (1<<TEENSYPRESENT))
   {
      lcd_putc('$');
   }
   else
   {
      lcd_putc('X');
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
   
   timer0();
   timer1();
   sei();
   
   
   
   //  masterstatus |= (1<<SUB_READ_EEPROM_BIT); // sub soll EE lesen
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
      lcd_puts("CD err");
      lcd_puthex(initerr);
      lcd_putc('*');
      
   }
   else
   {
      lcd_gotoxy(0,0);
      lcd_puts("CD OK ");
      lcd_puthex(initerr);
      lcd_putc('*');
      
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
   
   uint16_t mmcwritecounter=0;
   
   OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
   
   
   //  OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer OSZI
   //  OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
   
   //DDRD |= (1<<4);
   //PORTD |= (1<<4);
   //OSZIA_HI;
   //OSZIPORT |= (1<<OSZI_PULS_A);
   
   
   
   // MARK:  while
   
  if (TEST || (!(OSZIPORT & (1<<TEST_PIN)))) // Testdaten schreiben
  {
     uint16_t ii=0;
     while (ii<0xFF)
     {
        uint16_t wert =500 - ii;
        mmcbuffer[2*ii] = wert & 0x00FF;
        mmcbuffer[2*ii+1] = (wert & 0xFF00)>>8;
        ii += 1;
        
     }
     
     writeerr = mmc_disk_write ((void*)mmcbuffer,2 ,1); // Block 1 ist system
     
     // OSZIA_HI;
     
     lcd_gotoxy(6,3);
     lcd_puts("save ");
     lcd_puthex(writeerr);
     lcd_putc(' ');
     lcd_puthex(blockcounter);
  }
   
   while (1)
   {
      //OSZI_B_LO;
      //Blinkanzeige
      loopcount0+=1;
      
      
      /* **** spi_buffer abfragen **************** */
      // MARK:  spi_rxdata
      
      /* **** end spi_buffer abfragen **************** */
      
      // MARK:  MMC write
      //   if ((mmcstatus & (1<<WRITENEXT)) )
      if (mmcstatus & (1<<WRITENEXT) ) //
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
            /*
             lcd_putc('l');
             lcd_puthex((tempdata & 0x00FF));
             lcd_putc('h');
             lcd_puthex(((tempdata & 0xFF00)>>8));
             lcd_putc(' ');
             */
         }
         else
         {
            usbstatus = DEFAULT;
         }
         
         
         //        writeerr = mmc_disk_write ((void*)mmcbuffer,1 + (mmcwritecounter & 0x800),1);
         // OSZIA_HI;
         
         //         lcd_gotoxy(14,3);
         //         lcd_puthex(writeerr);
         
         //lcd_putc('*');
         
         mmcstatus &= ~(1<<WRITENEXT);
         //         mmcwritecounter++;
         
      }
      //OSZI_A_TOGG;
      
      
#pragma mark Mess-Intervall
      
      if (hoststatus & (1<<MESSUNG_OK)) // Intervall abgelaufen. In ISR gesetzt, Messungen vornehmen
      {
         /*
          ADC 0 lo
          ADC 0 hi
          ADC 1 lo
          ADC 1 hi
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          ADC 12bit lo
          ADC 12bit hi
          
          Digi Ausgang
          Digi Ausgang
          Digi Ausgang
          Digi Ausgang
          Digi Ausgang
          Digi Ausgang
          Digi Ausgang
          Digi Ausgang
          
          Digi Eingang
          Digi Eingang
          Digi Eingang
          Digi Eingang
          Digi Eingang
          Digi Eingang
          Digi Eingang
          Digi Eingang
          
          DA 1 lo
          DA 1 hi
          DA 2 lo
          DA 2 hi
          
          
          
          */
         hoststatus &= ~(1<<MESSUNG_OK);
         // ADC
         uint16_t adcwert = adc_read(0);
         _delay_ms(100);
         
         
         // adcwert *=10;
         // vor Korrektur
         
         // code fuer
         
         sendbuffer[0]= MESSUNG_DATA;
         
         // Data von ADC laden
         sendbuffer[ADCLO]= (adcwert & 0x00FF);
         sendbuffer[ADCHI]= ((adcwert & 0xFF00)>>8);
         
         //zaehler laden
         sendbuffer[DATACOUNT_LO] = (messungcounter & 0x00FF);
         sendbuffer[DATACOUNT_HI] = ((messungcounter & 0xFF00)>>8);
         
         /*
          usbstatus1
          #define SAVE_SD_BIT             0
          #define SAVE_SD_RUN_BIT         1
          #define SAVE_SD_STOP_BIT        2
          */
         if (usbstatus1 & (1<<SAVE_SD_RUN_BIT)) // Daten in mmcbuffer speichern, immer 2 bytes
         {
            
            lcd_gotoxy(8,1);
            lcd_putint(saveSDposition); // 0 .. 255, pos im mmcbuffer, immer 2 byte pro messung
            
            mmcbuffer[2*saveSDposition] = (adcwert & 0x00FF);
            mmcbuffer[2*saveSDposition+1] = ((adcwert & 0xFF00)>>8);
            mmcbuffer[2*saveSDposition+2] = 0;
            mmcbuffer[2*saveSDposition+3] = 0;
            mmcbuffer[2*saveSDposition+4] = 0;
            mmcbuffer[2*saveSDposition+5] = 0;
            mmcbuffer[2*saveSDposition+6] = 0;
            mmcbuffer[2*saveSDposition+7] = 0;
            lcd_putc(' ');
            lcd_putint12(mmcwritecounter % 0x800);
            lcd_putc('b');
            lcd_puthex(blockcounter);
            
            if (saveSDposition == 0) // Data an 0
            {
               lcd_gotoxy(3,3);
               lcd_puthex(mmcbuffer[saveSDposition]);
            }
            saveSDposition += 8;
            mmcwritecounter += 16; // Zaehlung write-Prozesse, immer 2 bytes pro messung
            
            if ((saveSDposition ) >= 0xFF) // Block voll, 2*255 Bytes = 512
            {
               writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
               // OSZIA_HI;
               
               lcd_gotoxy(6,3);
               lcd_puts("save ");
               lcd_puthex(writeerr);
               lcd_putc(' ');
               lcd_puthex(blockcounter);
               saveSDposition = 0;
               sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Nummer des geschriebenen Blocks lo
               sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
               
               blockcounter++;
            }
         }
         else if (usbstatus1 & (1<<SAVE_SD_STOP_BIT)) // Schreiben beenden, letzten Block noch schreiben
         {
            usbstatus1 &= ~(1<<SAVE_SD_STOP_BIT);
            usbstatus1 &= ~(1<<SAVE_SD_RUN_BIT);   // fortlaufendes Schreiben beenden
            // Daten sichern
            // ** DIFF writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
            writeerr = mmc_disk_write ((void*)mmcbuffer,1 + blockcounter,1); // Block 1 ist system
            // OSZIA_HI;
            
            lcd_gotoxy(6,3);
            lcd_puts("resc ");
            lcd_puthex(writeerr);
            lcd_putc(' ');
            lcd_puthex(blockcounter);
            saveSDposition = 0;
            sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF;
            sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi

         }
         
         //adcwert /= 2;
         //lcd_gotoxy(0,1);
         //lcd_putint12(adcwert/4); // *256/1024
         // lcd_putc(' ');
         //OSZIA_LO;
         double adcfloat = adcwert;
         adcfloat = adcfloat *2490/1024; // kalibrierung VREF, 1V > 0.999, Faktor 10, 45 us
         
         adcwert = (((uint16_t)adcfloat)&0xFFFF);
         //OSZIA_HI;
         
         uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         lcd_gotoxy(10,0);
         //lcd_putint(adcwert & 0x00FF);
         //lcd_putc(' ');
         //lcd_putint2((adcwert & 0xFF00)>>8);
         lcd_putc('t');
         lcd_putint12(adcwert);
         //messungcounter++;
         messungcounter += 8; // 8 bytes, naechste zeile
      } // if hoststatus & (1<<MESSUNG_OK)
      
      if (hoststatus & (1<<DOWNLOAD_OK))
      {
         
      }// if (hoststatus & (1<<DOWNLOAD_OK))
      
      
      if (loopcount0==0xCFFF)
      {
         //SPI_PORT ^= (1<<SPI_CLK);
         //SPI_PORT ^= (1<<SPI_MISO);
         //SPI_PORT ^= (1<<SPI_MOSI);
         //SPI_PORT ^= (1<<SPI_SS);
         /*
          uint8_t err = ow_reset();
          lcd_gotoxy(12,0);
          lcd_puthex(err);
          //PORTD &= ~(1<<4);
          OSZIA_LO;
          //clearbit(PORTD, PD4);
          ow_delay_us(250);
          ow_delay_us(250);
          ow_delay_us(50);
          OSZIA_HI;
          //setbit(PORTD, PD4);
          //PORTD |=  (1<<4);
          */
         loopcount0=0;
         loopcount1+=1;
         LOOPLEDPORT ^=(1<<LOOPLED);
         
         //OSZI_A_LO;
         //lcd_gotoxy(18,0);
         //lcd_puthex(loopcount1);
         
         //    sendbuffer[1] = ((usb_readcount));
         //    sendbuffer[2] = (loopcount1 & 0xFF);
         //    sendbuffer[3] = 16;
         // sendbuffer[5] = spi_rxbuffer[0];
         
         for (int i=0;i<SPI_BUFSIZE;i++)
         {
            //lcd_puthex(spi_rxbuffer[i]);
            //            sendbuffer[i+CODE_OFFSET] = spi_rxbuffer[i];
         }
         
         //lcd_gotoxy(0,1);
         //lcd_putc('S');
         //lcd_putc(' ');
         for (int i=0;i<4;i++)
         {
            //   lcd_puthex(sendbuffer[i]);
         }
         
         /*
          lcd_gotoxy(0,1);
          lcd_putc('R');
          lcd_puthex(spi_rxbuffer[0]);
          lcd_putc('*');
          lcd_puthex(spi_rxbuffer[1]);
          
          lcd_puthex(spi_rxbuffer[2]);
          lcd_puthex(spi_rxbuffer[3]);
          lcd_putc('-');
          
          lcd_puthex(spi_rxbuffer[4]);
          lcd_puthex(spi_rxbuffer[5]);
          lcd_puthex(spi_rxbuffer[6]);
          lcd_puthex(spi_rxbuffer[7]);
          */
         /*
          if (OHNE_INTERFACE)
          {
          spi_txbuffer[0] = 0x81;
          tempwert++;
          spi_txbuffer[1] = tempwert & 0x00FF;
          spi_txbuffer[2] = (tempwert & 0xFF00)>>8;
          spi_txbuffer[3] = 7;
          }
          */
         /*
          // Ausgabe auf soft-SPI
          lcd_gotoxy(0,0);
          lcd_putc('T');
          lcd_puthex(spi_txbuffer[0]);
          lcd_putc('+');
          lcd_puthex(spi_txbuffer[1]);
          
          lcd_puthex(spi_txbuffer[2]);
          lcd_puthex(spi_txbuffer[3]);
          */
         // lcd_putc('i');
         
         // lcd_puthex(isrcontrol);
         uint8_t testbit=1;
         if (!((old_H == spi_rxbuffer[testbit])) )
         {
            teensy_err++;
            old_H = spi_rxbuffer[testbit];
            testfix = loopcount1;
         }
         
         
         /*
          if (!((old_H == spi_rxbuffer[3]) && (old_L == spi_rxbuffer[2])) )
          
          {
          teensy_err++;
          old_L = spi_rxbuffer[2];
          old_H = spi_rxbuffer[3];
          }
          */
         
         //lcd_gotoxy(14,1);
         //lcd_puthex(spi_count);
         
         //lcd_gotoxy(16,1);
         //lcd_puthex(spi_count);
         
         //         lcd_gotoxy(18,1);
         //         lcd_puthex(isrcontrol);
         /*
          lcd_puthex(sendbuffer[CODE_OFFSET]);
          lcd_puthex(sendbuffer[CODE_OFFSET+1]);
          lcd_puthex(sendbuffer[ROTARY_OFFSET]);
          lcd_puthex(sendbuffer[ROTARY_OFFSET+1]);
          lcd_putc(' ');
          uint16_t rot = (sendbuffer[ROTARY_OFFSET+1]<<8) | sendbuffer[ROTARY_OFFSET];
          lcd_putint16(rot);
          */
         //lcd_puthex(sendbuffer[9]);
         //lcd_putc(' ');
         //lcd_puthex(sendbuffer[8]);
         //lcd_putc(' ');
         //lcd_puthex(SPI_Data_counter);
         //   sendbuffer[5] = isrcontrol;
         //   sendbuffer[4] = 28;
         //   sendbuffer[6] = 33;
         //   sendbuffer[7] = 34;
         //         lcd_gotoxy(11,0);
         //        lcd_putc('x');
         //         lcd_puthex(usb_readcount);
         
         if (usbstatus & (1<<WRITEAUTO))
         {
            uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         }
         //lcd_puthex(usberfolg);
         
         if ((loopcount1%2 == 0) && (usbstatus & (1<<WRITEAUTO)))
         {
#pragma mark ADC
            /*
             _delay_ms(10);
             uint16_t adcwert = adc_read(0);
             _delay_ms(100);
             
             lcd_gotoxy(10,0);
             lcd_putint(adcwert & 0x00FF);
             lcd_putc(' ');
             lcd_putint2((adcwert & 0xFF00)>>8);
             
             // adcwert *=10;
             // vor Korrektur
             sendbuffer[ADCLO]= (adcwert & 0x00FF);
             sendbuffer[ADCHI]= ((adcwert & 0xFF00)>>8);
             
             //adcwert /= 2;
             lcd_gotoxy(0,1);
             //       lcd_putint12(adcwert/4); // *256/1024
             //       lcd_putc(' ');
             //OSZIA_LO;
             double adcfloat = adcwert;
             adcfloat = adcfloat *2490/1024; // kalibrierung VREF, 1V > 0.999, Faktor 10, 45 us
             
             adcwert = (((uint16_t)adcfloat)&0xFFFF);
             //OSZIA_HI;
             lcd_putint12(adcwert);
             
             */
            //           sendbuffer[ADCLO+2]= (adcwert & 0x00FF);
            //           sendbuffer[ADCHI+2]= ((adcwert & 0xFF00)>>8);
            /*
             lcd_putc(' ');
             lcd_gotoxy(10,1);
             lcd_putint(adcwert & 0x00FF);
             lcd_putc(' ');
             lcd_putint2((adcwert & 0xFF00)>>8);
             lcd_putc('*');
             lcd_puthex(usb_configured());
             */
            
         }
         
         
         if(loopcount1%32 == 0)
         {
            
            
#pragma mark Sensors
            // Temperatur messen mit DS18S20
            
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
         } //
         
         
         
         // MARK:  USB send
         // neue Daten abschicken
         //         if ((usbtask & (1<<EEPROM_WRITE_PAGE_TASK) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         //OSZI_C_LO;
         
         //   uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
         //OSZI_C_HI;
         //OSZI_A_HI;
         
         // OSZI_B_HI;
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
      // MARK: USB_READ
      
      if (r > 0)
      {
         //OSZI_A_LO ;
         cli();
         usb_readcount++;
         uint8_t code = recvbuffer[0];
         lcd_gotoxy(18,2);
         lcd_puthex(code);
         
         if (!(usbstatus == code))
         {
            usbstatus = code;
            lcd_clr_line(2);
            lcd_gotoxy(18,3);
            lcd_puthex(code);
         }
         
         switch (code)
         {
               
            case DEFAULT: // cont read
            {
               //lcd_clr_line(2);
               sendbuffer[0] = DEFAULT;
               sendbuffer[3] = 0;
               mmcwritecounter = 0;
               sendbuffer[0] = DEFAULT;
               //            code = WRITE_MMC_TEST;
               //lcd_putc('c');
               //lcd_puthex(code); // code
               usbstatus1 = recvbuffer[1]; // bit 0: sd mit testdaten beschreiben
               mmcwritecounter = 0;
               //lcd_putc('-');
               //lcd_puthex(usbstatus1); // code
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
               // PWM fuer Channel A
               OCR1A = (recvbuffer[10] | (recvbuffer[11]<<8));
               // lcd_putc('*');
               // lcd_gotoxy(12,0);
               // lcd_putint12(OCR1A);
               
            }break;
               
               // MARK: LOGGER_START
               
            case LOGGER_START:
            {
               hoststatus &= ~(1<<MESSUNG_OK);
               hoststatus |= (1<<DOWNLOAD_OK); // Download von SD, Messungen unterbrechen
               
               lcd_clr_line(2);
               lcd_gotoxy(0,1);
               lcd_putc('l');
               lcd_putc(':');
               lcd_puthex(code);
               
               sendbuffer[0] = LOGGER_START;
               
               
               //code = LOGGER_START; // read block starten
               //lcd_putc('c');
               //lcd_puthex(code); // packetcount
               
               // Block lesen
               lcd_putc('l');
               lcd_puthex(recvbuffer[1]); // startblock lo
               lcd_putc('h');
               lcd_puthex(recvbuffer[2]); // startblock hi
               lcd_putc('*');
               lcd_puthex(recvbuffer[3]); // packetcount
               
               // old
               //startblock = recvbuffer[1] | (recvbuffer[2]<<8); // zu lesender Block auf mmc
               
               startblock = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8); // zu lesender Block auf mmc
               uint8_t paketindex = 0;
               
               // old
               //packetcount = recvbuffer[3] ;// laufender Index Paket, beim Start 0
               
               packetcount = recvbuffer[PACKETCOUNT_BYTE] ;// laufender Index Paket, beim Start 0
               //               packetcount=0;
               // lcd_gotoxy(12,1);
               // lcd_puts(">mmc");
               lcd_gotoxy(0,3);
               lcd_puthex(startblock);
               
               // Beim Start Block aus SD lesen
               readerr = mmc_disk_read((void*)mmcbuffer,1+ startblock,1);
               
               
               if (readerr == 0)
               {
                  lcd_gotoxy(15,1);
                  lcd_puts(">OK ");
                  sendbuffer[DATA_START_BYTE -1] = 11;
                  
                  // Header uebertragen
                  uint8_t headerindex=0;
                  for (headerindex = 0;headerindex < DATA_START_BYTE-2;headerindex++)
                  {
                     sendbuffer[headerindex+2] = mmcbuffer[headerindex];
                     //sendbuffer[headerindex+2] = headerindex;
                  }
                  /*
                   for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 48 bytes fuer sendbuffer
                   {
                   sendbuffer[PACKET_START + paketindex] = mmcbuffer[paketindex];
                   
                   
                   }
                   */
               } // if readerr==0
               else
               {
                  lcd_gotoxy(14,1);
                  lcd_puts(">err");
               }
               
               // old
               //sendbuffer[3] = 0;
               
               sendbuffer[PACKETCOUNT_BYTE] = 0; //
               sendbuffer[1] = readerr;
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               lcd_gotoxy(18,1);
               lcd_puthex(usberfolg);
               
            }break;
               
               
               // MARK: LOGGER_CONT
            case LOGGER_CONT:
            {
               lcd_clr_line(2);
               lcd_gotoxy(0,2);
               lcd_putc('c');
               lcd_putc(':');
               sendbuffer[0] = LOGGER_CONT;
               usbstatus1 = recvbuffer[1];
               //code = LOGGER_CONT;
               //lcd_putc('c');
               lcd_puthex(code); // code
               
               
               // Block lesen
               //lcd_puthex(recvbuffer[STARTBLOC]); // startblock lo
               //lcd_puthex(recvbuffer[2]); // startblock hi
               //lcd_putc(' ');
               lcd_putc(' ');
               lcd_puthex(recvbuffer[PACKETCOUNT_BYTE]); // packetcount
               
               // old
               //packetcount = recvbuffer[3];

               packetcount = recvbuffer[PACKETCOUNT_BYTE];
               
               
               
               uint8_t paketindex = 0;
               for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 48 bytes fuer sendbuffer
               {
                  sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
               }
               
               // Old
               //sendbuffer[3] = ++packetcount; //
               sendbuffer[PACKETCOUNT_BYTE] = ++packetcount; //
               
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
               lcd_gotoxy(18,2);
               lcd_puthex(usberfolg);
               
            }break;
               
               // MARK: LOGGER_STOP
            case LOGGER_STOP:
            {
               hoststatus &= ~(1<<DOWNLOAD_OK); // Download von SD beendet, Messungen fortsetzen
               //lcd_clr_line(1);
               lcd_clr_line(2);
               lcd_gotoxy(0,2);
               lcd_putc('s');
               lcd_putc(':');
               lcd_puthex(code); // code
               
               mmcwritecounter = 0;
               usbstatus = 0;
               sendbuffer[0] = LOGGER_STOP;
               
               // old
               //sendbuffer[3] = 0; // packetcount

               sendbuffer[PACKETCOUNT_BYTE] = 0; // packetcount
               
            }break;
               
               // MARK: WRITE_MMC_TEST
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
               usbstatus1 = recvbuffer[1]; // bit 0: sd mit testdaten beschreiben
               mmcwritecounter = 0;
               //lcd_putc('-');
               //lcd_puthex(usbstatus1); // code
               
            }break;
               
               // MARK: USB_STOP
            case USB_STOP: // Host ist ausgeschaltet
            {
               lcd_clr_line(2);
               lcd_gotoxy(0,2);
               lcd_putc('h');
               lcd_putc(':');
               lcd_puthex(code); // code
               //sendbuffer[0] = USB_STOP;
               
            }break;
               
               // MARK: LOGGER_SETTING
            case LOGGER_SETTING:
            {
               sendbuffer[0] = LOGGER_SETTING;
               usbstatus1 = recvbuffer[1];
               intervall = recvbuffer[TAKT_LO_BYTE] | (recvbuffer[TAKT_HI_BYTE]<<8);
               
               
               lcd_clr_line(2);
               lcd_gotoxy(0,2);
               lcd_putc('s');
               lcd_putc(':');
               lcd_puthex(code); // code
               lcd_putc(' ');
               lcd_putint12(intervall);
               lcd_gotoxy(6,3);
               lcd_puts("set  ");
               
               
            }break;
               
               
               // MARK: MESSUNG_START
            case MESSUNG_START:
            {
               messungcounter = 0;
               sendbuffer[0] = MESSUNG_START;
               lcd_clr_line(2);
               lcd_gotoxy(0,2);
               lcd_putc('m');
               lcd_putc(':');
               lcd_puthex(code); // code
               usbstatus = code;
               usbstatus1 = recvbuffer[1];
               
               // usb_init();
               
               mmcwritecounter = 0;
               
               saveSDposition = 0; // Start der Messung immer am Anfang des Blocks
               
               abschnittnummer = recvbuffer[ABSCHNITT_BYTE]; // Abschnitt,
               
               blockcounter = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8);
               
               lcd_putc(' ');
               lcd_puthex(blockcounter);
               
               lcd_putc(' ');
               lcd_puthex(usbstatus1);
               lcd_putc(' ');
               lcd_puthex(saveSDposition);
               
               lcd_gotoxy(6,3);
               lcd_puts("start ");
               sendbuffer[1] = usbstatus1;
               sendbuffer[2] = 17;
               sendbuffer[7] = 19;
               sendbuffer[15] = 21;

               
               mmcbuffer[0] = 17;
               mmcbuffer[7] = 19;
               mmcbuffer[15] = 21;
               saveSDposition = 8;
               
               
               /*lcd_puthex(saveSDposition);
                lcd_putc(' ');
                lcd_puthex(mmcbuffer[saveSDposition]);
                lcd_putc('*');
                //delay_ms(200);
                */
               uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
               lcd_gotoxy(18,2);
               lcd_puthex(usberfolg);

            }break;
               
               // MARK: MESSUNG_STOP
            case MESSUNG_STOP:
            {
               sendbuffer[0] = MESSUNG_STOP;
               lcd_clr_line(2);
               lcd_gotoxy(0,2);
               lcd_putc('h');
               lcd_putc(':');
               lcd_puthex(code); // code
               usbstatus = code;
               usbstatus1 = recvbuffer[1];
               sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF;
               sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0x00FF)>>8;
               lcd_gotoxy(6,3);
               lcd_puts("stop ");
               
            }break;
               
            default:
            {
               break;
            }
               
         }
         
         code=0;
         sei();
         
         //OSZI_D_HI;
         //OSZI_A_HI ;
      } // r>0, neue Daten
      else
      {
         //OSZI_B_LO;
      }
      
      /**	End USB-routinen	***********************/
      
      
      
      //OSZI_B_HI;
      
   }//while
   //free (sendbuffer);
   
   // return 0;
}
