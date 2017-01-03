#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*******************************************/
/* Hardware connection                     */
/*******************************************/

#define MAXSENSORS 5

/*
// Atmega8 PORTC PIN3 als Anschluss fuer DS-Sensor
#define OW_PIN  PORTC3
#define OW_IN   PINC
#define OW_OUT  PORTC
#define OW_DDR  DDRC
*/
   
   
//
//#if(MCU == atmega32u4)//        # Teensy 2.0
   
#define OW_PIN  PORTB4
   
#define OW_IN   PINB
#define OW_OUT  PORTB
#define OW_DDR  DDRB

   /*
#elif(MCU == atmega644p)//        # Atmega644p
#define OW_PIN  PORTA4
   
#define OW_IN   PINA
#define OW_OUT  PORTA
#define OW_DDR  DDRA
 
#endif
*/
#define OW_CONF_DELAYOFFSET 0


// Recovery time (T_Rec) minimum 1usec - increase for long lines 
// 5 usecs is a value give in some Maxim AppNotes
// 30u secs seem to be reliable for longer lines
//#define OW_RECOVERY_TIME        5  /* usec */
//#define OW_RECOVERY_TIME      300 /* usec */
//#define OW_RECOVERY_TIME         10 /* usec */
#define OW_RECOVERY_TIME         30 /* usec */

// Use AVR's internal pull-up resistor instead of external 4,7k resistor.
// Based on information from Sascha Schade. Experimental but worked in tests
// with one DS18B20 and one DS18S20 on a rather short bus (60cm), where both 
// sensores have been parasite-powered.
#define OW_USE_INTERNAL_PULLUP     1  /* 0=external, 1=internal */

/*******************************************/


#define OW_MATCH_ROM    0x55
#define OW_SKIP_ROM     0xCC
#define OW_SEARCH_ROM   0xF0

#define OW_SEARCH_FIRST 0xFF        // start new search
#define OW_PRESENCE_ERR 0xFF
#define OW_DATA_ERR     0xFE
#define OW_LAST_DEVICE  0x00        // last device found

// rom-code size including CRC
#define OW_ROMCODE_SIZE 8

extern uint8_t ow_reset(void);

extern uint8_t ow_bit_io( uint8_t b );
extern uint8_t ow_byte_wr( uint8_t b );
extern uint8_t ow_byte_rd( void );

extern uint8_t ow_rom_search( uint8_t diff, uint8_t *id );

extern void ow_command( uint8_t command, uint8_t *id );
extern void ow_command_with_parasite_enable( uint8_t command, uint8_t *id );

extern void ow_parasite_enable( void );
extern void ow_parasite_disable( void );
extern uint8_t ow_input_pin_state( void );

#ifndef OW_ONE_BUS
extern void ow_set_bus( volatile uint8_t* in,
	volatile uint8_t* out,
	volatile uint8_t* ddr,
	uint8_t pin );
#endif

#ifdef __cplusplus
}
#endif

#endif
