//************************************************************************
// vim:sw=8:ts=8:si:et: 
/*                                                                    
*        Access Dallas 1-Wire Device with ATMEL AVRs                 
*                                                                   
*              Author: Peter Dannegger                              
*                      danni@specs.de                              
*                                                                 
* modified by Martin Thomas <eversmith@heizung-thomas.de> 9/2004 
*
*          Modifications and updates by Guido Socher, June 2009
*         The code was also reduced to only the needed function for
*         this application.
*         http://tuxgraphics.org/electronics/
*Copyright: GPL
************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include "timeout.h"
#include "onewire.h"

/*******************************************/
/* Hardware connection, edit as needed     */
/*******************************************/
// the bus is on PB7:
#define OW_GET_IN    (OW_IN&(1<<OW_PIN))
// output and low:
#define OW_DIR_OUT_LOW()   OW_OUT&=~(1<<OW_PIN);OW_DDR|=1<<OW_PIN
// output and high:
#define OW_DIR_OUT_HIGH()   OW_OUT|=(1<<OW_PIN);OW_DDR|=1<<OW_PIN
// if input tie also pullup on:
#define OW_DIR_IN()    OW_DDR &= ~(1<<OW_PIN);OW_OUT|=1<<OW_PIN

// there is a bit more delay here due to overhead.
// the overhead is about 7us

/*
 http://www.atmel.com/webdoc/AVRLibcReferenceManual/group__util__delay__basic_1ga4e3957917c4c447d0f9166dac881b4e3.html
 Function _delay_loop_1()
 void _delay_loop_1(
	uint8_t __count)
 Delay loop using an 8-bit counter __count, so up to 256 iterations are possible. (The value 256 would have to be passed as 0.) The loop executes three CPU cycles per iteration, not including the overhead the compiler needs to setup the counter register.
 Thus, at a CPU speed of 1 MHz, delays of up to 768 microseconds can be achieved.
 
 */

void ow_delay_us(uint8_t us)
{
        while(us--)
        {
                _delay_loop_1(1); // 4 would be 1us without overhead
           //_delay_us(1);
        }
}

inline uint8_t ow_input_pin_state()
{
        if (OW_GET_IN)
        {
                return(1);
        }
        return(0);
}

// see http://www.maxim-ic.com/app-notes/index.mvp/id/162
uint8_t ow_reset(void)
{
        uint8_t err;
        OW_DIR_OUT_LOW(); // pull OW-Pin low for 480us
   
        ow_delay_us(230);  // min 480us in total
        ow_delay_us(230);

   ow_delay_us(90);  // min 480us in total
   
        // set Pin as input - wait for clients to pull low
        OW_DIR_IN(); // input and pull up
   //return(0);
    //    ow_delay_us(53);   // 60 us
   ow_delay_us(33);   // 60 us
        err = ow_input_pin_state(); // devices should now pull low
        // nobody pulled to low, still high-> error
        // after a delay the clients should release the line
        // and input-pin gets back to high due to pull-up-resistor
  
   //ow_delay_us(233);  // max 240us in total
      ow_delay_us(255);
   if (err==1)
        {
                return(2); // no presence puls
        }
   
    
   
        if( ow_input_pin_state() == 0 )                // short circuit
                return(2);
        return(0);
}

/* Timing issue when using runtime-bus-selection (!OW_ONE_BUS):
   The master should sample at the end of the 15-slot after initiating
   the read-time-slot. The variable bus-settings need more
   cycles than the constant ones so the delays had to be shortened 
   to achive a 15uS overall delay 
   Setting/clearing a bit in I/O Register needs 1 cyle in OW_ONE_BUS
   but around 14 cyles in configureable bus (us-Delay is 4 cyles per uS) */
uint8_t ow_bit_io1( uint8_t b )
{
   OW_DIR_OUT_LOW(); // drive bus low
   ow_delay_us(1); // Recovery-Time 1us
   if ( b )
   {
      OW_DIR_OUT_HIGH();
   }
   else
   {
      OW_DIR_OUT_LOW();
   }
   ow_delay_us(38);
   OW_DIR_IN();
   ow_delay_us(2); // Recovery-Time 1us
   // wuffwuff delay was 15uS-1 see comment above
   OW_DIR_OUT_LOW(); // drive bus low
   ow_delay_us(1); // Recovery-Time 1us
   OW_DIR_IN();
   ow_delay_us(7);
   b=ow_input_pin_state();
   ow_delay_us(38);
   return b;
}

uint8_t ow_bit_io( uint8_t b )
{
   
        OW_DIR_OUT_LOW(); // drive bus low
        ow_delay_us(1); // Recovery-Time
        if ( b )
        {
                OW_DIR_IN(); // if bit is 1 set bus high (by ext. pull-up)
        }
        // wuffwuff delay was 15uS-1 see comment above
        ow_delay_us(7);
        b=ow_input_pin_state();
        ow_delay_us(50);
        OW_DIR_IN();
        return b;
}

uint8_t ow_byte_wr( uint8_t b )
{
        uint8_t i = 8, j;
        
        do {
                j = ow_bit_io( b & 1 );
                b >>= 1;
                if( j ) b |= 0x80;
        } while( --i );
        
        return b;
}


uint8_t ow_byte_rd( void )
{
  // read by sending 0xff (a dontcare?)
  return ow_byte_wr( 0xFF ); 
}


uint8_t ow_rom_search( uint8_t diff, uint8_t *id )
{
        uint8_t i, j, next_diff;
        uint8_t b;
        
        if( ow_reset() ) return OW_PRESENCE_ERR;   // error, no device found
        
        ow_byte_wr( OW_SEARCH_ROM );               // ROM search command
        next_diff = OW_LAST_DEVICE;                // unchanged on last device
        
        i = OW_ROMCODE_SIZE * 8;                   // 8 bytes
        
        do {
                j = 8;                              // 8 bits
                do {
                        b = ow_bit_io( 1 );         // read bit
                        if( ow_bit_io( 1 ) ) 
								{      // read complement bit
                                if( b )             // 11
                                return OW_DATA_ERR; // data error
                        } 
								else 
								{
                                if( !b ) 
										  {          // 00 = 2 devices
                                        if( diff > i || ((*id & 1) && diff != i) ) 
													 {
                                        b = 1;      // now 1
                                        next_diff = i;     // next pass 0
                                        }
                                }
                        }
                        ow_bit_io( b );              // write bit
                        *id >>= 1;
                        if( b ) *id |= 0x80;         // store bit
                        i--;
                } while( --j );
                id++;                                // next byte
        } while( i );
        return next_diff;                            // to continue search
}


void ow_command( uint8_t command, uint8_t *id )
{
        uint8_t i;
        ow_reset();
        if( id ) {
                ow_byte_wr( OW_MATCH_ROM );                        // to a single device
                i = OW_ROMCODE_SIZE;
                do {
                        ow_byte_wr( *id );
                        id++;
                } while( --i );
        }else{
                ow_byte_wr( OW_SKIP_ROM );                        // to all devices
        }
        
        ow_byte_wr( command );
}
/*
void ow_read_rom(int *d, int sensor) 
{
//http://www.phanderson.com/PIC/PICC/CCS_PCM/ds1820.html
   int n;

   _1w_init(sensor);
   _1w_out_byte(0x33, sensor);
   for(n=0; n&lt8; n++)
   {
      d[n]=_1w_in_byte(sensor);
   }
}
*/