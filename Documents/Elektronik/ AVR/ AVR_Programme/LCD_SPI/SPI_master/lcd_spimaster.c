//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//


#include <stdlib.h>
//#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#include <avr/wdt.h>

#include "lcd.c"

#include "adc.c"
#include "defines.h"
#include "spi_function_master.c"

volatile    	uint16_t loopcount0=0;
volatile       uint16_t loopcount1=0;

volatile    uint16_t timercount0=0;
volatile    uint16_t timercount1=0;
volatile    uint16_t adccount0=0;
volatile    uint8_t blinkcount=0;

volatile    uint8_t pwmpos=0;
volatile    uint8_t sollwert=0;
volatile    uint8_t istwert=0;
volatile    uint8_t lastwert=0;
volatile    int8_t fehler=0;
volatile    int8_t lastfehler=0;
volatile    int16_t fehlersumme=0;

volatile    int16_t stellwert=0;
volatile    uint8_t masterstatus=0;




void delay_ms(unsigned int ms);

void r_itoa16(int16_t zahl, char* string)
{
   uint8_t i;
   int16_t original=zahl;
   string[7]='\0';                  // String Terminator
   if( zahl < 0 ) {                  // ist die Zahl negativ?
      string[0] = '-';
      zahl = -zahl;
   }
   else string[0] = ' ';             // Zahl ist positiv
   
   for(i=6; i>=1; i--)
   {
      string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   if (abs(original) < 1000)
   {
      string[1]= ' ';
   }
  if (abs(original) < 100)
   {
      string[2]= ' ';
   }
   if (abs(original) < 10)
   {
      string[3]= ' ';
   }

}
void r_itoa12(int16_t zahl, char* string)
{
   uint8_t i;
   int16_t original=zahl;
   string[5]='\0';                  // String Terminator
   if( zahl < 0 ) {                  // ist die Zahl negativ?
      string[0] = '-';
      zahl = -zahl;
   }
   else string[0] = ' ';             // Zahl ist positiv
   
   for(i=4; i>=1; i--)
   {
      string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   
   if (abs(original) < 1000)
   {
      string[1]= ' ';
   }
   if (abs(original) < 100)
   {
      string[2]= ' ';
   }
   if (abs(original) < 10)
   {
      string[3]= ' ';
   }
   
}


void r_itoa8(int8_t zahl, char* string)
{
   uint8_t i;
   int8_t original=zahl;
   string[4] = '\0';                  // String Terminator
   if( zahl < 0 )
   {                  // ist die Zahl negativ?
      string[0] = '-';
      zahl = -zahl;
   }
   else
   {
      string[0] = ' ';             // Zahl ist positiv
   }
   for(i = 3; i >= 1; i--)
   {
      uint8_t temp = zahl % 10;
      string[i] = temp +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      
      zahl /= 10;
   }
   if (abs(original) < 100)
   {
      string[1]= ' ';
   }
   if (abs(original) < 10)
   {
      string[2]= ' ';
   }
}


void slaveinit(void)
{
 	OUTDDR |= (1<<PWMPIN);		//Pin 0 von PORT D als Ausgang fuer PWM
	OUTPORT |= (1<<PWMPIN);		//HI
	
   OSZIDDR |= (1<<OSZIA);		//Pin 1 von PORT D als Ausgang fuer OSZI
 	OSZIPORT |= (1<<OSZIA);		//HI
   //DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
   LOOPLEDDDR |= (1<<LOOPLED);		//Pin 4 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED);		//Pin 4 von PORT D als Ausgang fuer LED Loop

   LOOPLEDDDR |= (1<<TOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Heizung

   
	
	DDRB &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang fŸr Taste 1
	PORTB |= (1<<PB0);	//Pull-up

	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang fŸr Taste 2
	PORTB |= (1<<PB1);	//Pull-up
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

//	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
   ADCDDR &= ~(1<<ADC_SOLL_PIN);	//Pin 1 von PORT C als Eingang fuer ADC soll-Wert
	ADCDDR &= ~(1<<ADC_IST_PIN);	//Pin 2 von PORT C als Eingang fuer ADC ist-Wert
//	ADCPORT |= (1<<ADCPIN); //Pull-up
//	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC3); //Pull-up
//	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur
//	PORTC |= (1<<DDC3); //Pull-up


	
	
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


void timer0(void)
{
	//----------------------------------------------------
	// Set up timer 0
	//----------------------------------------------------
   /*
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS00) | _BV(CS02);
    OCR0A = 0x2;
    TIMSK0 = _BV(OCIE0A);
    */
   
//   DDRD |= (1<< PORTD6);   // OC0A Output
   /*
   TCCR0A |= (1<<WGM00);   // fast PWM  top = 0xff
   TCCR0A |= (1<<WGM01);   // PWM
   //TCCR0A |= (1<<WGM02);   // PWM
   
   TCCR0A |= (1<<COM0A1);   // set OC0A at bottom, clear OC0A on compare match
   TCCR0B |= 1<<CS02;
   TCCR0B |= 1<<CS00;
   
   OCR0A=10;
   TIMSK0 |= (1<<OCIE0A);
   */
   
   //TCCR0B |= (1<<CS00)|(1<<CS02);	//Takt /64 Intervall 64 us
   TCCR0B |= (1<<CS02);	//Takt /256 Intervall
   
   TIFR0 |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
   TIMSK0 |= (1<<TOIE0);			//Overflow Interrupt aktivieren
   TCNT0 = 0x00;					//RŸcksetzen des Timers

   
}


ISR(TIMER0_COMPA_vect)
{
   //PORTD ^= (1<< PORTD6);
   //OCR0A = anzeigewert;
   //OCR0A++;
   
}

ISR(TIMER0_OVF_vect)
{
   adccount0++;
   if (adccount0 > 0x1E)
   {
      //OSZITOG;
      adccount0=0;
      //blinkcount++;
      
      //OSZILO;
      masterstatus |= (1<<PWM_ADC);// ADC

      
   }
   
   timercount0++;
  // OSZITOG;
   //if (timercount0 > 0x0002)
   {
      timercount0=0;
      //OSZITOG;
      timercount1++;
      if (timercount1==0x200)
      {
         
         masterstatus |= (1<<PWM_ON);
         OUTPORT |= (1<<PWMPIN);
         timercount1=0;
      }
      
      else if (fehler > 30) // schneller aufheizen
       {
         if (timercount1 > 2*(uint8_t)stellwert)
         {
             
             OUTPORT &= ~(1<<PWMPIN);
         }
       }
      else if (timercount1 > (uint8_t)stellwert)
      {
         
         OUTPORT &= ~(1<<PWMPIN);
         //masterstatus |= (1<<PWM_ON);// ADC
      }
      
   }
   
}




void main (void) 
{
   MCUSR = 0;
	//wdt_disable();

	slaveinit();
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	
   spi_master_init();
		
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts("SPI_Master\0");
	

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t Tastenmasterstatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	//initADC(TASTATURPIN);
	
	//uint16_t startdelay1=0;

	//uint8_t twierrcount=0;
	//LOOPLEDPORT |=(1<<LOOPLED);
	
	delay_ms(800);

	//lcd_clr_line(0);
   timer0();
   
   initADC(1);
   sollwert = readKanal(1)>>2;
   sei();
   uint8_t i=0;
	while (1)
   {
      // PORTD |= (1<<0);
      // _delay_ms(1000);
      // PORTD &= ~(1<<0);
      //Blinkanzeige
      loopcount0++;
      if (loopcount0>=LOOPSTEP)
      {
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         
         //delay_ms(10);
         //PORTD ^= (1<<4);
         loopcount1++;
         if ((loopcount1 >4) )
         {
            loopcount1=0;
            lcd_gotoxy(12,0);
            set_LCD(0x0D);// CR
            _delay_us(2);
            set_LCD(LCD_GOTOXY);
            _delay_us(2);
            lcd_gotoxy(0,1);
            uint8_t line = 1;
            uint8_t col = 8;
            
            // pos auf LCD
            int goto_pos = (col <<3) | (line & 0x07); // 5 bit col, 3 bit line
            lcd_puthex(goto_pos);
            lcd_putc(' ');
            set_LCD(goto_pos);
            
            
            for (i=0;i<8;i++)
            {
               
               char c ='A'+ (blinkcount++ & 0x0F);
               //lcd_putc(c);
               //
               set_LCD(LCD_PUTC);
               //_delay_us(2);
               set_LCD(c);
            }
            //_delay_us(2);
            set_LCD(0);
            
            //blinkcount++;
            //delay_ms(2);
            //lcd_gotoxy(12,0);
            //lcd_putint(blinkcount);
         }
        //pwmpos = readKanal(1)>>2;
         //lcd_putc('*');
         //lcd_putint(pwmpos);
         //lcd_putc('*');
         //lcd_putint(timercount1);
         //OSZITOG;
         if (sollwert > istwert)
         {
            LOOPLEDPORT ^=(1<<TOPLED);
         }
         else
         {
            LOOPLEDPORT |=(1<<TOPLED);
         }
                  
      }
      /*
      if (masterstatus & (1<<PWM_ADC))
      {
         masterstatus |= (1<<PWM_ON);
         OSZIHI;
      }
      */
      if (masterstatus & (1<<PWM_ADC)) // soll- und ist-werte lesen, PI aktualisieren
      {
   //      OSZIHI;
         masterstatus &= ~(1<<PWM_ADC);
         sollwert = readKanal(1)>>2;
         lcd_gotoxy(0,2);
         lcd_putc('s');
         lcd_putint12(sollwert);
         
         lcd_gotoxy(8,2);
         lcd_putc('i');
         istwert = readKanal(2)>>2;
         lcd_putint12(istwert);
 //     }

      
         
#pragma mark PID
         /*
          e = w - x;                       //Vergleich
          esum = esum + e;                 //Integration I-Anteil
          if (esum < -400) {esum = -400;}  //Begrenzung I-Anteil
          if (esum > 400) {esum = 400;}
          y = Kp*e + Ki*Ta*esum;           //Reglergleichung
          if (y < 0) {y = 0;}              //Begrenzung Stellgrš§e
          if (y > 255) {y = 255;}
          PWM = y;                         //†bergabe Stellgrš§e
          
          
          int8_t fehler=0;
          int16_t fehlersumme=0;
          FŸhrungsgrš§e (Sollwert) w
          Vorgegebener Wert, auf dem die Regelgrš§e durch die Regelung gehalten werden soll. Sie ist eine von der Regelung nicht beeinflusste Grš§e und wird von au§en zugefŸhrt.
          Regelgrš§e (Istwert) x
          */
         char buffer[16];
         fehler = sollwert - istwert;
         lcd_gotoxy(0,3);
         lcd_putc('f');
         if (fehler>=0)
         {
         //   lcd_putc(' '); // Platz des Minuszeichens
         }
         if (fehler < 10)
         {
         //   lcd_putc(' ');
         }
         //itoa(fehler,buffer,10);
         r_itoa8(fehler,buffer);
         lcd_puts(buffer);
         lcd_putc(' ');
         /*
         lcd_gotoxy(0,1);
         if (fehler<0)
         {
            lcd_putint12(fehler * (-1));
         }
         else
         {
         lcd_putint12(fehler);
         }
        // lcd_putc(' ');
         */
         
         fehlersumme += fehler;
         if (fehlersumme < -20)
         {
            fehlersumme = -20;
         }
         if (fehlersumme > 20)
         {
            fehlersumme = 20;
         }
         
         
         
         itoa(fehlersumme,buffer,10);
         //r_itoa16(fehlersumme,buffer);

         lcd_gotoxy(6,3);
         lcd_putc('f');
         lcd_putc('s');
         lcd_gotoxy(9,3);
         lcd_puts("    ");
         lcd_gotoxy(9,3);

         if (fehlersumme>=0)
         {
            lcd_putc(' '); // Platz des Minuszeichens
         }
          lcd_puts(buffer);

         stellwert = K_PROP * fehler + K_INT_DIV_TIME * fehlersumme +   ((fehler-lastfehler) >> K_DIFF_TIME);
         lastfehler = fehler;
         if (stellwert < 0)
         {
            stellwert = 0;
         }
         if (stellwert > 255)
         {
            stellwert = 255;
         }
         
         lcd_gotoxy(14,3);
         
         
         lcd_putc('s');lcd_putc(' ');
         lcd_putint(stellwert);
         
         lcd_gotoxy(19,2);
         if (blinkcount%2)
         {
              lcd_putc('*');
         }
         else
         {
             lcd_putc(' ');
         }
        
      
      }
      
      
      /**	Ende Startroutinen	***********************/
      
      
      
      
      if (!(PINB & (1<<PB0))) // Taste 0
      {
         //lcd_gotoxy(12,1);
         //lcd_puts("P0 Down\0");
         
         if (! (Tastenmasterstatus & (1<<PB0))) //Taste 0 war nich nicht gedrueckt
         {
            //RingD2(5);
            Tastenmasterstatus |= (1<<PB0);
            Tastencount=0;
            //lcd_gotoxy(0,1);
            //lcd_puts("P0 \0");
            //lcd_putint(Tastenmasterstatus);
            //delay_ms(800);
         }
         else
         {
            
            
            Tastencount ++;
            //lcd_gotoxy(7,1);
            //lcd_puts("TC \0");
            //lcd_putint(Tastencount);
            
            if (Tastencount >= Tastenprellen)
            {
            }
         }//else
         
      }	// Taste 0
      
      
      if (!(PINB & (1<<PB1))) // Taste 1
      {
         //lcd_gotoxy(12,1);
         //lcd_puts("P1 Down\0");
         
         if (! (Tastenmasterstatus & (1<<PB1))) //Taste 1 war nicht nicht gedrueckt
         {
            Tastenmasterstatus |= (1<<PB1);
            Tastencount=0;
            //lcd_gotoxy(3,1);
            //lcd_puts("P1 \0");
            //lcd_putint(Servoimpulsdauer);
            //delay_ms(800);
            
         }
         else
         {
            //lcd_gotoxy(3,1);
            //lcd_puts("       \0");
            
            Tastencount ++;
            if (Tastencount >= Tastenprellen)
            {
               
               
               Tastencount=0;
               Tastenmasterstatus &= ~(1<<PB1);
            }
         }//	else
         
      } // Taste 1
      
      /* ******************** */
      //		initADC(TASTATURPIN);
      //		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
      
      Tastenwert=0;
      
      //lcd_gotoxy(3,1);
      //lcd_putint(Tastenwert);
      
      if (Tastenwert>23)
      {
         /*
          0:
          1:
          2:
          3:
          4:
          5:
          6:
          7:
          8:
          9:
          */
         
         TastaturCount++;
         if (TastaturCount>=50)
         {
            
            //lcd_clr_line(1);
            //				lcd_gotoxy(8,1);
            //				lcd_puts("T:\0");
            //				lcd_putint(Tastenwert);
            
            uint8_t Taste=0;//=Tastenwahl(Tastenwert);
            
            
            
            TastaturCount=0;
            Tastenwert=0x00;
            
            switch (Taste)
            {
               case 0://
               { 
                  
               }break;
                  
               case 1://
               { 
               }break;
                  
               case 2://
               { 
                  
               }break;
                  
               case 3://
               { 
                  
               }break;
                  
               case 4://
               { 
                  
               }break;
                  
               case 5://
               { 
                  
                  
               }break;
                  
               case 6://
               { 
               }break;
                  
               case 7://
               { 
                  
               }break;
                  
               case 8://
               { 
                  
               }break;
                  
               case 9://
               { 
               }break;
                  
                  
            }//switch Tastatur
         }//if TastaturCount	
         
      }//	if Tastenwert
      
      //	LOOPLEDPORT &= ~(1<<LOOPLED);
   }//while


// return 0;
}
