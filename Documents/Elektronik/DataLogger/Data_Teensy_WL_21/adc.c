/*
 *  adc.c
 *  TWI_Master
 *
 *  Created by Sysadmin on 12.11.07.
 *  Copyright 2007 Ruedi Heimlicher. All rights reserved.
 *
 */

#include "adc.h"
#include <avr/io.h>




void initADC(uint8_t derKanal)
{
   VREF_Quelle = ADC_REF_INTERNAL;
   
   //ADCSRA = (1<<ADEN) |(1<<ADPS2) | (1<<ADPS0);
   ADCSRA = (1<<ADEN) | ADC_PRESCALER;       // Frequenzvorteiler auf 32 setzen und ADC aktivieren
   ADCSRB = (1<<ADHSM) | (derKanal & 0x20);  // high speed mode
  
   ADMUX = ADMUX = VREF_Quelle;         // configure mux input und Ÿbergebenen Kanal waehlen
   
   
   
   /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
    also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
   ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
   while ( ADCSRA & (1<<ADSC) ) {
      ;     // auf Abschluss der Wandlung warten
   }
}



int16_t adc_read(uint8_t derKanal) 
{
   uint16_t result = 0;
   uint8_t low, i ;
   ADCSRA = (1<<ADEN) | ADC_PRESCALER;             // enable ADC  f/64
   
   ADCSRB = (1<<ADHSM) | (derKanal & 0x20);             // high speed mode
   ADMUX = 0;
   ADMUX = ADC_REF_INTERNAL | (derKanal & 0x1F); // Interne Rev als Referenz
   
   ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
   while (ADCSRA & (1<<ADSC)) ;                    // wait for result
   for(i=0;i<4;i++)
   {
      ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0) | (1<<ADSC); // start the conversion
      while (ADCSRA & (1<<ADSC)) ;                    // wait for result
      low = ADCL;                                     // must read LSB first
      result +=((ADCH << 8) | low);
   }
   result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
   return result;
}


uint16_t readKanal(uint8_t derKanal) //Unsere Funktion zum ADC-Channel aus lesen
{
   uint8_t i;
   uint16_t result = 0;         //Initialisieren wichtig, da lokale Variablen
   //nicht automatisch initialisiert werden und
   //zufŠllige Werte haben. Sonst kann Quatsch rauskommen
   ADMUX = 0;
   ADMUX = VREF_Quelle | (derKanal & 0x3F);
   
   // Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen
   for(i=0;i<4;i++)
   {
      ADCSRA |= (1<<ADSC);            // eine Wandlung
      while ( ADCSRA & (1<<ADSC) )
      {
         ;     // auf Abschluss der Wandlung warten
      }
      result += ADCW;            // Wandlungsergebnisse aufaddieren
   }
   //  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
   
   result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
   
   return result;
}

uint16_t readKanal11(uint8_t derKanal) //Unsere Funktion zum ADC-Channel aus lesen
{
   uint8_t i;
   uint16_t result = 0;         
   //Initialisieren wichtig, da lokale Variablen
   //nicht automatisch initialisiert werden und
   //zufŠllige Werte haben. Sonst kann Quatsch rauskommen
   ADMUX = 0;
   ADMUX |= (1 << REFS1);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
   ADMUX |= (1 << REFS0);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin

   ADMUX |= (1 << MUX1);  //Temperature Sensor - 100011
   ADMUX |= (1 << MUX0);  //Temperature Sensor - 100111  
   ADMUX = 0xE3;
   // Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen
   for(i=0;i<4;i++)
   {
      ADCSRA |= (1<<ADSC);            // eine Wandlung
      while ( ADCSRA & (1<<ADSC) )
      {
         ;     // auf Abschluss der Wandlung warten
      }
      result += ADCW;            // Wandlungsergebnisse aufaddieren
   }
   //  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
   
   result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
   
   return result;
}



void closeADC()
{
ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
}


// http://www.narkidae.com/research/atmega-core-temperature-sensor/
// https://github.com/Synapseware/examples/blob/master/chip-data/atmega32u4.md
// http://yopero-tech.blogspot.ch/2013/01/reading-12-adc-channels-on-atmega32u4.html
void setupADC()
{
   
   //ADC Multiplexer Selection Register
   ADMUX = 0;
   ADMUX |= (1 << REFS1);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
   ADMUX |= (1 << REFS0);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
   ADMUX |= (0 << MUX4);  //Temperature Sensor - 100111
   ADMUX |= (0 << MUX3);  //Temperature Sensor - 100111
   ADMUX |= (0 << MUX2);  //Temperature Sensor - 100111
   ADMUX |= (1 << MUX1);  //Temperature Sensor - 100111
   ADMUX |= (1 << MUX0);  //Temperature Sensor - 100111
   ADMUX = 0xE3;
   //ADC Control and Status Register A 
   ADCSRA = 0;
   ADCSRA |= (1 << ADEN);  //Enable the ADC
   ADCSRA |= (1 << ADPS2);  //ADC Prescaler - 16 (16MHz -> 1MHz)
   
   //ADC Control and Status Register B 
   ADCSRB = 0;
   ADCSRB |= (1 << MUX5);  //Temperature Sensor - 100111
}

int getTemp()
{
   ADMUX = 0;
   ADMUX |= (1 << REFS1);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
   ADMUX |= (1 << REFS0);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
   ADMUX |= (0 << MUX4);  //Temperature Sensor - 100111
   ADMUX |= (0 << MUX3);  //Temperature Sensor - 100111
   ADMUX |= (1 << MUX2);  //Temperature Sensor - 100111
   ADMUX |= (1 << MUX1);  //Temperature Sensor - 100111
   ADMUX |= (1 << MUX0);  //Temperature Sensor - 100111

   ADCSRA |= (1 << ADSC);  //Start temperature conversion
   while (bit_is_set(ADCSRA, ADSC));  //Wait for conversion to finish
   uint8_t low  = ADCL;
   uint8_t high = ADCH;
   int temperature = (high << 8) | low;  //Result is in kelvin
   return temperature - 273;
}



