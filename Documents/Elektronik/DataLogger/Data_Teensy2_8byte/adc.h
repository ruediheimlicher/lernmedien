/*
 *  adc.h
 *  TWI_Master
 *
 *  Created by Sysadmin on 12.11.07.
 *  Copyright 2007 Ruedi Heimlicher. All rights reserved.
 *
 */

// aus adc_simple PJRC

static uint8_t aref = (1<<REFS0)|(1<<REFS1); // default to AREF = Vcc

#define ADC_REF_POWER     (1<<REFS0)
#define ADC_REF_INTERNAL  ((1<<REFS1) | (1<<REFS0))
#define ADC_REF_EXTERNAL  (0)

// These prescaler values are for high speed mode, ADHSM = 1
#if F_CPU == 16000000L
#define ADC_PRESCALER ((1<<ADPS2) | (1<<ADPS1)) // f/64
#elif F_CPU == 8000000L
#define ADC_PRESCALER ((1<<ADPS2) | (1<<ADPS0)) // f/32
#elif F_CPU == 4000000L
#define ADC_PRESCALER ((1<<ADPS2))
#elif F_CPU == 2000000L
#define ADC_PRESCALER ((1<<ADPS1) | (1<<ADPS0))
#elif F_CPU == 1000000L
#define ADC_PRESCALER ((1<<ADPS1))
#else
#define ADC_PRESCALER ((1<<ADPS0))
#endif


// some avr-libc versions do not properly define ADHSM
#if defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#if !defined(ADHSM)
#define ADHSM (7)
#endif
#endif


extern struct adcwert16 ADCWert16;

//struct adcwert16 readKanal16Bit(uint8_t kanal);
void closeADC(void);
uint16_t readKanal(uint8_t derKanal);//Unsere Funktion zum ADC-Channel aus lesen

uint16_t readKanalOrig(uint8_t derKanal, uint8_t num); //Unsere Funktion zum ADC-Channel aus lesen
