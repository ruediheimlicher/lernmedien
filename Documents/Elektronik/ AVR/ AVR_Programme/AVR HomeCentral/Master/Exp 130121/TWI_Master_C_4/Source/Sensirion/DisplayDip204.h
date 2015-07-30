#ifndef DISPLAYDIP204_H
#define DISPLAYDIP204_H
//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  DisplayDip204.h
// Author    :  SWE
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  Display Interface (DIP204)
//==============================================================================

//---------- Includes ----------------------------------------------------------
#include "system.h"

//---------- Defines -----------------------------------------------------------
// Define used pins
#define DISPLAY_RS_PIN             P9H_bit.no2
#define DISPLAY_RS_PIN_MODE        PM9H_bit.no2

#define DISPLAY_nRES_PIN           P9H_bit.no3
#define DISPLAY_nRES_PIN_MODE      PM9H_bit.no3

#define DISPLAY_RW_PIN             P9H_bit.no1
#define DISPLAY_RW_PIN_MODE        PM9H_bit.no1

#define DISPLAY_E_PIN              P9H_bit.no0
#define DISPLAY_E_PIN_MODE         PM9H_bit.no0

#define DISPLAY_BACKLIGHT_PIN      P3L_bit.no3
#define DISPLAY_BACKLIGHT_PIN_MODE PM3L_bit.no3

// Define used port
#define DISPLAY_PORT               P9L
#define DISPLAY_PORT_MODE          PM9L

//==============================================================================
void DisplayInit(void);
//==============================================================================
// Initializes display.

//==============================================================================
void DisplayEnableBacklight(void);
//==============================================================================
// Backlight enable

//==============================================================================
void DisplayDisableBacklight(void);
//==============================================================================
// Backlight disable

//==============================================================================
void DisplayWriteString(unsigned char aLine, unsigned char aColumn, char* aStringToWrite);
//==============================================================================
// Writes a character string to LCD on a specific position. If the string size
// exeeds the character size on the display, the surplus character are cut.
// input : aLine          line number [0...3]
//         aRow           start position of string in line [0...19]
//         aCharToWrite   output string
// output: -
// return: -

//==============================================================================
void DisplayWriteChar(unsigned char aLine, unsigned char aColumn, char aCharToWrite);
//==============================================================================
// Writes one character to LCD on a specific position.
// input : aLine          line number [0...3]
//         aRow           position of printing character line [0...19]
//         aCharToWrite   output character
// output: -
// return: -

//==============================================================================
void DisplayClear(void);
//==============================================================================
// Clears Display

//==============================================================================
void DisplayPrivateWriteByte(unsigned char aRs, unsigned char aCode);
//==============================================================================
// Internal function for writing a byte

//==============================================================================
void DisplayPrivateSetCursor(unsigned char aLine, unsigned char aColumn);
//==============================================================================
//Internal function to set the cursor

//==============================================================================
char DisplayPrivateConvertChar(char aChar);
//==============================================================================
// Internal function to convert a character (ASCII -> Displaycode)

#endif
