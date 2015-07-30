//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  DisplayDip204.c
// Author    :  SWE
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  Display Interface (DIP204)
//==============================================================================

//---------- Includes ----------------------------------------------------------
#include "DisplayDip204.h"

//==============================================================================
void DisplayInit(void)
//==============================================================================
{
  // Defines port direction
  DISPLAY_RS_PIN_MODE        = 0;     // output
  DISPLAY_nRES_PIN_MODE      = 0;     // output
  DISPLAY_RW_PIN_MODE        = 0;     // output
  DISPLAY_E_PIN_MODE         = 0;     // output
  DISPLAY_PORT_MODE          = 0x00;  // output
  DISPLAY_BACKLIGHT_PIN_MODE = 0;     // output

  // Defines initial port state
  DISPLAY_RW_PIN   = 1;
  DISPLAY_E_PIN    = 1;
  DISPLAY_nRES_PIN = 1;      // reset inactive
  DISPLAY_BACKLIGHT_PIN = 0; // backlight off

  // Power up time display
  DelayMicroSeconds(10000);

  // Initialize display
  DisplayPrivateWriteByte(0, 0x34);  // 8 bit data length, extension bit RE = 1
  DelayMicroSeconds(50);
  DisplayPrivateWriteByte(0, 0x09);  // 4 line mode
  DelayMicroSeconds(50);
  DisplayPrivateWriteByte(0, 0x30);  // 8 bit data length, extension bit RE = 0
  DelayMicroSeconds(50);
  DisplayPrivateWriteByte(0, 0x0C);  // display on, cursor off, blink off
  DelayMicroSeconds(50);
  DisplayPrivateWriteByte(0, 0x01);  // clear display, cursor 1st. row, 1st. line
  DelayMicroSeconds(2000);
  DisplayPrivateWriteByte(0, 0x06);  // cursor will be automatically incremented
  DelayMicroSeconds(50);
}

//==============================================================================
void DisplayEnableBacklight(void)
//==============================================================================
{
  DISPLAY_BACKLIGHT_PIN = 1;
}

//==============================================================================
void DisplayDisableBacklight(void)
//==============================================================================
{
  DISPLAY_BACKLIGHT_PIN = 0;
}
//==============================================================================
void DisplayWriteString(unsigned char aLine, unsigned char aColumn, char* aStringToWrite)
//==============================================================================
{
  // set cursor
  DisplayPrivateSetCursor(aLine, aColumn);
  // write character
  for(int i=aColumn; i<20; i++) // start at given position in line (row)
  {
    if(aStringToWrite[i-aColumn] == 0x00) break; // if NUL character -> exit
    else
    {
      DisplayPrivateWriteByte(1, DisplayPrivateConvertChar(aStringToWrite[i-aColumn]));
      DelayMicroSeconds(50);
    }
  }
}

//==============================================================================
void DisplayWriteChar(unsigned char aLine, unsigned char aColumn, char aCharToWrite)
//==============================================================================
{
  // set cursor
  DisplayPrivateSetCursor(aLine, aColumn);
  // write character
  DisplayPrivateWriteByte(1, DisplayPrivateConvertChar(aCharToWrite));
  DelayMicroSeconds(50);
}

//==============================================================================
void DisplayClear(void)
//==============================================================================
{
  DisplayPrivateWriteByte(0, 0x01);  // clear display, cursor 1st. row, 1st. line
  DelayMicroSeconds(2000);
}

//==============================================================================
void DisplayPrivateWriteByte(unsigned char aRs, unsigned char aCode)
//==============================================================================
{
  // set Register Select (RS)
  DISPLAY_RS_PIN = aRs;
  // set R/W  to write
  DISPLAY_RW_PIN = 0;
  // set data on bus
  DISPLAY_PORT = aCode;
  // enable to low
  DISPLAY_E_PIN = 0;
  // wait minimal low time
  DelayMicroSeconds(1);
  // enable to high
  DISPLAY_E_PIN = 1;
}

//==============================================================================
void DisplayPrivateSetCursor(unsigned char aLine, unsigned char aColumn)
//==============================================================================
{
  // Line number must be between 0..3
  assert(aLine < 4);
  // Row number must be between 0..19
  assert(aColumn < 20);

  DisplayPrivateWriteByte(0, 0x80 | (aLine * 0x20 + aColumn)); // Set DDRAM Adr.
  DelayMicroSeconds(2000);
}

//==============================================================================
char DisplayPrivateConvertChar(char aChar)
//==============================================================================
{
  return aChar;
}