//==============================================================================
//
//
//  CRYSTALFONTZ CFAH0802Z 8x2 Character Display
//
//  This code uses the 8-bit parallel MCU mode of the display.
//
//  https://www.crystalfontz.com/family/cfah0802z
//
//  The controller is a Sitronix ST7066U:
//    https://www.crystalfontz.com/controllers/Sitronix/ST7066U
//
//  Seeeduino v4.2, an open-source 3.3v capable Arduino clone.
//    https://www.seeedstudio.com/Seeeduino-V4.2-p-2517.html
//    https://github.com/SeeedDocument/SeeeduinoV4/raw/master/resources/Seeeduino_v4.2_sch.pdf
//==============================================================================
//
//  2018-08-22 Max Roberg / Crystalfontz
//
//===========================================================================
//This is free and unencumbered software released into the public domain.
//
//Anyone is free to copy, modify, publish, use, compile, sell, or
//distribute this software, either in source code form or as a compiled
//binary, for any purpose, commercial or non-commercial, and by any
//means.
//
//In jurisdictions that recognize copyright laws, the author or authors
//of this software dedicate any and all copyright interest in the
//software to the public domain. We make this dedication for the benefit
//of the public at large and to the detriment of our heirs and
//successors. We intend this dedication to be an overt act of
//relinquishment in perpetuity of all present and future rights to this
//software under copyright law.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
//OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//OTHER DEALINGS IN THE SOFTWARE.
//
//For more information, please refer to <http://unlicense.org/>
//==============================================================================
#include <Arduino.h>
// Our dev board, an Arduino UNO R3 runs at 16MHz,
// and this define makes the delays come out correctly.
#define F_CPU 16000000UL
//==============================================================================
// LCD & USD control lines
//  ----------+-------+-------------+-------------------------+-----------------
// 	ARD       |	Port	|	CFAH0802Z   |	Function                | Wire
//  ----------+-------+-------------+-------------------------+-----------------
//  GND	      |       | #1          | GROUND                  | Black
//  5V	      |       | #2          | POWER 5V                | Red
//  N/A	      |       | #3          | Vo                      | Orange
//  ----------+-------+-------------+-------------------------+-----------------
//  #8/D8     | PB0   | #4          | Data/Instruction  (RS)  | Purple
//  #10/D10   | PB2   | #5          | Read/Write Select (RW)  | Grey
//  #9/D9     | PB1   | #6          | Read/Write Enable (E)   | Yellow
//  ----------+-------+-------------+-------------------------+-----------------
//  #0/D0     | PD0   | #7          | LCD_D10 (DB0)           | Black
//  #1/D1     | PD1   | #8          | LCD_D11 (DB1)           | Brown
//  #2/D2     | PD2   | #9          | LCD_D12 (DB2)           | Red
//  #3/D3     | PD3   | #10         | LCD_D13 (DB3)           | Orange
//  #4/D4     | PD4   | #11         | LCD_D14 (DB4)           | Yellow
//  #5/D5     | PD5   | #12         | LCD_D15 (DB5)           | Green
//  #6/D6     | PD6   | #13         | LCD_D16 (DB6)           | Blue
//  #7/D7     | PD7   | #14         | LCD_D17 (DB7)           | Purple
//  ----------+-------+-------------+-------------------------+-----------------
//  N/A       |       | #15         | A (LED+)                | Green
//  N/A	      |       | #16         | K (LED-)                | Black
//  ----------+-------+-------------+-------------------------+-----------------
//==============================================================================
#define LCD_CTRL  (PORTB)
// PB0 (0x01) is RS (output)  Display pin 4
#define RS_MASK (0x01)
#define CLR_RS     (LCD_CTRL &= ~(RS_MASK)) //pin #8  - Data/Instruction
#define SET_RS     (LCD_CTRL |=  (RS_MASK)) //pin #8  - Data/Instruction
// PB1 (0x02) is E (output)  Display pin 6
#define E_MASK (0x02)
#define CLR_E      (LCD_CTRL &= ~(E_MASK)) //pin #9  - Chip Enable Signal
#define SET_E      (LCD_CTRL |=  (E_MASK)) //pin #9  - Chip Enable Signal
// PB2 (0x04) is RW (output)  Display pin 5
#define RW_MASK (0x04)
#define CLR_RW	   (LCD_CTRL &= ~(RW_MASK)) //pin #10 - Read/Write
#define SET_RW	   (LCD_CTRL |=  (RW_MASK)) //pin #10 - Read/Write

#define LCD_DATA   (PORTD)
//==============================================================================
#define ST7066_ClearDisplay (0x01)
#define ST7066_ReturnHome (0x02)
#define ST7066_EntryModeSet(ID, S) (0x04 | (ID << 1) | (S << 0))
#define ST7066_DisplayOnOffControl(D, C, B) (0x08 | (D << 2) | (C << 1) | (B << 0))
#define ST7066_CursorOrDisplayShift(SC, RL) (0x10 | (SC << 3) | (RL << 2))
#define ST7066_FunctionSet(DL, N, F) (0x20 | (DL << 4) | (N << 3) | (F << 2))
//==============================================================================
void sendCommand(uint8_t command)
{
  checkBusyAndAddress();

  //Put the command on the port
  LCD_DATA = command;
  
  // Select the LCD's command register
  CLR_RS;

  CLR_RW;

  // Deselect the LCD controller
  SET_E;
  CLR_E;
}
//==============================================================================
void sendData(uint8_t data)
{
  checkBusyAndAddress();
  //Put the data on the port
  LCD_DATA = data;

  // Select the LCD's command register
  SET_RS;

  CLR_RW;

  // Deselect the LCD controller
  SET_E;
  CLR_E;
  
}
//==============================================================================
uint8_t checkBusyAndAddress()
{
  PORTD = 0x80;
  DDRD = 0x00;
  
  CLR_RS;
  SET_RW;

  // Select the LCD controller
  SET_E;
  //Watch Pin 7 for the busy flag
  while (0x80 == (PIND & 0x80))
  {
    delayMicroseconds(80);
    CLR_E;
    delayMicroseconds(80);
    SET_E;
  }
  CLR_E;
  CLR_RW;

  DDRD = 0xFF;

  return LCD_DATA;
}
//==============================================================================
void init_0802z()
{

  //The following block is the power on sequence. Each time the module is power
  //  cycled, this is the code that is initiated internally.
  
  //This is for reference only, it is not necessary unless you are changing
  //  modes without power cycling the controller
  //----------------------------------------------------------------------------
  //Start Power On Sequence
  //Display clear
  sendCommand(ST7066_ClearDisplay);

  //Cursor / Display Shift Instruction (2 parts)
  sendCommand(ST7066_CursorOrDisplayShift(0, 1));
    //	00010100
    //	0001XX00
    //	  	||----  Shift Function
    //              00: Shifts cursor position to the left
    //                (AC is decremented by 1)
    //            >>01: Shifts cursor position to the right
    //                (AC is incremented by 1)
    //              10: Shifts entire display to the left
    //                The cursor follows the display shift
    //              11: Shifts entire display to the right
    //                The cursor follows the display shift
  //sendCommand(0x17);
    //	00010111
    //	0001XX11
    //	  	||----	Enable / Disable Internal Power
    //	  	|		  0: Internal Power OFF
    //	  	|	 	>>1: Internal Power ON
    //	  	|-----	Graphic Mode / Character Mode Selection
    //	  			>>0: Character Mode
    //	  			  1: Graphic Mode
  delay(500);

  checkBusyAndAddress();
  //Function Set
  sendCommand(ST7066_FunctionSet(1, 1, 0));	//00111000
    //	001XXX00
    //	   |||-----	Character Font Set
    //	   ||		>>0: 5x8 dot character font
    //	   ||		  1: 5x10 dot character font
    //	   ||------	Display line number control bit	
    //	   |		  0: 1-line display mode is set
    //	   |		>>1: 2-line display mode is set
    //	   |-------	Interface data length control bit
    //				  0: 4-bit bus mode with MPU
    //				>>1: 8-bit bus mode with MPU

  //Display ON/OFF Control
  sendCommand(ST7066_DisplayOnOffControl(1,0,0));	//0001100
    //	00001XXX
    //  		 |||--	Blinking Control Bit
    //  		 ||		  1: Cursor Blinking ON
    //  		 ||		>>0: Cursor Blinking OFF
    //  		 ||---	Cursor Display Control
    //  		 |		>>0: Cursor OFF
    //  		 |		  1: Cursor ON
    //  		 |----	Display ON/OFF
    //  		  		  0: Display OFF
    //  		  		>>1: Display ON

  //Display Clear 00000001
  sendCommand(ST7066_ClearDisplay);

  //Return Home 00000010
  sendCommand(ST7066_ReturnHome);

  //sendCommand(0x17);
  //Graphic vs character mode setting, RS=0,R/W=0
  // 7654 3210
  // 0001 GP11
  //  G = Mode: 1=graphic, 0=character
  //  C = Power: 1=0n, 0=off

  //Entry Mode Set
  sendCommand(ST7066_EntryModeSet(1,0)); //00000110
    //	000001XX
    //	  	  ||--	Shift Entire Display Control Bit
    //	  	  |   >>0: Decrement DDRAM Address by 1 when a character
    //	  	  |       code is written into or read from DDRAM
    //	  	  |     1: Increment DDRAM Address by 1 when a character
    //	  	  |       code is written into or read from DDRAM
    //	  	  |
    //	  	  |---	Increment/Decrement bit
    //	  	  	    0: when writing to DDRAM, each
    //	  	  		    entry moves the cursor to the left
    //	  	  	  >>1: when writing to DDRAM, each 
    //	  	  		    entry moves the cursor to the right

  //set DDRAM Address
  sendCommand(0x80);
}
//==============================================================================
void writeString(char* myString)
{
  uint8_t i = 0;
  do
  {
    sendData((uint8_t)myString[i]);
    i++;
  } while (myString[i] != NULL);
}
//============================================================================
void moveCursor(uint8_t y, uint8_t x)
{
  uint8_t cursorPosition = 0x80;
  switch (y)
  {
    case 0:
    {
      cursorPosition += 0x00;
      break;
    }
    case 1:
    {
      cursorPosition += 0x40;
      break;
    }
  }
  sendCommand(cursorPosition + x);
}
//==============================================================================
void setup()
{
  DDRD = 0xFF;
  PORTD = 0x00;

  DDRB = 0x17;
  PORTB = 0x00;
  delay(500);

  CLR_RW;
  init_0802z();
  delay(10);
}
//==============================================================================
void loop()
{
  //clear screen
  sendCommand(ST7066_ClearDisplay);
  //set DDRAM address to first row first column
  moveCursor(0,0);
  writeString("CFAH0802");
  //set DDRAM address to second row first column
  moveCursor(1,0);
  writeString("Z SERIES");
  delay(5000);

  moveCursor(0,0);
  writeString("01234567");
  moveCursor(1,0);
  writeString("ABCDEFGH");
  delay(5000);
}
