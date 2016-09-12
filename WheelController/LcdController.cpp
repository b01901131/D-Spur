//YWROBOT
//last updated on 21/12/2011
//Tim Starling Fix the reset bug (Thanks Tim)
//wiki doc http://www.dfrobot.com/wiki/index.php?title=I2C/TWI_LCD1602_Module_(SKU:_DFR0063)
//Support Forum: http://www.dfrobot.com/forum/
//Compatible with the Arduino IDE 1.0
//Library version:1.1


#include "LcdController.h"
#include <inttypes.h>

#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
inline void LcdController::write(uint8_t value) {
  send(value, Rs);
}

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

LcdController::LcdController( I2CBusController* i2c_bus_controller,
                              uint8_t sl_addr,
                              uint8_t lcd_cols, 
                              uint8_t lcd_rows) : _sl_addr(sl_addr)
{
  _i2c_bus_controller = i2c_bus_controller;
  
  _cols = lcd_cols;
  _rows = lcd_rows;
  _backlightval = LCD_NOBACKLIGHT;
}

void LcdController::init(){
  init_priv();
}

void LcdController::init_priv()
{
  _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  begin(_cols, _rows);  
}

void LcdController::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  usleep(50000);
  
  // Now we pull both RS and R/W low to begin commands
  expanderWrite(_backlightval); // reset expanderand turn backlight off (Bit 8 =1)
  usleep(1000000);

    //put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46
  
    // we start in 8bit mode, try to set 4 bit mode
   write4bits(0x03 << 4);
   usleep(4500); // wait min 4.1ms

   // second try
   write4bits(0x03 << 4);
   usleep(4500); // wait min 4.1ms

   // third go!
   write4bits(0x03 << 4); 
   usleep(150); 
   
   // finally, set to 4-bit interface
   write4bits(0x02 << 4); 


  // set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);  
  
  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  display();
  
  // clear it off
  clear();
  
  // Initialize to default text direction (for roman languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  
  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);
  
  home();
}

/********** high level commands, for the user! */
void LcdController::clear(){
  command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero  
  usleep(2000);             // this command takes a long time!
}

void LcdController::home(){
  command(LCD_RETURNHOME);  // set cursor position to zero
  usleep(2000);             // this command takes a long time!
}

void LcdController::setCursor(uint8_t col, uint8_t row){
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row > _numlines ) {
    row = _numlines-1;    // we count rows starting w/0
  }
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/*void LcdController::set_device_handle(int dh){
  _device_handle = dh;
}*/

void LcdController::print(const char* buffer, uint8_t len){
  for(uint8_t i = 0; i < len; i++){
    send(*(uint8_t*)(buffer+i), Rs);
  }
}

// Turn the display on/off (quickly)
void LcdController::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LcdController::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LcdController::noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LcdController::cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LcdController::noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LcdController::blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LcdController::scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LcdController::scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LcdController::leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LcdController::rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LcdController::autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LcdController::noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LcdController::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}

// Turn the (optional) backlight off/on
void LcdController::noBacklight(void) {
  _backlightval=LCD_NOBACKLIGHT;
  expanderWrite(0);
}

void LcdController::backlight(void) {
  _backlightval=LCD_BACKLIGHT;
  expanderWrite(0);
}

/*********** mid level commands, for sending data/cmds */

inline void LcdController::command(uint8_t value) {
  send(value, 0);
}


/************ low level data pushing commands **********/

// write either command or data
void LcdController::send(uint8_t value, uint8_t mode) {
  uint8_t highnib=value&0xf0;
  uint8_t lownib=(value<<4)&0xf0;
       write4bits((highnib)|mode);
  write4bits((lownib)|mode); 
}

void LcdController::write4bits(uint8_t value) {
  expanderWrite(value);
  pulseEnable(value);
}

void LcdController::expanderWrite(uint8_t _data){   
  // TODO                                     
  //i2c_smbus_write_byte(_device_handle, (_data | _backlightval));
  _i2c_bus_controller->sendByte(_sl_addr, (_data | _backlightval));
}

void LcdController::pulseEnable(uint8_t _data){
  expanderWrite(_data | En);  // En high
  usleep(1);                  // enable pulse must be >450ns
  
  expanderWrite(_data & ~En); // En low
  usleep(50);                 // commands need > 37us to settle
} 
