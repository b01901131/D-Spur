
#define BT_BAUD 38400

/*
 * PORTA: 29    28    27    26    25    24    23    22
 *        DIR1  STEP1 DIR2  STEP2 DIR3  STEP3 DIR4  STEP4
 *
 * PORTC: 30    31    32    33    34    35    36    37
 *        EN1   [     MS1    ]    EN2   [     MS2    ]
 *
 * PORTL: 42    43    44    45    46    47    48    49
 *        EN3   [     MS3    ]    EN4   [     MS4    ]
 *
 */
int i;
unsigned int loop_count = 0; 

int wait_us = 1000;
int target_wait_us = 1000;
byte dir_byte  = B00100010;  // set DIRx to LOW,HIGH,LOW,HIGH
volatile byte step_byte = B00000000;
int multipliers[4];

void setup() 
{
  Serial.begin(9600);
  
  dir_byte = B00100010;
  
  
  // set pin directions to OUTPUT.
  DDRA = B11111111;
  DDRC = B11111111;
  DDRL = B11111111;
  
  // init all DIRx,STEPx to LOW.
  PORTA = dir_byte | step_byte;
  
  // set EN1,EN2 to low. set MS1.MS2 to (HIGH,LOW,LOW)
  PORTC = B01110111;
  
  // set EN3,EN4 to low. set MS3.MS4 to (HIGH,LOW,LOW)
  PORTL = B01110111;
  
  // set timer interrupt of timer3 (16-bit)
  TCCR3A = 0x00;
  TCCR3B &= ~_BV(CS12);  // no prescaling. timer3 freq = 12 MHz.
  TCCR3B &= ~_BV(CS11);  
  TCCR3B |=  _BV(CS10);  
  TIMSK3 |= _BV(TOIE3);  // enable timer overflow interrupt
  TCNT3 = 65536 - 12 * wait_us;
  
  multipliers[0] = 1;
  multipliers[1] = 1;
  multipliers[2] = 1;
  multipliers[3] = 1;
  
}

void loop() 
{ 
}

ISR (TIMER3_OVF_vect) {
  loop_count++;
  PORTA = (PORTA & B01010101) | dir_byte;
  // Not in stand-by mode.
  if(target_wait_us != -1) {
    // ramp up/down wait_us
    if(wait_us != target_wait_us && (loop_count % 20 == 0)){
      wait_us += (target_wait_us > wait_us)? 10: -10;
    }
    if(loop_count == 2000) {
      target_wait_us = 2000;
    }
    if(loop_count == 3500) {
      //dir_byte = B00001010; // move LEFT
      dir_byte = B00000000; // rotate CCW
      target_wait_us = 1000;
    }
    // set output pin port register for STEPx
    step_byte = B00000000;
    for (i = 0; i < 4; i++){
      if(loop_count % multipliers[i] == 0)
        step_byte += ( 1 << (6-2*i) );
    }
    PORTA |= step_byte;
    delayMicroseconds(2);  
    PORTA &= B10101010;
  }
  else {
    // stand-by. set wait_us init val.
    wait_us = 5000;
  }
  // reset timer
  TCNT3 = 65536 - 12 * wait_us;
}

