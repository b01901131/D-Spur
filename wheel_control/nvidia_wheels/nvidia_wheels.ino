
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
 * TIMER3: 2    3     5
 *
 * SPI:   50    51    52    53
 *        MISO  MOSI  SCK   SS
 *
 */
int i;
unsigned int loop_count = 0; 

byte rcv_count;
byte rcv_buffer[21];

int wait_us = 1000;
int target_wait_us = -1;
byte dir_byte  = B00100010;  // set DIRx to LOW,HIGH,LOW,HIGH
volatile byte step_byte = B00000000;
int multipliers[4];
//unsigned long loop_counts[4];

union u_tag {
   byte b[4];
   int ival;
} u;

void setup() 
{
  Serial.begin(9600);
  //Serial3.begin(BT_BAUD);
  unsigned int baud_setting = (F_CPU / 16 / BT_BAUD) - 1;
  // Set the baud rate
  UBRR3H = (unsigned char)(baud_setting >> 8);
  UBRR3L = (unsigned char)baud_setting;
  // Enable receiver and transmitter, no interrupt
  UCSR3B |= _BV(RXEN3) | _BV(TXEN3);
  UCSR3B |= _BV(RXCIE3);
  UCSR3B &= ~_BV(TXCIE3);   
  // Set frame format: No parity check, 8 Data bits, 1 stop bit
  UCSR3C |= _BV(UCSZ31) | _BV(UCSZ30);

  
  // set pin directions to OUTPUT.
  DDRA = B11111111;
  DDRC = B11111111;
  DDRL = B11111111;
  
  // init all DIRx,STEPx to LOW.
  PORTA = dir_byte | step_byte;
  
  // set EN1,EN2 to low. set MS1.MS2 to (HIGH,HIGH,HIGH)
  PORTC = B01110111;
  
  // set EN3,EN4 to low. set MS3.MS4 to (HIGH,HIGH,HIGH)
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
  /*while(true){
    if(Serial3.read() == 1)
      break;
  }*/
  
  // beep to notify once wheels are ready for connection 
  //tone(8, 3000);
}

void loop() 
{ 
}

// Interrupt Service Routine for motor control.
ISR (TIMER3_OVF_vect) {
  loop_count++;
  // Not in stand-by mode.
  if(target_wait_us != -1) {
    // set output pin port register for DIRx
    PORTA = (PORTA & B01010101) | dir_byte;  
    // ramp up/down wait_us
    if(wait_us != target_wait_us && (loop_count % 20 == 0)){
      wait_us += (target_wait_us > wait_us)? 10: -10;
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
    wait_us = 1000;
  }
  // reset timer
  TCNT3 = 65536 - 12 * wait_us;
}

// Interrrupt Service Routine for receiving data from HC-05
ISR (USART3_RX_vect){
  rcv_buffer[rcv_count] = UDR3;
  rcv_count++;
  
  // transmission complete
  if(rcv_count == 21){
    target_wait_us = (rcv_buffer[0] << 24) + (rcv_buffer[1] << 16) +
                     (rcv_buffer[2] <<  8) +  rcv_buffer[3];
    dir_byte = rcv_buffer[4];
    for (i = 0; i< 4; i++){
      u.b[3] = rcv_buffer[5+4*i];
      u.b[2] = rcv_buffer[6+4*i];
      u.b[1] = rcv_buffer[7+4*i];
      u.b[0] = rcv_buffer[8+4*i];
      multipliers[i] = u.ival;
    }
    rcv_count = 0;
  }
}

