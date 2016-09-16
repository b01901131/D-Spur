const int gndPin0 = 23;
const int trigPin0 = 22;
const int echoPin0 = 21;

volatile bool echoPin0_state = false;

volatile unsigned long tStart0;

volatile unsigned long dist;

volatile unsigned long timer_count = 0;
volatile unsigned long step_count = 0;
volatile unsigned long curr_count = 0;
volatile unsigned long wait_us = 4000;
volatile int data_idx=0;
volatile int dist_array[400];

volatile int angle = 0;
volatile byte motor_byte = B01000100;
bool dir = false; // 0: CCW, 1:CW
volatile bool en  = true;
volatile bool transmit  = false;

union u_tag {
   byte b[4];
   int ival;
} u;

/* stepper motor: 30    31    32    33    34    35    36    37
 *                EN1   [     MS1    ]    DIR   STEP 
 *
 * timer4: 6 7 8
 *
 * ultrasonic: 21     22     23
 *             echo0  trig0  gnd0  
 *
 */

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  pinMode(trigPin0, OUTPUT);
  pinMode(gndPin0, OUTPUT);
  digitalWrite(gndPin0, LOW);
  pinMode(echoPin0, INPUT);
 
  DDRC = B11111100;
  PORTC = motor_byte;
 
  // set timer interrupt of timer4 (16-bit)
  TCCR4A = 0x00;
  TCCR4B &= ~_BV(CS12);  // no prescaling. timer4 freq = 12 MHz.
  TCCR4B &= ~_BV(CS11);  
  TCCR4B |=  _BV(CS10);  
  TIMSK4 |= _BV(TOIE4);  // enable timer overflow interrupt
  TCNT4 = 65536 - 12 * wait_us;
  
  //dist_array[0] = 5;
  //while(!Serial);
}

void loop() {
  //delay(1000);
  if(echoPin0_state){
    dist = pulseIn(echoPin0, HIGH, 6000) / 58;
    data_idx = (dir)? 400-curr_count: curr_count;
    if(dist != 0)
      dist_array[data_idx] = dist;
    
    /*
    Serial.print(step_count);
    Serial.print(" ");
    Serial.print(curr_count);
    Serial.print(" ");
    Serial.println(dist);
    */
    
    digitalWrite(13, LOW);
    echoPin0_state = false;
    en = true;
  }
  
  if(transmit){
    //dist_array[0] = 5;
    for(int x = 0; x < 400; x++){
      // transmit byte sequences.
      Serial.write((char*) (dist_array+x), 2);
    }
    transmit = false;
  }
}

void serialEvent() {
  char inChar = (char)Serial.read();
  transmit = true;
}

ISR (TIMER4_OVF_vect) {
  timer_count++;
  if(timer_count % 1 == 0){
    // control motor.
    PORTC = motor_byte;
    delayMicroseconds(2);  
    PORTC &= B11111011;
    step_count++;
    
    // handle direction change.
    if(step_count == (400+0)){
      if(!dir){
        motor_byte = B01001100;
        PORTC = (motor_byte & B11111011);
        dir = true;
      } else {
        motor_byte = B01000100;
        PORTC = (motor_byte & B11111011);
        dir = false;
      }
      step_count = 0;
    }
    
    // trigger HC-SR04
    //if(en && (step_count < 100) && (step_count > 137)){
    if(en){
      curr_count = step_count;
      
      digitalWrite(trigPin0, LOW);
      delayMicroseconds(4);
      digitalWrite(trigPin0, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin0, LOW);
        
      //digitalWrite(13, LOW);
      echoPin0_state = true;
      en = false;
    } 
  }   
  
  // reset timer
  TCNT4 = 65536 - 12 * wait_us;
}
