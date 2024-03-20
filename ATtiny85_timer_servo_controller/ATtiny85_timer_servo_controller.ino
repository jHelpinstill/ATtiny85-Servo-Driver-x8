#define DEBUG 0
//#define PIN1 1

#define SCLK 1
#define SER 2
#define LATCH 4

volatile uint8_t compare_value;
volatile uint8_t overflow_cntr;
volatile uint8_t max_overflows;
volatile uint8_t event_counter;
volatile uint8_t highest_event;

volatile bool waiting = false;
volatile bool double_compare_flag = false;

int servo[8]{};
int timerSequence[16]{};
uint8_t SR_state[16]{};
uint8_t servoOrder[8]{};

void runServos();
void wait();
void debugPrint(byte);
void shiftByte(byte); //19us
void latch(); //246ns

void setup() {
  DDRB = (1<<DEBUG) | (1<<SCLK) | (1<<SER) | (1<<LATCH);    //set pins to output
  PORTB = 0;   //set pins low
  TCCR1 = 4; //set clk/8 prescaler

  DDRB &= ~(1<<3); //set pin 3 as input for pot

  OSCCAL = 137; //tuning value for RC oscillator

  for(uint8_t i = 0; i < 8; i++){
    servo[i] = 700 + i * 200;
  }
}
int spd = 0;
char dir = 1;

void loop() {
  runServos();
  delay(20);
  int temp = analogRead(A3);
  servo[5] = 500 + temp*2;
  spd = temp / 3;
  servo[6] = servo[5] + temp /10 - 50;
//  servo[6] += spd * dir;
//  if(servo[6] > 2000){
//    servo[6] = 2000;
//    dir = -1;
//  }
//  else if(
//    servo[6] < 1000){
//      servo[6] = 1000;
//      dir = 1;
//    }
}

void runServos(){
  /// fill servoSorted with the servo signal lengths and fill servoOrder with the numbers 0 -> 7
  const byte minDelay = 50;
  int servoSorted[8]{};
  for(byte i = 0; i < 8; i++){
    servoSorted[i] = servo[i];
    servoOrder[i] = i;
  }

  /// sort servo pulse times from least to greatest and keep track of which servo is in which position ///
  for(uint8_t i = 0; i < 7; i++){ //bubble sort
    for(uint8_t j = 0; j < 7-i; j++){ 
      if(servoSorted[j] > servoSorted[j+1]){
        int temp;
        temp = servoSorted[j+1];
        servoSorted[j+1] = servoSorted[j];
        servoSorted[j] = temp;
        
        temp = servoOrder[j+1];
        servoOrder[j+1] = servoOrder[j];
        servoOrder[j] = temp;
      }
    }
  }

  uint8_t numHEs = 0; //HE: High events
  uint8_t HE[8]{};
  
  /// find LOWs which are too close together but not coincident///
  for(uint8_t i = 1; i < 8; i++){ //i starts as 1: don't need to check the first servo (0)
    byte difference = servoSorted[i] - servoSorted[i-1]; 
    if(difference < minDelay){
      if(difference == 0){
        continue;
      }
      else{
        HE[numHEs] = i;
        numHEs++;
      }
    }
  }
  
  
  /// find how many 50us waits need to happen for offset starts and fill appropriate bytes for shift reg, add time to appropriate servo pulse lengths
  for(uint8_t i = 0; i < numHEs; i++){
    timerSequence[i] = minDelay; //add a wait of minDelay us

    //remove servos which don't turn on until after this step
    byte SR_temp = ~0; //all on
    for(uint8_t j = 1; j < 8; j++){//servo 0 will always be on from the first step, no need to turn off here
      if(j >= HE[i]){
        SR_temp &= ~(1<<servoOrder[j]); //in this step we need the servo's port index, not its index in the sorted list
        servoSorted[j] += minDelay;// * (i+1); //add time to servo pulse length to account for delayed start
      }
    }
    SR_state[i] = SR_temp;
  }
  SR_state[numHEs] = ~0; //next SR state will always be 'all on'
  timerSequence[numHEs] = servoSorted[0] - minDelay*numHEs; //time to wait between end of delayed starts and first low
  SR_state[numHEs + 1] = ~0 & ~(1<<servoOrder[0]); //next next SR state will always be 'all on' except shortest signal
  ///note: there is always 1 more SR_state than timerSequence, this is the point where SR_state stops occuring before gaps and starts happening afterward

  /// find the gap lengths between pulse endings, and assign appropriate shift reg values ///
  uint8_t numCLs = 0;
  for(uint8_t i = 1; i < 8; i++){ //do the rest of the servos
    int difference = servoSorted[i] - servoSorted[i - 1]; //wait time is just difference between subsequent signal lengths
    if(difference == 0){
      SR_state[i + numHEs - numCLs] &= ~(1<<servoOrder[i]); //add a servoLow to the previous index (note the lack of ' + 1' in SR_state index field)
      numCLs++;
      continue; //skip to next i
    }
    timerSequence[i + numHEs - numCLs] = difference; //'-numCLs' because if there was a cl then 'i' will have increased without filling that index

    //turn off next servo
    SR_state[i + numHEs + 1 - numCLs] = SR_state[i + numHEs - numCLs] & ~(1<<servoOrder[i]); //this byte is the previous byte with the servo in question turned off
  }


  ///// TRANSMISSION /////
//  debugPrint(servo[5]>>8);
//  debugPrint(servo[5]);

  PORTB |= (1<<DEBUG); PORTB &= ~(1<<DEBUG);//pulse debug to show start of transmission
  
  
  TIMSK &= ~((1<<TOIE1) | (1<<OCIE1A)); //disable overflow and compare interrupts
  TIFR |= (1<<TOV1) | (1<<OCF1A); //clear overflow and compare interrupt flags
  
  shiftByte(SR_state[0]); //shift out initial shift reg state
  GTCCR |= (1<<PSR1);   //reset prescaler to zero
  TCNT1 = 0; //reset timer to zero -THIS IS OFFICIAL T0 OF TRANSMISSION-
  PORTB |= (1<<LATCH);
  PORTB &= ~(1<<LATCH);

//  event_counter = 0;
//  highest_event = 8 + numHEs - numCLs;

  for(uint8_t gap = 0; gap < (8 + numHEs - numCLs); gap++){ //repeat until transmission is finished
    cli();
    shiftByte(SR_state[gap+1]); //gap will never be larger than 14 (highest gap number is 15: 8 lows + 7 delayed starts), so gap+1 is safe to use without undefined behavior   
    max_overflows = timerSequence[gap] >> 8;
    overflow_cntr = 0;
    compare_value = timerSequence[gap];
    OCR1A = compare_value;
      
    if(OCR1A < 16 && max_overflows > 0){
      max_overflows--;
      OCR1A = (static_cast<uint16_t>(256) + OCR1A) / 2;
      compare_value = static_cast<uint16_t>(256) + compare_value - OCR1A; //"compare_value" still holds the original value here, two variables accounts for odd numbers
      double_compare_flag = true;
      debugPrint(OCR1A);
    }
    
    if(max_overflows > 0) TIMSK |= (1<<TOIE1); //enable overflow interrupts
    else TIMSK |= (1<<OCIE1A); //enable compare interrupts
  
    waiting = true;
    sei();
    
    while(waiting); //wait for interupts to complete and shift reg to latch
    
    TIMSK &= ~((1<<TOIE1) | (1<<OCIE1A)); //disable overflow and compare interrupts
    TIFR |= (1<<TOV1) | (1<<OCF1A); //clear overflow and compare interrupt flags
//    PORTB |= (1<<DEBUG);PORTB &= ~(1<<DEBUG); //pulse debug;
  }

//    for(int i = 0; i < 8; i++){
//      //debugPrint(SR_state[i] >> 8);
//      debugPrint(SR_state[i]);
//    }
//  }
}

ISR(TIMER1_OVF_vect){
  PORTB |= (1<<DEBUG); PORTB &= ~(1<<DEBUG);//pulse debug
  overflow_cntr++;
  if(overflow_cntr == max_overflows){
    TIFR |= (1<<OCF1A); //clear compare interrupt flag
    TIMSK |= (1<<OCIE1A); //enable compare interrupts
    TIMSK &= ~(1<<TOIE1); //disable overflow interrupts
    overflow_cntr = 0;
  }
}

ISR(TIMER1_COMPA_vect){ //run on timer1 compare match with compA
  //PORTB ^= (1<<DEBUG); PORTB ^= (1<<DEBUG);//pulse debug
  GTCCR |= (1<<PSR1);   //reset prescaler to zero
  TCNT1 = 0; //reset timer to zero
  
  if(!double_compare_flag){
    PORTB |= (1<<LATCH); //show next shift reg state to servos
    PORTB &= ~(1<<LATCH);

    waiting = false;
  }
  else{
    double_compare_flag = false;
    OCR1A = compare_value;
  }
  
}

void debugPrint(byte data){
  
  for(int i = 0; i < 8; i++){
    PORTB |= (1<<DEBUG);
    if(!((data << i) & (1<<7)))PORTB &= ~(1<<DEBUG);
    for(uint8_t i = 0; i < 20; i++){
      _NOP();
    }
    PORTB &= ~((1<<DEBUG) | (1<<PIN1));
    for(uint8_t i = 0; i < 5; i++){
      _NOP();
    }
  }
}

void shiftByte(byte data){
  uint8_t mask = (1<<7);
  while(mask > 0){
    if(data & mask) PORTB |= (1<<SER); // if there is a bit at this place value, set the serial pin high
    else PORTB &= ~(1<<SER);
    PORTB |= (1<<SCLK); //set shift clock pin high and then low. 
    PORTB &= ~(1<<SCLK);
    
    mask >>= 1;
  }
}

void latch(){
  PORTB |= (1<<LATCH); 
  PORTB &= ~(1<<LATCH);
}
