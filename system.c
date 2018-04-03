/*************************************************************************************
 * PID Controller (Small)
 *
 * PID Controller.c is developped by group C7 for motor position control  

 * PID Controller is designed for Arduino mega 2560
 *             +-- calculates PID values at 4Hkz (timer1)
 *             +-- updates setpoint at 25Hz (timer3).
 *             +-- integrates the interface to quadrature decoders and motor drivers
 **************************************************************************************/

/*
* test procedures:
    Done - set the reset pin high for the decoder
    Done - test the effective pwm range
    Done - test the minimum execution time for one cycle: 5.525KHz
    - tune the pid value
*/
//define pins
#define LAZER_PIN 2
#define Dir1_PIN 4
#define Dir2_PIN 5
#define Dir1_PIN_TOP 11
#define Dir2_PIN_TOP 12
#define BUTTON_PIN 6
#define EN_PIN_TOP 9
#define EN_PIN 10
#define SEL_PIN 7
#define SEL_PIN_TOP 8
#define RST_RIGHT_PIN 52
#define RST_LEFT_PIN 53
#define reload 63100 //311us 3.213kHz increase to increase the frequency// measure again 
#define interval 64500 //65000 60Hz, 64500 30hz, 64000 20hz  CS32
#define SETPOINT_NUMBER 16

//0.4 0.135 0.09

// PID coefficients
const float Kp_top = 0.2;
const float Ki_top = 1; //
const float Kd_top = 0.009; // > 0.0036

const float Kp = 0.333;
const float Ki = 0.00667;// 0.0054
const float Kd = 0.5;// < 0.0062

// PID variables
const float dt = 0.000311; //250us consistent with Timer1 reload freq
const float dtInverse = 3215;
float previous_error = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float previous_error_top = 0;
float error_top = 0;
float integral_top = 0;
float derivative_top = 0;

// moving average filter variable
int avgPulse[4];
float averagePulse = 0;
int avgPulse_top[4];
float averagePulse_top = 0;

// setpoint update variables
int desiredPos_top = 0;
int desiredPos = 0;                                             //              ||          ||
//const int setPoint[SETPOINT_NUMBER] =     {-12,-12, -5,  5, 12, 12,  5, -5,-12,  5,  5,  7,  5,  5,  2, -2, -5};
//const int setPoint_top[SETPOINT_NUMBER] = { -5,  5, 12, 12,  5, -5,-12,-12, -5, -1, -1, -1,  4,  4,  4,  4,  4};
//const bool lazer[SETPOINT_NUMBER] =       {  0,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  1,  0,  0,  1,  0,  1};

const int setPoint[SETPOINT_NUMBER] =     {10, 9, 7, 4, 0,-4,-7, -9,-10,-9,-7,-4,0,4, 7, 9};
const int setPoint_top[SETPOINT_NUMBER] = {0, 4, 7, 9,10, 9, 7, 4, 0,-4,-7,-9,-10,-9,-7,-4};
const bool lazer[SETPOINT_NUMBER] =       {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1};
//const bool lazer[SETPOINT_NUMBER] =       {0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0,0};
//const int setPoint[SETPOINT_NUMBER] =     {15,15,10, 5, 0, 0, 5,10};
//const int setPoint_top[SETPOINT_NUMBER] = {10, 5, 0, 0, 5,10,15,15};
//const bool lazer[SETPOINT_NUMBER] =       { 0, 1, 1, 1, 1, 1, 0, 0};
int setPointIndex = 0;
bool lazerStatus = 0;

// control flags
volatile bool toChange = true;
bool readyToGo = false;
int i =0;
bool test = 0;

int PID2PWM(float PID);
int PID2PWM_TOP(float PID);

void setup() {
  // disable all interrupts during timer configurations
  noInterrupts();

  // configure timer1 to control PID calcultion frequency @ 4KHz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = reload;           // preload timer1, 250us 4kHz
  TCCR1B |= (CS11);         // 1 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer1 overflow interrupt

  // configure time3 for setpoint update
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = interval;         // preload timer3, 40ms 25Hz
  TCCR3B = _BV(CS32);       // 1 prescaler
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt

  // configure timer2 for pwm frequency output
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2A = 0;                // PWM duty cycle register

  // set all pin mode
  //Serial.begin(115200);
  pinMode(LAZER_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(EN_PIN_TOP, OUTPUT);
  pinMode(Dir1_PIN, OUTPUT);
  pinMode(Dir2_PIN, OUTPUT);
  pinMode(Dir1_PIN_TOP, OUTPUT);
  pinMode(Dir2_PIN_TOP, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(SEL_PIN, OUTPUT);
  pinMode(SEL_PIN_TOP, OUTPUT);
  pinMode(RST_RIGHT_PIN, OUTPUT);
  pinMode(RST_LEFT_PIN, OUTPUT);
  digitalWrite(RST_RIGHT_PIN, HIGH);
  digitalWrite(RST_LEFT_PIN, HIGH);
  //digitalWrite(Dir1_PIN, HIGH);
  //digitalWrite(Dir2_PIN, LOW);
  //PORTG = PORTG | 0b00100000;
  //PORTE = PORTE & 0b11110111;
  pinMode(43, OUTPUT);
  OCR2A = 0;
  OCR2B = 0; //top

  // port A register, set pin 22-29 input
  DDRA = 0;
  DDRC = 0;
  interrupts();
}

void loop() {

  // wait to start till button pressed
  if (digitalRead(BUTTON_PIN) && readyToGo == false) { // change
    readyToGo = true;
    setPointIndex = 0;
    interrupts();
  }

  if (setPointIndex == SETPOINT_NUMBER) {
    setPointIndex = 0;
  }
  
  desiredPos = setPoint[setPointIndex];
  desiredPos_top = (setPoint_top[setPointIndex]);
  lazerStatus = lazer[setPointIndex];
  digitalWrite(LAZER_PIN, lazerStatus);

  if (toChange && readyToGo) {
    noInterrupts();
    /*****************************
     * Bottom motor PID
     ****************************/
    static int pwmSignal = 0;
    static float PIDoutput = 0;
    
    // read data from quadrature decoder
    // PINA stores input from pin 22-29
    PORTH = PORTH | 0b00010000;   //digitalWrite(SEL_PIN, LOW);
    signed int currentPos = PINA << 8;
    PORTH = PORTH & 0b11101111;   //digitalWrite(SEL_PIN, HIGH);
    currentPos += PINA;
    currentPos = ((unsigned int)(currentPos & 0b0101010101010101) << 1) + \
                  ((unsigned int)(currentPos & 0b1010101010101010) >> 1);
    //Serial.println(currentPos);
    
    // moving average filter of 4 stages
    if(i == 4) i = 0;
    avgPulse[i] = currentPos;
    averagePulse = (avgPulse[0] + avgPulse[1] + avgPulse[2] + avgPulse[3]) / 4.0;

    // PID controller
    error = desiredPos - averagePulse;
    integral = integral + error * dt; //dt is the fixed time between every measurement 250us
    derivative = (error - previous_error) * dtInverse;
    PIDoutput = Kp * error + Ki * integral + Kd * derivative; //output is the voltage going into motor
    previous_error = error;

    // map the PID controller result to PWM signal
    pwmSignal = PID2PWM(PIDoutput);

    // set PWM motor enable signal according to pwmSignal
    // OCR2A: reload register for timer 2
    OCR2A = pwmSignal;
    
    /********************
     * top motor PID
     ********************/
    static int pwmSignal_top = 0;
    static float PIDoutput_top = 0;
    
    // read data from quadrature decoder
    // PINC stores input from pin 30-37
    PORTH = PORTH | 0b00100000;   //digitalWrite(SEL_PIN, LOW);
    signed int currentPos_top = PINC << 8;
    PORTH = PORTH & 0b11011111;   //digitalWrite(SEL_PIN, HIGH);
    currentPos_top += PINC;
    currentPos_top = ((unsigned int)(currentPos_top & 0b0101010101010101) << 1) + \
                  ((unsigned int)(currentPos_top & 0b1010101010101010) >> 1);
    
    // moving average filter of 4 stages
    avgPulse_top[i] = currentPos_top;
    averagePulse_top = (avgPulse_top[0] + avgPulse_top[1] + avgPulse_top[2] + avgPulse_top[3]) / 4.0;
    i++;

    // PID controller
    error_top = desiredPos_top - averagePulse_top;
    integral_top = integral_top + error_top * dt; //dt is the fixed time between every measurement 250us
    derivative_top = (error_top - previous_error_top) * dtInverse;
    PIDoutput_top = Kp_top * error_top + Ki_top * integral_top + Kd_top * derivative_top; //output is the voltage going into motor
    previous_error_top = error_top;

    // map the PID controller result to PWM signal
    pwmSignal_top = PID2PWM_TOP(PIDoutput_top);

    // set PWM motor enable signal according to pwmSignal
    // OCR2A: reload register for timer 2
    OCR2B = pwmSignal_top;
    toChange = false;
    interrupts();
  }
}

//-----------------------------------------------------------
// timer 1 compare interrupt service routine
// interruption occour every 250us (4kHz) for PID calculation
// toChange flag enables a new PID calculation
//-----------------------------------------------------------
ISR(TIMER1_OVF_vect)          
{
  TCNT1 = reload;
//  test = !test;
//  digitalWrite(43, test);
  toChange = 1;
}

//----------------------------------------------------------
// timer compare interrupt service routine
// interruption occour every 40ms (25Hz) for setpoint update
// setPointIndex is set back to 0 in the main function
//----------------------------------------------------------
ISR(TIMER3_OVF_vect)          
{
  TCNT3 = interval;
  //test = !test;
  //digitalWrite(43, test);
  setPointIndex++;
}


//-------------------------------------------------------------
// PID2PWM function process PID otuput with a (-5, 5) saturator
//                  it also set driver direction based on input
//-------------------------------------------------------------
int PID2PWM(float PIDoutput) {
  static bool forward = true;
  int pwmSignal = 0;

  // apply a saturator of 5V and determine the direction
  if (PIDoutput >= 5) {
    pwmSignal = 255;
    forward = true;
  }
  else if (PIDoutput <= -5) {
    pwmSignal = 255;
    forward = false;
  }
  else if (PIDoutput >= 0 && PIDoutput < 5) {
    forward = true;
    pwmSignal = (int)(PIDoutput*24 + 135);//fmap(PIDoutput, 0.0, 5.0, 145.0, 255.0);
  }
  else if (PIDoutput < 0 && PIDoutput > -5) {
    forward = false;
    pwmSignal = (int)(135 - PIDoutput*24);//fmap(PIDoutput, -5.0, 0.0, 255.0, 145.0);
  }

  // set motor driver direction signal
  if (forward == true) {
    PORTG = PORTG | 0b00100000;
    PORTE = PORTE & 0b11110111;
    //digitalWrite(Dir1_PIN, HIGH);
    //digitalWrite(Dir2_PIN, LOW);
  }
  else {
    PORTG = PORTG & 0b11011111;
    PORTE = PORTE | 0b00001000;
    //digitalWrite(Dir1_PIN, LOW);
    //digitalWrite(Dir2_PIN, HIGH);
  }
  return pwmSignal;
}

int PID2PWM_TOP(float PIDoutput) {
  static bool forward = true;
  int pwmSignal = 0;

  // apply a saturator of 5V and determine the direction
  if (PIDoutput >= 5) {
    pwmSignal = 255;
    forward = true;
  }
  else if (PIDoutput <= -5) {
    pwmSignal = 255;
    forward = false;
  }
  else if (PIDoutput >= 0 && PIDoutput < 5) {
    forward = true;
    pwmSignal = (int)(PIDoutput*25 + 130);//fmap(PIDoutput, 0.0, 5.0, 145.0, 255.0);
  }
  else if (PIDoutput < 0 && PIDoutput > -5) {
    forward = false;
    pwmSignal = (int)(130 - PIDoutput*25);//fmap(PIDoutput, -5.0, 0.0, 255.0, 145.0);
  }

    // set motor driver direction signal
    if (forward == true) {
      PORTB = PORTB | 0b00100000;
      PORTB = PORTB & 0b10111111;
      //digitalWrite(Dir1_PIN, HIGH);
      //digitalWrite(Dir2_PIN, LOW);
    }
    else {
      PORTB = PORTB | 0b01000000;
      PORTB = PORTB & 0b11011111;
    //digitalWrite(Dir1_PIN, LOW);
    //digitalWrite(Dir2_PIN, HIGH);
  }
  return pwmSignal;
}

//----------------------------------------------------------
// fmap function maps PID result to PWM signal (0-255)
//----------------------------------------------------------
/*int fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}*/