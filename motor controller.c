/*************************************************************************************
 * PID_controller.c
 *
 * PID_controller.c is developped by group C7 for motor position control  
 *
 * PID_controller.c has been tested with commercial motor with the following performance:
 *      * 3% overshoot
 *      * 30ms raising time for 9 degree change
 *      * ~0 steady state error
 * 
 * PID_controller is designed for Arduino mega 2560
 *             ├── calculates PID values at 4Hkz (timer1)
 *             ├── updates setpoint at 25Hz (timer3).
 *             └── integrates the interface to quadrature decoders and motor drivers
 **************************************************************************************/

//define pins
#define LAZER_PIN 2
#define Dir1_PIN 4
#define Dir2_PIN 5
#define BUTTON_PIN 6
#define EN_PIN 10
#define SEL_PIN 8
#define reload 63780 //250us 4kHz increase to increase the frequency
#define interval 26000 //25hz

// PID coefficients
const float Kp = 0.65;
const float Ki = 0.21;
const float Kd = 0.14;

// PID variables
const float dt = 0.00025; //250us consistent with Timer1 reload freq
float previous_error = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float averagePulse = 0;

// moving average filter variable
int avgPulse[4];

// setpoint update variables
const int setPoint[4] = {0, 10, 20, 30};
const bool lazer[4] = {0, 1, 0, 1};
int desiredPos = 0;
int setPointIndex = 0;
bool lazerStatus = 0;

// control flags
volatile bool toChange = true;
bool readyToGo = false;


int fmap(float x, float in_min, float in_max, float out_min, float out_max);
int PID2PWM(float PID);

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
  TCCR3B = _BV(CS31);       // 1 prescaler
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt

  // configure timer2 for pwm frequency output
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2A = 0;                // PWM duty cycle register

  // set all pin mode
  pinMode(EN_PIN, OUTPUT);
  pinMode(Dir1_PIN, OUTPUT);
  pinMode(Dir2_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(SEL_PIN, OUTPUT);
  pinMode(LAZER_PIN, OUTPUT);

  // port A register, set pin 22-29 output
  PORTA = 0;
}

void loop() {

  // wait to start till button pressed
  if (digitalRead(BUTTON_PIN) && readyToGo == false) {
    readyToGo = true;
    interrupts();
  }

  // reset setPointIndex when it reaches 4
  if (setPointIndex == 4) {
    setPointIndex = 0;
  }

  // update setpoint and control lazer status
  desiredPos = setPoint[setPointIndex];
  lazerStatus = lazer[setPointIndex];
  digitalWrite(LAZER_PIN, lazerStatus);

  if (toChange && readyToGo) {
    static int pwmSignal = 0;
    static float PIDoutput = 0;

    // read data from quadrature decoder
    // PINA stores input from pin 22-29
    digitalWrite(SEL_PIN, LOW);
    signed int currentPos = PINA;
    currentPos = currentPos << 8;
    digitalWrite(SEL_PIN, HIGH);
    currentPos += PINA;

    // moving average filter of 4 stages
    avgPulse[3] = currentPos;
    averagePulse = (avgPulse[0] + avgPulse[1] + avgPulse[2] + avgPulse[3]) / 4.0;
    avgPulse[2] = avgPulse[3];
    avgPulse[1] = avgPulse[2];
    avgPulse[0] = avgPulse[1];

    // PID controller
    error = desiredPos - averagePulse;
    integral = integral + error * dt; //dt is the fixed time between every measurement 250us
    derivative = (error - previous_error) / dt;
    PIDoutput = Kp * error + Ki * integral + Kd * derivative; //output is the voltage going into motor
    previous_error = error;
    toChange = false;

    // map the PID controller result to PWM signal
    pwmSignal = PID2PWM(PIDoutput);

    // set PWM motor enable signal according to pwmSignal
    // OCR2A: reload register for timer 2
    OCR2A = pwmSignal;
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
  setPointIndex++;
}


//----------------------------------------------------------
// fmap function maps PID result to PWM signal (0-255)
//----------------------------------------------------------
int fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
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
    pwmSignal = fmap(PIDoutput, 0.0, 5.0, 15.0, 255.0);
  }
  else if (PIDoutput < 0 && PIDoutput > -5) {
    forward = false;
    pwmSignal = fmap(PIDoutput, -5.0, 0.0, 255.0, 15.0);
  }

  // set motor driver direction signal
  if (forward == false) {
    digitalWrite(Dir1_PIN, HIGH);
    digitalWrite(Dir2_PIN, LOW);
  }
  else {
    digitalWrite(Dir1_PIN, LOW);
    digitalWrite(Dir2_PIN, HIGH);
  }
  return pwmSignal;
}