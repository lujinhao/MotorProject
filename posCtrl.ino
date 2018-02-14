#define ENCODER_PIN 2
#define ENCODERREF_PIN 3
#define BUTTON_PIN 6
#define EN_PIN 39
#define Dir1_PIN 4
#define Dir2_PIN 5


const int desiredPos1 = 200; //pulses
//const int desiredPos2 = 30; //pulses

const float Kp = ???;
const float Ki = ???;
const float Kd = ???;

const char TAB = 9;


//interrupt variables:
int currentPos = 0;
int currentTime = 0;
int previousTime = 0;
bool toChange = true;
bool direction = 1; //1 for forward 0 for backward;
bool reference = 1;
bool readyToGo = false;


//PID variables:
bool forward = true;
float PIDVolOut = 0;
int pwmSignal = 0;
int previous_error = 0;
int error = 0;
float integral = 0;
float derivative = 0;
float dt = 0;

void feedback (void);
void feedbackRef (void);
int fmap(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(Dir1_PIN, OUTPUT);
  pinMode(Dir2_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
	//digitalWrite(Dir1_PIN, HIGH);
	//digitalWrite(Dir2_PIN, LOW);
  // attach feedback interrupt to pin 2 and call feedback() on interrupt
  attachInterrupt(ENCODER_PIN, feedback, CHANGE);
  attachInterrupt(ENCODERREF_PIN, feedbackRef, FALLING);
  Serial.begin(115200);
}

void loop() {
  
  if(digitalRead(BUTTON_PIN) && readyToGo == false){
    readyToGo = true;
    previousTime = micros();
  }
  
  if(toChange && readyToGo){
    error = desiredPos1 - currentPos; //measured value is readings after encoder.(Reading the number of HIGH. Assume 1 dgr = 1 HIGH)
    integral = integral + error*dt;  //dt is the time between every measurement
    derivative = (error - previous_error)/dt;
    PIDVolOut = Kp*error + Ki*integral + Kd*derivative; //output is the voltage going into motor
    previous_error = error;
    toChange = false;
/*
    Serial.print(PIDVolOut);
    Serial.print(TAB);
    Serial.print(Kp*error);
    Serial.print(TAB);
    Serial.print(Ki*integral);
    Serial.print(TAB);
    Serial.println(Kd*derivative);
*/
    Serial.print(currentTime);
    Serial.print(TAB);
    Serial.println(currentPos);

    if(PIDVolOut >= 5){
      	pwmSignal = 0;
      	forward = true;
    }
    else if(PIDVolOut <= -5){
      	pwmSignal = 0;
      	forward = false;
    }
  	else if(PIDVolOut >= 0 && PIDVolOut < 5){
    		forward = true;
    		pwmSignal = fmap(PIDVolOut, 0.0, 5.0, 93.0, 0.0);
  	} 
  	else if (PIDVolOut < 0 && PIDVolOut > -5){
    		forward = false;
    		pwmSignal = fmap(PIDVolOut, -5.0, 0.0, 0.0, 93.0);
  	}

  	if(forward == false){
  		digitalWrite(Dir1_PIN, HIGH);
  		digitalWrite(Dir2_PIN, LOW);
  	}
  	else {
  		digitalWrite(Dir1_PIN, LOW);
  		digitalWrite(Dir2_PIN, HIGH);
  	}

    PMC->PMC_PCER1 |= PMC_PCER1_PID36;                   // PWM on
    REG_PIOC_ABSR |= PIO_ABSR_P7;                        // Set PWM pin perhipheral type B
    REG_PIOC_PDR |= PIO_PDR_P7;                          // Set PWM pin to an output
    REG_PWM_ENA = PWM_ENA_CHID2;                         // Enable the PWM channel 2 (see datasheet page 973) 
    REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);    // Set the PWM clock rate to 2MHz (84MHz/42). Adjust DIVA for the resolution you are looking for
    REG_PWM_CMR2 = PWM_CMR_CALG |PWM_CMR_CPRE_CLKA;      // The period is left aligned, clock source as CLKA on channel 2
    REG_PWM_CPRD2 = 100;    //10kHz                         // Channel 2 : Set the PWM frequency 2MHz/(2 * CPRD) = F ; 1< CPRD < 2exp24  -1 
    REG_PWM_CDTY2 = pwmSignal; 
    
  }
}

void feedback (void){
	direction = reference;
	if(direction)
		currentPos++;
	else 
		currentPos--;
	toChange = true;
	currentTime = micros();
	dt = (currentTime - previousTime)/1000000.0;
	previousTime = currentTime;
}

void feedbackRef (void){
	if(digitalRead(ENCODER_PIN) == LOW)
		reference = 1;
	else 
		reference = 0;
	//direction = reference;
}

int fmap(float x, float in_min, float in_max, float out_min, float out_max){
 	return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}