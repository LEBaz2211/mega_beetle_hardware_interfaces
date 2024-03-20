#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Adafruit_MotorShield.h>
#include <PinChangeInterrupt.h>

#define ENCA 8 // RR
#define ENCB 11 // RL
#define ENCC 10 // FL
#define ENCD 9  // FR

#define LENGTH 20  
#define WIDTH 23   
#define WHEEL_RADIUS 3

int currentCommand = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *FR = AFMS.getMotor(2); //2
Adafruit_DCMotor *RR = AFMS.getMotor(1); //1
Adafruit_DCMotor *FL = AFMS.getMotor(4); //4
Adafruit_DCMotor *RL = AFMS.getMotor(3); //3

volatile int posiM1 = 0; 
volatile int posiM2 = 0; 
volatile int posiM3 = 0; 
volatile int posiM4 = 0; 
float rpmM1 = 0;
float rpmM2 = 0;
float rpmM3 = 0;
float rpmM4 = 0;
float prevTime = 0;
float currentTime = 0;
float prevTimePID = 0;
float currentTimePID = 0;

float kp = 1.9; //1.9
float kd = 0.002;//0.002
float ki = 0.002;//0.0

float eM1 = 0;
float eM2 = 0;
float eM3 = 0;
float eM4 = 0;
float prevEM1 = 0;
float prevEM2 = 0;
float prevEM3 = 0;
float prevEM4 = 0;
float eIntegralM1 = 0;
float eIntegralM2 = 0;
float eIntegralM3 = 0;
float eIntegralM4 = 0;

uint8_t pwmM1 = 0;
uint8_t pwmM2 = 0;
uint8_t pwmM3 = 0;
uint8_t pwmM4 = 0;

float m1Speed = 0; 
float m2Speed = 0;
float m3Speed = 0;
float m4Speed = 0;


float vX = 0.0;
float vY = 0.0;
float vZ = 0.0;

int directionM1 = FORWARD;
int directionM2 = FORWARD;
int directionM3 = FORWARD;
int directionM4 = FORWARD;


void readMotorSpeedsFromUART() {
  if (Serial.available() > 0) {
    String speedData = Serial.readStringUntil('\n');

    

    // Assuming the format of speed data is "X:Y:Z\n"
    int colonIndex1 = speedData.indexOf(':');
    if (colonIndex1 != -1) {
      vX = speedData.substring(0, colonIndex1).toFloat();
      int colonIndex2 = speedData.indexOf(':', colonIndex1 + 1);
      if (colonIndex2 != -1) {
        vY = speedData.substring(colonIndex1 + 1, colonIndex2).toFloat();
        vZ = speedData.substring(colonIndex2 + 1).toFloat();
        
      }
    }

    
    
    
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(ENCC, INPUT_PULLUP);
  pinMode(ENCD, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCA), readEncoderM1, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCD), readEncoderM2, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCB), readEncoderM3, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCC), readEncoderM4, RISING);
  
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  FR->setSpeed(pwmM1);
  FR->run(FORWARD);
  RR->setSpeed(pwmM2);
  RR->run(FORWARD);
  FL->setSpeed(pwmM3);
  FL->run(FORWARD);
  RL->setSpeed(pwmM4);
  RL->run(FORWARD);
}

void loop() {
  int posM1, posM2, posM3, posM4;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posM1 = posiM1;
    posM2 = posiM2;
    posM3 = posiM3;
    posM4 = posiM4;
  }
  computeSpeed();
  computePower();
  computeVectors();
  readMotorSpeedsFromUART();
  moveRobot();
  
  
}

void readEncoderM1() {
  posiM1++;
}

void readEncoderM2() {
  posiM2++;
}

void readEncoderM3() {
  posiM3++;
}

void readEncoderM4() {
  posiM4++;
}

void computeSpeed() {
  int posM1, posM2, posM3, posM4;
  currentTime = millis();
  unsigned long timeDiff = currentTime - prevTime;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posM1 = posiM1;
    posM2 = posiM2;
    posM3 = posiM3;
    posM4 = posiM4;
  }

  rpmM1 = (posM1 / 800.0) * (60000.0 / timeDiff);
  rpmM2 = (posM2 / 800.0) * (60000.0 / timeDiff);
  rpmM3 = (posM3 / 800.0) * (60000.0 / timeDiff);
  rpmM4 = (posM4 / 800.0) * (60000.0 / timeDiff);

  //Serial.print("speedFR:");
  //Serial.println(rpmM1);
  //Serial.println(rpmM1);
  //Serial.print("speedRR:");
  //Serial.println(rpmM2);
  //Serial.print("speedFL:");
  //Serial.println(rpmM3);
  //Serial.print("speedRL:");
  //Serial.println(rpmM4);

  prevTime = currentTime;
  posiM1 = 0;
  posiM2 = 0;
  posiM3 = 0;
  posiM4 = 0;
}

void computePower() {
  currentTimePID = millis();
  unsigned long timeDiffPID = currentTimePID - prevTimePID;

  // Calculate errors
  
  eM1 = m1Speed - rpmM1;
  eM2 = m2Speed - rpmM2;
  eM3 = m3Speed - rpmM3;
  eM4 = m4Speed - rpmM4;
  
  
  // Calculate derivative errors
  float dedtM1 = (eM1 - prevEM1) / (timeDiffPID);
  float dedtM2 = (eM2 - prevEM2) / (timeDiffPID);
  float dedtM3 = (eM3 - prevEM3) / (timeDiffPID);
  float dedtM4 = (eM4 - prevEM4) / (timeDiffPID);
  
  // Calculate integral errors
  eIntegralM1 = eIntegralM1 + eM1 * timeDiffPID;
  eIntegralM2 = eIntegralM2 + eM2 * timeDiffPID;
  eIntegralM3 = eIntegralM3 + eM3 * timeDiffPID;
  eIntegralM4 = eIntegralM4 + eM4 * timeDiffPID;
  if (eIntegralM1 > 2000) {
    eIntegralM1 = 2000;
  }
  if (eIntegralM2 > 2000) {
    eIntegralM2 = 2000;
  }
  if (eIntegralM3 > 2000) {
    eIntegralM3 = 2000;
  }
  if (eIntegralM4 > 2000) {
    eIntegralM4 = 2000;
  }
  
  // Calculate PID outputs
  float PIDpwmM1 = kp * eM1 + kd * dedtM1 + ki * eIntegralM1;
  float PIDpwmM2 = kp * eM2 + kd * dedtM2 + ki * eIntegralM2;
  float PIDpwmM3 = kp * eM3 + kd * dedtM3 + ki * eIntegralM3;
  float PIDpwmM4 = kp * eM4 + kd * dedtM4 + ki * eIntegralM4;

  

  // Constrain PID outputs to stay within [0, 255] range
  pwmM1 = constrain(PIDpwmM1, 0, 255);
  pwmM2 = constrain(PIDpwmM2, 0, 255);
  pwmM3 = constrain(PIDpwmM3, 0, 255);
  pwmM4 = constrain(PIDpwmM4, 0, 255);
  
  
  Serial.print("pwmM1:");
  Serial.println(pwmM1);
  Serial.print("pwmM2:");
  Serial.println(pwmM2);
  Serial.print("pwmM3:");
  Serial.println(pwmM3);
  Serial.print("pwmM4:");
  Serial.println(pwmM4);

  // Update previous errors and time
  prevEM1 = eM1;
  prevEM2 = eM2;
  prevEM3 = eM3;
  prevEM4 = eM4;
  prevTimePID = currentTimePID;
  
}


void moveRobot() {
  FL->setSpeed(pwmM4);
  FL->run(directionM4);
  FR->setSpeed(pwmM2);
  FR->run(directionM2);
  RL->setSpeed(pwmM3);
  RL->run(directionM3);
  RR->setSpeed(pwmM1);
  RR->run(directionM1);
}

void stop() {
  FL->run(RELEASE);
  FR->run(RELEASE);
  RL->run(RELEASE);
  RR->run(RELEASE);
}

void computeVectors() {
  m1Speed = abs(vX)*320 + abs(vY)*320 + abs(vZ)*50;
  Serial.println(m1Speed);
  
  if ((vX-vY+vZ)>=0){
    directionM1 = FORWARD;  
  }
  if ((vX-vY+vZ)<0){
    directionM1 = BACKWARD;  
  }
  


  m2Speed = abs(vX)*320 + abs(vY)*320 + abs(vZ)*50;
  Serial.println(m2Speed);
  if ((vX+vY-vZ)>=0){
    directionM2 = FORWARD;  
  }
  if ((vX+vY-vZ)<0){
    directionM2 = BACKWARD;  
  }

  m3Speed = abs(vX)*320 + abs(vY)*320 + abs(vZ)*50;
  Serial.println(m3Speed);
  if ((vX+vY+vZ)>=0){
    directionM3 = FORWARD;  
  }
  if ((vX+vY+vZ)<0){
    directionM3 = BACKWARD;  
  }

  m4Speed = abs(vX)*320 + abs(vY)*320 + abs(vZ)*50;
  Serial.println(m4Speed);
  if ((vX-vY-vZ)>=0){
    directionM4 = FORWARD;  
  }
  if ((vX-vY-vZ)<0){
    directionM4 = BACKWARD;  
  }
  
}
