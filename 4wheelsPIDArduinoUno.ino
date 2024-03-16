#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Adafruit_MotorShield.h>
#include <PinChangeInterrupt.h>

#define ENCA 8 // RR
#define ENCB 11 // RL
#define ENCC 10 // FL
#define ENCD 9  // FR

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
unsigned long prevTime = 0;
unsigned long currentTime = 0;
unsigned long prevTimePID = 0;
unsigned long currentTimePID = 0;

float kp = 1.9; //0.9
float kd = 0.002;//0.002
float ki = 0.02;//0.0

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


float m1Speed = 20; //a modifier pour imposer la vitesse
float m2Speed = 20;
float m3Speed = 20;
float m4Speed = 20;

uint8_t pwmM1 = 0;
uint8_t pwmM2 = 0;
uint8_t pwmM3 = 0;
uint8_t pwmM4 = 0;


void readMotorSpeedsFromUART() {
  if (Serial.available() > 0) {
    String speedData = Serial.readStringUntil('\n');

    if (speedData.indexOf("F1") != -1) {
      currentCommand = 1; //  Forward
    } else if (speedData.indexOf("R1") != -1) {
      currentCommand = 2; //  Round
    } else if (speedData.indexOf("C1") != -1) {
      currentCommand = 3; //  Sideways
    } else if (speedData.indexOf("F0") != -1) {
      currentCommand = 4; //  Sideways
    } else if (speedData.indexOf("R0") != -1) {
      currentCommand = 5;
    } else if (speedData.indexOf("C0") != -1){
      currentCommand = 6;
    }
     else if (speedData.indexOf("S") != -1) {
      currentCommand = 0; // Stop
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
  FR->run(FORWARD);
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
 

  readMotorSpeedsFromUART();

  switch (currentCommand) {
    case 1:
      moveStraight(1); 
      break;
    case 2:
      turnRound(1);
      break;
    case 3:
      moveSide(1);
      break;
    case 4:
      moveStraight(0);
      break;
    case 5:
      turnRound(0);
      break;
    case 6:
      moveSide(0);
      break;
    
      
    case 0:
    default:
      stop(); 
      break;
  }
  
  
  
  
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
  Serial.print("speedFR:");
  Serial.println(rpmM1);
  Serial.print("speedRR:");
  Serial.println(rpmM2);
  Serial.print("speedFL:");
  Serial.println(rpmM3);
  Serial.print("speedRL:");
  Serial.println(rpmM4);

  prevTime = currentTime;
  posiM1 = 0;
  posiM2 = 0;
  posiM3 = 0;
  posiM4 = 0;
}

void computePower(){
  
  currentTimePID = millis();
  unsigned long timeDiffPID = currentTimePID - prevTimePID;

  //erreur
  eM1 = m1Speed - rpmM1;
  eM2 = m2Speed - rpmM2;
  eM3 = m3Speed - rpmM3;
  eM4 = m4Speed - rpmM4;
  //erreur dérivée
  float dedtM1 = (eM1-prevEM1)/(timeDiffPID);
  float dedtM2 = (eM2-prevEM2)/(timeDiffPID);
  float dedtM3 = (eM3-prevEM3)/(timeDiffPID);
  float dedtM4 = (eM4-prevEM4)/(timeDiffPID);

  //erreur intégrale
  eIntegralM1 = eIntegralM1 + eM1*timeDiffPID;
  eIntegralM2 = eIntegralM2 + eM2*timeDiffPID;
  eIntegralM3 = eIntegralM3 + eM3*timeDiffPID;
  eIntegralM4 = eIntegralM4 + eM4*timeDiffPID;

  //signal de contrôle
  pwmM1 = kp*eM1 + kd*dedtM1 + ki*eIntegralM1;
  pwmM2 = kp*eM2 + kd*dedtM2 + ki*eIntegralM2;
  pwmM3 = kp*eM3 + kd*dedtM3 + ki*eIntegralM3;
  pwmM4 = kp*eM4 + kd*dedtM4 + ki*eIntegralM4;

  if (pwmM1>255) {
    pwmM1 = 255;
  }
  if (pwmM1<0) {
    pwmM1 = 0;
  }
  if (pwmM2<0){
    pwmM2 = 0;
  }
  if (pwmM2>255){
    pwmM2 = 255;
  }
  if (pwmM3<0){
    pwmM3 = 0;
  }
  if (pwmM3>255){
    pwmM3 = 255;
  }
  if (pwmM4<0){
    pwmM4 = 0;
  }
  if (pwmM4>255){
    pwmM4 = 255;
  }
  

  prevEM1 = eM1;
  prevEM2 = eM2;
  prevEM3 = eM3;
  prevEM4 = eM4;
  prevTimePID = currentTimePID;
}


void moveStraight(int direction){
  
  if (direction == 1){
  
    FL->setSpeed(pwmM3);
    FL->run(FORWARD);
    FR->setSpeed(pwmM1);
    FR->run(FORWARD);
    RL->setSpeed(pwmM4);
    RL->run(FORWARD);
    RR->setSpeed(pwmM2);
    RR->run(FORWARD);
  }

  else if (direction == 0) {
    FL->setSpeed(pwmM3);
    FL->run(BACKWARD);
    FR->setSpeed(pwmM1);
    FR->run(BACKWARD);
    RL->setSpeed(pwmM4);
    RL->run(BACKWARD);
    RR->setSpeed(pwmM2);
    RR->run(BACKWARD);
  }

  else {
    stop();
  }

}

void moveSide(int direction){
  if (direction == 1){
  
    FL->setSpeed(pwmM3);
    FL->run(BACKWARD);
    FR->setSpeed(pwmM1);
    FR->run(FORWARD);
    RL->setSpeed(pwmM4);
    RL->run(FORWARD);
    RR->setSpeed(pwmM2);
    RR->run(BACKWARD);
  }

  else if (direction == 0) {
    FL->setSpeed(pwmM3);
    FL->run(FORWARD);
    FR->setSpeed(pwmM1);
    FR->run(BACKWARD);
    RL->setSpeed(pwmM4);
    RL->run(BACKWARD);
    RR->setSpeed(pwmM2);
    RR->run(FORWARD);
  }
  else {
    stop();
  }
}

void moveDiagonal(int direction){
  if (direction == 1){
  
    FL->setSpeed(pwmM3);
    FL->run(FORWARD);
    FR->run(RELEASE);
    RL->run(RELEASE);
    RR->setSpeed(pwmM2);
    RR->run(FORWARD);
  }

  else if (direction == 0) {
    FL->run(RELEASE);
    FR->setSpeed(pwmM1);
    FR->run(FORWARD);
    RL->setSpeed(pwmM4);
    RL->run(FORWARD);
    RR->run(RELEASE);
  }
  else {
    stop();
  }
}

void turnRound(int direction){
  if (direction == 1){
  
    FL->setSpeed(pwmM3);
    FL->run(BACKWARD);
    FR->setSpeed(pwmM1);
    FR->run(BACKWARD);
    RL->setSpeed(pwmM4);
    RL->run(FORWARD);
    RR->setSpeed(pwmM2);
    RR->run(FORWARD);
  }

  else if (direction == 0) {
    FL->setSpeed(pwmM3);
    FL->run(FORWARD);
    FR->setSpeed(pwmM1);
    FR->run(FORWARD);
    RL->setSpeed(pwmM4);
    RL->run(BACKWARD);
    RR->setSpeed(pwmM2);
    RR->run(BACKWARD);
  }
  else {
    stop();
  }

}

void stop(){

  FL->run(RELEASE);
  FR->run(RELEASE);
  RL->run(RELEASE);
  RR->run(RELEASE);

}
