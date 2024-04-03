#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_MotorShield.h>
#include <Arduino.h>

#define CLK_PIN 3
#define SDO_PIN 2

const int trigPin = 9;  
const int echoPin = 10; 

float duration, distance;  

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *MotorLeft = AFMS.getMotor(1);//Moteur gauche
Adafruit_DCMotor *MotorRight= AFMS.getMotor(2);//Moteur droit

//########code variables##########

int posY = 0;
int posX = 0;
float Xcm = 0;

int i = 0;
float prevAngle;
int leftSpeed = 0;
int rightSpeed = 0;
int leftDirection = FORWARD;
int rightDirection = FORWARD;
float angle = 0;



//#######pathToFollow

bool wayPoints[6] = {false, false, false, false, false, false};
int XPoints[3] = {20, 10, 30};
int turnDirection[3] = {1, 0, 1};








MPU6050 mpu(Wire);
unsigned long timer = 0;

//############optical sensor functions###############

void sendByte(uint8_t byte) {
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (byte >> i) & 0x01);
    delayMicroseconds(3);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(CLK_PIN, LOW);
  }
  digitalWrite(CLK_PIN, HIGH);
}

uint8_t readByte() {
  uint8_t byte = 0;
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(1);
    byte |= (digitalRead(SDO_PIN) << i);
    //****Serial.print(digitalRead(SDO_PIN));
  }
  digitalWrite(CLK_PIN, HIGH);
  

  return byte;
}
void WriteConfig(uint8_t addr,uint8_t config, int delay){
  digitalWrite(CLK_PIN, HIGH);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (addr >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
  }
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (config >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
  }
}
int8_t Ask(uint8_t addr, int delay) {
  int8_t byte = 0;
  pinMode(SDO_PIN,OUTPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (addr >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
  }
  pinMode(SDO_PIN,INPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    byte |= (digitalRead(SDO_PIN) << i);;
  }
  digitalWrite(CLK_PIN, HIGH);
  return byte;
  //Serial.println();
}
void AskSilent(uint8_t addr, int delay) {
  pinMode(SDO_PIN,OUTPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (addr >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
  }
  //digitalWrite(CLK_PIN, HIGH);
  pinMode(SDO_PIN,INPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    //Serial.print(digitalRead(SDO_PIN));
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    //byte |= (digitalRead(SDO_PIN) << i);;
  }
  digitalWrite(CLK_PIN, HIGH);
  //Serial.println();
}


//############ Motors functions ###############

void moveMotors(int leftSpeed, int rightSpeed, int leftDirection, int rightDirection) {
  MotorLeft->run(leftDirection);
  MotorLeft->setSpeed(leftSpeed);
  MotorRight->run(rightDirection);
  MotorRight->setSpeed(rightSpeed);
}


void stopMotors() {
  MotorLeft->setSpeed(0);
  MotorLeft->run(RELEASE);
  MotorRight->setSpeed(0);
  MotorRight->run(RELEASE);
}




//############# UART functions ###################

void sendSpeedsViaUART(float Xcm, float angle) {
  
  Serial.print("linear: ");
  Serial.println(Xcm);
  Serial.print("angular: ");
  Serial.println(angle);
}

void turn(int direction, int desiredAngle) {
  int speed = constrain(abs(desiredAngle-angle)*1.5, 20, 150);
  if (direction == 0) { //turn right 
    
    moveMotors(speed, speed, BACKWARD, FORWARD);
    
  } else if (direction == 1) {

    moveMotors(speed, speed, FORWARD, BACKWARD);
    
  } 
}
void moveForward(int desiredDestination){
  int speed = constrain(abs(desiredDestination-Xcm)*3, 20, 150);
  moveMotors(speed, speed, BACKWARD, BACKWARD);
}





void followPath() {
  for (int i = 0; i < 6; i++) {
    if (!wayPoints[0]) {
      moveForward(XPoints[0]);
      if (Xcm >= XPoints[0]) {
        wayPoints[0] = true;
        Xcm = 0;
        posX = 0;
      }
    } else if (!wayPoints[1]) {
      turn(turnDirection[0], 90);
      if (abs(angle) >= 90) {
        wayPoints[1] = true;
        Xcm = 0;
        posX = 0;
        mpu.update();
        prevAngle = mpu.getAngleZ();
      }
    } else if (!wayPoints[2]) {
      moveForward(XPoints[1]);
      if (Xcm >= XPoints[1]) {
        wayPoints[2] = true;
        Xcm = 0;
        posX = 0;
      }
    } else if (!wayPoints[3]) {
      turn(turnDirection[1], 90);
      if (abs(abs(prevAngle - mpu.getAngleZ()) - 90) < 5) {
        wayPoints[3] = true;
        Xcm = 0;
        posX = 0;
        mpu.update();
        prevAngle = mpu.getAngleZ();
      }
    } else if (!wayPoints[4]) {
      moveForward(XPoints[2]);
      if (Xcm >= XPoints[2]) {
        wayPoints[4] = true;
        Xcm = 0;
        posX = 0;
      }
    } else if (!wayPoints[5]) {
      turn(turnDirection[2], 90);
      if (abs(abs(prevAngle - mpu.getAngleZ()) - 90) < 5) {
        wayPoints[5] = true;
      }
    }
    else {
    stopMotors();
    }
  }
}





void setup() {

  //setup gyro : 

  Serial.begin(115200);
  Wire.begin();
  byte status = mpu.begin();

  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  

  //setupMotors

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    
    while (1);
  }


  
  MotorLeft->setSpeed(0);
  MotorLeft->run(FORWARD);
  MotorLeft->run(RELEASE);
  MotorRight->setSpeed(0);
  MotorRight->run(FORWARD);
  MotorRight->run(RELEASE);


 //setup optical:

  pinMode(CLK_PIN,OUTPUT);
  pinMode(SDO_PIN,INPUT);
  digitalWrite(CLK_PIN,HIGH);
  digitalWrite(SDO_PIN,HIGH);
  
  delayMicroseconds(10000);
  WriteConfig(0b10000101,0b10111000,3);
  delayMicroseconds(10000);
  WriteConfig(0b10000110,0b10000100,3); //reg 0x06
  delayMicroseconds(10000); 
  WriteConfig(0b10001110,0b11101001,3);

  //setup ultrasound

  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 


}


void loop() {
  mpu.update();
  angle = mpu.getAngleZ();
  AskSilent(0x02,3);
  
  int8_t Xvel = Ask(0x03,3);
  

  posX = posX + Xvel; //-2650 = 10cm
  Xcm = -posX * 0.0037735849;
  
  


  delay(40);
  //sendSpeedsViaUART(Xcm, angle);

  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);  

  duration = pulseIn(echoPin, HIGH);  

  distance = (duration*.0343)/2;  
  if (distance <= 20) {
  stopMotors();
  }
  else {
  followPath();
  }


  
  
  

}



