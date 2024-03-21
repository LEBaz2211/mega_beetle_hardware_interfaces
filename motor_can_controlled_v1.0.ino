#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Adafruit_MotorShield.h>
#include <PinChangeInterrupt.h>
#include "AA_MCP2515.h"

const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_8MHz_500kbps;
const uint8_t CAN_CS = 10;
const int8_t CAN_INT = 2;

// Initialize CANConfig with the settings for bitrate, chip select pin, and interrupt pin
CANConfig config(CAN_BITRATE, CAN_CS, CAN_INT);
CANController CAN(config);

#define ENCA 3 // RR
#define ENCB 7 // RL
#define ENCC 6 // FL
#define ENCD 4  // FR

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

float linearX = 0.0;
float linearY = 0.0;
float angularZ = 0.0;

unsigned long lastCommandTime = 0;



void readMotorSpeedsFromUART() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    // Assuming the command format is "X,Y,Z\n"
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    
    linearX = command.substring(0, firstComma).toFloat();
    linearY = command.substring(firstComma + 1, secondComma).toFloat();
    angularZ = command.substring(secondComma + 1).toFloat();

    Serial.println(linearX);
    Serial.println(linearY);
    Serial.println(angularZ);

    lastCommandTime = millis();
    // Now linearX, linearY, and angularZ contain the new velocities
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

    // Initialize CAN controller in Normal mode
  while (CAN.begin(CANController::Mode::Normal) != CANController::OK) {
    Serial.println("CAN begin FAIL - retrying");
    delay(1000); // Delay a bit before retrying
  }

  Serial.println("CAN controller initialized successfully");

}

void loop() {
    CANFrame frame;
  if (CAN.read(frame) == CANController::IOResult::OK) {
    if (frame.getDlc() == 6) { // Assuming 6-byte commands for linearX, linearY, angularZ
      // Process the frame to extract linearX, linearY, angularZ
      int16_t linear_x = (int16_t)(frame.getData()[1] << 8 | frame.getData()[0]);
      int16_t linear_y = (int16_t)(frame.getData()[3] << 8 | frame.getData()[2]);
      int16_t angular_z = (int16_t)(frame.getData()[5] << 8 | frame.getData()[4]);
      
      // Convert to float for calculations (if needed based on your scaling)
      linearX = linear_x / 100.0;
      linearY = linear_y / 100.0;
      angularZ = angular_z / 100.0;
      
      lastCommandTime = millis(); // Update the last command time
    }
    // Optional: frame.print("RX"); // Debugging
  }

  int posM1, posM2, posM3, posM4;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posM1 = posiM1;
    posM2 = posiM2;
    posM3 = posiM3;
    posM4 = posiM4;
  }

  // readMotorSpeedsFromUART(); // Now reads X, Y, Z velocities
  calculateWheelSpeeds(); // New function to calculate wheel speeds

    // Check if more than 1 second has passed since the last command
  if (millis() - lastCommandTime > 1000) {
      // If more than a second has passed, stop the robot by setting all speeds to 0
      m1Speed = 0;
      m2Speed = 0;
      m3Speed = 0;
      m4Speed = 0;
  }

  applyMotorSpeeds();

  computeSpeed();
  computePower(); // This function uses the updated m1Speed, m2Speed, m3Speed, and m4Speed

  delay(10);

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

void calculateWheelSpeeds() {
  // Placeholder values for robot geometry - adjust these to your robot's specifications
  float r = 3.0; // Distance from center to wheel in cm (example value)
  float c = 18.85; // Wheel circumference in cm (example value, corresponds to a 10cm diameter wheel)

  // Convert angular velocity from rad/s to cm/s using the radius
  // This assumes the robot rotates around its center point
  float angularVelocityCmS = angularZ * r;

  // Calculate wheel speeds in cm/s considering X, Y movements and rotation
  // These formulas depend on your mecanum wheel configuration
  float wheelSpeedCmS_FL = linearX + linearY + angularVelocityCmS;
  float wheelSpeedCmS_FR = linearX - linearY - angularVelocityCmS;
  float wheelSpeedCmS_RL = linearX - linearY + angularVelocityCmS;
  float wheelSpeedCmS_RR = linearX + linearY - angularVelocityCmS;

  // Convert cm/s to RPM for each wheel
  // RPM = (wheelSpeedCmS * 60) / circumference
  m1Speed = (wheelSpeedCmS_FR * 60) / c;
  m2Speed = (wheelSpeedCmS_RR * 60) / c;
  m3Speed = (wheelSpeedCmS_FL * 60) / c;
  m4Speed = (wheelSpeedCmS_RL * 60) / c;

  Serial.println(m1Speed);
  Serial.println(m2Speed);
  Serial.println(m3Speed);
  Serial.println(m4Speed);
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
  // Serial.print("speedFR:");
  // Serial.println(rpmM1);
  // Serial.print("speedRR:");
  // Serial.println(rpmM2);
  // Serial.print("speedFL:");
  // Serial.println(rpmM3);
  // Serial.print("speedRL:");
  // Serial.println(rpmM4);

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

  Serial.println(pwmM1);

  

  prevEM1 = eM1;
  prevEM2 = eM2;
  prevEM3 = eM3;
  prevEM4 = eM4;
  prevTimePID = currentTimePID;
}

void applyMotorSpeeds() {
    // Apply speed and direction for each motor based on the PID-controlled speed variables
    if (m1Speed >= 0) {
        FR->setSpeed(min(abs(m1Speed), 255)); // Ensure the speed does not exceed 255
        FR->run(FORWARD);
    } else {
        FR->setSpeed(min(abs(m1Speed), 255));
        FR->run(BACKWARD);
    }

    if (m2Speed >= 0) {
        RR->setSpeed(min(abs(m2Speed), 255));
        RR->run(FORWARD);
    } else {
        RR->setSpeed(min(abs(m2Speed), 255));
        RR->run(BACKWARD);
    }

    if (m3Speed >= 0) {
        FL->setSpeed(min(abs(m3Speed), 255));
        FL->run(FORWARD);
    } else {
        FL->setSpeed(min(abs(m3Speed), 255));
        FL->run(BACKWARD);
    }

    if (m4Speed >= 0) {
        RL->setSpeed(min(abs(m4Speed), 255));
        RL->run(FORWARD);
    } else {
        RL->setSpeed(min(abs(m4Speed), 255));
        RL->run(BACKWARD);
    }
}

void stop(){

  FL->run(RELEASE);
  FR->run(RELEASE);
  RL->run(RELEASE);
  RR->run(RELEASE);

}
