#include <Adafruit_MCP2515.h>
#include "Wire.h"

#define CS_PIN    17
#define CAN_BAUDRATE (500000)

Adafruit_MCP2515 mcp(CS_PIN);

const int trigPin = 9;
const int echoPin = 10;
const int trigPin1 = 11;
const int echoPin1 = 12;
const int trigPin2 = 14;
const int echoPin2 = 15;
const int trigPin3 = 7;
const int echoPin3 = 8;

float duration, distance;
float duration1, distance1;
float duration2, distance2;
float duration3, distance3;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  Serial.begin(115200);
  Serial.println("MCP2515 multiUltrasonic!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;
  delay(10);

  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1 * 0.0343) / 2;
  delay(10);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2 * 0.0343) / 2;
  delay(10);

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3 * 0.0343) / 2;
  delay(10);

  Serial.print("0: ");
  Serial.println(distance);

  Serial.print("1: ");
  Serial.println(distance1);

  Serial.print("2: ");
  Serial.println(distance2);

  Serial.print("3: ");
  Serial.println(distance3);

  if (distance < 10 || distance1 < 10 || distance2 < 10 || distance3 < 10) {
    Serial.println("Obstacle detected, sending CAN packet...");
    digitalWrite(LED_BUILTIN, HIGH);

    mcp.beginPacket(0x12);
    Serial.println("1");
    mcp.write('S');
    mcp.write('T');
    mcp.write('O');
    mcp.write('P');
    Serial.println("2");
    mcp.endPacket();
    

    Serial.println("CAN packet sent");
    delay(10);
  }

  digitalWrite(LED_BUILTIN, LOW);
}
