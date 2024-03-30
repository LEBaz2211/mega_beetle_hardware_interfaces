#include <Adafruit_MCP2515.h>


#define CS_PIN    17


#define CAN_BAUDRATE (1000000)

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





// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.

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
  //while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}


void loop() {
//1
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  delay(10);

//2
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1*.0343)/2;
  delay(10);


//3
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2*.0343)/2;
  delay(10);

//4
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3*.0343)/2;
  delay(10);

  Serial.print("0: ");
  Serial.println(distance);

  Serial.print("1: ");
  Serial.println(distance1);

  Serial.print("2: ");
  Serial.println(distance2);

  Serial.print("3: ");
  Serial.println(distance3);
  if (distance < 20) {
    digitalWrite(LED_BUILTIN, HIGH);
    mcp.beginPacket(0x12);
    mcp.write('S');
    mcp.write('T');
    mcp.write('O');
    mcp.write('P');

    mcp.endPacket();
    delay(10);
      }

  else if (distance1 < 20) {
    digitalWrite(LED_BUILTIN, HIGH);
    mcp.beginPacket(0x12);
    mcp.write('S');
    mcp.write('T');
    mcp.write('O');
    mcp.write('P');

    mcp.endPacket();
    delay(10);
      }

  else if (distance2 < 20) {
    digitalWrite(LED_BUILTIN, HIGH);
    mcp.beginPacket(0x12);
    mcp.write('S');
    mcp.write('T');
    mcp.write('O');
    mcp.write('P');

    mcp.endPacket();
    delay(10);
      }
  else if (distance3 < 20) {
    digitalWrite(LED_BUILTIN, HIGH);
    mcp.beginPacket(0x12);
    mcp.write('S');
    mcp.write('T');
    mcp.write('O');
    mcp.write('P');

    mcp.endPacket();
    delay(10);
      }
  
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  
}
