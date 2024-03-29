#include <Adafruit_MCP2515.h>


#define CS_PIN    17


#define CAN_BAUDRATE (1000000)

Adafruit_MCP2515 mcp(CS_PIN);




const int trigPin = 9;
const int echoPin = 10;



float duration, distance;





// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.println(distance);
  if (distance < 20) {
    digitalWrite(LED_BUILTIN, HIGH);
    mcp.beginPacket(0x12);
    mcp.write('1');
    mcp.write('2');
    mcp.write('3');
    mcp.write('4');

    mcp.endPacket();
    delay(10);
      }
  
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  
}
