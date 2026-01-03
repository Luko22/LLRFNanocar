//importing libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "SR04.h"

//defining pins for ultrasonic sensor (looks like two speakers and measures distance)
#define TRIG_PIN 4
#define ECHO_PIN 5
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;
unsigned long lastUltraRead = 0;
const unsigned long ULTRA_INTERVAL = 60; // ms

#define warnLED A0

RF24 radio(10, 9); // CE, CSN
const byte address[6] = "00001"; // the address the the module
///pins D10,D9,D13,D4,D5 : DO NOT USE THESE PINS
#include <Servo.h>

/* Motor A connections: 
in1 and in2 pins are used to control the direction of Motor A
connected to pin 13, pin 12 */
int enA = 3; 
int in1 = 2; 
int in2 = 7;
/* Motor B connections: 
in3 and in4 pins are used to control the direction of Motor B
connected to pin 11, pin 10 */
int enB = 6; 
int in3 = 8; 
int in4 = A2; 

//////blinking lights////////////
bool warnBlink = false;
unsigned long lastBlink = 0;
const unsigned long BLINK_INTERVAL = 100;
bool ledState = false;

void setup() {
  Serial.begin(9600);
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  pinMode(warnLED, OUTPUT);
  digitalWrite(warnLED, LOW);
  delay(100);
  digitalWrite(warnLED, HIGH);
  delay(100);
  digitalWrite(warnLED, LOW);
  delay(100);
  digitalWrite(warnLED, HIGH);
  delay(100);
  digitalWrite(warnLED, LOW);

// pins initiation
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    delay(1000);
    initDrive(60);

}

void loop() {
  if (radio.available()) {
    int receivedData[2];
    radio.read(&receivedData, sizeof(receivedData));

    int joyposV = receivedData[0];
    int joyposH = receivedData[1];

    int decLim = 300;
    int incLim = 700;

    int driveD = map(joyposV, decLim, 0, 80, 255);
    int driveB = map(joyposV, incLim, 1023, 50, 255);

    int driveR = map(joyposH, incLim, 0, 50, 150);
    int driveL = map(joyposH, incLim, 1023, 50, 150);

    Serial.print(joyposV);
    Serial.print("  |  ");
    Serial.print(driveD);
    Serial.print("  |  ");
    Serial.print(driveB);
    Serial.print("  |||  ");
    Serial.print(joyposH);
    Serial.print("  |  ");
    Serial.print(driveL);
    Serial.print("  |  ");
    Serial.println(driveR);


//Forward and Back
  if(joyposV<decLim){
    //map then drive forwards
    analogWrite(enA, driveD);
    analogWrite(enB, driveD);
  
    // motor A CW ^ (forward)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // motor B CW ^ (forward)
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    }else if(joyposV>incLim){
      //moves back
    analogWrite(enA, driveB);
    analogWrite(enB, driveB);
   // motor A CCW (backwards)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // motor B CCW (backwards)
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else{
    // Turn off motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

//Rotation
  if(joyposH<decLim){
    //rot R
    analogWrite(enA, driveR);
    analogWrite(enB, driveR);
  
    // motor A CCW
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // motor B CW ^
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    } else if(joyposH>incLim){
      //rot L
    analogWrite(enA, driveL);
    analogWrite(enB, driveL);
  
   // motor A CW ^ 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // motor B CCW 
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    }



//////////////ULTRASOUND///////////////
    if (millis() - lastUltraRead >= ULTRA_INTERVAL) {
      lastUltraRead = millis();
      a = sr04.Distance();
    }
    Serial.print(a);
    Serial.println("cm");

    if(a<30 && a>10){ 
    digitalWrite(warnLED, HIGH);
    }else if(a<10){
      signal();

    }
    else{
    digitalWrite(warnLED, LOW);
    

    }
  }
}

/////////////////////////////////////////////////

void initDrive(int s){
  analogWrite(enA, s);
  analogWrite(enB, s);
 
  // motor A CW ^ (forward)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);
    
  // motor B CW ^ (forward)
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    delay(200);

 // Turn off motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(500);


 // motor A CCW (backwards)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // digitalWrite(in1, HIGH);
  // digitalWrite(in2, LOW);

  // motor B CCW (backwards)
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(200);

 // Turn off motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(1000);

}


//FUNCTIONS////////////
void signal(){
if (!warnBlink) {
    digitalWrite(warnLED, LOW);
    return;
  }

  if (millis() - lastBlink >= BLINK_INTERVAL) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(warnLED, ledState);
  }
}

