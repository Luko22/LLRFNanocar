// Arduino + nrf24L01+ module transmitter

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(10, 9); // CE, CSN
const byte address[6] = "00001"; // the address the the module
///pins 10,9,13,4,5 DO NOT USE
#define JoyStick_X_PIN A0
#define JoyStick_Y_PIN A1
int joyposVert;
int joyposHorz;

void setup() {
  Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  
  pinMode(JoyStick_X_PIN, INPUT);
  pinMode(JoyStick_Y_PIN, INPUT);
}

void loop() {
  int joyposVert = analogRead(JoyStick_X_PIN);
  int joyposHorz = analogRead(JoyStick_Y_PIN);

  Serial.print(joyposVert);
  Serial.print("  |  ");
  Serial.println(joyposHorz);

  int joystickData[2] = {joyposVert, joyposHorz};

  // Send joystick data over NRF24L01+
  radio.write(&joystickData, sizeof(joystickData));

  delay(50); // Add a delay to avoid overwhelming the radio
}