//  Charles P
// Version 4
// 4/21/2024

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins

struct JoystickData {
  int x; // X-axis for steering
  int y; // Y-axis for forward/backward
  int toggle; // State of the toggle switch
  int toggleARM;
  int toggleRECORD;
  int pot1; // First potentiometer
  int pot2; // Second potentiometer
  int pot3; // Third potentiometer
  int buttonOne; // State of the push button
} joystickData;

const int toggleSwitchPin = 3; // Interrupt pin for the toggle switch
const int toggleSwitchPinRECORD = 2; // Interrupt pin for the toggle switch
const int LEDPin = 9; // LED pin
const int LEDPinRECORD = 4; // LED pin

// ARM -------------------------------------------------------------
const int toggleSwitchPinARM = 29; // Interrupt pin for the toggle switch
const int LEDPinARM = 34; // LED pin

// Potentiometers --------------------------------------------------
const int potPin1 = A2; // Analog pin for the first potentiometer
const int potPin2 = A3; // Analog pin for the second potentiometer
const int potPin3 = A4; // Analog pin for the second potentiometer

volatile int switchState = LOW; // Use volatile for variables accessed within interrupt
volatile int switchStateARM = LOW;
volatile int switchStateRECORD = LOW;

const int pushButtonPin = 12; // Assign an appropriate pin for the push button
int pushButtonState = LOW; // Variable to store the push button state

void toggleSwitchISR() {
  switchState = !switchState; // Toggle the state
  digitalWrite(LEDPin, switchState); // Reflect state on LED immediately
}

void toggleSwitchISR_Arm() {
  switchStateARM = !switchStateARM; // Toggle the state
  digitalWrite(LEDPinARM, switchStateARM); // Reflect state on LED immediately
}

void toggleSwitchISR_RECORD() {
  switchStateRECORD = !switchStateRECORD; // Toggle the state
  digitalWrite(LEDPinRECORD, switchStateRECORD); // Reflect state on LED immediately
}

void setup() {
  Serial.begin(9600);
  pinMode(toggleSwitchPin, INPUT_PULLUP); // Set the toggle switch pin as input with pull-up
  pinMode(LEDPin, OUTPUT); // Set the LED pin as output

  pinMode(toggleSwitchPinARM, INPUT_PULLUP); // Set the toggle switch pin as input with pull-up
  pinMode(LEDPinARM, OUTPUT); // Set the LED pin as output

  pinMode(toggleSwitchPinRECORD, INPUT_PULLUP); // Set the toggle switch pin as input with pull-up
  pinMode(LEDPinRECORD, OUTPUT); // Set the LED pin as output

  pinMode(potPin1, INPUT); // Set the first potentiometer pin as input
  pinMode(potPin2, INPUT); // Set the second potentiometer pin as input
  pinMode(potPin3, INPUT); // Set the third potentiometer pin as input

  pinMode(pushButtonPin, INPUT_PULLUP);

  // Attach an interrupt to the toggleSwitchPin
  attachInterrupt(digitalPinToInterrupt(toggleSwitchPin), toggleSwitchISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(toggleSwitchPinARM), toggleSwitchISR_Arm, CHANGE);

  radio.begin();
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.stopListening();
}

void loop() {
  joystickData.x = analogRead(A0); // Read X-axis of joystick
  joystickData.y = analogRead(A1); // Read Y-axis of joystick
  //  joystickData.toggle = switchState; // Use the volatile switch state updated by the ISR
  //  joystickData.toggleARM = switchStateARM;
  joystickData.pot1 = analogRead(potPin1); // Read the first potentiometer
  joystickData.pot2 = analogRead(potPin2); // Read the second potentiometer
  joystickData.pot3 = analogRead(potPin3); // Read the second potentiometer

  // Update LED based on the toggle switch state
  digitalWrite(LEDPin, digitalRead(toggleSwitchPin));

  // Update LED for the ARM based on the toggle switch state
  digitalWrite(LEDPinARM, digitalRead(toggleSwitchPinARM));

  // Update LED for the ARM based on the toggle switch state
  digitalWrite(LEDPinRECORD, digitalRead(toggleSwitchPinRECORD));

  int setDATAtoggle = digitalRead(toggleSwitchPin);
  Serial.print("Toggle Switch ONE:");
  Serial.println(setDATAtoggle);
  joystickData.toggle = setDATAtoggle;

  // Correct way to read the ARM switch state
  int setDATA = digitalRead(toggleSwitchPinARM);  // Read the state of the ARM toggle switch pin
  Serial.print("Toggle Switch TWO:");
  Serial.println(setDATA);
  // Update joystickData with the ARM switch state for transmission
  joystickData.toggleARM = setDATA;  // Assign the read state to toggleARM for transmitting

    // Correct way to read the ARM switch state
  int setDATARECORD = digitalRead(toggleSwitchPinRECORD);  // Read the state of the ARM toggle switch pin
  Serial.print("Toggle Switch THREE:");
  Serial.println(setDATARECORD);
  // Update joystickData with the ARM switch state for transmission
  joystickData.toggleRECORD = setDATARECORD;  // Assign the read state to toggleARM for transmitting

  // Read the state of the push button
  pushButtonState = digitalRead(pushButtonPin);
  Serial.print("Push Button State:");
  Serial.println(pushButtonState);
 
  // Add the push button state to the joystick data
  joystickData.buttonOne = pushButtonState;

  bool ok = radio.write(&joystickData, sizeof(joystickData));
  if (ok) {
    Serial.println("Transmission successful");
  } else {
    Serial.println("Transmission failed");
  }

  delay(20); // Small delay to limit the data rate
}
