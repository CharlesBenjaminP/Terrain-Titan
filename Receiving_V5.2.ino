//  Charles P
// Version 5.2
// 4/28/2024

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Stepper.h>
#include <Arduino.h>

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

// Create an MPU6050 object
MPU6050 mpu;

// Setup for your balance servo
Servo balanceServo;
const int balanceServoPin = 6;
int balanceServoPos = 105; // Start at the midpoint, adjust based on setup

// Define variables for MPU6050 readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Rear Motors
const int leftMotorForwardPin = 2;
const int leftMotorBackwardPin = 3;
const int rightMotorForwardPin = 30;
const int rightMotorBackwardPin = 32;

// Middle Motors
const int leftMMotorForwardPin = 36;
const int leftMMotorBackwardPin = 38;
const int rightMMotorForwardPin = 40;
const int rightMMotorBackwardPin = 42;

// Front Motors
const int leftFMotorForwardPin = 37;
const int leftFMotorBackwardPin = 39;
const int rightFMotorForwardPin = 41;
const int rightFMotorBackwardPin = 43;

const int deadzoneX = 250; // Increased deadzone for the X-axis
const int deadzoneY = 15; // Deadzone for the Y-axis
const int maxSpeed = 255; // Maximum speed value for motors

//----------------------------------------------------------------------------------------------------------------------------------------------

Servo leftServos[3];  // Left bank servos
Servo rightServos[3]; // Right bank servos

int leftServoPins[3] = {34, 24, 26};  // Left bank servo pins
int rightServoPins[3] = {23, 29, 27}; // Right bank servo pins

// Edit these arrays to set the initial positions for each servo
int leftServoInitialPositions[3] = {104, 96, 93};  // Initial positions for left bank servos [FRONT, MIDDLE, REAR] WHITE
int rightServoInitialPositions[3] = {60, 78, 73}; // Initial positions for right bank servos [FRONT, MIDDLE, REAR] GREEN

// Define desired steering positions for left and right turns
int leftTurnPosition = 65; // Desired position when turning left
int rightTurnPosition = 20; // Desired position when turning right

// Define the Trigger and Echo pin:
#define TRIG_PIN 19
#define ECHO_PIN 18

// Define variables for the duration and the distance:
long duration;
float distance; // Use float for higher precision

//----------------------------------------------------------------------------------------------------------------------------------------------

// Connection LED
const int connectionLEDPin = 48;

// ARM SERVO ------------------------------------------------------------------------------------------------------------------------------------

Servo armServo;  // create servo object for the arm
Servo armServoTwo;
Servo armServoThree;

const int stepsPerRevolution = 500;  // Change this to fit the number of steps per revolution for motor
const int motorSpeed = 60;           // RPM

// Initialize the stepper library on the appropriate pins
Stepper myStepper(stepsPerRevolution, 49, 46, 53, 47);

int lastButtonState = LOW; // Tracks the last state of the button

int buzzerPin = 35; // Connect Piezo Buzzer to Pin 8

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.startListening();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU Online");
  }

  // Initialize motor control pins
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);

  pinMode(leftMMotorForwardPin, OUTPUT);
  pinMode(leftMMotorBackwardPin, OUTPUT);
  pinMode(rightMMotorForwardPin, OUTPUT);
  pinMode(rightMMotorBackwardPin, OUTPUT);

  pinMode(leftFMotorForwardPin, OUTPUT);
  pinMode(leftFMotorBackwardPin, OUTPUT);
  pinMode(rightFMotorForwardPin, OUTPUT);
  pinMode(rightFMotorBackwardPin, OUTPUT);

  // Define the TRIG_PIN as Output and ECHO_PIN as Input:
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(buzzerPin, OUTPUT);

  for (int i = 0; i < 3; i++) {
    leftServos[i].attach(leftServoPins[i]);
    rightServos[i].attach(rightServoPins[i]);
    leftServos[i].write(leftServoInitialPositions[i]);  // Initialize to specified initial positions
    delay(150);
    rightServos[i].write(rightServoInitialPositions[i]); // Initialize to specified initial positions
    delay(150);

    // Attach and initialize the balance servo
    balanceServo.attach(balanceServoPin);
    balanceServo.write(balanceServoPos);
    delay(150);

    // ARM SERVO ------------------------------------------------------------------------------------------------------------------------------------

    armServo.attach(28);
    armServo.write(180);
    armServoTwo.attach(9);
    armServoTwo.write(0);
    armServoThree.attach(10);
    armServoThree.write(90);

    myStepper.setSpeed(motorSpeed);

    pinMode(connectionLEDPin, OUTPUT);
  }
  stopMotors();

  if (radio.available() == true) {
    digitalWrite(connectionLEDPin, HIGH);
    playMoonlightSonataDarker();
  }
}

// Akermann desiered steering end points
// Modify these arrays to define the steering positions
int leftTurnPositions[2] = {55, 120}; // [FRONT, REAR]
int rightTurnPositions[2] = {25, 85}; // [FRONT, REAR]

int currentArmTarget = 180;  // Initialize with the default position
int currentArmPosition = 180;  // Current position of the arm, initialize with the default position

int currentArmTargetTwo = 0;  // Initialize with the default position
int currentArmPositionTwo = 0;  // Current position of the arm, initialize with the default position

int currentArmTargetThree = 85;  // Initialize with the default position
int currentArmPositionThree = 85;  // Current position of the arm, initialize with the default position

unsigned long gripperPreviousMillis = 0; // Tracks the last time the gripper moved
const long gripperInterval = 1000; // Interval at which to activate the gripper (milliseconds)
bool gripperActivated = false; // Tracks whether the gripper has been activated

bool armIsActive = false; // Global flag for arm activity

void loop() {
  if (radio.available()) {
    //BalanceServoControl ();
    DistanceSensor ();

    if (distance <= 0.1) {
      stopMotors();
      Forward();
      Serial.println("Taking evasive maneuvers, REVERSING");
      delay(500);
      stopMotors();
    }

    radio.read(&joystickData, sizeof(JoystickData));
    digitalWrite(connectionLEDPin, HIGH);

    int steeringAdjustment = map(abs(joystickData.x - 512), 0, 512 - deadzoneX, 0, maxSpeed);
    bool isTurningLeft = joystickData.x < 512 - deadzoneX;
    bool isTurningRight = joystickData.x > 512 + deadzoneX;

    // Toggle switch state handling
    if (joystickData.toggle) { // If toggle is HIGH
      if (isTurningLeft || isTurningRight) {
        Stance(); // Enter crab stance
      } else {
        ResetPostion(); // Return to default position if not turning
      }
    } else { // If toggle is LOW
      if (isTurningLeft) {
        LeftTurn(); // Turn left
      } else if (isTurningRight) {
        RightTurn(); // Turn right
      } else {
        ResetPostion(); // Return to default position if not turning
      }
    }

    if (joystickData.toggle) { // If toggle is HIGH
      // Correcting the drive forward/backward logic
      if (joystickData.y > 512 + deadzoneY) {
        // Assuming higher values mean forward movement
        int speed = map(joystickData.y, 512 + deadzoneY, 1023, 0, maxSpeed);
        drive(speed, speed, isTurningLeft, isTurningRight, steeringAdjustment);
      } else if (joystickData.y < 512 - deadzoneY) {
        // Assuming lower values mean backward movement
        int speed = map(joystickData.y, 512 - deadzoneY, 0, 0, -maxSpeed);
        drive(speed, speed, isTurningLeft, isTurningRight, steeringAdjustment);
      } else {
        // Explicitly stop motors if within the deadzone
        stopMotors();
      }
    } else {
      if (joystickData.y > 512 + deadzoneY) {
        int speed = map(joystickData.y, 512 + deadzoneY, 1023, 0, maxSpeed);
        drive(speed, speed, 0, 0, 0);
      } else if (joystickData.y < 512 - deadzoneY) {
        int speed = map(joystickData.y, 512 - deadzoneY, 0, 0, -maxSpeed);
        drive(speed, speed, 0, 0, 0);
      } else {
        stopMotors();
      }
    }

    // ARM CONTROL
    if (joystickData.toggleARM == HIGH) {
      //armIsActive = true;
      // Control arms based on potentiometer values only when toggleARM is HIGH
      currentArmTarget = map(joystickData.pot1, 0, 1023, 0, 180);  // Adjust for arm 1 based on pot1
      delay(10);
      currentArmTargetTwo = map(joystickData.pot2, 0, 1023, 0, 180);  // Adjust for arm 2 based on pot2
      delay(10);
      currentArmTargetThree = map(joystickData.pot3, 0, 1023, 0, 180);  // Adjust for arm 3 based on pot2
      delay(10);
      GripperControl();
    } else if (joystickData.toggleARM == LOW) {
      // Return arms to their default positions when toggleARM is LOW
      currentArmTarget = 210;  // Default position for arm 1
      currentArmTargetTwo = 0;  // Default position for arm 2
      currentArmTargetThree = 105;  // Default position for arm 2
      //armIsActive = false;
    }

    // Move the arms towards their target positions
    updateArmPositions();

    // Call balance control only if the arm is not active
    if (!armIsActive) {
      BalanceServoControl();
    }

    if (joystickData.toggleRECORD == HIGH) {
      FigureEight();
    } else {
      radio.available();
    }
    //    GripperControl();
  }
  digitalWrite(connectionLEDPin, LOW);
}

void drive(int leftSpeed, int rightSpeed, bool isTurningLeft, bool isTurningRight, int steeringAdjustment) {
  // Adjust speeds for turning
  if (isTurningLeft) {
    // To turn left, the left motors go backward, and the right motors go forward
    leftSpeed = -steeringAdjustment;
    rightSpeed = steeringAdjustment;
  } else if (isTurningRight) {
    // To turn right, the right motors go backward, and the left motors go forward
    leftSpeed = steeringAdjustment;
    rightSpeed = -steeringAdjustment;
  }

  // Ensure the speeds are within the valid range
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Set motor speeds for rear motors
  setMotorSpeed(leftMotorForwardPin, leftMotorBackwardPin, leftSpeed);
  setMotorSpeed(rightMotorForwardPin, rightMotorBackwardPin, rightSpeed);

  // Set motor speeds for middle motors
  setMotorSpeed(leftMMotorForwardPin, leftMMotorBackwardPin, leftSpeed);
  setMotorSpeed(rightMMotorForwardPin, rightMMotorBackwardPin, rightSpeed);

  // Set motor speeds for front motors
  setMotorSpeed(leftFMotorForwardPin, leftFMotorBackwardPin, leftSpeed);
  setMotorSpeed(rightFMotorForwardPin, rightFMotorBackwardPin, rightSpeed);
}

void Forward() {
  // Change HIGH to LOW and LOW to HIGH to reverse direction
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);

  digitalWrite(leftMMotorForwardPin, LOW);
  digitalWrite(leftMMotorBackwardPin, HIGH);
  digitalWrite(rightMMotorForwardPin, LOW);
  digitalWrite(rightMMotorBackwardPin, HIGH);

  digitalWrite(leftFMotorForwardPin, LOW);
  digitalWrite(leftFMotorBackwardPin, HIGH);
  digitalWrite(rightFMotorForwardPin, LOW);
  digitalWrite(rightFMotorBackwardPin, HIGH);
}

void Backwards() {
  // Change HIGH to LOW and LOW to HIGH to reverse direction
  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);

  digitalWrite(leftMMotorForwardPin, HIGH);
  digitalWrite(leftMMotorBackwardPin, LOW);
  digitalWrite(rightMMotorForwardPin, HIGH);
  digitalWrite(rightMMotorBackwardPin, LOW);

  digitalWrite(leftFMotorForwardPin, HIGH);
  digitalWrite(leftFMotorBackwardPin, LOW);
  digitalWrite(rightFMotorForwardPin, HIGH);
  digitalWrite(rightFMotorBackwardPin, LOW);
}

void setMotorSpeed(int forwardPin, int backwardPin, int speed) {
  if (speed > 0) {
    analogWrite(forwardPin, speed);
    digitalWrite(backwardPin, LOW);
  } else if (speed < 0) {
    digitalWrite(forwardPin, LOW);
    analogWrite(backwardPin, -speed);
  } else {
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
  }
}

void stopMotors() {
  // Stop rear motors
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);

  // Stop middle motors
  digitalWrite(leftMMotorForwardPin, LOW);
  digitalWrite(leftMMotorBackwardPin, LOW);
  digitalWrite(rightMMotorForwardPin, LOW);
  digitalWrite(rightMMotorBackwardPin, LOW);

  // Stop front motors
  digitalWrite(leftFMotorForwardPin, LOW);
  digitalWrite(leftFMotorBackwardPin, LOW);
  digitalWrite(rightFMotorForwardPin, LOW);
  digitalWrite(rightFMotorBackwardPin, LOW);
}

void ResetPostion() {
  leftServos[0].write(leftServoInitialPositions[0]); // Front Left Servo
  rightServos[0].write(rightServoInitialPositions[0]); // Front Right Servo
  leftServos[1].write(leftServoInitialPositions[1]); // Middle Left Servo
  rightServos[1].write(rightServoInitialPositions[1]); // Middle Right Servo
  leftServos[2].write(leftServoInitialPositions[2]); // Rear Left Servo
  rightServos[2].write(rightServoInitialPositions[2]); // Rear Right Servo
}


void Stance() {
  // Adjust front and rear servos only
  leftServos[0].write(180 - leftTurnPositions[0]); // Front Left Servo
  rightServos[0].write(rightTurnPositions[0]);    // Front Right Servo
  leftServos[2].write(180 - leftTurnPositions[1]); // Rear Left Servo
  rightServos[2].write(180 - rightTurnPositions[1]);    // Rear Right Servo

  delay(150); // Short delay for smooth transition
}

void LeftTurn() {
  // Set front wheels to left turn position
  leftServos[0].write(leftTurnPosition); // Front Left Servo
  rightServos[0].write(20 - leftTurnPosition); // Front Right Servo

  leftServos[2].write(leftTurnPositions[1]); // Rear Left Servo
  rightServos[2].write(180 - rightTurnPositions[1]);    // Rear Right Servo
}

void RightTurn() {
  // Set front wheels to right turn position
  leftServos[0].write(170 - rightTurnPosition);  // Front Left Servo
  rightServos[0].write(150 - rightTurnPosition); // Front Right Servo

  leftServos[2].write(180 - leftTurnPositions[1]); // Rear Left Servo
  rightServos[2].write(100 - rightTurnPositions[1]);  // Rear Right Servo
}

void updateArmPositions() {
  // Gradually move the first arm to the target position for smoother motion
  if (currentArmPosition != currentArmTarget) {
    currentArmPosition += (currentArmPosition < currentArmTarget) ? 1 : -1;
    armServo.write(currentArmPosition - 30);
  }

  // Gradually move the second arm to the target position for smoother motion
  if (currentArmPositionTwo != currentArmTargetTwo) {
    currentArmPositionTwo += (currentArmPositionTwo < currentArmTargetTwo) ? 1 : -1;
    armServoTwo.write(currentArmPositionTwo);
  }

  // Gradually move the second arm to the target position for smoother motion
  if (currentArmPositionThree != currentArmTargetThree) {
    currentArmPositionThree += (currentArmPositionThree < currentArmTargetThree) ? 1 : -1;
    armServoThree.write(currentArmPositionThree - 42);
  }
}

void GripperControl() {
  unsigned long currentMillis = millis();

  // Check if the button state has changed from its last state
  if (joystickData.buttonOne != lastButtonState) {
    // Update the last button state to the current state
    lastButtonState = joystickData.buttonOne;
    gripperActivated = false; // Reset the activation flag whenever the button state changes
    gripperPreviousMillis = currentMillis; // Reset the timer
  }

  // Check if the interval has passed since the last activation
  if (!gripperActivated && currentMillis - gripperPreviousMillis >= gripperInterval) {
    // Perform action based on the button state
    if (joystickData.buttonOne == HIGH) {
      Serial.println("Clockwise");
      myStepper.step(-stepsPerRevolution); // Move the stepper clockwise
    } else {
      Serial.println("Counterclockwise");
      myStepper.step(stepsPerRevolution); // Move the stepper counterclockwise
    }
    gripperActivated = true; // Set the flag to prevent reactivation until the button state changes again
    gripperPreviousMillis = currentMillis; // Reset the timer for the next activation
  }
}


void BalanceServoControl() {
  static unsigned long previousMillis = 0; // Stores the last time the servo was updated
  const long interval = 10; // Interval at which to update the servo (milliseconds)

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time you updated the servo
    // PID constants
    float Kp = 0.0006;  // Proportional gain
    float Ki = 0.00001; // Integral gain
    float Kd = 200.5;  // Derivative gain

    // PID variables
    static float integral = 0;
    static float previous_error = 0;
    const int maxIntegral = 0.3; // Max integral wind-up limit
    const int maxPosChange = 0.005; // Max position change per loop iteration

    // Read the MPU6050's accelerometer and gyro data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Assuming ax gives us the tilt angle we need:
    float angleX = (ax / 16384.0) * 90; // Convert accelerometer data to an angle
    Serial.println(angleX);
    // Desired angle for balancing, adjust based on your setup's equilibrium position
    float desiredAngle = -8.6;

    // Calculate the error between the desired angle and the current angle
    float error = angleX - abs(desiredAngle);

    // Prevent integral windup
    integral = constrain(integral + error, -maxIntegral, maxIntegral);

    // Determine the change in error (rate of change of error)
    float derivative = error - previous_error;

    // Compute the total correction amount
    float correction = Kp * error + Ki * integral + Kd * derivative;

    // Update the previous error with the current error for the next iteration
    previous_error = error;

    int posChange = balanceServoPos + correction;
    if (abs(posChange - balanceServoPos) > 1) {
      posChange = balanceServoPos + (posChange > balanceServoPos ? 1 : -1);
    }
    balanceServoPos = constrain(posChange, 100, 90);
    balanceServo.write(balanceServoPos);
  }
}

void DistanceSensor () {
  // Clear the TRIG_PIN by setting it LOW:
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the TRIG_PIN high for 10 microseconds:
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the time for the echo:
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in meters using floating-point arithmetic:
  distance = (duration * 0.034) / 2 / 100.0; // Speed of sound wave divided by 2 (go and back) and convert from cm to m

  // Print the distance on the Serial Monitor with higher precision:
  Serial.print("Distance: ");
  Serial.print(distance, 3); // Print distance with three decimal places
  Serial.println(" m");
}

void playMoonlightSonataDarker() {
  // Moonlight Sonata main notes, simplified and transposed to lower octaves
  int melody[] = {
    329,  // E4
    311,  // D#4
    329,  // E4
    311,  // D#4
    329,  // E4
    247,  // B3
    293,  // D4
    261,  // C4
    220,  // A3
    131,  // E3
    165,  // E3
    220,  // A3
    247,  // B3
    165,  // E3
    207,  // G#3
    247,  // B3
    261,  // C4
    165,  // E3
    329,  // E4
    311,  // D#4
    329,  // E4
    311,  // D#4
    329,  // E4
    247,  // B3
    293,  // D4
    261,  // C4
    220,  // A3
    131,  // E3
    165,  // E3
    220,  // A3
    247,  // B3
    165,  // E3
    207,  // G#3
    247,  // B3
    261,  // C4
    165   // E3
  };
  int noteDurations[] = {400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400};

  for (int thisNote = 0; thisNote < 36; thisNote++) {
    int noteDuration = noteDurations[thisNote];
    tone(buzzerPin, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * .5;
    delay(pauseBetweenNotes);
    noTone(buzzerPin);
  }
}

void playToneForCase(int caseNumber) {
  int toneFrequency = 1000 + (100 * caseNumber);
  int toneDuration = 200; // og 200 milliseconds duration

  tone(buzzerPin, toneFrequency, toneDuration);
}

void FigureEight() {
  static unsigned long lastMillis = millis();  // Initialize lastMillis with the current time once
  static int step = 0;  // Step counter to keep track of which part of the sequence we're in

  unsigned long currentMillis = millis();  // Current time

  switch (step) {
    case 0:  // Start moving backwards
      Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 2000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 1:  // Left turn while moving backwards
      LeftTurn(),Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 48000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 2:  // Continue moving backwards
      Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 2000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 3:  // Briefly stop
      stopMotors();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 10) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 4:  // Left turn into another backwards motion
      LeftTurn(),Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 17750) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 5:  // Right turn while moving backwards
      RightTurn(),Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 17750) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 6:  // Move backwards again
      Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 2000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 7:  // Continue moving backwards
      //Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 2000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 8:  // Right turn while moving backwards
      RightTurn(),Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 48000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 9:  // Continue moving backwards
      Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 2000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 10:  // Brief stop before continuing
      stopMotors();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 10) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 11:  // Final right turn and move backwards
      RightTurn(),Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 17750) {
        lastMillis = currentMillis;
        step++;
      }
      break;
    case 12:  // Final left turn and move backwards to complete the figure eight
      LeftTurn(),Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 17750) {
        lastMillis = currentMillis;
        step = 0;  // Reset to start the pattern over or move to a stop state
        stopMotors();
      }
      break;
    case 13:  // Continue moving backwards
      //Backwards();
      //playToneForCase(step);
      if (currentMillis - lastMillis >= 2000) {
        lastMillis = currentMillis;
        step++;
      }
      break;
  }
}