///////// CONFIGURATION STARTS HERE /////////

// Joystick calibration
#define CONTROL_CENTER_X  509 // joystick's centered X-value
#define CONTROL_CENTER_Y  511 // joystick's centered Y-VALUE
#define CONTROL_RIGHT     632 // joystick's right-most X-value
#define CONTROL_LEFT      385 // joystick's left-most X-value
#define CONTROL_UP        638 // joystick's full-forward Y-value
#define CONTROL_DOWN      380 // joystick's full-backward Y-value

// Acceleration configuration (larger number means quicker acceleration)
#define ACCELERATION_FORWARD   0.2 // amount of forward acceleration
#define DECELERATION_FORWARD   0.6 // amount of forward deceleration
#define ACCELERATION_BACKWARD  0.1 // amount of backward acceleration
#define DECELERATION_BACKWARD  0.8 // amount of backward deceleration

// Speed configuration
#define FASTEST_FORWARD   1750 // forward limit, 2000=max, 1500=no movement
#define FASTEST_BACKWARD  1250 // backward limit, 2000=max, 1500=no movement

#define TURN_SPEED  0.6 // turning limit, 1=max, 0=no turn

#define SLOW_TURNING_WHEN_MOVING  0.6 // TURN_SPEED multiplier while vehicle is moving

// Pin configuration
#define joyXPin  15 // pin connected to the X-axis of the joystick
#define joyYPin  16 // pin connected to the Y-axis of the joystick
#define leftMotorControllerPin   10 // pin connected to the signal pin of the left motor controller
#define rightMotorControllerPin  9  // pin connected to the signal pin of the right motor controller

// Motor controller configuration
#define startGoingLargeVal  1516 // when the motor should start moving
#define startGoingSmallVal  1484 // when the motor should start moving

///////// CONFIGURATION ENDS HERE /////////

#include <Servo.h>

Servo rightMotorController; // right motor controller (uses servo signals)
Servo leftMotorController; // left motor controller (uses servo signals)
float leftMotorWriteVal = 1500.0; // left motor controller speed
float rightMotorWriteVal = 1500.0; // right motor controller speed
float rightMotorTryVal = 1500.0; // right motor controller speed before acceleration calculations
float leftMotorTryVal = 1500.0; // left motor controller speed before acceleration calculations
float joyXVal = 0.0; // joystick's current X value
float joyYVal = 0.0; // joystick's current Y value
float speedVal = 1500.0; // forward speed
float turnVal = 0.0; // turn amount

unsigned int timezie = 0; // helps with print timing for debugging and calibrating

void setup() {  // run once when the car is turned on
  Serial.begin(19200); // start serial port (for debugging and calibrating)
  Serial.println();
  Serial.println("  wait 5 seconds for the ESC to calibrate itself");
  Serial.println("current values:");
  Serial.println("---------");
  Serial.print("CONTROL_RIGHT: ");
  Serial.println(CONTROL_RIGHT);
  Serial.print("CONTROL_CENTER_X: ");
  Serial.println(CONTROL_CENTER_X);
  Serial.print("CONTROL_LEFT: ");
  Serial.println(CONTROL_LEFT);
  Serial.print("CONTROL_UP: ");
  Serial.println(CONTROL_UP);
  Serial.print("CONTROL_CENTER_Y: ");
  Serial.println(CONTROL_CENTER_Y);
  Serial.print("CONTROL_DOWN: ");
  Serial.println(CONTROL_DOWN);
  Serial.print("ACCELERATION_FORWARD: ");
  Serial.println(ACCELERATION_FORWARD);
  Serial.print("DECELERATION_FORWARD: ");
  Serial.println(DECELERATION_FORWARD);
  Serial.print("ACCELERATION_BACKWARD: ");
  Serial.println(ACCELERATION_BACKWARD);
  Serial.print("DECELERATION_BACKWARD: ");
  Serial.println(DECELERATION_BACKWARD);
  Serial.print("FASTEST_FORWARD: ");
  Serial.println(FASTEST_FORWARD);
  Serial.print("FASTEST_BACKWARD: ");
  Serial.println(FASTEST_BACKWARD);
  Serial.print("TURN_SPEED: ");
  Serial.println(TURN_SPEED);
  Serial.print("SLOW_TURNING_WHEN_MOVING");
  Serial.println(SLOW_TURNING_WHEN_MOVING);
  Serial.print("joyXPin: ");
  Serial.println(joyXPin);
  Serial.print("joyYPin: ");
  Serial.println(joyYPin);
  Serial.print("leftMotorControllerPin: ");
  Serial.println(leftMotorControllerPin);
  Serial.print("rightMotorControllerPin: ");
  Serial.println(rightMotorControllerPin);
  Serial.print("startGoingLargeVal: ");
  Serial.println(startGoingLargeVal);
  Serial.print("startGoingSmallVal: ");
  Serial.println(startGoingSmallVal);
  Serial.println();

  pinMode(joyXPin, INPUT); // set joystick pin as an input
  pinMode(joyYPin, INPUT); // set joystick pin as an input

  leftMotorController.attach(leftMotorControllerPin); // use pin for the motor controller
  rightMotorController.attach(rightMotorControllerPin); // use pin for the motor controller

  leftMotorController.writeMicroseconds(1500); // tell the motor controller to not move
  rightMotorController.writeMicroseconds(1500); // tell the motor controller to not move

  delay(3000); // wait for the motor controller to calibrate itself

  leftMotorController.writeMicroseconds(1540); // pulse motors slightly
  rightMotorController.writeMicroseconds(1540); // pulse motors slightly

  delay(250); // pause while motors are pulsing

  leftMotorController.writeMicroseconds(1500); // stop the motors
  rightMotorController.writeMicroseconds(1500); // stop the motors
}

void loop() {
  joyXVal = analogRead(joyXPin); // read joystick X value
  joyYVal = analogRead(joyYPin); // read joystick Y value

  if (CONTROL_LEFT < CONTROL_RIGHT) { //if the value for when the joystick is towards the left is less then the value for when the joystick is towards the right...
    if (joyXVal < CONTROL_CENTER_X) { //...and if going left...
      turnVal = map(joyXVal, CONTROL_LEFT, CONTROL_CENTER_X, TURN_SPEED * -500, 0); //set turnVal from the joystick
    }

    if (joyXVal > CONTROL_CENTER_X) {
      turnVal = map(joyXVal, CONTROL_RIGHT, CONTROL_CENTER_X, TURN_SPEED * 500, 0);
    }
  }

  if (CONTROL_LEFT > CONTROL_RIGHT) {
    if (joyXVal > CONTROL_CENTER_X) {
      turnVal = map(joyXVal, CONTROL_LEFT, CONTROL_CENTER_X, TURN_SPEED * -500, 0);
    }

    if (joyXVal < CONTROL_CENTER_X) {
      turnVal = map(joyXVal, CONTROL_RIGHT, CONTROL_CENTER_X, TURN_SPEED * 500, 0);
    }
  }

  if (CONTROL_UP < CONTROL_DOWN) {
    if (joyYVal < CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_UP, CONTROL_CENTER_Y, FASTEST_FORWARD, 1500);
    }

    if (joyYVal > CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_DOWN, CONTROL_CENTER_Y, FASTEST_BACKWARD, 1500);
    }
  }

  if (CONTROL_UP > CONTROL_DOWN) {
    if (joyYVal > CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_UP, CONTROL_CENTER_Y, FASTEST_FORWARD, 1500);
    }

    if (joyYVal < CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_DOWN, CONTROL_CENTER_Y, FASTEST_BACKWARD, 1500);
    }
  }

  if (speedVal >= 1500) {
    turnVal = map(turnVal, TURN_SPEED * -500, TURN_SPEED * 500, map(abs(speedVal - 1500), 0, FASTEST_FORWARD - 1500, TURN_SPEED * -500, TURN_SPEED * -SLOW_TURNING_WHEN_MOVING * 500), map(abs(speedVal - 1500), 0, FASTEST_FORWARD - 1500, TURN_SPEED * 500, TURN_SPEED * SLOW_TURNING_WHEN_MOVING * 500));
  } else {
    turnVal = map(turnVal, TURN_SPEED * -500, TURN_SPEED * 500, map(abs(1500 - speedVal), 0, 1500 - FASTEST_BACKWARD, TURN_SPEED * -500, TURN_SPEED * -SLOW_TURNING_WHEN_MOVING * 500), map(abs(1500 - speedVal), 0, 1500 - FASTEST_BACKWARD, TURN_SPEED * 500, TURN_SPEED * SLOW_TURNING_WHEN_MOVING * 500));
  }

  leftMotorTryVal = speedVal + turnVal; //use the forward-backward speed value and the turn value to find the left motor value
  rightMotorTryVal = speedVal - turnVal; //use the forward-backward speed value and the turn value to find the right motor value
  ////////////////////////////////////code to make the car accelerate slowly
  ////for left motor
  if (leftMotorTryVal < startGoingLargeVal && leftMotorTryVal > startGoingSmallVal) { //if the motor wouldn't be moving enough anyway...
    leftMotorTryVal = 1500; //...don't turn on the motor.
  }

  if (leftMotorTryVal < 1500) { //if going backwards...
    if (leftMotorTryVal <= leftMotorWriteVal - ACCELERATION_BACKWARD) { //...if trying to go faster backwards...
      leftMotorWriteVal = leftMotorWriteVal - ACCELERATION_BACKWARD; //...accelerate by ACCELERATION_BACKWARD
    }

    if (leftMotorTryVal > leftMotorWriteVal + DECELERATION_BACKWARD) { //if trying to go slower backwards...
      leftMotorWriteVal = leftMotorWriteVal + DECELERATION_BACKWARD; //...decelerate by DECELERATION_BACKWARD
    }
  }

  if (leftMotorTryVal >= 1500) { //if going forwards...
    if (leftMotorTryVal >= leftMotorWriteVal + ACCELERATION_FORWARD) { //...if trying to go faster forwards...
      leftMotorWriteVal = leftMotorWriteVal + ACCELERATION_FORWARD; //...accelerate by ACCELERATION_FORWARD
    }

    if (leftMotorTryVal < leftMotorWriteVal - DECELERATION_FORWARD) { // if trying to go slower forwards...
      leftMotorWriteVal = leftMotorWriteVal - DECELERATION_FORWARD; //...decelerate by DECELERATION_FORWARD
    }
  }

  ////////for right motor
  if (rightMotorTryVal < startGoingLargeVal && rightMotorTryVal > startGoingSmallVal) { //if the motor wouldn't be moving enough anyway...
    rightMotorTryVal = 1500; //...don't turn on the motor.
  }

  if (rightMotorTryVal < 1500) { //if going backwards...
    if (rightMotorTryVal <= rightMotorWriteVal - ACCELERATION_BACKWARD) { //...if trying to go faster backwards...
      rightMotorWriteVal = rightMotorWriteVal - ACCELERATION_BACKWARD; //...accelerate by ACCELERATION_BACKWARD
    }
    if (rightMotorTryVal > rightMotorWriteVal + DECELERATION_BACKWARD) { //if trying to go slower backwards...
      rightMotorWriteVal = rightMotorWriteVal + DECELERATION_BACKWARD; //...decelerate by DECELERATION_BACKWARD
    }
  }

  if (rightMotorTryVal >= 1500) { //if going forwards...
    if (rightMotorTryVal >= rightMotorWriteVal + ACCELERATION_FORWARD) { //...if trying to go faster forwards...
      rightMotorWriteVal = rightMotorWriteVal + ACCELERATION_FORWARD; //...accelerate by ACCELERATION_FORWARD
    }
    if (rightMotorTryVal < rightMotorWriteVal - DECELERATION_FORWARD) { // if trying to go slower forwards...
      rightMotorWriteVal = rightMotorWriteVal - DECELERATION_FORWARD; //...decelerate by DECELERATION_FORWARD
    }
  }

  ///////////////////////////////////////////////send values to servo and motor controller////////////////////////
  leftMotorWriteVal = constrain(leftMotorWriteVal, FASTEST_BACKWARD, FASTEST_FORWARD); //make sure the value is in the right range
  rightMotorWriteVal = constrain(rightMotorWriteVal, FASTEST_BACKWARD, FASTEST_FORWARD); //make sure the value is in the right range
  rightMotorController.writeMicroseconds(int(rightMotorWriteVal)); //send a servo signal to the motor controller for the Motor value (after acceleration)
  leftMotorController.writeMicroseconds(int(leftMotorWriteVal)); //send a servo signal to the motor controller for the Motor value (after acceleration)
  /////////////////////////////////////print values to help figure out problems

  if (timezie > 500) { //around twice a second
    timezie = 0;

    Serial.print("X joystick value: ");
    Serial.print(joyXVal);
    Serial.print(", ");
    Serial.print("Y joystick value: ");
    Serial.print(joyYVal);

    Serial.println();
    Serial.print("speedVal: ");
    Serial.println(speedVal);

    Serial.print("turnVal: ");
    Serial.println(turnVal);
    Serial.println();

    Serial.println("motor values: ");
    Serial.print(leftMotorTryVal);
    Serial.print(", ");
    Serial.print(rightMotorTryVal);
    Serial.println();

    Serial.print(leftMotorWriteVal);
    Serial.print(", ");
    Serial.print(rightMotorWriteVal);
    Serial.println();
    Serial.println();
    Serial.println();
  }

  timezie++;
}
