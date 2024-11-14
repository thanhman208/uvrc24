
#define DEBUG

#include <Wire.h>
#include <PS4Controller.h>
#include <Adafruit_PWMServoDriver.h>

// PS4 controller MAC address
#define MAC_ADDRESS "44:17:93:DF:AB:50"

// Create instances
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // PWM instance for motor control

// Define motor control PWM pins
const int LEFT_FWD = 10;    // Left motor forward
const int LEFT_BCK = 11;    // Left motor backward
const int RIGHT_FWD = 12;   // Right motor forward
const int RIGHT_BCK = 13;   // Right motor backward

// PWM pins for IntakeIn and IntakeOut
const int INTAKEIN_FWD = 14;  // IntakeIn forward
const int INTAKEIN_BCK = 15;  // IntakeIn backward
const int INTAKEOUT_FWD = 8;  // IntakeOut forward
const int INTAKEOUT_BCK = 9;  // IntakeOut backward

// Servo control pins
const int SERVO1_PIN = 2;     // Servo 1 control pin
const int SERVO2_PIN = 3;     // Servo 2 control pin

// PS4 control variables
bool pov_control = false;     // Toggle between joystick and POV control
int boost_factor = 1;         // Factor to control boost
const int MAX_SPEED = 127;    // Max speed limit
const int NORMAL_SPEED = 63;  // Reduced speed limit for normal mode
const int DEADZONE = 10;      // Deadzone threshold to filter small signal noise

// Servo positions
int servo1Position = 0;       // Initial position for Servo 1
int servo2Position = 0;       // Initial position for Servo 2
bool servo1Toggle = false;    // Toggle state for Servo 1
bool servo2Toggle = false;    // Toggle state for Servo 2

void setup() {
  Serial.begin(115200);

  // Initialize PS4 controller
  PS4.begin(MAC_ADDRESS);
  Serial.println("PS4 controller initialized.");

  // Initialize motor controller (PWM)
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Set frequency for PCA9685
  pwm.setPWMFreq(50); // PWM frequency
  Wire.setClock(400000); // Set I2C frequency to 400kHz
}

void printButtonStates() {
  Serial.println("Button States:");
  Serial.printf("Right: %d, Down: %d, Up: %d, Left: %d\n", PS4.Right(), PS4.Down(), PS4.Up(), PS4.Left());
  Serial.printf("Square: %d, Cross: %d, Circle: %d, Triangle: %d\n", PS4.Square(), PS4.Cross(), PS4.Circle(), PS4.Triangle());
  Serial.printf("L1: %d, R1: %d, L2: %d, R2: %d\n", PS4.L1(), PS4.R1(), PS4.L2(), PS4.R2());
  Serial.printf("Share: %d, Options: %d, L3: %d, R3: %d\n", PS4.Share(), PS4.Options(), PS4.L3(), PS4.R3());
  Serial.printf("PS Button: %d, Touch Pad: %d\n", PS4.PSButton(), PS4.Touchpad());
  Serial.printf("L2 Value: %d, R2 Value: %d\n", PS4.L2Value(), PS4.R2Value());
  Serial.printf("Left Stick X: %d, Left Stick Y: %d\n", PS4.LStickX(), PS4.LStickY());
  Serial.printf("Right Stick X: %d, Right Stick Y: %d\n", PS4.RStickX(), PS4.RStickY());
  Serial.printf("Charging: %d, Audio: %d, Mic: %d\n", PS4.Charging(), PS4.Audio(), PS4.Mic());
  Serial.printf("Battery Level : %d\n", PS4.Battery());
  Serial.println("-------------------------");
}

void moveServoMicroseconds(int pin, int angle) {
  int microseconds = map(angle, 0, 180, 500, 2500); // Convert angle to microseconds (500-2500us)
  pwm.writeMicroseconds(pin, microseconds);
}

void loop() {
  if (PS4.isConnected()) {
    printButtonStates(); // Print all button states

    // Toggle POV control mode using the Options button
    if (PS4.Options()) {
      pov_control = !pov_control;
      delay(300); // debounce
      Serial.print("POV Control: ");
      Serial.println(pov_control ? "Enabled" : "Disabled");
    }

    // Boost speed control using Triangle button
    boost_factor = PS4.R1() ? 2 : 1; // Boost factor doubles speed when Triangle is pressed

    int leftMotorSpeed = 0, rightMotorSpeed = 0;
    int speedLimit = (boost_factor == 2) ? MAX_SPEED : NORMAL_SPEED; // Set speed limit based on boost

    // Control motor based on joystick inputs with deadzone filtering
    if (pov_control) {
      // POV mode: use D-pad for control with reduced speed in normal mode
      if (PS4.Up()) {
        leftMotorSpeed = speedLimit;
        rightMotorSpeed = speedLimit;
      } else if (PS4.Down()) {
        leftMotorSpeed = -speedLimit;
        rightMotorSpeed = -speedLimit;
      } else if (PS4.Left()) {
        leftMotorSpeed = -speedLimit;
        rightMotorSpeed = speedLimit;
      } else if (PS4.Right()) {
        leftMotorSpeed = speedLimit;
        rightMotorSpeed = -speedLimit;
      }
    } else {
      // Joystick mode with reduced speed in normal mode and deadzone filtering
      int leftStickY = PS4.LStickY();
      int rightStickY = PS4.RStickY();

      // Apply deadzone to filter out small signals
      leftMotorSpeed = (abs(leftStickY) > DEADZONE) ? constrain(leftStickY * boost_factor, -speedLimit, speedLimit) : 0;
      rightMotorSpeed = (abs(rightStickY) > DEADZONE) ? constrain(rightStickY * boost_factor, -speedLimit, speedLimit) : 0;
    }

    // Control left motor (forward and backward)
    pwm.setPWM(LEFT_FWD, 0, map(leftMotorSpeed, -MAX_SPEED, MAX_SPEED, 0, 4095));  // Left forward
    pwm.setPWM(LEFT_BCK, 0, map(-leftMotorSpeed, -MAX_SPEED, MAX_SPEED, 0, 4095)); // Left backward

    // Control right motor (forward and backward)
    pwm.setPWM(RIGHT_FWD, 0, map(rightMotorSpeed, -MAX_SPEED, MAX_SPEED, 0, 4095)); // Right forward
    pwm.setPWM(RIGHT_BCK, 0, map(-rightMotorSpeed, -MAX_SPEED, MAX_SPEED, 0, 4095)); // Right backward

    // Control IntakeIn (using R1 and R2)
    if (PS4.R2()) {
      pwm.setPWM(INTAKEIN_FWD, 0, 4095);  // IntakeIn forward (clockwise)
      pwm.setPWM(INTAKEIN_BCK, 0, 0);     // Stop IntakeIn backward
    } else if (PS4.Cross()) {
      pwm.setPWM(INTAKEIN_BCK, 0, 4095);  // IntakeIn backward (counter-clockwise)
      pwm.setPWM(INTAKEIN_FWD, 0, 0);     // Stop IntakeIn forward
    } else {
      pwm.setPWM(INTAKEIN_FWD, 0, 0);     // Stop IntakeIn forward
      pwm.setPWM(INTAKEIN_BCK, 0, 0);     // Stop IntakeIn backward
    }

    // Control IntakeOut (using L1 and L2)
    if (PS4.L1()) {
      pwm.setPWM(INTAKEOUT_BCK, 0, 4095);  // IntakeOut backward (counter-clockwise)
      pwm.setPWM(INTAKEOUT_FWD, 0, 0);     // Stop IntakeOut forward
    } else if (PS4.L2()) {
      pwm.setPWM(INTAKEOUT_FWD, 0, 4095);  // IntakeOut forward (clockwise)
      pwm.setPWM(INTAKEOUT_BCK, 0, 0);     // Stop IntakeOut backward
    } else {
      pwm.setPWM(INTAKEOUT_FWD, 0, 0);     // Stop IntakeOut forward
      pwm.setPWM(INTAKEOUT_BCK, 0, 0);     // Stop IntakeOut backward
    }

    // Control Servo 1 with Square button
    if (PS4.Square()) {
      servo1Toggle = !servo1Toggle;    // Toggle between 0 and 90 degrees
      servo1Position = servo1Toggle ? 90 : 0;
      moveServoMicroseconds(SERVO1_PIN, servo1Position);
      delay(200); // debounce
    }

    // Control Servo 2 with Circle button
    if (PS4.Circle()) {
      servo2Toggle = !servo2Toggle;    // Toggle between 0 and 90 degrees
      servo2Position = servo2Toggle ? 90 : 0;
      moveServoMicroseconds(SERVO2_PIN, servo2Position);
      delay(200); // debounce
    }

    delay(100); // Reduce serial print frequency
  }
}

