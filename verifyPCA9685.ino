#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define the servo channel on the PCA9685
const int servoChannel = 0;
int servoMin = 150; // Minimum pulse length out of 4096
int servoMax = 600; // Maximum pulse length out of 4096

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 Servo Driver Test!");

  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz

  // Give a brief pause to let the servos reach their home positions
  delay(1000);

  // Move servo to minimum position
  pwm.setPWM(servoChannel, 0, servoMin);
  delay(1000); // Wait for 1 second

  // Move servo to maximum position
  pwm.setPWM(servoChannel, 0, servoMax);
  delay(1000); // Wait for 1 second

  // Move servo to middle position
  int midPosition = (servoMin + servoMax) / 2;
  pwm.setPWM(servoChannel, 0, midPosition);

    // Move servo to minimum position
  pwm.setPWM(servoChannel, 0, servoMin);
  delay(1000); // Wait for 1 second

}

void loop() {
  // No need to do anything in the loop for this test
}
