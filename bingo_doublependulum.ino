#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define the servos connected to the PCA9685
const int numServos = 5;
const int servoPins[numServos] = {0, 1, 2, 3, 4}; // Channels on PCA9685
int servoMin = 150; // Minimum pulse length out of 4096
int servoMax = 600; // Maximum pulse length out of 4096

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 Servo Driver Test!");

  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz

  // Give a brief pause to let the servos reach their home positions
  delay(1000);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming angles
    float angles[numServos];
    for (int i = 0; i < numServos; i++) {
      while (Serial.available() == 0) {
        // Wait for the next angle
      }
      angles[i] = Serial.parseFloat();
      Serial.read(); // To clear the newline character
    }

    // Set the servo positions
    for (int i = 0; i < numServos; i++) {
      int pulse = map(angles[i] * 10, 0, 1800, servoMin, servoMax);
      pwm.setPWM(servoPins[i], 0, pulse);
    }
  }
}
