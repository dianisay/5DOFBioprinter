#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 Servo test!");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    // Read angles for three servos
    int theta0 = readAngleFromSerial();
    int theta1 = readAngleFromSerial();
    int theta2 = readAngleFromSerial();
    
    if (theta0 >= 0 && theta0 <= 180 && theta1 >= 0 && theta1 <= 180 && theta2 >= 0 && theta2 <= 180) {
      // Convert angles to pulse lengths and set the servos
      int pulse0 = map(theta0, 0, 180, SERVOMIN, SERVOMAX);
      int pulse1 = map(theta1, 0, 180, SERVOMIN, SERVOMAX);
      int pulse2 = map(theta2, 0, 180, SERVOMIN, SERVOMAX);
      
      pwm.setPWM(0, 0, pulse0);
      pwm.setPWM(1, 0, pulse1);
      pwm.setPWM(2, 0, pulse2);
      
      Serial.print("Servo 0 set to ");
      Serial.print(theta0);
      Serial.print(" degrees, pulse: ");
      Serial.println(pulse0);
      
      Serial.print("Servo 1 set to ");
      Serial.print(theta1);
      Serial.print(" degrees, pulse: ");
      Serial.println(pulse1);
      
      Serial.print("Servo 2 set to ");
      Serial.print(theta2);
      Serial.print(" degrees, pulse: ");
      Serial.println(pulse2);
    }
  }
}

int readAngleFromSerial() {
  while (Serial.available() == 0) {
    // Wait for input
  }
  int angle = Serial.parseInt();
  Serial.read(); // Clear the newline character from the buffer
  return angle;
}
