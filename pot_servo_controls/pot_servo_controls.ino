#include <ESP32Servo.h>
#define potPin 12
#define motor1Pin 1
#define motor2Pin 2
Servo servo1;
Servo servo2;

float x_est = 0.0;    // Initial estimate
float P = 1.0;        // Initial estimate uncertainty
float Q = 0.05;       // Process noise (tune this)
float R = 5.0;        // Measurement noise (tune this)
float K;              // Kalman gain

void setup() {
  Serial.begin(115200);
  servo1.attach(motor1Pin);
  servo2.attach(motor2Pin);
}

void loop() {
  int adcValue = (analogRead(potPin));

  //predict
  P = P + Q; //use this to calculate Kalman Gain

  //update
  K = P / (P + R); //Calculate Kalman Gain
  x_est = x_est + K * (adcValue - x_est); //Calculate current estimate
  P = (1 - K) * P; //calculate the error of the new estimate, and loop

  int servoAngle = map(x_est, 100, 8000, 0, 180);
  servoAngle = constrain(servoAngle, 0, 180); // Ensure the angle is within bounds

  servo1.write(servoAngle);
  servo2.write(servoAngle); //I have one pot controlling two motors. THe two sensors move in unison, not independently.

  Serial.println(x_est);
  delay(10);
}
