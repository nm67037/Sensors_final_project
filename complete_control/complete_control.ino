//the code for the FSR was sourced from the Adafruit website
//the ultrasonic code was sourced from arduino's website
//PIR code from RobotsDinosaurs:
////https://github.com/RobotsDinosaurs/Arduino/blob/main/PIR/PIR.ino
//^source

#include <ESP32Servo.h>
#include <Arduino.h>
//potentiometer parameters:
#define potPin 12
#define motor1Pin 1
#define motor2Pin 2

#define MINUTE 60000 // for converting milliseconds to a minute
#define SECOND 1000 // for converting milliseconds to a second

Servo servo1;
Servo servo2;
//#define motor2Pin 10

float x_est = 0.0;    // Initial estimate
float P = 1.0;        // Initial estimate uncertainty
float Q = 0.05;       // Process noise (tune this)
float R = 5.0;        // Measurement noise (tune this)
float K;              // Kalman gain

//pir parameters
int ledPin = 11;                // pin for the LED
int pirPin = 3;                // signal pin of the PIR sensor

int lastPirVal = LOW;          // the last value of the PIR sensor
int pirVal;                    // the current value of the PIR sensor
unsigned long myTime;          // number of milliseconds passed since the Arduino started running the code itself
char printBuffer[128]; 

//ultrasonic parameters:
const int trig_pin = 4;
const int echo_pin = 6;
float timing = 0.0;
float distance = 0.0;

//Hall parameters:
#define Hall_Sensor_Pin 10

int fsrAnalogPin = 5; 
//int Hall_Sensor_Pin = 10;
int fsrReading;       
int threshold = 600;
 
bool state = false;   
bool running = false;
bool motor_condition = false;


void setup() {
  Serial.begin(115200);
  pinMode(Hall_Sensor_Pin, INPUT);
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(Hall_Sensor_Pin,INPUT);
  pinMode(ledPin, OUTPUT);    // declare LED as output
  pinMode(pirPin, INPUT);     // declare PIR sensor as input
  digitalWrite(trig_pin, LOW);
  servo1.attach(motor1Pin);
  servo2.attach(motor2Pin);
}

void loop() {
  // Polling algorithm to check FSR state
  while (!running) { // Keep polling until "running" is true
    fsrReading = analogRead(fsrAnalogPin);
    Serial.println(fsrReading);
    if (fsrReading >= threshold) {
      state = true;
      running = true;
      motor_condition = true; 
      Serial.println("FSR pressed. Running = true");
    } else {
      state = false;
      Serial.println("FSR not pressed. Polling...");
    }

    delay(500); // Short delay for stability
  }

  // Code that runs once "running" is true
  if (running) {
    //Serial.println("Running part of the code...");
    
    float voltage;
    voltage = analogRead(Hall_Sensor_Pin);
    //voltage = analogRead(Hall_Sensor_Pin);
   // Serial.println(voltage);
    if (voltage>=3700)
    {
     // Serial.println("Bait is gone!");
    }
    
   // Serial.println(hall_voltage);
    
    digitalWrite(trig_pin, LOW);
    delay(2);
    
    digitalWrite(trig_pin, HIGH);
    delay(10);
    digitalWrite(trig_pin, LOW);
    
    timing = pulseIn(echo_pin, HIGH);
    distance = (timing * 0.034) / 2;
    distance = constrain(distance, 0, 300);
    
     // Serial.print("Distance: ");
      //Serial.print(distance);
      //Serial.println("cm | ");
  

    int pot_adcValue = (analogRead(potPin));
    //Serial.println(pot_adcValue);

  //predict
    P = P + Q; //use this to calculate Kalman Gain

    //update
    K = P / (P + R); //Calculate Kalman Gain
    x_est = x_est + K * (pot_adcValue - x_est); //Calculate current estimate
    P = (1 - K) * P; //calculate the error of the new estimate, and loop

    int servoAngle = map(x_est, 100, 8100, 0, 180);
    servoAngle = constrain(servoAngle, 0, 180); // Ensure the angle is within bounds
    //Serial.println(x_est);
    //Serial.println(servoAngle);

    servo1.write(servoAngle);
    servo2.write(servoAngle); //I have one pot controlling two motors. THe two sensors move in unison, not independently.

    //Serial.println(x_est);
    bool animal;
    pirVal = digitalRead(pirPin);  // read current input value
    if (pirVal == HIGH) { // movement detected  
      digitalWrite(ledPin, HIGH); }  // turn LED on
    else { // no movement detected
      digitalWrite(ledPin, LOW);} // turn LED off
    Serial.print(distance);
    Serial.print('\t');
    Serial.println(pirVal), //something wrong with this line

    if (distance < 500) {
      if (pirVal == HIGH) {
        animal == HIGH;
        Serial.println("Animal Detected!");
        //WRITE ALGORITHM HERE!!!
      }
    }
   
    delay(50); // Simulate processing time
  }
}
